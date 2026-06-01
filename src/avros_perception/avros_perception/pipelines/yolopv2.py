"""
YOLOPv2 lane-line segmentation pipeline (learned, ONNX Runtime).

YOLOPv2 (CAIC-AD/YOLOPv2, arXiv:2208.11434, MIT licence) is a panoptic
driving-perception net with three heads: object detection, drivable-area
segmentation, and LANE-LINE segmentation. We use ONLY the lane-line head and
map every detected lane pixel to ``class_id_lane`` (1) — the exact same
single-class mono8 contract the classical pipelines (sooner25/adaptive) emit,
so nothing downstream (kiwicampus semantic layer / Nav2) changes.

Why a learned model at all
--------------------------
The classical ``adaptive`` pipeline is at its limit on faint/worn paint and on
asphalt whose aggregate speckle has line-comparable local contrast — the code's
own comments repeatedly conclude "ONNX is the robust answer"
(``perception.yaml``, ``adaptive.py``). YOLOPv2 was validated zero-shot on the
captured IGVC practice frames (``docs/cv_yolopv2_2026_06_01/``): on continuous
high-contrast white tape it traces the boundary cleanly with NO false positives
on barrels / grass / tent, where the classical pipeline needs per-scene tuning.
The honest failure mode is SHORT isolated stubs (it was trained on long
continuous/dashed highway lanes) — see the validation report.

Ground-truth I/O (derived from + empirically confirmed against the official
``demo.py`` / ``utils/utils.py`` and the exported ONNX):

  * INPUT  ``images`` : (1, 3, 384, 640) float32, RGB, /255.0 only (NO ImageNet
    mean/std), NCHW. Built by: optional resize to ``pre_resize`` (the demo's
    hard-coded 1280x720 BDD framing) -> keep-aspect letterbox to the net size
    with pad value 114 -> BGR->RGB -> CHW -> /255.
  * OUTPUT lane head : (1, 1, 384, 640), values already in [0, 1] (sigmoid). The
    official ``lane_line_mask`` does ``torch.round`` == threshold 0.5. We expose
    the threshold (``yolopv2_lane_thresh``) so it is tunable for recall.
  * (the drivable-area head (1, 2, 384, 640) and the detection head are present
    in the full model but unused here; requesting only the lane output at
    inference prunes the detection compute.)

The lane output is identified by SHAPE (the (1, 1, H, W) output), not by the
brittle traced tensor name, so a re-export with different node names still works.

Runtime / Jetson notes
----------------------
onnxruntime is imported lazily (in ``warmup``) so this module stays importable
for unit tests on a box without onnxruntime or the 156 MB model. ExecutionProviders
are filtered against ``ort.get_available_providers()`` and tried in the configured
order (TensorRT -> CUDA -> CPU on the Jetson), so a CPU-only box (the dev laptop)
degrades to ``CPUExecutionProvider`` instead of raising. ``intra_op_num_threads``
is capped (default 2) so CPU pre/post-processing cannot starve the 20 Hz MPPI loop
on the CPU-saturated Jetson. A missing model / failed load logs ONCE and the
pipeline returns an empty mask every frame — it must never crash ``_on_synced``
(that would kill ALL perception).
"""

import os

import cv2
import numpy as np

from avros_perception.pipelines.base import Pipeline, PipelineResult


# ---- pure helpers (module level so tests need neither onnxruntime nor the model) ----

def letterbox_exact(img, net_hw, color=(114, 114, 114)):
    """Keep-aspect resize + pad ``img`` to EXACTLY ``net_hw`` (H, W).

    Returns (padded_img, (top, bottom, left, right)). Unlike the demo's
    ``auto=True`` minimum-rectangle variant, this pads to the fixed net size so
    it always matches a fixed-shape ONNX input regardless of source aspect.
    """
    h, w = img.shape[:2]
    nh, nw = int(net_hw[0]), int(net_hw[1])
    r = min(nh / h, nw / w)
    new_w, new_h = int(round(w * r)), int(round(h * r))
    if (w, h) != (new_w, new_h):
        img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    dw, dh = nw - new_w, nh - new_h
    top, left = dh // 2, dw // 2
    bottom, right = dh - top, dw - left
    img = cv2.copyMakeBorder(
        img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )
    return img, (top, bottom, left, right)


def preprocess(bgr, net_hw, pre_resize=None):
    """BGR HxWx3 uint8 -> (NCHW float32 input, pads).

    Replicates the official YOLOPv2 demo preprocessing: optional pre-resize to
    the BDD-style framing (``pre_resize`` = (W, H), the demo uses 1280x720),
    letterbox to the net size, BGR->RGB, CHW, /255.0.
    """
    src = bgr
    if pre_resize:
        src = cv2.resize(bgr, (int(pre_resize[0]), int(pre_resize[1])),
                         interpolation=cv2.INTER_LINEAR)
    lb, pads = letterbox_exact(src, net_hw)
    rgb = lb[:, :, ::-1].transpose(2, 0, 1)            # BGR->RGB, HWC->CHW
    x = np.ascontiguousarray(rgb, dtype=np.float32) / 255.0
    return x[None], pads                                # (1,3,H,W)


def decode_lane(lane_out, pads, orig_hw, thresh):
    """Lane head tensor -> (binary uint8 HxW, prob uint8 HxW) at original size.

    Handles both the YOLOPv2 single-channel sigmoid head ((1,1,H,W), thresholded)
    and a hypothetical 2-channel head ((1,2,H,W), argmax==1). Strips the letterbox
    padding (``pads``) then NEAREST-resizes to the original frame so thin lines
    survive.
    """
    arr = np.asarray(lane_out)
    if arr.ndim == 4:
        c = arr.shape[1]
        if c == 1:
            prob = arr[0, 0]
        else:                                           # 2-ch: softmax, take ch1
            a = arr[0].astype(np.float32)
            e = np.exp(a - a.max(axis=0, keepdims=True))
            prob = (e[1] / e.sum(axis=0))
    elif arr.ndim == 3:                                 # (1,H,W) or (C,H,W)
        prob = arr[0] if arr.shape[0] == 1 else arr[1]
    else:
        prob = arr
    top, bottom, left, right = pads
    h, w = prob.shape
    prob = prob[top:h - bottom if bottom else h, left:w - right if right else w]
    binm = (prob > float(thresh)).astype(np.uint8)
    oh, ow = orig_hw
    binm = cv2.resize(binm, (ow, oh), interpolation=cv2.INTER_NEAREST)
    probu = cv2.resize(
        np.clip(prob * 255.0, 0, 255).astype(np.uint8), (ow, oh),
        interpolation=cv2.INTER_NEAREST,
    )
    return binm, probu


def filter_min_area(binm, min_area):
    """Drop connected components smaller than ``min_area`` px. 0 disables."""
    if min_area <= 0:
        return binm
    n, labels, stats, _ = cv2.connectedComponentsWithStats(binm, 8)
    out = np.zeros_like(binm)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            out[labels == i] = 1
    return out


# ---- pipeline ----

class YolopV2Pipeline(Pipeline):
    """YOLOPv2 lane-line head -> single-class (class_id_lane) mono8 mask."""

    def __init__(self, params, logger=None):
        super().__init__(params, logger)
        self._sess = None
        self._input_name = None
        self._lane_name = None
        self._net_hw = None          # (H, W) read from the ONNX input
        self._load_failed = False    # latch so we log the failure only once

    # -- params --
    def _p(self, key, default):
        v = self.params.get(key, default)
        return default if v is None else v

    def _resolve_model_path(self):
        path = str(self._p('yolopv2_model_path', '') or '')
        if not path:
            path = os.environ.get('YOLOPV2_ONNX_PATH', '')
        if not path:
            path = os.path.expanduser('~/yolopv2_lane_384x640.onnx')
        return os.path.expanduser(path)

    def _log(self, level, msg):
        if self.logger is not None:
            getattr(self.logger, level)(f'[yolopv2] {msg}')

    def warmup(self):
        """Build the ONNX Runtime session and run one dummy inference.

        The dummy run is load-bearing on the Jetson: the first TensorRT-EP
        inference BUILDS the engine (can take minutes) — doing it here avoids
        stalling the first real frame's _on_synced callback.
        """
        if self._sess is not None or self._load_failed:
            return
        try:
            import onnxruntime as ort
        except Exception as e:                          # noqa: BLE001
            self._load_failed = True
            self._log('error', f'onnxruntime import failed ({e}); '
                               'pipeline will emit empty masks')
            return

        model_path = self._resolve_model_path()
        if not os.path.isfile(model_path):
            self._load_failed = True
            self._log('error',
                      f'model not found at {model_path} — run '
                      'scripts/fetch_yolopv2_model.py and set yolopv2_model_path '
                      '(or $YOLOPV2_ONNX_PATH). Emitting empty masks.')
            return

        requested = list(self._p('yolopv2_providers',
                                 ['TensorrtExecutionProvider',
                                  'CUDAExecutionProvider',
                                  'CPUExecutionProvider']))
        avail = set(ort.get_available_providers())
        providers = [p for p in requested if p in avail] or ['CPUExecutionProvider']

        fp16 = bool(self._p('yolopv2_fp16', True))
        trt_cache = os.path.expanduser(str(self._p('yolopv2_trt_cache',
                                                   '~/.cache/yolopv2_trt')))
        prov_with_opts = []
        for p in providers:
            if p == 'TensorrtExecutionProvider':
                os.makedirs(trt_cache, exist_ok=True)
                prov_with_opts.append((p, {
                    'trt_fp16_enable': fp16,
                    'trt_engine_cache_enable': True,
                    'trt_engine_cache_path': trt_cache,
                }))
            elif p == 'CUDAExecutionProvider':
                prov_with_opts.append((p, {}))
            else:
                prov_with_opts.append(p)

        so = ort.SessionOptions()
        # Cap BOTH thread pools (default 1/1). The Jetson Orin is CPU-saturated;
        # an unbounded ORT pool grabs all cores and starves the 20 Hz MPPI loop
        # + EKF + the in-process kiwicampus layer (same failure mode as RViz-on-
        # Jetson — see CLAUDE.md). The conv math runs on the GPU EP anyway; only
        # pre/post-process touch the CPU.
        n_threads = int(self._p('yolopv2_intra_op_threads', 1))
        so.intra_op_num_threads = n_threads
        so.inter_op_num_threads = 1
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        try:
            self._sess = ort.InferenceSession(
                model_path, sess_options=so, providers=prov_with_opts)
        except Exception as e:                          # noqa: BLE001
            self._load_failed = True
            self._log('error', f'session creation failed ({e}); empty masks')
            return

        self._input_name = self._sess.get_inputs()[0].name
        ishape = self._sess.get_inputs()[0].shape       # e.g. [1,3,384,640]
        try:
            self._net_hw = (int(ishape[2]), int(ishape[3]))
        except (TypeError, ValueError):                 # dynamic dims -> fallback
            self._net_hw = (int(self._p('yolopv2_net_h', 384)),
                            int(self._p('yolopv2_net_w', 640)))

        # Identify the lane output by shape: the (1, 1, H, W) one.
        lane = None
        for o in self._sess.get_outputs():
            s = o.shape
            if len(s) == 4 and (s[1] == 1 or s[1] == 2):
                lane = o.name
                if s[1] == 1:                           # prefer the 1-ch lane head
                    break
        self._lane_name = lane or self._sess.get_outputs()[-1].name

        self._log('info',
                  f'loaded {os.path.basename(model_path)} | EP={self._sess.get_providers()} '
                  f'| net_hw={self._net_hw} | lane_out={self._lane_name}')
        # Build engine / prime caches now, not on the first frame.
        try:
            dummy = np.zeros((1, 3, self._net_hw[0], self._net_hw[1]), np.float32)
            self._sess.run([self._lane_name], {self._input_name: dummy})
            self._log('info', 'warmup inference OK')
        except Exception as e:                          # noqa: BLE001
            self._log('warn', f'warmup inference raised (continuing): {e}')

    def _reshape_poly(self, seq):
        if not seq:
            return []
        if len(seq) % 2 != 0:
            raise ValueError(
                f'sky_roi_poly flat list must have even length, got {len(seq)}')
        return [(float(seq[i]), float(seq[i + 1])) for i in range(0, len(seq), 2)]

    def _roi_polygon_px(self, h, w):
        raw = self.params.get('sky_roi_poly',
                              [0.0, 0.0, 1.0, 0.0, 1.0, 0.40, 0.0, 0.40])
        poly = self._reshape_poly(raw)
        if not poly:
            return None
        return np.array([(int(x * (w - 1)), int(y * (h - 1))) for x, y in poly],
                        dtype=np.int32)

    def run(self, bgr, depth=None):
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f'YolopV2Pipeline expects HxWx3 BGR; got {bgr.shape}')
        h, w = bgr.shape[:2]
        class_id_lane = int(self._p('class_id_lane', 1))

        if self._sess is None and not self._load_failed:
            self.warmup()
        if self._sess is None:                          # load failed -> empty mask
            z = np.zeros((h, w), dtype=np.uint8)
            return PipelineResult(mask=z, confidence=z)

        # Live-tunable per-frame params.
        thresh = float(self._p('yolopv2_lane_thresh', 0.5))
        min_area = int(self._p('yolopv2_min_area', 0))
        pre = list(self._p('yolopv2_pre_resize', [1280, 720]) or [])
        pre_resize = (pre[0], pre[1]) if len(pre) == 2 and pre[0] > 0 else None

        x, pads = preprocess(bgr, self._net_hw, pre_resize)
        try:
            out = self._sess.run([self._lane_name], {self._input_name: x})[0]
        except Exception as e:                          # noqa: BLE001
            self._log('warn', f'inference raised, empty frame: {e}')
            z = np.zeros((h, w), dtype=np.uint8)
            return PipelineResult(mask=z, confidence=z)

        binm, prob = decode_lane(out, pads, (h, w), thresh)
        binm = filter_min_area(binm, min_area)

        mask = np.zeros((h, w), dtype=np.uint8)
        mask[binm > 0] = class_id_lane

        # Sky / out-of-interest ROI zeroed LAST (mirrors the classical pipelines).
        poly = self._roi_polygon_px(h, w)
        if poly is not None:
            cv2.fillPoly(mask, [poly], 0)

        confidence = np.where(mask > 0, prob, 0).astype(np.uint8)
        return PipelineResult(mask=mask, confidence=confidence)
