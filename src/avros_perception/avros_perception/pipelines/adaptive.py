"""
Local-adaptive lane threshold — exposure-invariant replacement for sooner25.

Field problem (2026-05-30): the ZED X (POLARIZED) front camera's auto-exposure
hunts in a limit cycle even while stationary (V_median ~141<->167 every
~0.3-0.4 s). sooner25 thresholds asphalt on a FIXED absolute brightness
(sooner25_upper V) and inverts, so the AE swing makes the lane mask flicker
(detected lane px swung 375<->909, 2.4x, frame-to-frame at the same threshold).

This pipeline replaces the fixed-brightness threshold with a LOCAL ADAPTIVE
threshold (cv2.adaptiveThreshold, Gaussian) on a lighting-stable single channel
(HLS-Lightness by default). It is exposure-invariant BY CONSTRUCTION: per the
OpenCV docs the threshold T(x,y) is the local-neighborhood Gaussian mean minus
C, so any monotone exposure transform lifts a pixel AND its neighborhood mean
together, preserving the src > mean + |C| relation. Keep auto-exposure ON.

Polarity (doc-exact, docs.opencv.org/4.x/d7/d1b):
  THRESH_BINARY fires "maxval if src(x,y) > T(x,y)" and C "may be negative".
  C < 0  =>  T = local_mean + |C|  =>  only pixels BRIGHTER than their local
  neighborhood by |C| fire = white paint. (THRESH_BINARY_INV / C>0 would mark
  dark cracks + shadows — WRONG polarity for bright lines on asphalt.)

Deliberately NOT included:
  - Sobel/Canny gradient OR: measured to triple px (280->923) and push the
    exposure CV 0.031->0.273 by importing asphalt aggregate/crack texture —
    directly defeats "keep asphalt clean". There is no IPM-warp + sliding-window
    stage downstream to reject that noise (unlike Udacity-style pipelines).
  - MORPH_OPEN by default: open erases marginal thin lines (verified: a real
    line component dropped 114px -> 17px). Optional via adaptive_use_open.

Speckle is removed with a connected-component min-area filter instead of
morphology, so the single clean line component survives while 4-7px
asphalt-texture specks are dropped.

Single output class — the whole mask maps to class_id_lane (1), identical
kiwicampus / Nav2 contract to sooner25.py: a mono8 class-ID mask + a uint8
confidence plane, same HxW as the input, with the sky/horizon ROI zeroed last.
"""

import cv2
import numpy as np

from avros_perception.pipelines.base import Pipeline, PipelineResult


class AdaptivePipeline(Pipeline):
    """Exposure-invariant local-adaptive lane threshold.

    Output contract is identical to Sooner25Pipeline: a mono8 class-ID mask
    (everything detected -> class_id_lane) + a uint8 confidence plane, same
    HxW as the input, with the sky/horizon ROI zeroed last.
    """

    # NOTE (DRY): _reshape_poly + _roi_polygon_px are byte-identical across
    # adaptive.py, sooner25.py, and hsv.py. The standard fix is to hoist them
    # once into the Pipeline base class (base.py, alongside warmup) and have all
    # three call self._roi_polygon_px. That edit spans two files, so it is left
    # to the base-class refactor; the logic itself is correct and load-bearing
    # (the sky/horizon ROI is non-negotiable and zeroed LAST — see run()).

    def _reshape_poly(self, seq):
        """Flat [x0,y0,x1,y1,...] normalized list -> [(x0,y0),(x1,y1),...]."""
        if not seq:
            return []
        if len(seq) % 2 != 0:
            raise ValueError(
                f'sky_roi_poly flat list must have even length, got {len(seq)}'
            )
        return [(float(seq[i]), float(seq[i + 1])) for i in range(0, len(seq), 2)]

    def _roi_polygon_px(self, h, w):
        """sky_roi_poly (normalized) -> int32 pixel polygon, or None if empty.

        Default mirrors the field-tuned perception.yaml value (top 40% zeroed).
        """
        raw_poly = self.params.get(
            'sky_roi_poly',
            [0.0, 0.0, 1.0, 0.0, 1.0, 0.40, 0.0, 0.40],
        )
        poly_norm = self._reshape_poly(raw_poly)
        if not poly_norm:
            return None
        return np.array(
            [(int(x * (w - 1)), int(y * (h - 1))) for x, y in poly_norm],
            dtype=np.int32,
        )

    def run(self, bgr, depth=None):
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f'AdaptivePipeline expects HxWx3 BGR; got {bgr.shape}')

        # ---- Live-tunable params (re-read every frame, same pattern as sooner25).
        # In-code defaults match the field-tuned perception.yaml so a YAML-less
        # launch matches the known-good field config.
        block = int(self.params.get('adaptive_block_size', 21))
        C = float(self.params.get('adaptive_C', -8.0))
        min_area = int(self.params.get('adaptive_min_area', 80))
        use_open = bool(self.params.get('adaptive_use_open', False))
        channel = str(self.params.get('adaptive_channel', 'L'))
        class_id_lane = int(self.params.get('class_id_lane', 1))
        max_sat = int(self.params.get('adaptive_max_sat', 70))
        blur = int(self.params.get('adaptive_blur', 9))

        # MANDATORY odd-coercion (load-bearing, verified on cv2 4.13.0):
        # cv2.adaptiveThreshold RAISES on an even blockSize (thresh.cpp:1909) and
        # cv2.GaussianBlur RAISES on an even kernel. The rqt IntegerRange(step=2)
        # only constrains sliders, NOT a raw `ros2 param set adaptive_block_size
        # 20`, so without these guards one bad set stalls _on_synced and kills
        # ALL perception. Lower bounds (block<3, blur<1) are also reachable via
        # raw set, hence the floor clamps.
        bs = block if block % 2 == 1 else block + 1
        bs = max(bs, 3)
        bk = blur if blur % 2 == 1 else blur + 1
        bk = max(bk, 1)

        # ---- Single 8-bit lighting-stable channel + (optional) saturation plane.
        # adaptiveThreshold REQUIRES an 8-bit single-channel image — passing the
        # 3ch image RAISES (thresh.cpp:1908). HLS-L is the default; 'V'
        # (HSV-Value) and 'gray' are near-identical here (CV 0.029-0.031).
        #
        # The low-saturation gate (below) needs HLS-S. When channel == 'L' we do
        # ONE BGR2HLS and slice both L (idx 1) and S (idx 2) — verified
        # byte-identical to two separate conversions on cv2 4.13.0, so the old
        # double-convert was pure dead work. The 'V'/'gray' paths use a different
        # colorspace, so they take their own BGR2HLS only if the sat gate is on.
        sat = None
        if channel == 'gray':
            chan = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        elif channel == 'V':
            chan = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:, :, 2]
        else:  # 'L' (HLS-Lightness) — default
            hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
            chan = hls[:, :, 1]
            sat = hls[:, :, 2]   # reuse — no second conversion

        # Gaussian de-speckle (kernel = adaptive_blur, odd-coerced). A larger
        # kernel low-passes the 1-3px asphalt-aggregate spikes (which otherwise
        # out-contrast the faint paint and speckle past min_area) while the wider
        # painted line survives. Default 9 is the field-tuned value.
        chan = cv2.GaussianBlur(chan, (bk, bk), 0)

        # Local Gaussian adaptive threshold. THRESH_BINARY + C<0 marks pixels
        # brighter than their local neighborhood mean by |C| = white paint.
        raw = cv2.adaptiveThreshold(
            chan, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            bs, C,
        )

        # ---- Low-saturation (white-paint) gate. White lane paint is near-grey
        # (low HLS saturation); colored clutter — orange barrels, tan pillar,
        # green grass, tents — is high-S and sits AT lane-band height where the
        # ROI cannot separate it from the far line. Dropping high-S pixels
        # removes that clutter while keeping near-grey paint. 255 disables.
        if max_sat < 255:
            if sat is None:  # 'V'/'gray' channels didn't compute HLS-S yet
                sat = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)[:, :, 2]
            raw[sat > max_sat] = 0

        # ---- Optional MORPH_OPEN — OFF by default (open erases marginal thin
        # lines: verified line comp 114px -> 17px). A guarded escape hatch for a
        # rougher surface that speckles past min_area; zero cost when off.
        if use_open:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            raw = cv2.morphologyEx(raw, cv2.MORPH_OPEN, kernel)

        # ---- Connected-component speckle filter, mapped straight to class IDs.
        # Keep only components with area >= min_area, writing class_id_lane
        # directly into the mask (no intermediate binary array). The Python loop
        # is intentional and measured FASTER than the np.where(keep[labels])
        # vectorization (it short-circuits on the few surviving large comps and
        # only writes their pixels, rather than fancy-indexing the whole frame).
        n, labels, stats, _ = cv2.connectedComponentsWithStats(raw, 8)
        h, w = raw.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        for i in range(1, n):  # skip label 0 (background)
            if stats[i, cv2.CC_STAT_AREA] >= min_area:
                mask[labels == i] = class_id_lane

        # ---- Sky / out-of-interest ROI zeroed LAST (background tents/cones are
        # bright — the ROI is non-negotiable and must run after detection).
        poly = self._roi_polygon_px(h, w)
        if poly is not None:
            cv2.fillPoly(mask, [poly], 0)

        confidence = np.where(mask > 0, 255, 0).astype(np.uint8)
        return PipelineResult(mask=mask, confidence=confidence)
