"""
Unit tests for the YOLOPv2 ONNX lane pipeline.

These exercise the pure preprocess/decode helpers and the run() contract using
an INJECTED fake session, so they need neither onnxruntime nor the 156 MB model
present (CI-safe). A separate, auto-skipped test runs the real model if it is
found on disk.
"""

import os

import numpy as np
import pytest

from avros_perception.pipelines import PIPELINES, build_pipeline
from avros_perception.pipelines.yolopv2 import (
    YolopV2Pipeline,
    decode_lane,
    filter_min_area,
    letterbox_exact,
    preprocess,
)


class _NullLog:
    """Swallow logger calls so the pipeline can run headless in tests."""

    def info(self, *_):
        pass

    warn = error = info


# ---- registration ----

def test_yolopv2_registered():
    assert PIPELINES.get('yolopv2') is YolopV2Pipeline


# ---- pure helpers ----

def test_letterbox_720p_to_384x640_pads_12():
    """1280x720 -> 384x640 letterbox pads 12 px top/bottom, 0 px sides."""
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    out, pads = letterbox_exact(img, (384, 640))
    assert out.shape == (384, 640, 3)
    assert pads == (12, 12, 0, 0)


def test_letterbox_arbitrary_aspect_hits_exact_net_size():
    """Any source aspect must pad to EXACTLY the net size (fixed ONNX input)."""
    img = np.zeros((300, 480, 3), dtype=np.uint8)
    out, _ = letterbox_exact(img, (384, 640))
    assert out.shape == (384, 640, 3)


def test_preprocess_shape_dtype_and_normalization():
    bgr = np.full((300, 480, 3), 255, dtype=np.uint8)
    x, pads = preprocess(bgr, (384, 640), pre_resize=(1280, 720))
    assert x.shape == (1, 3, 384, 640)
    assert x.dtype == np.float32
    assert 0.0 <= float(x.min()) and float(x.max()) <= 1.0
    assert len(pads) == 4


def test_decode_lane_1ch_thresholds_and_resizes():
    """Single-channel sigmoid head: > thresh marks lane, output at orig size."""
    lane = np.zeros((1, 1, 384, 640), dtype=np.float32)
    lane[0, 0, 100:200, 200:300] = 0.9          # a confident blob in-content
    binm, prob = decode_lane(lane, (12, 12, 0, 0), (300, 480), thresh=0.5)
    assert binm.shape == (300, 480)
    assert prob.shape == (300, 480)
    assert binm.max() == 1 and binm.min() == 0
    assert binm.sum() > 0
    assert prob.max() > 200                      # 0.9 * 255


def test_decode_lane_threshold_controls_recall():
    lane = np.full((1, 1, 384, 640), 0.4, dtype=np.float32)
    high = decode_lane(lane, (0, 0, 0, 0), (384, 640), thresh=0.5)[0]
    low = decode_lane(lane, (0, 0, 0, 0), (384, 640), thresh=0.3)[0]
    assert high.sum() == 0                       # 0.4 < 0.5 -> nothing
    assert low.sum() > 0                         # 0.4 > 0.3 -> everything


def test_decode_lane_2ch_argmax_path():
    """A 2-channel head decodes by argmax==1 (drivable-style)."""
    lane = np.zeros((1, 2, 384, 640), dtype=np.float32)
    lane[0, 1, :, :] = 5.0                        # channel-1 wins everywhere
    binm, _ = decode_lane(lane, (0, 0, 0, 0), (384, 640), thresh=0.5)
    assert binm.mean() > 0.9


def test_filter_min_area_drops_small_keeps_large():
    m = np.zeros((100, 100), dtype=np.uint8)
    m[0:2, 0:2] = 1                               # 4 px speck
    m[40:70, 40:70] = 1                           # 900 px blob
    out = filter_min_area(m, min_area=50)
    assert out[0:2, 0:2].sum() == 0
    assert out[40:70, 40:70].sum() == 900
    # min_area 0 is a no-op
    assert np.array_equal(filter_min_area(m, 0), m)


# ---- run() contract with an injected fake session (no model needed) ----

class _FakeSession:
    """Minimal onnxruntime.InferenceSession stand-in returning a fixed lane map."""

    def __init__(self, value=0.9):
        self._value = value

    def run(self, _outputs, _feeds):
        return [np.full((1, 1, 384, 640), self._value, dtype=np.float32)]


def _wire_fake(pipeline, value=0.9):
    pipeline._sess = _FakeSession(value)
    pipeline._input_name = 'images'
    pipeline._lane_name = 'lane'
    pipeline._net_hw = (384, 640)
    pipeline._load_failed = False
    return pipeline


def test_run_contract_with_fake_session():
    """mask is uint8 HxW matching the input, single class == class_id_lane."""
    p = _wire_fake(YolopV2Pipeline({'class_id_lane': 1, 'sky_roi_poly': []},
                                   _NullLog()))
    bgr = np.zeros((300, 480, 3), dtype=np.uint8)
    res = p.run(bgr)
    assert res.mask.shape == (300, 480)
    assert res.mask.dtype == np.uint8
    assert res.confidence.dtype == np.uint8
    assert set(np.unique(res.mask).tolist()) == {1}      # all lane (thresh 0.5 < 0.9)


def test_run_custom_class_id():
    p = _wire_fake(YolopV2Pipeline({'class_id_lane': 7, 'sky_roi_poly': []},
                                   _NullLog()))
    res = p.run(np.zeros((64, 64, 3), dtype=np.uint8))
    assert set(np.unique(res.mask).tolist()) == {7}


def test_run_sky_roi_zeroes_top():
    """sky_roi_poly must zero the polygon region LAST (after detection)."""
    poly = [0.0, 0.0, 1.0, 0.0, 1.0, 0.5, 0.0, 0.5]      # top half
    p = _wire_fake(YolopV2Pipeline({'class_id_lane': 1, 'sky_roi_poly': poly},
                                   _NullLog()))
    res = p.run(np.zeros((100, 100, 3), dtype=np.uint8))
    assert res.mask[:40, :].sum() == 0                    # top zeroed
    assert res.mask[60:, :].sum() > 0                     # bottom kept


def test_run_below_threshold_is_empty():
    p = _wire_fake(YolopV2Pipeline({'yolopv2_lane_thresh': 0.5, 'sky_roi_poly': []},
                                   _NullLog()), value=0.2)
    res = p.run(np.zeros((50, 50, 3), dtype=np.uint8))
    assert res.mask.sum() == 0


# ---- graceful degradation (no model on disk) ----

def test_missing_model_returns_empty_mask_no_raise():
    p = YolopV2Pipeline({'yolopv2_model_path': '/definitely/not/a/model.onnx'},
                        _NullLog())
    res = p.run(np.zeros((40, 60, 3), dtype=np.uint8))
    assert res.mask.shape == (40, 60)
    assert res.mask.sum() == 0
    assert p._load_failed is True
    # second run must also not raise (failure is latched)
    assert p.run(np.zeros((40, 60, 3), dtype=np.uint8)).mask.sum() == 0


# ---- optional: real model if present ----

_MODEL = os.path.expanduser(
    os.environ.get('YOLOPV2_ONNX_PATH', '~/yolopv2_lane_384x640.onnx'))


@pytest.mark.skipif(not os.path.isfile(_MODEL),
                    reason='real YOLOPv2 ONNX not present')
def test_real_model_runs_and_emits_lane_class():
    import cv2
    p = build_pipeline('yolopv2',
                       {'yolopv2_model_path': _MODEL, 'class_id_lane': 1,
                        'sky_roi_poly': []}, _NullLog())
    p.warmup()
    bgr = np.full((300, 480, 3), 120, dtype=np.uint8)
    cv2.line(bgr, (50, 250), (430, 120), (255, 255, 255), 6)
    res = p.run(bgr)
    assert res.mask.shape == (300, 480)
    assert res.mask.dtype == np.uint8
    assert set(np.unique(res.mask).tolist()) <= {0, 1}
