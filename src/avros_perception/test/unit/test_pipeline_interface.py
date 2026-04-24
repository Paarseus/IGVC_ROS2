"""
Contract tests for the Pipeline abstract base + PipelineResult dataclass.

Every Pipeline subclass (stub, HSV, ONNX) must produce output with a
shape matching the input image and dtype=uint8 for both mask and
confidence. These tests pin that contract so swapping pipelines at
runtime never surprises the node or the kiwicampus plugin downstream.
"""

import numpy as np
import pytest

from avros_perception.pipelines.base import Pipeline, PipelineResult


class _Identity(Pipeline):
    """Minimal concrete subclass — fills the mask with a single class ID."""

    def run(self, bgr, depth=None):
        h, w = bgr.shape[:2]
        class_id = int(self.params.get('class_id', 1))
        mask = np.full((h, w), class_id, dtype=np.uint8)
        confidence = np.full((h, w), 255, dtype=np.uint8)
        return PipelineResult(mask=mask, confidence=confidence)


def test_base_pipeline_run_is_abstract():
    """Pipeline.run() must raise NotImplementedError on the ABC itself."""
    base = Pipeline(params={})
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    with pytest.raises(NotImplementedError):
        base.run(img)


def test_base_pipeline_warmup_is_noop():
    """warmup() has a default no-op implementation; must not raise."""
    assert Pipeline(params={}).warmup() is None


def test_params_default_to_empty_dict():
    """Passing params=None must produce an empty dict, not a None attr."""
    p = Pipeline(params=None)
    assert p.params == {}


def test_subclass_run_returns_pipeline_result_shape_matches_input():
    bgr = np.zeros((100, 160, 3), dtype=np.uint8)
    result = _Identity(params={'class_id': 3}).run(bgr)
    assert isinstance(result, PipelineResult)
    assert result.mask.shape == (100, 160)
    assert result.confidence.shape == (100, 160)


def test_subclass_mask_dtype_is_uint8():
    """Kiwicampus requires mono8; mask must be uint8."""
    bgr = np.zeros((50, 80, 3), dtype=np.uint8)
    result = _Identity(params={}).run(bgr)
    assert result.mask.dtype == np.uint8
    assert result.confidence.dtype == np.uint8


def test_subclass_mask_contains_expected_class_id():
    bgr = np.zeros((20, 30, 3), dtype=np.uint8)
    result = _Identity(params={'class_id': 7}).run(bgr)
    assert int(result.mask.min()) == 7
    assert int(result.mask.max()) == 7


def test_confidence_range_is_0_255():
    """Confidence plane is uint8 in [0, 255] per the perception contract."""
    bgr = np.zeros((40, 60, 3), dtype=np.uint8)
    result = _Identity(params={}).run(bgr)
    assert int(result.confidence.min()) >= 0
    assert int(result.confidence.max()) <= 255
