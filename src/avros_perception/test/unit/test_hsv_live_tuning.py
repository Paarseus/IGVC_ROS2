"""
Live-tuning behavior for HSVPipeline.

The pipeline reads its bounds + class IDs + adaptive_k from `self.params`
on every `run()` call. perception_node hands it the same dict it mutates
inside `_on_set_params`, so any `ros2 param set` takes effect on the next
frame. These tests pin that contract.
"""

import numpy as np
import pytest

from avros_perception.pipelines.hsv import HSVPipeline


def _gradient_frame(h=64, w=128):
    """V-channel ramp from 0 (top-left) to 255 (bottom-right) in HSV space.

    Yields a frame where any HSV-V threshold cleanly partitions pixels —
    perfect for asserting that bound mutations actually move the mask.
    """
    g = np.linspace(0, 255, w, dtype=np.uint8)
    v_plane = np.tile(g, (h, 1))                   # H x W ramp
    hsv = np.stack([np.zeros_like(v_plane),        # H = 0
                    np.zeros_like(v_plane),        # S = 0  (achromatic)
                    v_plane], axis=-1)             # V = ramp
    import cv2
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


@pytest.fixture
def frame():
    return _gradient_frame()


def _hsv_defaults():
    """Mirrors perception.yaml so each test starts from a known baseline."""
    return {
        'blur_iters': 0,        # disable blur so single-pixel changes are visible
        'adaptive_period': 1,
        'adaptive_k': 0.0,      # disable adaptive V-floor so it doesn't gate lane
        'lane_low':    [0,   0, 180],
        'lane_high':   [179, 60, 255],
        'class_id_lane': 1,
        'barrel_low':  [5, 120, 100],
        'barrel_high': [25, 255, 255],
        'class_id_barrel': 2,
        'pothole_low':  [0,   0, 200],
        'pothole_high': [179, 40, 255],
        'class_id_pothole': 3,
        'sky_roi_poly': [],     # disable ROI so no pixels are zeroed
    }


def test_lane_bounds_mutation_takes_effect_next_run(frame):
    """Tightening lane_low V should reduce the lane-pixel count."""
    params = _hsv_defaults()
    pipe = HSVPipeline(params=params)
    first = pipe.run(frame)
    lane_pixels_before = int((first.mask == params['class_id_lane']).sum())
    assert lane_pixels_before > 0, 'baseline must detect some lane pixels'

    # Raise the V floor — fewer pixels should pass.
    params['lane_low'] = [0, 0, 240]
    second = pipe.run(frame)
    lane_pixels_after = int((second.mask == params['class_id_lane']).sum())
    assert lane_pixels_after < lane_pixels_before


def test_class_id_mutation_relabels_mask(frame):
    """Changing class_id_lane changes the value written into mask pixels."""
    params = _hsv_defaults()
    pipe = HSVPipeline(params=params)
    pipe.run(frame)  # warm tick state

    params['class_id_lane'] = 7
    result = pipe.run(frame)
    nonzero = result.mask[result.mask > 0]
    assert nonzero.size > 0
    assert (nonzero == 7).all(), \
        f'expected only class_id=7 in mask, got values {set(nonzero.tolist())}'


def test_blur_iters_mutation_does_not_crash(frame):
    """blur_iters=0 must be valid (no preprocessing)."""
    params = _hsv_defaults()
    params['blur_iters'] = 0
    pipe = HSVPipeline(params=params)
    result = pipe.run(frame)
    assert result.mask.shape == frame.shape[:2]


def test_roi_poly_mutation_can_zero_full_frame(frame):
    """A unit-square ROI zeros every pixel — kiwicampus uses this to silence cams."""
    params = _hsv_defaults()
    pipe = HSVPipeline(params=params)
    first = pipe.run(frame)
    assert int(first.mask.sum()) > 0

    params['sky_roi_poly'] = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0]  # whole frame
    second = pipe.run(frame)
    assert int(second.mask.sum()) == 0


def test_adaptive_k_mutation_changes_v_floor(frame):
    """High k starves the V-floor so lane detection collapses."""
    params = _hsv_defaults()
    params['adaptive_k'] = 0.0
    pipe = HSVPipeline(params=params)
    baseline = int((pipe.run(frame).mask == params['class_id_lane']).sum())
    assert baseline > 0

    params['adaptive_k'] = 100.0  # V-floor far above any pixel
    starved = int((pipe.run(frame).mask == params['class_id_lane']).sum())
    assert starved < baseline
