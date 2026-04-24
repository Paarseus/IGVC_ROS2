"""
HSVPipeline synthetic-input tests.

Exercises the pipeline on hand-crafted numpy BGR frames where the
correct class assignment is known by construction. These tests are
deliberately resolution-small and fixture-free so they run fast in
CI; the real-image regression suite (test_mask_regression.py) will
land alongside the field-day fixture corpus.

Most tests disable the adaptive V-floor gate and the ROI polygon via
baseline params, so the HSV-only classification is the single variable
under test. Dedicated tests exercise the adaptive-V gate and the ROI
separately.
"""

import numpy as np
import pytest

from avros_perception.pipelines.hsv import HSVPipeline


CLASS_FREE = 0
CLASS_LANE = 1
CLASS_BARREL = 2
CLASS_POTHOLE = 3


def _mostly_dark_frame(h=100, w=160, base=50):
    """Return an HxWx3 BGR frame filled with a low-value background color."""
    return np.full((h, w, 3), base, dtype=np.uint8)


def _baseline_params(overrides=None):
    """
    Return pipeline params for synthetic-input unit tests.

    Disables the sky-ROI polygon and the adaptive V-floor gate so each
    test's intended HSV content is the sole driver of classification.
    """
    params = {
        'sky_roi_poly': [],
        'adaptive_k': -100.0,   # v_floor well below any uint8 value -> always pass
    }
    if overrides:
        params.update(overrides)
    return params


def test_output_shape_matches_input():
    """Output mask and confidence must match input HxW and be uint8."""
    frame = _mostly_dark_frame(h=48, w=72)
    result = HSVPipeline(_baseline_params()).run(frame)
    assert result.mask.shape == (48, 72)
    assert result.confidence.shape == (48, 72)
    assert result.mask.dtype == np.uint8
    assert result.confidence.dtype == np.uint8


def test_run_rejects_non_bgr_input():
    """Non-3-channel input must raise, not silently misclassify."""
    with pytest.raises(ValueError):
        HSVPipeline(_baseline_params()).run(np.zeros((10, 10), dtype=np.uint8))


def test_uniform_mid_gray_frame_is_all_free():
    """
    Gray value too low for lane, saturation too low for barrel.

    Mid-gray (V=128, S=0) sits below the lane V-floor (>=180) and below
    the barrel S-floor (>=120). All pixels must remain class 0 (free).
    """
    frame = np.full((60, 80, 3), 128, dtype=np.uint8)
    result = HSVPipeline(_baseline_params()).run(frame)
    assert int(result.mask.max()) == CLASS_FREE
    assert int(result.confidence.sum()) == 0


def test_bright_white_patch_is_lane():
    """
    A bright-white patch on a dark background must be classified as lane.

    White: high V, low S — passes the lane HSV band (V>=180, S<=60, H any).
    Dark background (V=30) stays free.
    """
    frame = _mostly_dark_frame(h=100, w=160, base=30)
    frame[40:60, 60:100] = 250
    result = HSVPipeline(_baseline_params()).run(frame)
    assert int(result.mask[50, 80]) == CLASS_LANE
    assert int(result.mask[5, 5]) == CLASS_FREE


def test_orange_patch_is_barrel():
    """
    A saturated-orange patch must be classified as barrel.

    BGR (0, 128, 255) -> HSV ~ (15, 255, 255), inside the barrel band.
    """
    frame = _mostly_dark_frame(h=100, w=160, base=20)
    frame[40:60, 60:100] = (0, 128, 255)
    result = HSVPipeline(_baseline_params()).run(frame)
    assert int(result.mask[50, 80]) == CLASS_BARREL


def test_lane_wins_priority_over_barrel_on_overlap():
    """
    Pixel matching both lane and barrel bands must be classified as lane.

    Write order is pothole -> barrel -> lane (ascending priority). Widen
    the barrel band to overlap lane's range to force the collision.
    """
    params = _baseline_params({
        'barrel_low': [0, 0, 180],
        'barrel_high': [179, 255, 255],
    })
    frame = _mostly_dark_frame(h=40, w=60, base=20)
    frame[10:30, 20:40] = 245
    result = HSVPipeline(params).run(frame)
    assert int(result.mask[20, 30]) == CLASS_LANE


def test_roi_polygon_zeros_its_region():
    """
    Sky-ROI polygon over the top half must zero every pixel inside it.

    Bright content above the polygon is suppressed; bright content below
    the polygon is still classified normally.
    """
    frame = _mostly_dark_frame(h=80, w=120, base=20)
    frame[:40, :] = 250                  # whole top half bright
    frame[40:, 20:50] = 250              # also a patch below the ROI
    params = _baseline_params({
        'sky_roi_poly': [[0.0, 0.0], [1.0, 0.0], [1.0, 0.5], [0.0, 0.5]],
    })
    result = HSVPipeline(params).run(frame)
    assert int(result.mask[10, 60]) == CLASS_FREE
    assert int(result.mask[:40, :].sum()) == 0
    assert int(result.mask[50, 35]) == CLASS_LANE


def test_confidence_is_255_where_mask_is_nonzero():
    """Confidence plane must carry 255 on classified pixels, 0 elsewhere."""
    frame = _mostly_dark_frame(h=60, w=80, base=20)
    frame[20:40, 30:50] = 250
    result = HSVPipeline(_baseline_params()).run(frame)
    nonzero_mask = result.mask > 0
    assert int(result.confidence[nonzero_mask].min()) == 255
    assert int(result.confidence[~nonzero_mask].max()) == 0


def test_adaptive_v_floor_is_computed_on_first_frame():
    """
    With default adaptive_k (>0), v_floor starts None and populates on run().

    This is the iscumd pattern — recompute mean+k*std every N frames to
    track ambient exposure changes.
    """
    pipe = HSVPipeline({'sky_roi_poly': []})  # leave adaptive_k at default
    assert pipe._v_floor is None
    pipe.run(_mostly_dark_frame(h=20, w=30, base=50))
    assert pipe._v_floor is not None
    assert pipe._v_floor > 0


def test_class_ids_respect_params():
    """Custom class IDs are written into the mask instead of the defaults."""
    params = _baseline_params({
        'class_id_lane': 42,
        'class_id_barrel': 77,
        'class_id_pothole': 99,
    })
    # V=190 falls inside lane (V>=180) but outside pothole (V>=200), so
    # this test isolates lane classification from the pothole band.
    frame = _mostly_dark_frame(h=40, w=60, base=20)
    frame[10:30, 20:40] = 190
    result = HSVPipeline(params).run(frame)
    nonzero_ids = set(int(v) for v in np.unique(result.mask)) - {0}
    assert nonzero_ids == {42}, f'expected {{42}} got {nonzero_ids}'


def test_build_pipeline_registers_hsv():
    """PIPELINES registry must expose 'hsv' so perception.yaml can select it."""
    from avros_perception.pipelines import PIPELINES, build_pipeline
    assert 'hsv' in PIPELINES
    pipe = build_pipeline('hsv', _baseline_params())
    assert isinstance(pipe, HSVPipeline)
