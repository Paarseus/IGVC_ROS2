"""
Tests for StubPipeline — the zero-mask + configurable-stripe pipeline.

The stub exists for infrastructure validation (plumbing from perception
through kiwicampus into the Nav2 costmap) before any real CV code lands.
These tests pin its behavior so refactors don't break the Phase 3
stripe-injection loop documented in CLAUDE.md.
"""

import numpy as np
import pytest

from avros_perception.pipelines.stub import StubPipeline


@pytest.fixture
def black_frame():
    """100x200 BGR black frame — the shape/content are irrelevant for the stub."""
    return np.zeros((100, 200, 3), dtype=np.uint8)


def test_stub_default_mask_is_all_zero(black_frame):
    """No inject params → all-zero mask (class 0 = free everywhere)."""
    result = StubPipeline(params={}).run(black_frame)
    assert int(result.mask.max()) == 0
    assert result.mask.shape == black_frame.shape[:2]


def test_stub_confidence_is_full_by_default(black_frame):
    """Stub reports full confidence (255) so kiwicampus never down-weights it."""
    result = StubPipeline(params={}).run(black_frame)
    assert int(result.confidence.min()) == 255
    assert int(result.confidence.max()) == 255


def test_stub_stripe_width_zero_is_noop(black_frame):
    """inject_stripe_width=0 is the documented 'disabled' state."""
    params = {'inject_stripe_width': 0, 'inject_class_id': 1}
    result = StubPipeline(params=params).run(black_frame)
    assert int(result.mask.sum()) == 0


def test_stub_stripe_width_50_marks_50_columns(black_frame):
    """Every row must carry exactly the requested stripe width."""
    h, w = black_frame.shape[:2]
    width = 50
    params = {'inject_stripe_width': width, 'inject_class_id': 1}
    result = StubPipeline(params=params).run(black_frame)
    nonzero_per_row = (result.mask > 0).sum(axis=1)
    assert (nonzero_per_row == width).all()
    assert int(result.mask.sum()) == h * width  # class_id=1 × cells


def test_stub_stripe_respects_class_id(black_frame):
    """inject_class_id is written verbatim into stripe pixels."""
    params = {'inject_stripe_width': 20, 'inject_class_id': 5}
    result = StubPipeline(params=params).run(black_frame)
    stripe_values = result.mask[result.mask > 0]
    assert (stripe_values == 5).all()


def test_stub_stripe_auto_centers_when_start_negative(black_frame):
    """inject_stripe_start=-1 centers the stripe (default behavior)."""
    h, w = black_frame.shape[:2]
    width = 40
    params = {
        'inject_stripe_width': width,
        'inject_stripe_start': -1,
        'inject_class_id': 1,
    }
    result = StubPipeline(params=params).run(black_frame)
    cols_touched = np.where((result.mask > 0).any(axis=0))[0]
    expected_start = (w - width) // 2
    assert cols_touched[0] == expected_start
    assert cols_touched[-1] == expected_start + width - 1


def test_stub_stripe_explicit_start_is_honored(black_frame):
    params = {
        'inject_stripe_width': 30,
        'inject_stripe_start': 10,
        'inject_class_id': 1,
    }
    result = StubPipeline(params=params).run(black_frame)
    cols_touched = np.where((result.mask > 0).any(axis=0))[0]
    assert cols_touched[0] == 10
    assert cols_touched[-1] == 39


def test_stub_stripe_clipped_to_image_width(black_frame):
    """Stripe wider than the frame is silently clipped, not an error."""
    h, w = black_frame.shape[:2]
    params = {'inject_stripe_width': w * 10, 'inject_class_id': 1}
    result = StubPipeline(params=params).run(black_frame)
    assert int(result.mask.sum()) == h * w  # whole frame marked, no overflow


def test_stub_stripe_params_mutation_takes_effect_next_run(black_frame):
    """Node updates self.params in-place on ros2 param set; next run sees it."""
    params = {'inject_stripe_width': 0, 'inject_class_id': 1}
    pipe = StubPipeline(params=params)
    first = pipe.run(black_frame)
    assert int(first.mask.sum()) == 0

    # Simulate a runtime ros2 param set call
    params['inject_stripe_width'] = 25
    second = pipe.run(black_frame)
    assert int(second.mask.sum()) == black_frame.shape[0] * 25
