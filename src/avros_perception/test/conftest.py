"""
Pytest shared fixtures + determinism knobs for avros_perception tests.

Unit tests are deterministic by design: OpenCV's internal threading and
numpy's default RNG both introduce nondeterminism that surfaces as
spurious regression-test failures. We pin both here so every test sees
the same execution.
"""

import os

import cv2
import numpy as np
import pytest


def pytest_configure(config):
    """Pin OpenCV thread count and numpy seed before any test runs."""
    cv2.setNumThreads(1)
    np.random.seed(0)


@pytest.fixture
def fixtures_dir():
    """Absolute path to test/fixtures/ — populated in Step D (Phase 4 HSV)."""
    return os.path.join(os.path.dirname(__file__), 'fixtures')


@pytest.fixture
def data_dir():
    """Absolute path to test/data/ — holds ndarrays_regression goldens."""
    return os.path.join(os.path.dirname(__file__), 'data')


@pytest.fixture
def load_image(fixtures_dir):
    """
    Load a fixture image by relative path; skip the test if absent.

    Lets tests declare their fixture expectations before the field corpus
    lands, rather than fail loudly on an empty repo.
    """
    def _load(name):
        path = os.path.join(fixtures_dir, name)
        img = cv2.imread(path)
        if img is None:
            pytest.skip(f"fixture {name!r} not yet collected")
        return img
    return _load
