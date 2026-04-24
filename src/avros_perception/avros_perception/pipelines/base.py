"""Abstract pipeline interface.

A Pipeline converts an RGB (and optional depth) frame into a mono8 class-ID
mask plus a uint8 confidence plane, both the same HxW as the input. Concrete
pipelines (stub, HSV, ONNX) implement run(); the node doesn't know which one
is active.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class PipelineResult:
    mask: np.ndarray        # (H, W) uint8 — class IDs, 0 == free/background
    confidence: np.ndarray  # (H, W) uint8 — 0..255


class Pipeline:
    """Base class. Subclass and override run()."""

    def __init__(self, params: dict, logger=None):
        self.params = params if params is not None else {}
        self.logger = logger

    def warmup(self) -> None:
        """Pre-allocate buffers / load models. Default no-op."""
        return None

    def run(self, bgr: np.ndarray, depth: np.ndarray | None = None) -> PipelineResult:
        """Classify each pixel. `bgr` is HxWx3 uint8 in OpenCV BGR order."""
        raise NotImplementedError
