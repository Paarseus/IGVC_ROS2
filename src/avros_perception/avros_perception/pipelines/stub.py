"""Stub pipeline for plumbing validation.

Returns a zero mask, except for a vertical stripe of configurable width/start
that is filled with a configurable class ID. Use it to drive the full
perception -> kiwicampus -> Nav2 costmap chain before writing real CV.

Runtime params (read from self.params every frame — updated by the node
whenever ros2 param set is called):
    inject_stripe_width  (int)  0 disables; otherwise stripe column count
    inject_stripe_start  (int)  leftmost column; defaults to image center
    inject_class_id      (int)  class ID written into the stripe (default 1)
"""

import numpy as np

from avros_perception.pipelines.base import Pipeline, PipelineResult


class StubPipeline(Pipeline):
    def run(self, bgr: np.ndarray, depth: np.ndarray | None = None) -> PipelineResult:
        h, w = bgr.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)

        width = int(self.params.get('inject_stripe_width', 0))
        if width > 0:
            width = min(width, w)
            default_start = (w - width) // 2
            start = int(self.params.get('inject_stripe_start', default_start))
            start = max(0, min(start, w - width))
            class_id = int(self.params.get('inject_class_id', 1)) & 0xFF
            mask[:, start:start + width] = class_id

        confidence = np.full((h, w), 255, dtype=np.uint8)
        return PipelineResult(mask=mask, confidence=confidence)
