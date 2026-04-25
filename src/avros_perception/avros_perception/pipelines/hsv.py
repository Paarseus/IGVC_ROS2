"""
HSV lane + obstacle segmentation pipeline.

Combines three ideas, all validated on IGVC-grass by prior teams:
  1. Sooner Robotics 2023/2024 — per-class HSV `cv2.inRange` with
     fixed bounds, plus 3x 5x5 box-blur preprocessing for noise.
  2. iscumd/white_line_detection — adaptive V-channel threshold
     (mean + k*sigma) recomputed every N frames, to follow ambient
     exposure changes across sun/shadow transitions.
  3. Sooner Robotics 2024 — polygonal ROI to reject sky/trees in
     the upper portion of the frame.

The adaptive-V gate is AND-ed with the fixed white-lane HSV range so
only pixels that are BOTH achromatic-at-high-value AND pass the
lighting-aware floor mark as lane. Colored classes (barrels, pothole
paint rings) use the plain HSV range alone — they are color-specific,
not brightness-specific.

Output: mono8 class-ID mask + uint8 confidence plane. Class-ID 0 is
free/background. Per-class IDs are passed in params; defaults match
the shipped class_map.yaml (1=lane_white, 2=barrel_orange, 3=pothole).
"""

import cv2
import numpy as np

from avros_perception.pipelines.base import Pipeline, PipelineResult


# Defaults chosen to match shipped class_map.yaml. Callers override via params.
_DEFAULT_CLASS_IDS = {
    'lane': 1,
    'barrel': 2,
    'pothole': 3,
}


class HSVPipeline(Pipeline):
    """Classical-CV lane/barrel/pothole detector for outdoor grass courses."""

    def __init__(self, params, logger=None):
        super().__init__(params, logger)

        # Stateful adaptive-V across frames (NOT a tunable; tick counter +
        # cached floor that's refreshed every adaptive_period frames).
        self._tick = 0
        self._v_floor = None

        # Morphology kernel — pebbles/speckle removal (iscumd 3x3)
        self._morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        # All HSV bounds, blur, adaptive_k, class IDs, and the ROI polygon
        # are re-read from self.params on every run() so live `ros2 param set`
        # takes effect on the next frame. Cost is six tiny np.array casts +
        # a few int/float coercions — negligible vs. the cv2 ops.

    @staticmethod
    def _as_hsv(triplet):
        """Coerce a param into a uint8 HSV triplet ndarray suitable for inRange."""
        return np.array(list(triplet), dtype=np.uint8)

    @staticmethod
    def _reshape_poly(flat_or_pairs):
        """Accept either a flat [x,y,...] list or a list of [x,y] pairs."""
        seq = list(flat_or_pairs)
        if not seq:
            return []
        if isinstance(seq[0], (list, tuple)):
            return [(float(x), float(y)) for x, y in seq]
        if len(seq) % 2 != 0:
            raise ValueError(
                f'sky_roi_poly flat list must have even length, got {len(seq)}'
            )
        return [(float(seq[i]), float(seq[i + 1])) for i in range(0, len(seq), 2)]

    def _roi_polygon_px(self, h, w):
        """Convert normalized polygon (read live from params) to pixel coords."""
        raw_poly = self.params.get(
            'sky_roi_poly',
            [0.0, 0.0, 1.0, 0.0, 1.0, 0.35, 0.0, 0.35],
        )
        poly_norm = self._reshape_poly(raw_poly)
        if not poly_norm:
            return None
        pts = np.array(
            [[int(round(x * (w - 1))), int(round(y * (h - 1)))]
             for x, y in poly_norm],
            dtype=np.int32,
        )
        return pts.reshape(-1, 1, 2)

    def run(self, bgr, depth=None):
        """Produce a class-ID mask + confidence from a BGR uint8 frame."""
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f'HSVPipeline expects HxWx3 BGR; got {bgr.shape}')

        # Re-read live tunables from params on every frame (cheap).
        blur_iters = int(self.params.get('blur_iters', 3))
        adaptive_period = int(self.params.get('adaptive_period', 5))
        adaptive_k = float(self.params.get('adaptive_k', 3.0))
        lane_low = self._as_hsv(self.params.get('lane_low', [0, 0, 180]))
        lane_high = self._as_hsv(self.params.get('lane_high', [179, 60, 255]))
        barrel_low = self._as_hsv(self.params.get('barrel_low', [5, 120, 100]))
        barrel_high = self._as_hsv(self.params.get('barrel_high', [25, 255, 255]))
        pothole_low = self._as_hsv(self.params.get('pothole_low', [0, 0, 200]))
        pothole_high = self._as_hsv(self.params.get('pothole_high', [179, 40, 255]))
        cid = _DEFAULT_CLASS_IDS
        id_lane = int(self.params.get('class_id_lane', cid['lane']))
        id_barrel = int(self.params.get('class_id_barrel', cid['barrel']))
        id_pothole = int(self.params.get('class_id_pothole', cid['pothole']))

        # Sooner 2023 preprocessing — N iterations of 5x5 box blur
        blurred = bgr
        for _ in range(blur_iters):
            blurred = cv2.blur(blurred, (5, 5))
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # iscumd adaptive V-threshold, refreshed every N frames
        v_channel = hsv[:, :, 2]
        if self._tick == 0 or self._v_floor is None:
            self._v_floor = float(v_channel.mean() + adaptive_k * v_channel.std())
        self._tick = (self._tick + 1) % max(adaptive_period, 1)
        bright = (v_channel >= self._v_floor).astype(np.uint8) * 255

        # Per-class thresholds
        lane_hsv = cv2.inRange(hsv, lane_low, lane_high)
        lane = cv2.bitwise_and(lane_hsv, bright)
        barrel = cv2.inRange(hsv, barrel_low, barrel_high)
        pothole = cv2.inRange(hsv, pothole_low, pothole_high)

        # Cleanup — erode lane to kill grass speckle, open barrel/pothole
        # to kill pepper noise without closing narrow features.
        lane = cv2.erode(lane, self._morph_kernel)
        barrel = cv2.morphologyEx(barrel, cv2.MORPH_OPEN, self._morph_kernel)
        pothole = cv2.morphologyEx(pothole, cv2.MORPH_OPEN, self._morph_kernel)

        # Compose: write in ascending priority so lanes win over ambiguous
        # pixels that also pass the barrel/pothole range.
        h, w = v_channel.shape
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[pothole > 0] = id_pothole
        mask[barrel > 0] = id_barrel
        mask[lane > 0] = id_lane

        # ROI — zero out the sky/tree region (top of image by default)
        poly = self._roi_polygon_px(h, w)
        if poly is not None:
            cv2.fillPoly(mask, [poly], 0)

        confidence = np.where(mask > 0, 255, 0).astype(np.uint8)
        return PipelineResult(mask=mask, confidence=confidence)
