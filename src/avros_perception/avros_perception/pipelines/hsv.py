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

        # Preprocessing
        self._blur_iters = int(self.params.get('blur_iters', 3))

        # iscumd adaptive V threshold
        self._adaptive_period = int(self.params.get('adaptive_period', 5))
        self._adaptive_k = float(self.params.get('adaptive_k', 3.0))
        self._tick = 0
        self._v_floor = None

        # Per-class HSV bounds — opencv HSV is H in [0,179], S/V in [0,255].
        # Starter values from Sooner 2023/2024; field-calibrate per camera.
        self._lane_low = self._as_hsv(self.params.get('lane_low', [0, 0, 180]))
        self._lane_high = self._as_hsv(self.params.get('lane_high', [179, 60, 255]))
        self._barrel_low = self._as_hsv(self.params.get('barrel_low', [5, 120, 100]))
        self._barrel_high = self._as_hsv(self.params.get('barrel_high', [25, 255, 255]))
        self._pothole_low = self._as_hsv(self.params.get('pothole_low', [0, 0, 200]))
        self._pothole_high = self._as_hsv(self.params.get('pothole_high', [179, 40, 255]))

        # Class IDs (must match class_map.yaml)
        cid = _DEFAULT_CLASS_IDS
        self._id_lane = int(self.params.get('class_id_lane', cid['lane']))
        self._id_barrel = int(self.params.get('class_id_barrel', cid['barrel']))
        self._id_pothole = int(self.params.get('class_id_pothole', cid['pothole']))

        # ROI polygon: list of [x,y] pairs in NORMALIZED 0..1 coordinates.
        # Scaled to image size on every frame so it's resolution-independent.
        # Default rejects the top 35% of the frame (sky / distant trees).
        self._roi_poly_norm = self.params.get(
            'sky_roi_poly',
            [[0.0, 0.0], [1.0, 0.0], [1.0, 0.35], [0.0, 0.35]],
        )

        # Morphology kernel — pebbles/speckle removal (iscumd 3x3)
        self._morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    @staticmethod
    def _as_hsv(triplet):
        """Coerce a param into a uint8 HSV triplet ndarray suitable for inRange."""
        return np.array(list(triplet), dtype=np.uint8)

    def _roi_polygon_px(self, h, w):
        """Convert normalized polygon to pixel coords for cv2.fillPoly."""
        if not self._roi_poly_norm:
            return None
        pts = np.array(
            [[int(round(x * (w - 1))), int(round(y * (h - 1)))]
             for x, y in self._roi_poly_norm],
            dtype=np.int32,
        )
        return pts.reshape(-1, 1, 2)

    def run(self, bgr, depth=None):
        """Produce a class-ID mask + confidence from a BGR uint8 frame."""
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f'HSVPipeline expects HxWx3 BGR; got {bgr.shape}')

        # Sooner 2023 preprocessing — 3 iterations of 5x5 box blur
        blurred = bgr
        for _ in range(self._blur_iters):
            blurred = cv2.blur(blurred, (5, 5))
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # iscumd adaptive V-threshold, refreshed every N frames
        v_channel = hsv[:, :, 2]
        if self._tick == 0 or self._v_floor is None:
            self._v_floor = float(v_channel.mean() + self._adaptive_k * v_channel.std())
        self._tick = (self._tick + 1) % max(self._adaptive_period, 1)
        bright = (v_channel >= self._v_floor).astype(np.uint8) * 255

        # Per-class thresholds
        lane_hsv = cv2.inRange(hsv, self._lane_low, self._lane_high)
        lane = cv2.bitwise_and(lane_hsv, bright)
        barrel = cv2.inRange(hsv, self._barrel_low, self._barrel_high)
        pothole = cv2.inRange(hsv, self._pothole_low, self._pothole_high)

        # Cleanup — erode lane to kill grass speckle, open barrel/pothole
        # to kill pepper noise without closing narrow features.
        lane = cv2.erode(lane, self._morph_kernel)
        barrel = cv2.morphologyEx(barrel, cv2.MORPH_OPEN, self._morph_kernel)
        pothole = cv2.morphologyEx(pothole, cv2.MORPH_OPEN, self._morph_kernel)

        # Compose: write in ascending priority so lanes win over ambiguous
        # pixels that also pass the barrel/pothole range.
        h, w = v_channel.shape
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[pothole > 0] = self._id_pothole
        mask[barrel > 0] = self._id_barrel
        mask[lane > 0] = self._id_lane

        # ROI — zero out the sky/tree region (top of image by default)
        poly = self._roi_polygon_px(h, w)
        if poly is not None:
            cv2.fillPoly(mask, [poly], 0)

        confidence = np.where(mask > 0, 255, 0).astype(np.uint8)
        return PipelineResult(mask=mask, confidence=confidence)
