"""
Sooner Robotics 2025 HSV pipeline — port for kiwicampus / Nav2 integration.

The two body methods (apply_blur, apply_hsv) below are copy-pasted from
Sooner Robotics' 2025 IGVC AutoNav winning code:

  https://github.com/SoonerRobotics/autonav_software_2025/blob/main/
  autonav_ws/src/autonav_vision/src/transformations.py

The pipeline is intentionally minimal: blur the image, threshold the asphalt
(low S, mid V), then INVERT the mask. The result marks anything that is NOT
asphalt as an obstacle — lane paint, barrels, potholes — in a single binary
mask. We then map that binary 255 -> class_id_lane (1) so the kiwicampus
semantic layer treats the whole mask as a single 'danger' class.

Why this is more robust than thresholding the foreground (our hsv.py):
the asphalt color is more stable across lighting than paint brightness, so
inverting a background threshold sidesteps the V-floor scene-dependence
that bit us in /tmp/hsv_iter all session.

Things we deliberately do NOT port from Sooner:
  - Inverse perspective transform (we get top-down geometry from the
    kiwicampus organized-cloud projection)
  - Their robot-blanker region of disinterest (we use sky_roi_poly)
  - Their per-camera rotation / multi-camera fusion (single ZED for now)
"""

import cv2
import numpy as np

from avros_perception.pipelines.base import Pipeline, PipelineResult


# ---- Sooner 2025 functions (copy-pasted, parameters threaded through args) ----

def apply_blur(img, blur_weight, blur_iterations):
    """Sooner 2025 transformations.py — apply_blur."""
    for _ in range(blur_iterations):
        img = cv2.blur(img, (blur_weight, blur_weight))
    return img


def apply_hsv(img, lower, upper):
    """Sooner 2025 transformations.py — apply_hsv.

    lower/upper are 3-tuples (H, S, V). Note the inversion: cv2.inRange
    marks pixels INSIDE the (dim+gray asphalt) range, then `255 - mask`
    flips it so the output is 255 where the pixel is OUTSIDE — i.e.
    lane paint (V > upper_val) or saturated objects (S > upper_sat).
    """
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img, lower, upper)
    return 255 - mask


# ---- Port wrapper: implements our Pipeline interface ----

class Sooner25Pipeline(Pipeline):
    """Thin wrapper that calls apply_blur + apply_hsv and emits a class-ID mask.

    Single output class — everything non-asphalt becomes class_id_lane (1)
    so the kiwicampus 'danger' costing applies to it. If we later want
    separate barrel/pothole channels we'd add a second pass; for now a
    unified obstacle blob matches Sooner's winning strategy.
    """

    def _reshape_poly(self, seq):
        if not seq:
            return []
        if len(seq) % 2 != 0:
            raise ValueError(
                f'sky_roi_poly flat list must have even length, got {len(seq)}'
            )
        return [(float(seq[i]), float(seq[i + 1])) for i in range(0, len(seq), 2)]

    def _roi_polygon_px(self, h, w):
        """Same sky_roi_poly handling as the legacy HSV pipeline."""
        raw_poly = self.params.get(
            'sky_roi_poly',
            [0.0, 0.0, 1.0, 0.0, 1.0, 0.35, 0.0, 0.35],
        )
        poly_norm = self._reshape_poly(raw_poly)
        if not poly_norm:
            return None
        return np.array(
            [(int(x * (w - 1)), int(y * (h - 1))) for x, y in poly_norm],
            dtype=np.int32,
        )

    def run(self, bgr, depth=None):
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f'Sooner25Pipeline expects HxWx3 BGR; got {bgr.shape}')

        # Live-tunable params (re-read every frame, same pattern as hsv.py).
        blur_weight = int(self.params.get('sooner25_blur_weight', 5))
        blur_iters = int(self.params.get('sooner25_blur_iters', 3))
        lower = tuple(int(x) for x in self.params.get('sooner25_lower', [0, 0, 0]))
        upper = tuple(int(x) for x in self.params.get('sooner25_upper', [255, 95, 210]))
        class_id_lane = int(self.params.get('class_id_lane', 1))

        # Sooner pipeline.
        blurred = apply_blur(bgr, blur_weight, blur_iters)
        binary = apply_hsv(blurred, lower, upper)   # 255 = obstacle, 0 = drivable

        # Map binary -> class IDs for kiwicampus.
        h, w = binary.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[binary > 0] = class_id_lane

        # Sky / out-of-interest ROI zeroed last so we don't paint trees as lanes.
        poly = self._roi_polygon_px(h, w)
        if poly is not None:
            cv2.fillPoly(mask, [poly], 0)

        confidence = np.where(mask > 0, 255, 0).astype(np.uint8)
        return PipelineResult(mask=mask, confidence=confidence)
