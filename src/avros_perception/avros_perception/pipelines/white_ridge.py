"""
Shadow-robust white-line pipeline (selected via pipeline:='white_ridge').

Wraps white_ridge_detector.WhiteRidgeDetector: Hessian ridges on log
luminance (a shadow is multiplicative, so it becomes an additive offset the
second derivative ignores), ridge scales chosen from the camera's ground
geometry, a both-sides-darker test, a measured ground-width gate
(2.5-15 cm, rejects sealed joints and panel seams), shadow-gap rejection
and a ground-length gate. Ridge orientation is local, so curves are kept.

The detector needs this camera's ground geometry, so it is calibrated per
camera with ridge_calib_<camera_name> = [fx, fy, cx, cy, ref_width,
height_m, pitch_deg]: intrinsics at ref_width pixels wide (scaled to the
processing size), camera height above the ground and downward pitch.

Frames are processed at most ridge_proc_width pixels wide (the resolution
the detector was validated at) and the mask is scaled back with NEAREST.

wants_depth: the node passes the organized cloud's forward (optical-axis)
depth, any resolution. Segments that stereo puts more than
ridge_obj_height_m above the ground (chair bases, drawer handles, table
edges, people) are dropped; tape has zero height. For a ray that would hit
the ground at depth d_g, a surface h above the ground is met at
d_g * (1 - h / camera_height), so the height becomes a depth margin.
"""

import cv2
import numpy as np

from avros_perception.pipelines.base import Pipeline, PipelineResult
from avros_perception.pipelines.white_ridge_detector import WhiteRidgeDetector


class WhiteRidgePipeline(Pipeline):
    """White lane lines only; single class (class_id_lane)."""

    wants_depth = True

    def __init__(self, params, logger=None):
        super().__init__(params, logger)
        self._det = None
        self._key = None

    def _calib(self):
        cam = str(self.params.get('camera_name', ''))
        calib = list(self.params.get(f'ridge_calib_{cam}', []) or [])
        if len(calib) != 7:
            raise ValueError(
                f'white_ridge: ridge_calib_{cam} must be [fx, fy, cx, cy, '
                f'ref_width, height_m, pitch_deg], got {calib}')
        return [float(v) for v in calib]

    def _detector(self, w, h):
        fx, fy, cx, cy, ref_w, height, pitch = self._calib()
        knobs = dict(
            t_high=float(self.params.get('ridge_t_high', 0.22)),
            t_low=float(self.params.get('ridge_t_low', 0.10)),
            side_log=float(self.params.get('ridge_side_log', 0.10)),
            w_min=float(self.params.get('ridge_w_min', 0.025)),
            w_max=float(self.params.get('ridge_w_max', 0.15)),
            min_len_m=float(self.params.get('ridge_min_len_m', 0.30)),
            max_range_m=float(self.params.get('ridge_max_range_m', 12.0)),
        )
        key = (fx, fy, cx, cy, ref_w, height, pitch, w, h,
               tuple(sorted(knobs.items())))
        if key != self._key:
            # pixel-centre-correct rescale of K from ref_width to w
            s = w / ref_w
            K = np.array([[fx * s, 0.0, (cx + 0.5) * s - 0.5],
                          [0.0, fy * s, (cy + 0.5) * s - 0.5],
                          [0.0, 0.0, 1.0]])
            self._det = WhiteRidgeDetector(K, height, pitch, w, h, **knobs)
            self._key = key
            if self.logger is not None:
                self.logger.info(
                    f'white_ridge: {w}x{h} fx={K[0, 0]:.1f} h={height:.3f} m '
                    f'pitch={pitch:.1f} deg {knobs}')
        return self._det

    def warmup(self):
        self._calib()      # fail at startup, not on the first frame

    def run(self, bgr, depth=None):
        h, w = bgr.shape[:2]
        pw = min(w, int(self.params.get('ridge_proc_width', 480)))
        ph = int(round(h * pw / w))
        small = bgr if pw == w else cv2.resize(
            bgr, (pw, ph), interpolation=cv2.INTER_AREA)
        if depth is not None and bool(self.params.get('ridge_use_depth', True)):
            depth = cv2.resize(depth.astype(np.float32), (pw, ph),
                               interpolation=cv2.INTER_NEAREST)
        else:
            depth = None
        lines = self._detector(pw, ph).detect(
            small, depth=depth,
            depth_margin=float(self.params.get('ridge_obj_height_m', 0.05)) / self._calib()[5],
            depth_vote=float(self.params.get('ridge_depth_vote', 0.3)))
        out = lines.astype(np.uint8)
        if pw != w:
            out = cv2.resize(out, (w, h), interpolation=cv2.INTER_NEAREST)
        mask = out * np.uint8(int(self.params.get('class_id_lane', 1)))
        confidence = np.where(out > 0, 255, 0).astype(np.uint8)
        return PipelineResult(mask=mask, confidence=confidence)
