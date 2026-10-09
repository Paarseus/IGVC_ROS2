#!/usr/bin/env python3
"""
Self-contained offline validation harness for the 'canny' lane pipeline.

Loads CannyPipeline DIRECTLY from the source file via importlib (NO colcon / ROS
needed — only cv2 + numpy), runs it on the captured IGVC field frames, writes
per-frame tinted overlays, and prints a JSON summary with:

  * per-frame lane-px count
  * frame-to-frame coefficient-of-variation (CV) across the 7 canonical
    exp_frames f00..f06 (exposure-stability metric; <0.15 is good)
  * a rough white-line-recall / asphalt-noise estimate

The harness stubs the avros_perception.pipelines.base module so canny.py's
`from avros_perception.pipelines.base import Pipeline, PipelineResult` resolves
without importing the whole ROS package tree.

Run:  python3 docs/cv_canny_pipeline_2026_06_01/validate_canny.py
"""

import glob
import importlib.util
import json
import os
import sys
import types
from dataclasses import dataclass

import cv2
import numpy as np

REPO = '/home/mspacman/IGVC_ROS2'
CANNY_SRC = os.path.join(
    REPO, 'src/avros_perception/avros_perception/pipelines/canny.py')
OUT_DIR = os.path.join(REPO, 'docs/cv_canny_pipeline_2026_06_01/validation')

EXP_FRAMES = sorted(glob.glob(os.path.join(REPO, 'exp_frames/*.png')))
# The 7 canonical near-identical-scene exposure-drift frames (design spec).
EXP7 = sorted(glob.glob(os.path.join(REPO, 'exp_frames/f0[0-6]_*.png')))
SCENE_FRAMES = sorted(
    glob.glob(os.path.join(
        REPO, 'docs/cv_adaptive_debug_2026_05_31/input_rgb_0*.png')))
# Skip the _2x upscaled duplicates so we test distinct scenes only.
SCENE_FRAMES = [f for f in SCENE_FRAMES if not f.endswith('_2x.png')]


# ---- Stub the base module so canny.py imports without the ROS package -------
def _install_base_stub():
    pkg = types.ModuleType('avros_perception')
    pkg.__path__ = []
    sub = types.ModuleType('avros_perception.pipelines')
    sub.__path__ = []
    base = types.ModuleType('avros_perception.pipelines.base')

    @dataclass
    class PipelineResult:
        mask: np.ndarray
        confidence: np.ndarray

    class Pipeline:
        def __init__(self, params, logger=None):
            self.params = params if params is not None else {}
            self.logger = logger

        def warmup(self):
            return None

        def run(self, bgr, depth=None):
            raise NotImplementedError

    base.Pipeline = Pipeline
    base.PipelineResult = PipelineResult
    sys.modules['avros_perception'] = pkg
    sys.modules['avros_perception.pipelines'] = sub
    sys.modules['avros_perception.pipelines.base'] = base


def _load_canny():
    _install_base_stub()
    spec = importlib.util.spec_from_file_location(
        'avros_perception.pipelines.canny', CANNY_SRC)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['avros_perception.pipelines.canny'] = mod
    spec.loader.exec_module(mod)
    return mod.CannyPipeline


# In-code defaults mirror perception.yaml (passing {} would also work since the
# pipeline reads via .get with defaults — but make exposure intent explicit).
DEFAULT_PARAMS = {
    'canny_block_size': 21, 'canny_C': -8.0, 'canny_blur': 9,
    'canny_max_sat': 70, 'canny_sigma': 0.33, 'canny_aperture': 3,
    # 2026-06-01 shipped defaults: edge_and FALSE (AND wrecks exposure CV +
    # eats faint paint), use_ipm FALSE (uncalibrated IPM extrapolates obstacle
    # edges into phantom lanes), elong>=2.5 AND fill<=0.10 shape gate.
    'canny_edge_and': False, 'canny_use_clahe': False, 'canny_clahe_clip': 2.0,
    'canny_min_area': 80, 'canny_elong_min': 2.5, 'canny_fill_max': 0.10,
    'canny_longaxis_min': 0, 'canny_longaxis_min_base': 45,
    'canny_use_ipm': False,
    'canny_ipm_src': [0.42, 0.42, 0.58, 0.42, 1.0, 1.0, 0.0, 1.0],
    'canny_bev_w': 300, 'canny_bev_h': 400,
    'canny_fit_mode': 'window', 'canny_close_v': 15, 'canny_nwindows': 10,
    'canny_col_k': 3.0, 'canny_col_floor': 8, 'canny_min_inliers': 150,
    'canny_line_draw_w': 7, 'canny_hough_rho': 2, 'canny_hough_thresh': 40,
    'canny_angle_band': 35, 'canny_use_depth': True, 'canny_height_tol': 0.15,
    'class_id_lane': 1,
    'sky_roi_poly': [0.0, 0.0, 1.0, 0.0, 1.0, 0.40, 0.0, 0.40],
}


def cv_of(values):
    a = np.asarray(values, dtype=np.float64)
    if a.size == 0 or a.mean() == 0:
        return None
    return float(a.std() / a.mean())


def roi_mask(h, w, poly_norm):
    """Boolean mask: True for the IN-INTEREST region (below the sky ROI)."""
    keep = np.ones((h, w), dtype=bool)
    if poly_norm:
        pts = np.array(
            [(int(poly_norm[i] * (w - 1)), int(poly_norm[i + 1] * (h - 1)))
             for i in range(0, len(poly_norm), 2)], dtype=np.int32)
        sky = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(sky, [pts], 1)
        keep &= (sky == 0)
    return keep


def estimate_recall_noise(bgr, mask, params):
    """Rough white-line-recall + asphalt-noise proxy (NO ground-truth labels).

    Recall proxy: of the truly-white-paint candidate pixels in the in-interest
    region (high HLS-L local contrast AND low-S — i.e. the adaptive white core
    before geometry), what fraction did the final mask keep? Captures whether
    the geometric stack threw away real line. Noise proxy: fraction of final
    lane px whose local HLS-S is high (colored clutter) or whose 5x5 L variance
    is texture-like (asphalt). Both are heuristics, not metrics.
    """
    h, w = bgr.shape[:2]
    hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
    L = cv2.GaussianBlur(hls[:, :, 1], (9, 9), 0)
    S = hls[:, :, 2]
    white = cv2.adaptiveThreshold(
        L, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, -8.0)
    white[S > int(params['canny_max_sat'])] = 0
    keep = roi_mask(h, w, params.get('sky_roi_poly'))
    white_core = (white > 0) & keep
    lane = (mask > 0)
    core_n = int(white_core.sum())
    recall = float((lane & white_core).sum() / core_n) if core_n else None
    lane_n = int(lane.sum())
    # Noise proxy: lane px that are high-S (should have been gated) — leakage.
    noise = float((lane & (S > int(params['canny_max_sat']))).sum() / lane_n) \
        if lane_n else 0.0
    return recall, noise, core_n


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    CannyPipeline = _load_canny()
    pipe = CannyPipeline(dict(DEFAULT_PARAMS))
    pipe.warmup()

    all_frames = EXP_FRAMES + SCENE_FRAMES
    per_frame = []
    overlays = []
    errors = []

    for path in all_frames:
        bgr = cv2.imread(path, cv2.IMREAD_COLOR)
        if bgr is None:
            errors.append(f'imread failed: {path}')
            continue
        try:
            res = pipe.run(bgr)  # node calls run(bgr) — no depth, by design
        except Exception as e:  # noqa: BLE001 — harness must report, not crash
            errors.append(f'{os.path.basename(path)}: {type(e).__name__}: {e}')
            continue

        mask = res.mask
        lane_px = int((mask > 0).sum())
        recall, noise, core_n = estimate_recall_noise(bgr, mask, DEFAULT_PARAMS)

        # Tinted overlay: lane px -> green.
        overlay = bgr.copy()
        overlay[mask > 0] = (0, 255, 0)
        blended = cv2.addWeighted(bgr, 0.5, overlay, 0.5, 0)
        outp = os.path.join(OUT_DIR, 'overlay_' + os.path.basename(path))
        cv2.imwrite(outp, blended)
        overlays.append(outp)

        per_frame.append({
            'frame': os.path.basename(path),
            'shape': [int(mask.shape[0]), int(mask.shape[1])],
            'lane_px': lane_px,
            'white_core_px': core_n,
            'recall_proxy': None if recall is None else round(recall, 3),
            'highS_leak_proxy': round(noise, 4),
        })

    # Exposure-stability CV across the 7 canonical exp frames f00..f06.
    exp7_names = {os.path.basename(p) for p in EXP7}
    exp7_counts = [r['lane_px'] for r in per_frame if r['frame'] in exp7_names]
    exp_cv = cv_of(exp7_counts)

    # CV across ALL exp_frames too (broader exposure sweep, f00..f13).
    exp_names = {os.path.basename(p) for p in EXP_FRAMES}
    exp_all_counts = [r['lane_px'] for r in per_frame if r['frame'] in exp_names]
    exp_all_cv = cv_of(exp_all_counts)

    recalls = [r['recall_proxy'] for r in per_frame
               if r['recall_proxy'] is not None]
    summary = {
        'pipeline': 'canny',
        'cv2_version': cv2.__version__,
        'frames_tested': len(per_frame),
        'exp7_frames': sorted(exp7_names),
        'exp7_lane_px': exp7_counts,
        'exposure_cv_exp7': None if exp_cv is None else round(exp_cv, 4),
        'exposure_cv_all_exp': None if exp_all_cv is None else round(exp_all_cv, 4),
        'mean_recall_proxy': round(float(np.mean(recalls)), 3) if recalls else None,
        'mean_lane_px': round(float(np.mean([r['lane_px'] for r in per_frame])), 1)
        if per_frame else None,
        'overlay_dir': OUT_DIR,
        'errors': errors,
        'per_frame': per_frame,
    }
    print(json.dumps(summary, indent=2))
    return 0 if not errors else 1


if __name__ == '__main__':
    sys.exit(main())
