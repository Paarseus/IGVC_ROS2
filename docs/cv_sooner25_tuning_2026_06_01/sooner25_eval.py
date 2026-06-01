#!/usr/bin/env python3
"""
sooner25 evaluation + robustness harness (drives the REAL Sooner25Pipeline).

Runs the production pipeline over a frame set with arbitrary params and optional
angle/lighting AUGMENTATIONS (to test "works in different angles/settings"),
emitting per-frame metrics + overlays. Used by the tuning workflow so every
agent measures identically instead of re-implementing sooner25.

Metrics (no ground truth needed; heuristics for "clean continuous line, no FP"):
  lane_pct        fraction of in-ROI pixels marked lane (too high=flood/FP, too low=miss)
  n_comp          # connected components (clean line=few; speckle/FP=many)
  largest_frac    largest component / total lane px (clean line -> high)
  comp_ge50       # components >=50 px (real line segments vs speckle)

Adaptive-V mode computes sooner25_upper[V] per-frame from the asphalt band
(Sooner 2025 run-start auto-calibration idea) so the threshold tracks lighting.

Usage:
  python3 sooner25_eval.py --frames <dir-or-glob> --out <dir> \
      [--vupper 195] [--supper 255] [--blur-weight 5] [--blur-iters 3] \
      [--adaptive p97|mean+2.0sd|none] [--band 0.40 1.0] \
      [--augment none] [--json]
"""
import argparse, glob, json, os, sys

import cv2
import numpy as np

sys.path.insert(0, os.path.expanduser('~/IGVC_ROS2/src/avros_perception'))
from avros_perception.pipelines.sooner25 import Sooner25Pipeline  # noqa: E402

DEFAULT_ROI = [0.0, 0.0, 1.0, 0.0, 1.0, 0.40, 0.0, 0.40]


# ---- angle / lighting augmentations (simulate "different settings") ----
def aug_identity(b): return b


def aug_bright_up(b): return cv2.convertScaleAbs(b, alpha=1.0, beta=45)


def aug_bright_down(b): return cv2.convertScaleAbs(b, alpha=1.0, beta=-45)


def aug_gamma_hi(b):
    lut = np.array([((i / 255.0) ** (1 / 1.6)) * 255 for i in range(256)]).astype(np.uint8)
    return cv2.LUT(b, lut)


def aug_warp_tilt(b):
    """Perspective warp: pitch the ground plane (different camera/approach angle)."""
    h, w = b.shape[:2]
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    dst = np.float32([[w * 0.18, 0], [w * 0.82, 0], [w, h], [0, h]])
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(b, M, (w, h), borderValue=(120, 120, 120))


def aug_rot(b):
    h, w = b.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2, h / 2), 7, 1.0)
    return cv2.warpAffine(b, M, (w, h), borderValue=(120, 120, 120))


def aug_wb_warm(b):
    out = b.astype(np.float32)
    out[:, :, 0] *= 0.82          # less blue
    out[:, :, 2] *= 1.12          # more red -> warm/yellow tint (WB drift)
    return np.clip(out, 0, 255).astype(np.uint8)


def aug_shadow(b):
    """Diagonal shadow band across the asphalt (sun/cloud)."""
    h, w = b.shape[:2]
    mask = np.zeros((h, w), np.float32)
    pts = np.array([[0, int(h * 0.3)], [w, int(h * 0.6)], [w, h], [0, h]], np.int32)
    cv2.fillPoly(mask, [pts], 1.0)
    mask = cv2.GaussianBlur(mask, (51, 51), 0)[..., None]
    return np.clip(b.astype(np.float32) * (1 - 0.45 * mask), 0, 255).astype(np.uint8)


AUGS = {
    'none': aug_identity, 'bright_up': aug_bright_up, 'bright_down': aug_bright_down,
    'gamma_hi': aug_gamma_hi, 'warp_tilt': aug_warp_tilt, 'rot': aug_rot,
    'wb_warm': aug_wb_warm, 'shadow': aug_shadow,
}


def roi_mask(h, w, roi):
    pts = np.array([(int(roi[i] * (w - 1)), int(roi[i + 1] * (h - 1)))
                    for i in range(0, len(roi), 2)], np.int32)
    m = np.ones((h, w), np.uint8)
    cv2.fillPoly(m, [pts], 0)        # 0 inside ROI (masked), 1 elsewhere
    return m


def adaptive_vupper(bgr, band, mode):
    """Per-frame V ceiling from the asphalt band stats."""
    h = bgr.shape[0]
    y0, y1 = int(band[0] * h), int(band[1] * h)
    v = cv2.cvtColor(bgr[y0:y1], cv2.COLOR_BGR2HSV)[:, :, 2]
    if mode.startswith('p'):
        return int(np.percentile(v, float(mode[1:])))
    if mode.startswith('mean+'):
        k = float(mode[len('mean+'):].rstrip('sd'))
        return int(v.mean() + k * v.std())
    raise ValueError(mode)


def metrics(mask, roi):
    h, w = mask.shape
    rm = roi_mask(h, w, roi)
    inroi = int(rm.sum())
    lane = ((mask > 0) & (rm > 0)).astype(np.uint8)
    tot = int(lane.sum())
    pct = 100.0 * tot / max(inroi, 1)
    n, _, stats, _ = cv2.connectedComponentsWithStats(lane, 8)
    areas = sorted((int(stats[i, cv2.CC_STAT_AREA]) for i in range(1, n)), reverse=True)
    largest = areas[0] if areas else 0
    return {
        'lane_pct': round(pct, 3),
        'n_comp': len(areas),
        'comp_ge50': sum(1 for a in areas if a >= 50),
        'largest_frac': round(largest / max(tot, 1), 3),
        'lane_px': tot,
    }


def run(args):
    files = sorted(glob.glob(os.path.join(args.frames, '*.png'))
                   if os.path.isdir(args.frames) else glob.glob(args.frames))
    files = [f for f in files if 'overlay' not in os.path.basename(f)
             and 'mask' not in os.path.basename(f)]
    os.makedirs(args.out, exist_ok=True)
    augfn = AUGS[args.augment]
    results = {}
    for f in files:
        bgr0 = cv2.imread(f)
        if bgr0 is None:
            continue
        bgr = augfn(bgr0)
        vup = (adaptive_vupper(bgr, args.band, args.adaptive)
               if args.adaptive != 'none' else args.vupper)
        params = {
            'sooner25_lower': [0, 0, int(args.slower)],
            'sooner25_upper': [255, int(args.supper), int(vup)],
            'sooner25_blur_weight': int(args.blur_weight),
            'sooner25_blur_iters': int(args.blur_iters),
            'sky_roi_poly': args.roi, 'class_id_lane': 1,
        }
        res = Sooner25Pipeline(params).run(bgr)
        m = metrics(res.mask, args.roi)
        m['vupper'] = int(vup)
        name = os.path.splitext(os.path.basename(f))[0]
        if args.augment != 'none':
            name += f'_{args.augment}'
        results[name] = m
        ov = bgr.copy()
        ov[res.mask > 0] = (0, 255, 255)
        cv2.imwrite(os.path.join(args.out, f'{name}_ov.png'), ov)
    # aggregate robustness summary
    pcts = [v['lane_pct'] for v in results.values()]
    lf = [v['largest_frac'] for v in results.values()]
    agg = {
        'n_frames': len(results),
        'lane_pct_mean': round(float(np.mean(pcts)), 3) if pcts else 0,
        'lane_pct_std': round(float(np.std(pcts)), 3) if pcts else 0,
        'lane_pct_min': round(min(pcts), 3) if pcts else 0,
        'lane_pct_max': round(max(pcts), 3) if pcts else 0,
        'largest_frac_mean': round(float(np.mean(lf)), 3) if lf else 0,
    }
    out = {'config': {'vupper': args.vupper, 'supper': args.supper,
                      'adaptive': args.adaptive, 'blur': [args.blur_weight, args.blur_iters],
                      'augment': args.augment},
           'aggregate': agg, 'per_frame': results}
    if args.json:
        print(json.dumps(out, indent=1))
    else:
        print(f"cfg V<={args.vupper} S<={args.supper} adapt={args.adaptive} aug={args.augment}: "
              f"lane% {agg['lane_pct_mean']}±{agg['lane_pct_std']} "
              f"[{agg['lane_pct_min']},{agg['lane_pct_max']}] "
              f"largest_frac {agg['largest_frac_mean']} over {agg['n_frames']} frames")
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', required=True)
    ap.add_argument('--out', default='/tmp/s25eval')
    ap.add_argument('--vupper', type=int, default=195)
    ap.add_argument('--supper', type=int, default=255)
    ap.add_argument('--slower', type=int, default=0)
    ap.add_argument('--blur-weight', type=int, default=5)
    ap.add_argument('--blur-iters', type=int, default=3)
    ap.add_argument('--adaptive', default='none')
    ap.add_argument('--band', type=float, nargs=2, default=[0.40, 1.0])
    ap.add_argument('--roi', type=float, nargs='+', default=DEFAULT_ROI)
    ap.add_argument('--augment', default='none', choices=list(AUGS))
    ap.add_argument('--json', action='store_true')
    run(ap.parse_args())
