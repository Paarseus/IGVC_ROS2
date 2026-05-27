#!/usr/bin/env python3
"""Test C (M3) analysis: closing error of a 1×1 m square trajectory.

Validates end-to-end accuracy. Benchmarks:
  - Ollman 1–5% of distance traveled (= 4–20 cm closing on 1×1m)
  - Mandow §5 <10° yaw closing

Usage:
    python3 scripts/analyze_M3.py <csv_dir> [--t-start S] [--t-end S]

Auto-detects the M3 window as: longest continuous activity after the
last rotation window (M2). If multiple goal segments are detected,
splits them and reports per-leg drift.
"""
import argparse
import math
import sys
from pathlib import Path

import pandas as pd


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def load(csv_dir: Path, name: str):
    p = csv_dir / name
    if not p.exists():
        return None
    return pd.read_csv(p)


def clip(df, t0, t1):
    return df[(df['t_rel'] >= t0) & (df['t_rel'] <= t1)].reset_index(drop=True)


def detect_window(cmd_vel: pd.DataFrame):
    """M3 window = activity after the LAST rotation pause.
    Heuristic: find the last time when cmd_vel is idle for >2s, then take
    everything after until the bag ends or another long idle."""
    if cmd_vel is None or len(cmd_vel) == 0:
        return None, None
    df = cmd_vel.copy()
    df['idle'] = (df['vx'].abs() < 0.05) & (df['wz'].abs() < 0.05)
    # find pauses (idle runs >2s)
    groups = (df['idle'] != df['idle'].shift()).cumsum()
    pauses = []
    for _, run in df[df['idle']].groupby(groups):
        if len(run) >= 2:
            t0 = run['t_rel'].iloc[0]
            t1 = run['t_rel'].iloc[-1]
            if t1 - t0 > 2.0:
                pauses.append((t0, t1))
    if not pauses:
        return df['t_rel'].iloc[0], df['t_rel'].iloc[-1]
    t_start = pauses[-1][1]
    t_end = df['t_rel'].iloc[-1]
    return t_start, t_end


def split_legs(cmd_vel: pd.DataFrame, t0, t1):
    """Split M3 window into per-goal legs by short idle pauses (<2s)
    between cmd_vel bursts."""
    df = clip(cmd_vel, t0, t1)
    if len(df) == 0:
        return [(t0, t1)]
    df['drive'] = (df['vx'].abs() > 0.05) | (df['wz'].abs() > 0.05)
    groups = (df['drive'] != df['drive'].shift()).cumsum()
    legs = []
    for _, run in df[df['drive']].groupby(groups):
        if len(run) >= 5:
            legs.append((run['t_rel'].iloc[0], run['t_rel'].iloc[-1]))
    return legs if legs else [(t0, t1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_dir', type=Path)
    ap.add_argument('--t-start', type=float, default=None)
    ap.add_argument('--t-end', type=float, default=None)
    args = ap.parse_args()

    ekf = load(args.csv_dir, 'odometry_filtered.csv')
    cmdvel = load(args.csv_dir, 'cmd_vel.csv')

    if ekf is None:
        sys.exit('missing odometry_filtered.csv')

    t0, t1 = args.t_start, args.t_end
    if t0 is None or t1 is None:
        t0_auto, t1_auto = detect_window(cmdvel)
        t0 = t0 if t0 is not None else t0_auto
        t1 = t1 if t1 is not None else t1_auto

    print('# M3 (Test C) — 1×1m closing square')
    print(f'window: t={t0:.2f}..{t1:.2f} s  ({t1-t0:.2f} s)')
    print()

    ekf_c = clip(ekf, t0, t1)
    if len(ekf_c) < 2:
        sys.exit('not enough EKF samples in window')

    # Global closing error
    x0, y0, yaw0 = ekf_c['x'].iloc[0], ekf_c['y'].iloc[0], ekf_c['yaw'].iloc[0]
    xN, yN, yawN = ekf_c['x'].iloc[-1], ekf_c['y'].iloc[-1], ekf_c['yaw'].iloc[-1]
    xy_close = math.hypot(xN - x0, yN - y0)
    yaw_close = math.degrees(wrap_pi(yawN - yaw0))

    # Path length
    dx = ekf_c['x'].diff().fillna(0)
    dy = ekf_c['y'].diff().fillna(0)
    path_len = float((dx**2 + dy**2).pow(0.5).sum())

    pct = 100 * xy_close / path_len if path_len > 0 else float('inf')

    print('## Square closing metrics')
    print()
    print(f'- path length (sum of EKF Δxy): **{path_len:.3f} m**')
    print(f'- xy closing error: **{xy_close:.3f} m**  '
          f'({pct:.1f}% of path)')
    print(f'- yaw closing error: **{yaw_close:+.2f}°**')
    print()

    # Verdict
    print('### Verdict')
    if xy_close < 0.1 and abs(yaw_close) < 10:
        print('**HEALTHY** — Ollman <2.5% and Mandow <10° both pass.')
    elif xy_close < 0.2 and abs(yaw_close) < 20:
        print('**MARGINAL** — Ollman 1–5% range. Acceptable but '
              'check per-leg drift for asymmetry.')
    else:
        print(f'**FAIL** — xy={pct:.1f}% (target <5%), yaw={yaw_close:+.2f}° '
              '(target <10°).')
    print()

    # Per-leg analysis
    legs = split_legs(cmdvel, t0, t1)
    print(f'## Per-leg analysis ({len(legs)} legs detected)')
    print()
    print('| Leg | t_start | t_end | Δx (m) | Δy (m) | leg length (m) | Δyaw (deg) |')
    print('|---:|---:|---:|---:|---:|---:|---:|')
    for i, (la, lb) in enumerate(legs, 1):
        leg = clip(ekf, la, lb)
        if len(leg) < 2:
            continue
        ldx = leg['x'].iloc[-1] - leg['x'].iloc[0]
        ldy = leg['y'].iloc[-1] - leg['y'].iloc[0]
        llen = math.hypot(ldx, ldy)
        ldyaw = math.degrees(wrap_pi(leg['yaw'].iloc[-1] - leg['yaw'].iloc[0]))
        print(f'| {i} | {la:.2f} | {lb:.2f} | {ldx:+.3f} | {ldy:+.3f} | '
              f'{llen:.3f} | {ldyaw:+.2f} |')


if __name__ == '__main__':
    main()
