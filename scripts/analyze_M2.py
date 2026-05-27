#!/usr/bin/env python3
"""Test B (M2) analysis: IMU vs ZED VIO vs wheel under pure rotation.

Three independent yaw sources rotate the chassis 360°. Whichever two
agree most closely is the truth; the dissenter is biased.

Usage:
    python3 scripts/analyze_M2.py <csv_dir> [--rate 0.5] [--rate 0.8]

Auto-detects rotation windows from /cmd_vel where |vx|<0.05 and
|wz|>0.3. If the --rate flags are given, expects two windows matching
those commanded rates; otherwise reports every detected rotation
window.
"""
import argparse
import math
import sys
from pathlib import Path

import pandas as pd


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def integrated_delta_yaw(times, yaws):
    if len(times) < 2:
        return 0.0
    dy = 0.0
    prev = yaws[0]
    for y in yaws[1:]:
        dy += wrap_pi(y - prev)
        prev = y
    return dy


def integrated_vyaw(times, vyaws):
    if len(times) < 2:
        return 0.0
    return float(((vyaws[:-1] + vyaws[1:]) * 0.5 *
                  (times[1:] - times[:-1])).sum())


def load(csv_dir: Path, name: str):
    p = csv_dir / name
    if not p.exists():
        return None
    return pd.read_csv(p)


def detect_rotation_windows(cmd_vel: pd.DataFrame):
    """Find contiguous windows where |vx|<0.05 and |wz|>0.3."""
    if cmd_vel is None or len(cmd_vel) == 0:
        return []
    df = cmd_vel.copy()
    df['rot'] = (df['vx'].abs() < 0.05) & (df['wz'].abs() > 0.3)
    groups = (df['rot'] != df['rot'].shift()).cumsum()
    windows = []
    for _, run in df[df['rot']].groupby(groups):
        if len(run) >= 5:  # at least 5 cmd samples = ~250ms
            t0 = run['t_rel'].iloc[0]
            t1 = run['t_rel'].iloc[-1]
            wz_cmd = run['wz'].median()
            windows.append((t0, t1, wz_cmd))
    return windows


def clip(df, t0, t1):
    return df[(df['t_rel'] >= t0) & (df['t_rel'] <= t1)].reset_index(drop=True)


def analyze_window(t0, t1, wz_cmd, imu, wheel, ekf, zed):
    duration = t1 - t0
    expected_deg = math.degrees(wz_cmd * duration)

    sources = []
    imu_c = clip(imu, t0, t1)
    wheel_c = clip(wheel, t0, t1)
    ekf_c = clip(ekf, t0, t1)

    if len(imu_c) >= 2:
        dy = integrated_delta_yaw(imu_c['t_rel'].to_numpy(),
                                  imu_c['yaw'].to_numpy())
        sources.append(('IMU (quaternion)', math.degrees(dy)))

    if len(wheel_c) >= 2:
        dy = integrated_vyaw(wheel_c['t_rel'].to_numpy(),
                             wheel_c['wz'].to_numpy())
        sources.append(('Wheel odom (∫vyaw)', math.degrees(dy)))

    if len(ekf_c) >= 2:
        dy = integrated_delta_yaw(ekf_c['t_rel'].to_numpy(),
                                  ekf_c['yaw'].to_numpy())
        sources.append(('EKF /odometry/filtered', math.degrees(dy)))

    if zed is not None:
        zed_c = clip(zed, t0, t1)
        if len(zed_c) >= 2:
            dy = integrated_delta_yaw(zed_c['t_rel'].to_numpy(),
                                      zed_c['yaw'].to_numpy())
            sources.append(('ZED VIO', math.degrees(dy)))

    return duration, expected_deg, sources


def find_truth(sources):
    """Pairwise compare; the pair with smallest |diff| are the agreers."""
    if len(sources) < 3:
        return None, 'need >=3 sources for majority'
    best_pair, best_diff = None, float('inf')
    for i in range(len(sources)):
        for j in range(i + 1, len(sources)):
            d = abs(sources[i][1] - sources[j][1])
            if d < best_diff:
                best_diff = d
                best_pair = (i, j)
    odd = [k for k in range(len(sources)) if k not in best_pair]
    return best_pair, odd, best_diff


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_dir', type=Path)
    args = ap.parse_args()

    imu = load(args.csv_dir, 'imu_data.csv')
    wheel = load(args.csv_dir, 'wheel_odom.csv')
    ekf = load(args.csv_dir, 'odometry_filtered.csv')
    cmdvel = load(args.csv_dir, 'cmd_vel.csv')
    zed = load(args.csv_dir, 'zed_front_zed_node_odom.csv')

    if imu is None or wheel is None or ekf is None:
        sys.exit('missing required CSV(s)')

    if zed is None:
        print('# M2 (Test B) — rotation analysis')
        print()
        print('⚠️  **ZED VIO CSV missing.** Without the third truth '
              'source, M2 cannot resolve disagreements. Verify '
              '`enable_zed_front:=true` was set and `/zed_front/zed_node/odom`'
              ' was recorded.')
        print()

    windows = detect_rotation_windows(cmdvel)
    if not windows:
        sys.exit('no rotation windows detected '
                 '(expected |vx|<0.05 and |wz|>0.3)')

    print('# M2 (Test B) — rotation yaw comparison')
    print(f'detected {len(windows)} rotation window(s)')
    print()

    for idx, (t0, t1, wz_cmd) in enumerate(windows, 1):
        dur, exp, sources = analyze_window(t0, t1, wz_cmd, imu, wheel,
                                           ekf, zed)
        print(f'## Window {idx}: t={t0:.2f}..{t1:.2f}s  (Δt={dur:.2f}s, '
              f'commanded wz={wz_cmd:+.3f} rad/s, expected '
              f'Δyaw={exp:+.1f}°)')
        print()
        print('| Source | Δyaw (deg) | error vs commanded (deg) | error % |')
        print('|---|---:|---:|---:|')
        for name, dy in sources:
            err = dy - exp
            pct = 100 * err / exp if exp != 0 else float('inf')
            print(f'| {name} | {dy:+.2f} | {err:+.2f} | {pct:+.1f}% |')
        print()
        if zed is not None and len(sources) >= 3:
            pair, odd, diff = find_truth(sources)
            agreers = ' + '.join(sources[k][0] for k in pair)
            dissenter = ', '.join(sources[k][0] for k in odd)
            print(f'**Truth (two-of-three):** {agreers} (agree to {diff:.2f}°)')
            print(f'**Dissenter (likely biased):** {dissenter}')
        elif len(sources) == 2:
            d = abs(sources[0][1] - sources[1][1])
            print(f'**Only 2 sources; gap = {d:.2f}°.** Cannot resolve '
                  'without ZED VIO.')
        print()

    # Cross-window: does the IMU error grow with rate?
    if len(windows) >= 2:
        print('## Cross-window: IMU rate-dependent bias check')
        print()
        imu_errs = []
        for t0, t1, wz_cmd in windows:
            _, exp, sources = analyze_window(t0, t1, wz_cmd, imu, wheel,
                                             ekf, zed)
            imu_row = next((s for s in sources if 'IMU (quat' in s[0]), None)
            if imu_row:
                imu_errs.append((wz_cmd, imu_row[1] - exp))
        for wz, err in imu_errs:
            print(f'  wz={wz:+.2f}: IMU error = {err:+.2f}°')
        if len(imu_errs) >= 2:
            # rough slope
            slope = ((imu_errs[-1][1] - imu_errs[0][1]) /
                     max(abs(imu_errs[-1][0] - imu_errs[0][0]), 1e-3))
            print()
            print(f'IMU-error-vs-rate slope: {slope:+.2f} °/(rad/s)')
            if abs(slope) > 10:
                print('**Strong rate dependence** → magnetic interference '
                      'under motor current likely → Fix 4 warranted.')


if __name__ == '__main__':
    main()
