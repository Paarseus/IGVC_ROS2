#!/usr/bin/env python3
"""Test A (M1) analysis: IMU vs wheel vs EKF yaw under straight drive.

Identifies which yaw source is biased during a forward drive. Per the
decision thresholds doc, the biased source shows non-zero accumulated
|Δyaw| while the other(s) stay near zero (since the robot is supposedly
going straight).

Usage:
    python3 scripts/analyze_M1.py <csv_dir> [--t-start S] [--t-end S]

If --t-start/--t-end are omitted, auto-detects the longest contiguous
window where |/cmd_vel.angular.z| < 0.1 AND /cmd_vel.linear.x > 0.1.

Outputs a markdown comparison table + verdict line to stdout.
"""
import argparse
import math
import sys
from pathlib import Path

import pandas as pd


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def unwrap_series(s):
    return pd.Series(s).interpolate().to_numpy()


def integrated_delta_yaw(times, yaws):
    """Δyaw from start to end, wrap-safe."""
    if len(times) < 2:
        return 0.0
    # Use atan2 on cumulative quaternion-like integration
    dy = 0.0
    prev = yaws[0]
    for y in yaws[1:]:
        d = wrap_pi(y - prev)
        dy += d
        prev = y
    return dy


def integrated_vyaw(times, vyaws):
    """∫ vyaw dt using trapezoidal rule. Returns radians."""
    if len(times) < 2:
        return 0.0
    return float(((vyaws[:-1] + vyaws[1:]) * 0.5 *
                  (times[1:] - times[:-1])).sum())


def load(csv_dir: Path, name: str):
    p = csv_dir / name
    if not p.exists():
        return None
    return pd.read_csv(p)


def detect_window(cmd_vel: pd.DataFrame):
    """Find the longest window where vx > 0.1 and |wz| < 0.1."""
    if cmd_vel is None or len(cmd_vel) == 0:
        return None, None
    df = cmd_vel.copy()
    df['drive'] = (df['vx'] > 0.1) & (df['wz'].abs() < 0.1)
    # find longest contiguous True run
    groups = (df['drive'] != df['drive'].shift()).cumsum()
    runs = df[df['drive']].groupby(groups)
    if len(runs) == 0:
        return None, None
    best = max(runs, key=lambda kv: len(kv[1]))
    t_start = best[1]['t_rel'].iloc[0]
    t_end = best[1]['t_rel'].iloc[-1]
    return t_start, t_end


def clip(df, t0, t1):
    return df[(df['t_rel'] >= t0) & (df['t_rel'] <= t1)].reset_index(drop=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_dir', type=Path)
    ap.add_argument('--t-start', type=float, default=None)
    ap.add_argument('--t-end', type=float, default=None)
    args = ap.parse_args()

    imu = load(args.csv_dir, 'imu_data.csv')
    wheel = load(args.csv_dir, 'wheel_odom.csv')
    ekf = load(args.csv_dir, 'odometry_filtered.csv')
    cmdvel = load(args.csv_dir, 'cmd_vel.csv')
    wdebug = load(args.csv_dir, 'avros_wheel_debug.csv')

    if imu is None or wheel is None or ekf is None:
        sys.exit('missing required CSV (imu_data, wheel_odom, or '
                 'odometry_filtered) — re-run extract_bag.py')

    t0, t1 = args.t_start, args.t_end
    if t0 is None or t1 is None:
        t0_auto, t1_auto = detect_window(cmdvel)
        if t0_auto is None:
            sys.exit('could not auto-detect M1 window; pass --t-start/--t-end')
        t0 = t0 if t0 is not None else t0_auto
        t1 = t1 if t1 is not None else t1_auto

    print(f'# M1 (Test A) — straight-drive yaw comparison')
    print(f'window: t={t0:.2f}..{t1:.2f} s  ({t1-t0:.2f} s)')
    print()

    # Clip each source
    imu_c = clip(imu, t0, t1)
    wheel_c = clip(wheel, t0, t1)
    ekf_c = clip(ekf, t0, t1)
    wdebug_c = clip(wdebug, t0, t1) if wdebug is not None else None

    # Compute distance traveled (from EKF)
    if len(ekf_c) >= 2:
        dx = ekf_c['x'].iloc[-1] - ekf_c['x'].iloc[0]
        dy = ekf_c['y'].iloc[-1] - ekf_c['y'].iloc[0]
        dist = math.hypot(dx, dy)
    else:
        dist = 0.0

    rows = []

    # IMU quaternion-derived Δyaw (drift-free, from Xsens internal filter)
    if len(imu_c) >= 2:
        dy = integrated_delta_yaw(imu_c['t_rel'].to_numpy(),
                                  imu_c['yaw'].to_numpy())
        rows.append(('IMU (quaternion)', math.degrees(dy),
                     imu_c['wz'].mean(), imu_c['wz'].std()))

    # IMU gyro-integrated
    if len(imu_c) >= 2:
        dy = integrated_vyaw(imu_c['t_rel'].to_numpy(),
                             imu_c['wz'].to_numpy())
        rows.append(('IMU (gyro integrated)', math.degrees(dy),
                     imu_c['wz'].mean(), imu_c['wz'].std()))

    # Wheel odom — Δyaw from integrated vyaw (orientation in /wheel_odom may
    # not be set; integrating wz is the canonical signal)
    if len(wheel_c) >= 2:
        dy = integrated_vyaw(wheel_c['t_rel'].to_numpy(),
                             wheel_c['wz'].to_numpy())
        rows.append(('Wheel odom (∫vyaw)', math.degrees(dy),
                     wheel_c['wz'].mean(), wheel_c['wz'].std()))

    # EKF
    if len(ekf_c) >= 2:
        dy = integrated_delta_yaw(ekf_c['t_rel'].to_numpy(),
                                  ekf_c['yaw'].to_numpy())
        rows.append(('EKF /odometry/filtered', math.degrees(dy),
                     ekf_c['wz'].mean(), ekf_c['wz'].std()))

    # Wheel debug yaw (mirrors EKF/IMU heading-hold input — redundant signal)
    if wdebug_c is not None and len(wdebug_c) >= 2:
        dy = wrap_pi(wdebug_c['yaw'].iloc[-1] - wdebug_c['yaw'].iloc[0])
        rows.append(('wheel_debug.yaw (heading-hold input)', math.degrees(dy),
                     wdebug_c['yaw_rate'].mean(), wdebug_c['yaw_rate'].std()))

    print('## Comparison')
    print()
    print(f'distance traveled (EKF xy): **{dist:.3f} m**')
    print()
    print('| Source | Δyaw (deg) | mean vyaw (rad/s) | σ vyaw (rad/s) |')
    print('|---|---:|---:|---:|')
    for name, dyaw, mwz, swz in rows:
        print(f'| {name} | {dyaw:+.3f} | {mwz:+.4f} | {swz:.4f} |')
    print()

    # Verdict
    imu_quat = next((r for r in rows if r[0] == 'IMU (quaternion)'), None)
    wheel_row = next((r for r in rows if 'Wheel odom' in r[0]), None)
    if imu_quat and wheel_row:
        imu_abs = abs(imu_quat[1])
        wheel_abs = abs(wheel_row[1])
        sign_imu = 1 if imu_quat[1] > 0 else -1
        sign_wheel = 1 if wheel_row[1] > 0 else -1

        print('## Verdict')
        print()
        if imu_abs < 1.0 and wheel_abs < 1.0:
            print('**BOTH CLEAN** — chassis really drove straight. '
                  '14.7° from original test may have been transient or '
                  'already fixed by commit 33d62ea. No fix needed.')
        elif imu_abs < 1.0 and wheel_abs > 3.0:
            print('**WHEELS BIASED** (likely grass slip / differential '
                  f'friction). |wheel Δyaw|={wheel_abs:.2f}°. '
                  '→ Apply **Fix 1** first.')
        elif imu_abs > 3.0 and wheel_abs < 1.0:
            print('**IMU BIASED UNDER MOTION** (likely mag interference). '
                  f'|IMU Δyaw|={imu_abs:.2f}°. → Apply **Fix 4** '
                  '(NoBaro profile), NOT Fix 1/2.')
        elif imu_abs > 3.0 and wheel_abs > 3.0 and sign_imu == sign_wheel:
            print('**BOTH BIASED SAME DIRECTION** — chassis actually '
                  'curved. No yaw fix; investigate Mandow α on grass.')
        elif imu_abs > 3.0 and wheel_abs > 3.0 and sign_imu != sign_wheel:
            print('**BOTH BIASED OPPOSITE DIRECTIONS** — worst case. '
                  'Hold. Use M2 ZED VIO as tiebreaker.')
        else:
            print(f'**MARGINAL** — IMU={imu_abs:.2f}°, wheel={wheel_abs:.2f}°. '
                  'Bag a longer drive (10–15 m) to amplify signal.')


if __name__ == '__main__':
    main()
