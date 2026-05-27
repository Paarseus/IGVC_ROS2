#!/usr/bin/env python3
"""Bag preflight — verifies extracted CSVs are complete and analyzable.

Run after extract_bag.py, before any analysis agent. Exits non-zero on
any failure so this can gate the pipeline.

Usage:
    python3 scripts/bag_preflight.py <csv_dir>
"""
import argparse
import sys
from pathlib import Path

import pandas as pd


REQUIRED = {
    'imu_data.csv':          {'min_hz': 50, 'critical': True},
    'wheel_odom.csv':        {'min_hz': 10, 'critical': True},
    'odometry_filtered.csv': {'min_hz': 15, 'critical': True},
    'cmd_vel.csv':           {'min_hz': 0,  'critical': True},   # variable rate
    'gnss.csv':              {'min_hz': 1,  'critical': True},
    'avros_wheel_debug.csv': {'min_hz': 25, 'critical': True},
    'tf.csv':                {'min_hz': 5,  'critical': True},
    'tf_static.csv':         {'min_hz': 0,  'critical': True},   # latched
}

OPTIONAL_V2 = {
    'imu_mag.csv':                       {'min_hz': 5,  'critical': False},
    'zed_front_zed_node_odom.csv':       {'min_hz': 5,  'critical': False},
    'zed_front_zed_node_imu_data.csv':   {'min_hz': 50, 'critical': False},
    'odometry_global.csv':               {'min_hz': 10, 'critical': False},
    'odometry_gps.csv':                  {'min_hz': 1,  'critical': False},
}

# Test-window motion signatures expected in cmd_vel.csv
MOTION_PATTERNS = {
    'M1 forward (fwd vx>0.3)':    lambda d: ((d['vx'] > 0.3) & (d['wz'].abs() < 0.1)).sum() > 20,
    'M1 reverse (vx<-0.3)':       lambda d: ((d['vx'] < -0.3) & (d['wz'].abs() < 0.1)).sum() > 20,
    'M2 CCW rotation (wz>0.3)':   lambda d: ((d['vx'].abs() < 0.1) & (d['wz'] > 0.3)).sum() > 20,
    'M2 CW rotation (wz<-0.3)':   lambda d: ((d['vx'].abs() < 0.1) & (d['wz'] < -0.3)).sum() > 20,
    'M3 multi-segment':           lambda d: ((d['vx'].abs() > 0.05) | (d['wz'].abs() > 0.05)).diff().fillna(0).abs().sum() >= 6,  # at least 3 motion bursts (M1 already counts)
}


def check_csv(path: Path, spec: dict) -> tuple:
    if not path.exists():
        return ('MISSING', None)
    df = pd.read_csv(path)
    if len(df) == 0:
        return ('EMPTY', None)
    # Duration + rate
    if 't_rel' in df.columns:
        dur = df['t_rel'].max() - df['t_rel'].min()
        rate = len(df) / dur if dur > 0 else 0
    else:
        dur = 0
        rate = 0
    # Monotonicity
    if 't_rel' in df.columns:
        non_monotonic = (df['t_rel'].diff() < 0).sum()
    else:
        non_monotonic = 0
    # Max gap
    if 't_rel' in df.columns and len(df) > 1:
        max_gap = df['t_rel'].diff().max()
    else:
        max_gap = 0
    return ('OK', {
        'rows': len(df),
        'duration': dur,
        'rate': rate,
        'min_hz': spec['min_hz'],
        'non_monotonic': non_monotonic,
        'max_gap': max_gap,
    })


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_dir', type=Path)
    args = ap.parse_args()

    if not args.csv_dir.is_dir():
        sys.exit(f'not a directory: {args.csv_dir}')

    print(f'# Bag preflight — {args.csv_dir}')
    print()

    failures = 0
    warnings = 0

    print('## Required CSVs')
    print('| file | status | rows | rate (Hz) | min_hz | max_gap (s) |')
    print('|---|---|---:|---:|---:|---:|')
    for name, spec in REQUIRED.items():
        status, info = check_csv(args.csv_dir / name, spec)
        if status != 'OK':
            print(f'| {name} | **{status}** | - | - | {spec["min_hz"]} | - |')
            if spec['critical']:
                failures += 1
            continue
        rate_ok = info['rate'] >= info['min_hz'] * 0.9 if info['min_hz'] > 0 else True
        gap_ok = info['max_gap'] < 0.2
        mono_ok = info['non_monotonic'] == 0
        all_ok = rate_ok and gap_ok and mono_ok
        marker = '✓' if all_ok else '✗'
        print(f'| {name} | {marker} | {info["rows"]} | {info["rate"]:.1f} | '
              f'{info["min_hz"]} | {info["max_gap"]:.3f} |')
        if not all_ok:
            failures += 1
            if not rate_ok:
                print(f'  → rate {info["rate"]:.1f} Hz < {info["min_hz"]} Hz target')
            if not gap_ok:
                print(f'  → max gap {info["max_gap"]:.3f}s > 0.2s')
            if not mono_ok:
                print(f'  → {info["non_monotonic"]} non-monotonic timestamps')

    print()
    print('## Optional v2 CSVs (warn if missing)')
    print('| file | status | rows | rate (Hz) |')
    print('|---|---|---:|---:|')
    for name, spec in OPTIONAL_V2.items():
        status, info = check_csv(args.csv_dir / name, spec)
        if status != 'OK':
            print(f'| {name} | {status} | - | - |')
            warnings += 1
            continue
        print(f'| {name} | ✓ | {info["rows"]} | {info["rate"]:.1f} |')

    print()
    print('## Test-window motion signatures (in cmd_vel.csv)')
    cmdvel_path = args.csv_dir / 'cmd_vel.csv'
    if cmdvel_path.exists():
        cmdvel = pd.read_csv(cmdvel_path)
        for label, fn in MOTION_PATTERNS.items():
            present = fn(cmdvel)
            mark = '✓' if present else '✗ MISSING'
            print(f'- {label}: {mark}')
            if not present:
                warnings += 1
    else:
        print('cmd_vel.csv missing — cannot verify motion patterns')

    print()
    print(f'## Summary: {failures} critical failures, {warnings} warnings')
    if failures > 0:
        print('FAIL — do not run analysis agents.')
        sys.exit(1)
    if warnings > 0:
        print('PASS WITH WARNINGS — analysis can proceed but expect reduced coverage.')
    else:
        print('PASS — all checks green.')


if __name__ == '__main__':
    main()
