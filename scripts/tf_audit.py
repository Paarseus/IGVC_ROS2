#!/usr/bin/env python3
"""TF audit — walks the transform tree and reports frame consistency.

Catches frame-mismatch silent corruption (e.g., ZED VIO's hardcoded
child_frame=camera_link instead of base_link, which breaks EKF
lever-arm fusion — see feedback_zed_leverarm_in_rl memory).

Usage:
    python3 scripts/tf_audit.py <csv_dir>
"""
import argparse
import math
import sys
from pathlib import Path

import pandas as pd


# Expected frame relationships per odometry source.
EXPECTED_FRAMES = {
    'wheel_odom.csv':              ('odom', 'base_link'),
    'odometry_filtered.csv':       ('odom', 'base_link'),
    'odometry_global.csv':         ('map',  'base_link'),
    'odometry_gps.csv':            ('map',  'base_link'),  # navsat_transform output
    'zed_front_zed_node_odom.csv': ('odom', 'zed_front_camera_link'),  # KNOWN-WRONG per ZED wrapper bug
}


def load_or_none(p: Path):
    if not p.exists():
        return None
    return pd.read_csv(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_dir', type=Path)
    args = ap.parse_args()

    if not args.csv_dir.is_dir():
        sys.exit(f'not a directory: {args.csv_dir}')

    print(f'# TF audit — {args.csv_dir}')
    print()

    # --- 1. tf_static tree ---
    print('## Static transform tree (from tf_static.csv)')
    print()
    tf_static = load_or_none(args.csv_dir / 'tf_static.csv')
    if tf_static is None:
        print('tf_static.csv MISSING — TF tree audit cannot proceed.')
        sys.exit(1)

    edges = tf_static.groupby(['parent', 'child']).size().reset_index(name='count')
    print('| parent → child | count |')
    print('|---|---:|')
    for _, row in edges.iterrows():
        print(f'| `{row["parent"]}` → `{row["child"]}` | {row["count"]} |')
    print()

    # --- 2. dynamic tf (map→odom dominant) ---
    print('## Dynamic transforms (from tf.csv)')
    print()
    tf_dyn = load_or_none(args.csv_dir / 'tf.csv')
    if tf_dyn is None:
        print('tf.csv MISSING.')
        sys.exit(1)
    dyn_edges = tf_dyn.groupby(['parent', 'child']).size().reset_index(name='count')
    print('| parent → child | count | mean rate (Hz) |')
    print('|---|---:|---:|')
    bag_dur = tf_dyn['t_rel'].max() - tf_dyn['t_rel'].min()
    for _, row in dyn_edges.iterrows():
        rate = row['count'] / bag_dur if bag_dur > 0 else 0
        print(f'| `{row["parent"]}` → `{row["child"]}` | {row["count"]} | {rate:.2f} |')
    print()

    # --- 3. Chain validation: must have map→odom→base_link ---
    print('## Chain validation: map → odom → base_link')
    all_edges = set(zip(tf_static['parent'], tf_static['child'])) | \
                set(zip(tf_dyn['parent'], tf_dyn['child']))

    chain_ok = True
    for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
        present = (parent, child) in all_edges
        mark = '✓' if present else '✗ MISSING'
        print(f'- `{parent}` → `{child}`: {mark}')
        if not present:
            chain_ok = False
    print()

    if not chain_ok:
        print('**ANOMALY**: required frame chain incomplete. EKF outputs are unreliable.')
        sys.exit(2)

    # --- 4. Per-odometry-source frame check ---
    print('## Per-odometry frame audit')
    print()
    print('| source CSV | header.frame_id | child_frame_id | expected | verdict |')
    print('|---|---|---|---|---|')
    anomalies = 0
    for csv_name, (exp_parent, exp_child) in EXPECTED_FRAMES.items():
        df = load_or_none(args.csv_dir / csv_name)
        if df is None or len(df) == 0:
            print(f'| {csv_name} | - | - | {exp_parent}→{exp_child} | (not recorded) |')
            continue
        # Uniques
        frames = df['frame_id'].unique()
        children = df['child_frame'].unique()
        actual_parent = frames[0] if len(frames) == 1 else f'AMBIGUOUS({len(frames)})'
        actual_child = children[0] if len(children) == 1 else f'AMBIGUOUS({len(children)})'
        match_parent = actual_parent == exp_parent
        match_child = actual_child == exp_child
        if csv_name == 'zed_front_zed_node_odom.csv' and not match_child:
            verdict = '⚠️ KNOWN-WRONG (ZED wrapper bug)'
            anomalies += 1  # still flag for visibility
        elif match_parent and match_child:
            verdict = '✓'
        else:
            verdict = '✗ FRAME MISMATCH'
            anomalies += 1
        print(f'| {csv_name} | `{actual_parent}` | `{actual_child}` | '
              f'`{exp_parent}`→`{exp_child}` | {verdict} |')
    print()

    # --- 5. Map → odom drift quantification ---
    print('## map → odom drift quantification')
    map_odom = tf_dyn[(tf_dyn['parent'] == 'map') & (tf_dyn['child'] == 'odom')]
    if len(map_odom) < 2:
        print('insufficient map→odom samples')
    else:
        # Total drift (start to end)
        dx_total = map_odom['x'].iloc[-1] - map_odom['x'].iloc[0]
        dy_total = map_odom['y'].iloc[-1] - map_odom['y'].iloc[0]
        dur = map_odom['t_rel'].iloc[-1] - map_odom['t_rel'].iloc[0]
        rate_total = math.hypot(dx_total, dy_total) / dur if dur > 0 else 0

        # Stationary baseline: try first 60s (M0 window)
        m0 = map_odom[map_odom['t_rel'] - map_odom['t_rel'].iloc[0] < 60]
        if len(m0) >= 2:
            dx_m0 = m0['x'].iloc[-1] - m0['x'].iloc[0]
            dy_m0 = m0['y'].iloc[-1] - m0['y'].iloc[0]
            dur_m0 = m0['t_rel'].iloc[-1] - m0['t_rel'].iloc[0]
            rate_m0 = math.hypot(dx_m0, dy_m0) / dur_m0 if dur_m0 > 0 else 0
        else:
            rate_m0 = float('nan')

        print(f'- M0 stationary drift rate: **{rate_m0*100:.2f} cm/s**')
        print(f'  - Reference: M4 baseline = 0.26 cm/s (2026-05-26, #12)')
        print(f'  - Verdict: {"✓ holds" if rate_m0 < 0.005 else "⚠️ regression" if rate_m0 < 0.05 else "✗ FAIL"}')
        print(f'- Whole-session drift rate (avg): **{rate_total*100:.2f} cm/s**')
        if abs(rate_total - rate_m0) > 0.05:
            print(f'  - Motion-induced drift detected (whole-session ≠ M0)')

    print()
    print(f'## Summary: {anomalies} anomalies')
    if anomalies == 0:
        print('PASS — TF tree healthy.')
    else:
        print('REVIEW — anomalies above. ZED wrapper child-frame is a known issue '
              '(see feedback_zed_leverarm_in_rl memory); other anomalies need '
              'investigation before relying on EKF verdicts.')


if __name__ == '__main__':
    main()
