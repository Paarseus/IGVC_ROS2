#!/usr/bin/env python3
"""Reversible test configuration for the obstacle-avoidance / waypoint tests (obstacle_waypoint_test_plan_2026_10_02.md).

  apply_test_config.py apply  --actuator PATH --nav2 PATH [--gains tuned|keep]
  apply_test_config.py revert --actuator PATH --nav2 PATH
  apply_test_config.py status --actuator PATH --nav2 PATH

Edits only specific keys, keeps the trailing comments, and stores a one-time backup next to each file
(<file>.navtest.orig). 'revert' restores the backups. Nothing is committed to git by this script.

actuator_params.yaml: heading_hold_deadband 0.0 (heading-hold off), max_linear_mps 0.4 (backstop),
                      and with --gains tuned: kFF 0.00211, kP 0.0004, kS_left 0.40, kS_right 0.39 (TUNING_LOG.md E1-E3a)
nav2 params:          vx_max 0.35, wz_max 1.5 (actuator cap), odom_topic /odometry/filtered under controller_server
"""
import argparse, os, re, shutil, sys

ACT = [('heading_hold_deadband', '0.0'), ('max_linear_mps', '0.4')]
ACT_TUNED = [('kFF', '0.00211'), ('kP', '0.0004'), ('kS_left', '0.40'), ('kS_right', '0.39')]
NAV = [('vx_max', '0.35'), ('wz_max', '1.5')]
BAK = '.navtest.orig'


def set_key(lines, key, value):
    pat = re.compile(r'^(\s*' + re.escape(key) + r':\s*)([-+0-9.eE]+)(.*)$')
    hits = [i for i, l in enumerate(lines) if pat.match(l)]
    if len(hits) != 1:
        sys.exit(f'{key}: expected exactly one match, found {len(hits)}')
    m = pat.match(lines[hits[0]])
    old = m.group(2)
    lines[hits[0]] = f'{m.group(1)}{value}{m.group(3)}\n' if lines[hits[0]].endswith('\n') else f'{m.group(1)}{value}{m.group(3)}'
    return old


def add_odom_topic(lines):
    start = next((i for i, l in enumerate(lines) if re.match(r'^controller_server:\s*$', l)), None)
    if start is None:
        sys.exit('controller_server: block not found')
    end = next((i for i in range(start + 1, len(lines)) if re.match(r'^\S', lines[i])), len(lines))
    block = lines[start:end]
    if any(re.match(r'^\s+odom_topic:', l) for l in block):
        return 'already set'
    fi = next((i for i in range(start, end) if re.match(r'^\s+controller_frequency:', lines[i])), None)
    if fi is None:
        sys.exit('controller_frequency not found in controller_server')
    lines.insert(fi + 1, '    odom_topic: /odometry/filtered   # navtest 2026-10-02: MPPI default is "odom" (nothing publishes it)\n')
    return 'added'


def backup(path):
    if not os.path.exists(path + BAK):
        shutil.copy2(path, path + BAK)


def get_vals(path, keys):
    text = open(path).read().splitlines()
    out = {}
    for k in keys:
        m = [re.match(r'^\s*' + re.escape(k) + r':\s*([-+0-9.eE]+)', l) for l in text]
        m = [x for x in m if x]
        out[k] = m[0].group(1) if len(m) == 1 else f'({len(m)} matches)'
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('action', choices=['apply', 'revert', 'status'])
    ap.add_argument('--actuator', required=True)
    ap.add_argument('--nav2', required=True)
    ap.add_argument('--gains', choices=['tuned', 'keep'], default='tuned')
    a = ap.parse_args()
    if a.action == 'status':
        for path, keys in ((a.actuator, [k for k, _ in ACT + ACT_TUNED]), (a.nav2, [k for k, _ in NAV])):
            print(os.path.basename(path), 'backup' if os.path.exists(path + BAK) else 'no backup', get_vals(path, keys))
        lines = open(a.nav2).read().splitlines()
        st = next((i for i, l in enumerate(lines) if re.match(r'^controller_server:\s*$', l)), None)
        en = next((i for i in range(st + 1, len(lines)) if re.match(r'^\S', lines[i])), len(lines)) if st is not None else 0
        print('controller_server odom_topic:', 'present' if st is not None and any(re.match(r'^\s+odom_topic:', l) for l in lines[st:en]) else 'missing')
        return
    if a.action == 'revert':
        for p in (a.actuator, a.nav2):
            if os.path.exists(p + BAK):
                shutil.move(p + BAK, p)
                print('restored', p)
            else:
                print('no backup for', p)
        return
    for p in (a.actuator, a.nav2):
        backup(p)
    lines = open(a.actuator).readlines()
    for k, v in ACT + (ACT_TUNED if a.gains == 'tuned' else []):
        print(f'{os.path.basename(a.actuator)} {k}: {set_key(lines, k, v)} -> {v}')
    open(a.actuator, 'w').writelines(lines)
    lines = open(a.nav2).readlines()
    for k, v in NAV:
        print(f'{os.path.basename(a.nav2)} {k}: {set_key(lines, k, v)} -> {v}')
    print(f'{os.path.basename(a.nav2)} controller_server odom_topic: {add_odom_topic(lines)}')
    open(a.nav2, 'w').writelines(lines)


if __name__ == '__main__':
    main()
