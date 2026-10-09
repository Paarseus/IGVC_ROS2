#!/usr/bin/env python3
"""Estimate SPARK MAX reported-velocity lag from logged wheel telemetry.

Input: an avros_wheel_debug.csv (columns t_rel, L/R_cmd_rpm, L/R_meas_rpm,
L/R_pos_rev, ...). Reported velocity (meas_rpm) comes from the SPARK MAX
velocity measurement; position (pos_rev) comes from the same encoder but is
not averaged. Differentiating position gives an unfiltered velocity reference.
The lag that best aligns reported velocity with the position-derived velocity
estimates the velocity-measurement delay. The same method applied to the
commanded RPM gives the command-to-reported-velocity delay.

Usage: velocity_lag.py <avros_wheel_debug.csv> [<more.csv> ...]
"""
import csv
import sys

import numpy as np


def load(path):
    with open(path) as f:
        r = list(csv.DictReader(f))
    t = np.array([float(x['t_rel']) for x in r])
    out = {'t': t}
    for k in ('L_cmd_rpm', 'R_cmd_rpm', 'L_meas_rpm', 'R_meas_rpm', 'L_pos_rev', 'R_pos_rev'):
        out[k] = np.array([float(x[k]) for x in r])
    return out


def resample(t, y, dt):
    tg = np.arange(t[0], t[-1], dt)
    return tg, np.interp(tg, t, y)


def best_lag(ref, sig, dt, max_lag_s=0.6):
    """Lag (s) by which sig trails ref, by maximizing normalized correlation."""
    ref = ref - ref.mean()
    sig = sig - sig.mean()
    best = (0.0, -1.0)
    for k in range(0, int(max_lag_s / dt) + 1):
        a, b = ref[:len(ref) - k], sig[k:]
        c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
        if c > best[1]:
            best = (k * dt, c)
    return best


def main():
    dt = 0.005
    for path in sys.argv[1:]:
        d = load(path)
        print(f'\n{path}  ({d["t"][-1]:.1f} s, {len(d["t"])} rows)')
        for side in ('L', 'R'):
            pos, meas, cmd = d[f'{side}_pos_rev'], d[f'{side}_meas_rpm'], d[f'{side}_cmd_rpm']
            # Use only rows where the position value changed (new E-line), so the
            # derivative is not polluted by repeated samples.
            keep = np.concatenate(([True], np.diff(pos) != 0))
            tp, pp = d['t'][keep], pos[keep]
            if len(tp) < 50:
                print(f'  {side}: not enough position updates')
                continue
            tg = np.arange(tp[0], tp[-1], dt)               # common grid for all signals
            pg = np.interp(tg, tp, pp)
            v_pos = np.gradient(pg, dt) * 60.0              # RPM from position
            v_pos = np.convolve(v_pos, np.ones(4) / 4, 'same')  # 20 ms box to match E-line spacing
            v_meas = np.interp(tg, d['t'], meas)
            v_cmd = np.interp(tg, d['t'], cmd)
            lag_meas, c1 = best_lag(v_pos, v_meas, dt)
            lag_cmd, c2 = best_lag(v_cmd, v_pos, dt, 1.0)
            moving = np.abs(v_pos) > 100
            gain = float(np.median(v_meas[moving] / v_pos[moving])) if moving.sum() > 50 else float('nan')
            print(f'  {side}: reported-velocity lag behind position-derived velocity = {lag_meas * 1e3:.0f} ms (corr {c1:.3f}); '
                  f'command -> actual (position) delay = {lag_cmd * 1e3:.0f} ms (corr {c2:.3f}); '
                  f'median reported/position-derived velocity = {gain:.3f}')


if __name__ == '__main__':
    main()
