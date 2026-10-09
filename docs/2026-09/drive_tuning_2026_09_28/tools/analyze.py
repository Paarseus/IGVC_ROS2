#!/usr/bin/env python3
"""Analysis for drive_tuner.py runs (docs/drive_tuning_2026_09_28/STRATEGY.md).

  analyze.py ff       RUN_DIR [RUN_DIR ...]   feedforward fit per side and direction: V = kS*sgn(v) + kV*v + kA*a   (step B)
  analyze.py lag      RUN_DIR [...]           speed-reading lag: reported STATUS_2 velocity vs d(position)/dt  (step B.5)
  analyze.py steps    RUN_DIR [...]           closed-loop step metrics per segment: 95% time, overshoot, steady error, L/R mismatch (steps C, D)
  analyze.py distance RUN_DIR [...]           m per motor revolution from RTK (/gnss) vs encoder revolutions (step E)
  analyze.py turn     RUN_DIR [...]           effective track width / multiplier from gyro vs track speeds (step F)
  analyze.py circle   RUN_DIR [...]           spin centre (spins) and driven radius (arcs) from a circle fit to /gnss
  analyze.py gyroscale RUN_DIR [...]          gyro scale error vs the RTK antenna angle (or --turns N counted)
  analyze.py still    RUN_DIR [...]           stationary: gyro bias/noise, RTK FIXED share, GNSS scatter (0.2, 0.4)
  analyze.py tape     RUN_DIR [...] --along A,.. --side S,..   m per motor rev from tape-measured start/end marks (4.1)
  analyze.py delivery RUN_DIR [...]           steady speed vs command over many repeats: error, 95 % band, ripple, stuck, L/R (3.1)
  analyze.py ffid     RUN_DIR [...] --kv-set .. --ks-l .. --ks-r ..   closed-form kV/kS identification from FF-only runs
  analyze.py guard    RUN_DIR                 safety check of one run (faults, bus V, current, ripple)
  analyze.py stops    RUN_DIR [...]           braked stops ("S:secs" segments): time, distance, rollback, bus V, current (1.3, 5.1)

Velocity is always computed from encoder POSITION (the STATUS_2 velocity lags ~112 ms by default).
Units: RPM and rotations are motor-side; m/s uses --m-per-rev.

The firmware prints one X line whenever EITHER side's STATUS_2 arrives, so each side's
s2_rx_us/pos repeat on the rows triggered by the other side. All per-side maths runs on
that side's unique STATUS_2 samples (REVIEW 2026-09-28: np.gradient on the repeated
timestamps returned NaN for every sample and the ff fit printed nothing).
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

TRACK_W = 0.7366
TEST_PREFIXES = ('ramp', 'step')     # drive_tuner.py labels of the open-loop test segments


def load_teensy(d):
    with open(os.path.join(d, 'teensy.csv')) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f'{d}: no telemetry rows')
    num = {}
    for k in rows[0]:
        if k in ('mode', 'cmd'):
            num[k] = np.array([r[k] for r in rows])
        else:
            num[k] = np.array([float(r[k]) for r in rows])
    return num


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')          # e.g. empty GGA fields when there is no fix


def load_csv(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    return {k: np.array([_f(r[k]) for r in rows]) for k in rows[0]}


def uniq(t_us):
    """Indices of the first row of each new STATUS_2 sample, and the row -> sample map."""
    _, first, inv = np.unique(t_us, return_index=True, return_inverse=True)
    return first, inv


def central_velocity(t, pos_rot, win):
    """Central-difference velocity (RPM) over +/- win/2 s on strictly increasing sample times t (s)."""
    v = np.full(len(t), np.nan)
    j0 = j1 = 0
    for i in range(len(t)):
        while t[i] - t[j0] > win / 2:
            j0 += 1
        while j1 + 1 < len(t) and t[j1 + 1] - t[i] <= win / 2:
            j1 += 1
        if t[j1] - t[j0] > 1e-3:
            v[i] = (pos_rot[j1] - pos_rot[j0]) / (t[j1] - t[j0]) * 60.0
    return v


def pos_velocity(t_us, pos_rot, win=0.06):
    """Per-row time (s, STATUS_2 arrival) and position-derived velocity (RPM), computed on unique samples."""
    first, inv = uniq(t_us)
    tu = (t_us[first] - t_us[first][0]) * 1e-6
    vu = central_velocity(tu, pos_rot[first], win)
    return tu[inv], vu[inv]


def side(num, s, win=0.06):
    t, v = pos_velocity(num[f'{s}_s2_rx_us'], num[f'{s}_pos_rot'], win)
    volts = num[f'{s}_applied'] * num[f'{s}_bus_V']
    return t, v, volts


# ------------------------------------------------------------------ ff
def _simulate(u, v0, dt, kS, kV, kA):
    """Velocity predicted by the fitted model from v0, exact first-order step per sample (SysId sim-velocity check)."""
    out = np.empty(len(u))
    vh = v0
    for i in range(len(u)):
        out[i] = vh
        if abs(vh) < 1e-6 and abs(u[i]) <= kS:
            vh = 0.0
            continue
        sg = np.sign(vh) if abs(vh) > 1e-6 else np.sign(u[i])
        vinf = (u[i] - kS * sg) / kV
        vn = vinf + (vh - vinf) * math.exp(-dt[i] * kV / kA) if kA > 0 else vinf
        if abs(vh) > 1e-6 and np.sign(vn) != np.sign(vh):
            vn = 0.0                                  # friction cannot reverse the motion
        vh = vn
    return out


def cmd_ff(args):
    for s in ('L', 'R'):
        segs = []     # per run and contiguous test segment: (t, v, a, u)
        for d in args.runs:
            num = load_teensy(d)
            first, _ = uniq(num[f'{s}_s2_rx_us'])
            t = (num[f'{s}_s2_rx_us'][first] - num[f'{s}_s2_rx_us'][first][0]) * 1e-6
            v = central_velocity(t, num[f'{s}_pos_rot'][first], args.win)
            a = np.gradient(v, t)                     # RPM/s, on unique, strictly increasing times
            u = (num[f'{s}_applied'] * num[f'{s}_bus_V'])[first]
            lab = num['cmd'][first]
            test = np.array([l.startswith(TEST_PREFIXES) for l in lab]) if not args.all_segments \
                else np.ones(len(lab), bool)
            # split into contiguous runs of "test" rows (ramp is one block, each step its own block)
            idx = np.flatnonzero(test)
            if not len(idx):
                continue
            breaks = np.flatnonzero(np.diff(idx) > 1) + 1
            for blk in np.split(idx, breaks):
                segs.append((t[blk], v[blk], a[blk], u[blk]))
        if not segs:
            print(f'{s}: no test segments (labels starting with {TEST_PREFIXES}); use --all-segments')
            continue
        v = np.concatenate([g[1] for g in segs]); a = np.concatenate([g[2] for g in segs])
        u = np.concatenate([g[3] for g in segs])
        ok = np.isfinite(v) & np.isfinite(a) & (np.abs(v) > args.vmin)
        for label, sel in (('fwd', ok & (v > 0)), ('rev', ok & (v < 0)), ('both', ok)):
            if sel.sum() < 30:
                continue
            # SysId form (C1-S20 FeedforwardAnalysis.cpp): a = alpha*v + beta*V + gamma*sgn(v), OLS on
            # ACCELERATION. Regressing V on a noisy, twice-differentiated a biases kA low and kS high.
            X = np.column_stack([v[sel], u[sel], np.sign(v[sel])])
            (al, be, ga), *_ = np.linalg.lstsq(X, a[sel], rcond=None)
            if be <= 0:
                # Fallback (C1 practice 6: steady-state friction identification): with too little
                # acceleration content (ramp-only data), fit V = kS*sgn(v) + kV*v on near-steady samples.
                slow = sel & (np.abs(a) < max(np.nanpercentile(np.abs(a[sel]), 50), 1.0))
                Xq = np.column_stack([np.sign(v[slow]), v[slow]])
                (kSq, kVq), *_ = np.linalg.lstsq(Xq, u[slow], rcond=None)
                rq = u[slow] - Xq @ np.array([kSq, kVq])
                r2q = 1 - np.sum(rq ** 2) / np.sum((u[slow] - u[slow].mean()) ** 2)
                print(f'{s} {label:4s}: QUASISTATIC ONLY (not enough acceleration for kA; add the dynamic steps): '
                      f'kS={kSq:.4f} V  kV={kVq:.6f} V/RPM  voltage r2={r2q:.3f}  n={slow.sum()}  '
                      f'(kS reads high by about ramp_rate x motor time constant)')
                continue
            kS, kV, kA = -ga / be, -al / be, 1.0 / be
            ra = a[sel] - X @ np.array([al, be, ga])
            r2a = 1 - np.sum(ra ** 2) / np.sum((a[sel] - a[sel].mean()) ** 2)
            # simulated-velocity r^2 over the same segments (SysId "sim velocity r^2 > 0.9")
            meas, pred = [], []
            for (tt, vv, _, uu) in segs:
                good = np.isfinite(vv)
                if good.sum() < 5:
                    continue
                tt, vv, uu = tt[good], vv[good], uu[good]
                dt = np.r_[np.diff(tt), 0.0]
                ph = _simulate(uu, vv[0], dt, kS, kV, kA)
                m = (np.abs(vv) > args.vmin) & ((vv > 0) if label == 'fwd' else (vv < 0) if label == 'rev' else True)
                meas.append(vv[m]); pred.append(ph[m])
            meas, pred = np.concatenate(meas), np.concatenate(pred)
            r2v = 1 - np.sum((meas - pred) ** 2) / np.sum((meas - meas.mean()) ** 2)
            rmsv = math.sqrt(np.mean((meas - pred) ** 2))
            print(f'{s} {label:4s}: kS={kS:.4f} V  kV={kV:.6f} V/RPM  kA={kA:.7f} V/(RPM/s)  '
                  f'sim-vel r2={r2v:.3f} (rms {rmsv:.0f} RPM)  accel r2={r2a:.3f}  n={sel.sum()}')
        print('   (SysId checks: sim-vel r2 > 0.9; accel r2 > ~0.2 for a usable kA; kV near 12/5676 = 0.00211 V/RPM\n'
              '    for a free NEO. Fit on 2 of 3 repeats and check sim-vel r2 on the third (C1 practice 9).)')


# ------------------------------------------------------------------ lag
def cmd_lag(args):
    for d in args.runs:
        num = load_teensy(d)
        for s in ('L', 'R'):
            first, _ = uniq(num[f'{s}_s2_rx_us'])
            t = (num[f'{s}_s2_rx_us'][first] - num[f'{s}_s2_rx_us'][first][0]) * 1e-6
            vpos = central_velocity(t, num[f"{s}_pos_rot"][first], 0.06)   # +/-1 sample (20 ms STATUS_2): zero-phase
            vrep = num[f'{s}_vel_rpm'][first]
            ok = np.isfinite(vpos)
            t, vpos, vrep = t[ok], vpos[ok], vrep[ok]
            if len(t) < 50:
                continue
            tu = np.arange(t[0], t[-1], 0.002)
            a = np.interp(tu, t, vpos)
            b = np.interp(tu, t, vrep)
            # the delay k that best maps position-derived velocity onto the reported one: b(t) ~ a(t - k)
            lags = np.arange(0, int(0.4 / 0.002))
            err = [np.mean((a[:len(a) - k] - b[k:]) ** 2) for k in lags]
            k = int(np.argmin(err))
            print(f'{os.path.basename(d)} {s}: reported velocity lags position-derived velocity by {k * 2} ms '
                  f'(default filter predicts ~112 ms; needs a run with speed changes)')


# ------------------------------------------------------------------ steps
def segments(num):
    cmd = num['cmd']
    out, start = [], 0
    for i in range(1, len(cmd) + 1):
        if i == len(cmd) or cmd[i] != cmd[start]:
            out.append((start, i, cmd[start]))
            start = i
    return out


def cmd_steps(args):
    print(f'run | segment | side | cmd RPM | steady RPM | err % | t95 s | overshoot %   (velocity window {args.win} s)')
    for d in args.runs:
        num = load_teensy(d)
        steady_lr = {}
        for s in ('L', 'R'):
            t, v, _ = side(num, s, args.win)
            sp = num[f'sp_{s}']
            for i0, i1, lab in segments(num):
                if i1 - i0 < 20 or lab in ('S', ''):
                    continue
                target = np.median(sp[i1 - 10:i1])
                if abs(target) < 1:
                    continue
                tt, vv = t[i0:i1] - t[i0], v[i0:i1]
                steady = np.nanmean(vv[tt > tt[-1] - min(1.0, tt[-1] / 3)])
                v0 = np.nanmean(v[max(0, i0 - 5):i0 + 1]) if i0 > 0 else 0.0
                span = target - v0
                t95 = next((x for x, y in zip(tt, vv) if abs(y - v0) >= 0.95 * abs(span)), float('nan'))
                peak = np.nanmax(vv) if span > 0 else np.nanmin(vv)
                over = 100 * (peak - target) / abs(span) * (1 if span > 0 else -1) if abs(span) > 1 else float('nan')
                steady_lr.setdefault(lab, {})[s] = steady
                print(f'{os.path.basename(d)} | {lab} | {s} | {target:7.0f} | {steady:7.0f} | '
                      f'{100 * (steady - target) / target:6.2f} | {t95:5.2f} | {over:6.1f}')
        for lab, lr in steady_lr.items():
            if 'L' in lr and 'R' in lr and lr['L'] != 0:
                print(f'{os.path.basename(d)} | {lab} | L/R steady mismatch {100 * (lr["R"] / lr["L"] - 1):+.2f} % (T5)')
    print('   (t95 is from the host command change, so it includes serial + Teensy slew + CAN delay.\n'
          '    Overshoot noise floor: hall position is 1/42 rev, so a +/-win/2 difference has ~+/-(1/42)/win*60 RPM\n'
          '    quantisation; at win 0.1 s that is ~14 RPM, i.e. ~1 % of a 1200 RPM step. Do not use win < 0.1.)')


# ------------------------------------------------------------------ distance
WGS_A, WGS_E2 = 6378137.0, 6.69437999014e-3


def enu(lat, lon, lat0, lon0):
    """Local tangent-plane E/N (m) with WGS-84 meridional (M) and prime-vertical (N) radii.
    (A single 6378137 m radius overstates north distances by ~0.2 % at 34-43 deg latitude.)"""
    s2 = math.sin(math.radians(lat0)) ** 2
    Rn = WGS_A / math.sqrt(1 - WGS_E2 * s2)                     # prime vertical
    Rm = WGS_A * (1 - WGS_E2) / (1 - WGS_E2 * s2) ** 1.5        # meridional
    x = np.radians(lon - lon0) * Rn * math.cos(math.radians(lat0))
    y = np.radians(lat - lat0) * Rm
    return x, y


def gyro_bias(imu, num, pre=0.3):
    """Mean gz while the setpoints are still zero at the start of the run (robot stationary)."""
    moving = np.abs(num['sp_L']) + np.abs(num['sp_R']) > 1
    if not moving.any():
        return float('nan'), 0
    t_move = num['host_t'][moving][0] - pre
    sel = (imu['host_t'] < t_move) & (imu['host_t'] > num['host_t'][0])
    return (float(np.mean(imu['gz'][sel])) if sel.sum() >= 50 else float('nan')), int(sel.sum())


def cmd_distance(args):
    print('run | RTK dist m | L rev | R rev | m/rev (mean) | L/R rev ratio | dpsi deg (gyro) | sagitta m (fit) | '
          'end offset m vs start heading | fixed %')
    for d in args.runs:
        num = load_teensy(d)
        g = load_csv(os.path.join(d, 'gnss.csv'))
        if g is None:
            print(f'{d}: no gnss.csv (run with --ros)'); continue
        moving = np.abs(num['sp_L']) + np.abs(num['sp_R']) > 1
        ht = num['host_t'][moving]
        t0, t1 = ht[0] + args.trim, ht[-1] - args.trim
        sel = (g['host_t'] >= t0) & (g['host_t'] <= t1)
        if sel.sum() < 5:
            print(f'{d}: fewer than 5 GNSS fixes in the trimmed window'); continue
        q = load_csv(os.path.join(d, 'gga.csv'))
        if q is not None:
            qs = (q['host_t'] >= t0) & (q['host_t'] <= t1)
            fixed = 100 * np.mean(q['quality'][qs] == 4) if qs.any() else 0
        else:
            fixed = float('nan')
        gt = g['host_t'][sel]
        x, y = enu(g['lat'][sel], g['lon'][sel], g['lat'][sel][0], g['lon'][sel][0])
        dist = math.hypot(x[-1] - x[0], y[-1] - y[0])
        cx, cy = (x[-1] - x[0]) / dist, (y[-1] - y[0]) / dist
        # straightness: least-squares parabola of lateral offset vs along-track distance in the chord frame.
        # (max |offset| over ~40-60 RTK fixes is biased up by ~2.5 sigma of the fix noise, ~2-3 cm vs K2 5 cm.)
        s_al = (x - x[0]) * cx + (y - y[0]) * cy
        l_lat = (x - x[0]) * cy - (y - y[0]) * cx          # + = right of the chord
        c2 = np.polyfit(s_al, l_lat, 2)[0]
        sag = abs(c2) * dist ** 2 / 4
        # encoder revolutions over exactly the same interval as the first and last GNSS fixes
        pl = np.interp([gt[0], gt[-1]], num['host_t'], num['L_pos_rot'])
        pr = np.interp([gt[0], gt[-1]], num['host_t'], num['R_pos_rot'])
        rl, rr = abs(pl[1] - pl[0]), abs(pr[1] - pr[0])
        # heading change over the same interval from the bias-corrected gyro; for a constant-curvature leg
        # the lateral offset at the end from the START heading line is L*dpsi/2 (= 4 x sagitta).
        dpsi = end_off = float('nan')
        imu = load_csv(os.path.join(d, 'imu.csv'))
        if imu is not None and len(imu['host_t']) > 10:
            b, _ = gyro_bias(imu, num)
            b = 0.0 if not np.isfinite(b) else b
            psi_rel = np.concatenate([[0.0], np.cumsum(np.diff(imu['host_t']) * (imu['gz'][1:] - b))])
            dpsi = float(np.diff(np.interp([gt[0], gt[-1]], imu['host_t'], psi_rel))[0])
            end_off = dist * dpsi / 2
        print(f'{os.path.basename(d)} | {dist:.3f} | {rl:.2f} | {rr:.2f} | {dist / ((rl + rr) / 2):.5f} | '
              f'{rl / rr:.4f} | {math.degrees(dpsi):+.2f} | {sag:.3f} | {end_off:+.3f} | {fixed:.0f}')
    print('   (fixed % = share of GGA quality 4 (RTK FIXED) during the run; use only runs at 100 %.\n'
          '    Distance is the chord between the first and last fix, so RTK noise does not inflate it. /gnss is the\n'
          '    ANTENNA (0.74 m ahead of base_link); on a straight leg it translates with base_link, the chord error is\n'
          '    ~lever^2/(2R^2) and negligible. dpsi at equal RPM, forward vs reverse, separates the left/right scale\n'
          '    ratio (sign flips with direction, UMBmark type B) from a heading/gyro bias (same sign).)')


# ------------------------------------------------------------------ turn
def cmd_turn(args):
    print('run | segment | v_L m/s | v_R m/s | gyro rad/s | cmd w | eff width m | multiplier | radius m')
    for d in args.runs:
        num = load_teensy(d)
        imu = load_csv(os.path.join(d, 'imu.csv'))
        if imu is None:
            print(f'{d}: no imu.csv (run with --ros)'); continue
        bias, nb = gyro_bias(imu, num)
        if np.isfinite(bias):
            print(f'{os.path.basename(d)}: gyro z bias {bias:+.5f} rad/s ({math.degrees(bias):+.3f} deg/s) '
                  f'from {nb} stationary samples; subtracted'
                  + ('   !! > 0.1 deg/s: Xsens bias suspect (CLAUDE.md known issue), fix before trusting K3'
                     if abs(math.degrees(bias)) > 0.1 else ''))
        else:
            print(f'{os.path.basename(d)}: no stationary IMU data before motion: bias NOT removed '
                  f'(start the --seq with "0,0:5")')
            bias = 0.0
        _, vl, _ = side(num, 'L')
        _, vr, _ = side(num, 'R')
        ht = num['host_t']
        for i0, i1, lab in segments(num):
            if not lab.startswith('v=') or i1 - i0 < 40:
                continue
            v_cmd, w_cmd = (float(p.split('=')[1]) for p in lab.split())
            if abs(w_cmd) < 1e-3:
                continue
            # skip the first --settle s (acceleration) of the segment
            h0, h1 = ht[i0] + args.settle, ht[i1 - 1]
            k = (ht >= h0) & (ht <= h1)
            mvl = np.nanmean(vl[k]) * args.m_per_rev / 60
            mvr = np.nanmean(vr[k]) * args.m_per_rev / 60
            gi = (imu['host_t'] >= h0) & (imu['host_t'] <= h1)
            gz = float(np.mean(imu['gz'][gi])) - bias
            eff = (mvr - mvl) / gz if abs(gz) > 1e-3 else float('nan')
            radius = ((mvl + mvr) / 2) / gz if abs(gz) > 1e-3 else float('nan')
            print(f'{os.path.basename(d)} | {lab} | {mvl:.3f} | {mvr:.3f} | {gz:.4f} | {w_cmd:.3f} | '
                  f'{eff:.4f} | {eff / args.track:.4f} | {radius:.2f}')
    print('   (use --m-per-rev from step E: the width scales 1:1 with it)')
    print('   (gz = /imu/data angular_velocity.z in imu_link; URDF imu_joint rpy = 0 so it is base_link yaw\n'
          '    rate, CCW positive per REP-103: a CCW spin must print gz > 0 and a positive width)')


# ------------------------------------------------------------------ circle (spin centre, driven radius)
ANTENNA_X = 0.764     # m, GNSS antenna ahead of base_link (control-stack audit V5; confirm with a tape)


def fit_circle(x, y):
    """Algebraic (Kasa) least-squares circle: centre, radius, RMS residual."""
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x ** 2 + y ** 2
    (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = math.sqrt(c + cx ** 2 + cy ** 2)
    res = np.hypot(x - cx, y - cy) - r
    return cx, cy, r, float(np.sqrt(np.mean(res ** 2)))


def cmd_circle(args):
    print('run | segment | fixes | antenna circle radius m | fit RMS cm | turn centre x from base_link m | '
          'base_link path radius m | commanded v/w m | fixed %')
    for d in args.runs:
        num = load_teensy(d)
        g = load_csv(os.path.join(d, 'gnss.csv'))
        if g is None:
            print(f'{d}: no gnss.csv (run with --ros)'); continue
        q = load_csv(os.path.join(d, 'gga.csv'))
        ht = num['host_t']
        for i0, i1, lab in segments(num):
            if not lab.startswith('v=') or i1 - i0 < 40:
                continue
            v_cmd, w_cmd = (float(p.split('=')[1]) for p in lab.split())
            if abs(w_cmd) < 1e-3:
                continue
            h0, h1 = ht[i0] + args.settle, ht[i1 - 1]
            sel = (g['host_t'] >= h0) & (g['host_t'] <= h1)
            if sel.sum() < 10:
                print(f'{os.path.basename(d)} | {lab} | fewer than 10 fixes'); continue
            x, y = enu(g['lat'][sel], g['lon'][sel], g['lat'][sel][0], g['lon'][sel][0])
            _, _, r, rms = fit_circle(x, y)
            fixed = float('nan')
            if q is not None:
                qs = (q['host_t'] >= h0) & (q['host_t'] <= h1)
                fixed = 100 * np.mean(q['quality'][qs] == 4) if qs.any() else float('nan')
            if abs(v_cmd) < 1e-3:
                # spin in place: the antenna circles the turn centre; centre assumed on the robot's x axis
                xc = args.antenna_x - r
                print(f'{os.path.basename(d)} | {lab} | {sel.sum()} | {r:.3f} | {100 * rms:.1f} | {xc:+.3f} | - | 0 | {fixed:.0f}')
            else:
                rb = math.sqrt(max(r ** 2 - args.antenna_x ** 2, 0.0))
                print(f'{os.path.basename(d)} | {lab} | {sel.sum()} | {r:.3f} | {100 * rms:.1f} | - | {rb:.3f} | '
                      f'{abs(v_cmd / w_cmd):.3f} | {fixed:.0f}')
    print('   (spins: turn centre x = antenna_x - radius; +0.31 m = middle of the tracks, 0 = base_link.\n'
          '    arcs: base_link radius assumes the turn centre is abeam base_link; compare with v/w.\n'
          '    Use segments with >= 1 full turn and 100 % RTK FIXED only.)')


# ------------------------------------------------------------------ gyroscale (spin reference)
def rtk_spin_angle(g, t_pre_end, t_post_start, t0, t1):
    """Yaw change of a spin in place from the antenna's angle around the fitted circle centre.
    Start/end angles are averaged over the stationary holds before and after the spin."""
    k = (g['host_t'] >= t0) & (g['host_t'] <= t1)
    x, y = enu(g['lat'][k], g['lon'][k], g['lat'][k][0], g['lon'][k][0])
    tt = g['host_t'][k]
    mv = (tt > t_pre_end) & (tt < t_post_start)
    if mv.sum() < 20:
        return float('nan')
    cx, cy, r, _ = fit_circle(x[mv], y[mv])
    ang = np.unwrap(np.arctan2(y - cy, x - cx))
    pre, post = tt <= t_pre_end, tt >= t_post_start
    if pre.sum() < 3 or post.sum() < 3:
        return float('nan')
    return math.degrees(float(np.mean(ang[post]) - np.mean(ang[pre])))


def cmd_gyroscale(args):
    print('run | gyro bias deg/s | gyro deg | RTK deg | scale vs RTK % | counted deg | scale vs count % | fixed %')
    for d in args.runs:
        num = load_teensy(d)
        imu = load_csv(os.path.join(d, 'imu.csv'))
        if imu is None:
            print(f'{d}: no imu.csv (run with --ros)'); continue
        b, nb = gyro_bias(imu, num)
        if not np.isfinite(b):
            print(f'{d}: no stationary data before motion (use --pre 5)'); continue
        moving = np.abs(num['sp_L']) + np.abs(num['sp_R']) > 1
        tm0, tm1 = num['host_t'][moving][0], num['host_t'][moving][-1]
        t0, t1 = tm0 - args.still, tm1 + args.still
        k = (imu['host_t'] >= t0) & (imu['host_t'] <= t1)
        ang = math.degrees(float(np.sum(np.diff(imu['host_t'][k]) * (imu['gz'][k][1:] - b))))
        g = load_csv(os.path.join(d, 'gnss.csv'))
        rtk = rtk_spin_angle(g, tm0 - 0.3, tm1 + 2.0, t0, t1) if g is not None else float('nan')
        q = load_csv(os.path.join(d, 'gga.csv'))
        fixed = float('nan')
        if q is not None:
            qs = (q['host_t'] >= t0) & (q['host_t'] <= t1)
            fixed = 100 * np.mean(q['quality'][qs] == 4) if qs.any() else float('nan')
        counted = (360.0 * args.turns + abs(args.residual_deg)) * (1 if ang >= 0 else -1) if args.turns else float('nan')
        print(f'{os.path.basename(d)} | {math.degrees(b):+.4f} | {ang:+.1f} | {rtk:+.1f} | '
              f'{100 * (ang / rtk - 1):+.2f} | {counted:+.1f} | {100 * (ang / counted - 1):+.2f} | {fixed:.0f}')
    print('   (Run: drive_tuner.py vel --pre 5 --seq "0,W:SECS 0,0:5" --ros. RTK reference needs 100 % FIXED and\n'
          '    stationary holds before and after; about 2 deg reference error over 5 turns = 0.1 %.\n'
          '    --turns N (whole turns counted on a ground mark, ending on the mark) is the fallback reference.\n'
          '    Pass (GROUND_TEST_PLAN 0.3): |scale error| <= 0.3 %.)')


# ------------------------------------------------------------------ still (stationary checks)
def cmd_still(args):
    print('run | secs | gyro z bias deg/s | gyro z noise deg/s | RTK FIXED % | GNSS static sd E/N cm | GNSS rate Hz')
    for d in args.runs:
        imu = load_csv(os.path.join(d, 'imu.csv'))
        g = load_csv(os.path.join(d, 'gnss.csv'))
        q = load_csv(os.path.join(d, 'gga.csv'))
        if imu is None:
            print(f'{d}: no imu.csv (run "drive_tuner.py listen --ros")'); continue
        secs = imu['host_t'][-1] - imu['host_t'][0]
        gz = np.degrees(imu['gz'])
        fixed = 100 * np.mean(q['quality'] == 4) if q is not None else float('nan')
        sde = sdn = rate = float('nan')
        if g is not None and len(g['lat']) > 5:
            x, y = enu(g['lat'], g['lon'], g['lat'][0], g['lon'][0])
            sde, sdn = 100 * np.std(x), 100 * np.std(y)
            rate = (len(g['lat']) - 1) / (g['host_t'][-1] - g['host_t'][0])
        print(f'{os.path.basename(d)} | {secs:.0f} | {np.mean(gz):+.4f} | {np.std(gz):.3f} | {fixed:.0f} | '
              f'{sde:.1f} / {sdn:.1f} | {rate:.1f}')
    print('   (GROUND_TEST_PLAN 0.2: bias <= 0.02 deg/s after >= 10 min warm-up; 0.4: 100 % FIXED, sd <= 2 cm)')


# ------------------------------------------------------------------ tape (ground-mark distance reference)
def arc_from_marks(along, side):
    """Path length of a constant-curvature run that starts along +x (robot squared to the start line)
    and ends at (along, side) measured from the start mark: returns (arc length m, heading change deg)."""
    if abs(side) < 1e-6:
        return along, 0.0
    r = (along ** 2 + side ** 2) / (2 * side)
    th = 2 * math.atan2(side, along)
    return abs(r * th), math.degrees(th)


def cmd_tape(args):
    if args.along is None:
        sys.exit('tape: give --along (m, along the start direction) and --side (m, + = left) for each run, '
                 'in the same order as the run folders, e.g. --along 20.105,20.098 --side 0.42,-0.31')
    al = [float(x) for x in args.along.split(',')]
    sd = [float(x) for x in args.side.split(',')] if args.side else [0.0] * len(al)
    if len(al) != len(args.runs) or len(sd) != len(args.runs):
        sys.exit('tape: one --along/--side value per run folder')
    print('run | tape along m | side m | path m | heading from marks deg | L rev | R rev | m/rev | gyro heading deg')
    for d, a, y in zip(args.runs, al, sd):
        num = load_teensy(d)
        path, dth = arc_from_marks(a, y)
        rl = abs(num['L_pos_rot'][-1] - num['L_pos_rot'][0])
        rr = abs(num['R_pos_rot'][-1] - num['R_pos_rot'][0])
        gdeg = float('nan')
        imu = load_csv(os.path.join(d, 'imu.csv'))
        if imu is not None and len(imu['host_t']) > 10:
            b, _ = gyro_bias(imu, num)
            b = 0.0 if not np.isfinite(b) else b
            gdeg = math.degrees(float(np.sum(np.diff(imu['host_t']) * (imu['gz'][1:] - b))))
        print(f'{os.path.basename(d)} | {a:.3f} | {y:+.3f} | {path:.3f} | {dth:+.2f} | {rl:.2f} | {rr:.2f} | '
              f'{path / ((rl + rr) / 2):.5f} | {gdeg:+.2f}')
    print('   (Tape from the start mark of the base_link pointer to its end mark: --along = distance along the start\n'
          '    direction, --side = sideways offset (+ left). Path assumes constant curvature. Revolutions cover the\n'
          '    whole run, so the robot must start and end at rest on the marks. Plan 4.1: |m/rev error| <= 1 %.)')


# ------------------------------------------------------------------ stops (braked stop segments)
def cmd_stops(args):
    print('run | side | speed before m/s | time to still s | stop distance m | backward m | '
          'max reverse RPM | min bus V | peak A | faults')
    for d in args.runs:
        num = load_teensy(d)
        ht = num['host_t']
        faults = ''
        lp = os.path.join(d, 'lines.log')
        if os.path.exists(lp):
            fl = [l.strip() for l in open(lp) if ' F ' in l or 'FAULT' in l.upper() or ' sf=' in l]
            sticky = [l for l in fl if 'sf=' in l and 'sf=0x00/0x00' not in l]
            faults = 'STICKY: ' + sticky[-1][-60:] if sticky else 'none'
        for i0, i1, lab in segments(num):
            if lab != 'stop' or i0 < 5:
                continue
            for sd in 'LR':
                t, v = pos_velocity(num[f'{sd}_s2_rx_us'], num[f'{sd}_pos_rot'], args.win)
                t0 = ht[i0]
                pre = (ht >= t0 - 0.5) & (ht < t0)
                v0 = float(np.nanmean(v[pre])) if pre.any() else float('nan')
                if not np.isfinite(v0) or abs(v0) < 50:
                    continue
                sgn = np.sign(v0)
                idx = np.arange(i0, i1)
                vv, tt = v[idx], ht[idx]
                still = np.abs(vv) < 30
                t_still = float('nan')
                for k in range(len(idx)):
                    w = (tt >= tt[k]) & (tt <= tt[k] + 0.2)
                    if still[w].all() and w.sum() >= 5:
                        t_still = tt[k] - t0
                        break
                pos = num[f'{sd}_pos_rot'][idx]
                p_ref = num[f'{sd}_pos_rot'][i0 - 1]        # last sample before the stop command
                travel = (pos - p_ref) * sgn * args.m_per_rev
                dist = float(np.max(travel))
                back = float(dist - travel[-1])
                rev = float(max(0.0, np.nanmax(-sgn * vv)))
                vb = float(np.min(num[f'{sd}_bus_V'][idx][num[f'{sd}_bus_V'][idx] > 1])) if (num[f'{sd}_bus_V'][idx] > 1).any() else float('nan')
                pk = float(np.max(np.abs(num[f'{sd}_current_A'][idx])))
                print(f'{os.path.basename(d)} | {sd} | {v0 * args.m_per_rev / 60:+.2f} | {t_still:.2f} | {dist:.3f} | '
                      f'{back:.3f} | {rev:.0f} | {vb:.1f} | {pk:.0f} | {faults}')
    print('   (distance from the stop command, from encoder position (track travel, no slip correction);\n'
          '    backward = travel lost after the furthest point; still = |speed| < 30 RPM for 0.2 s.\n'
          '    Plan 1.3: no backward motion (reverse RPM ~0, backward < 0.01 m), no faults.)')


# ------------------------------------------------------------------ delivery (steady speed vs command, many repeats)
def _run_geom(d, m_per_rev, track):
    import json
    try:
        meta = json.load(open(os.path.join(d, 'meta.json')))
    except OSError:
        meta = {}
    return float(meta.get('m_per_rev', m_per_rev)), float(meta.get('track', track)), float(meta.get('mult', 1.0)), meta


def _vw(lab):
    return tuple(float(p.split('=')[1]) for p in lab.split())


def delivery_rows(runs, win, window, min_seg, m_per_rev, track):
    """One row per (run, steady segment, side): steady speed from encoder position vs the command."""
    rows = []
    for d in runs:
        num = load_teensy(d)
        mpr, trk, mult, _ = _run_geom(d, m_per_rev, track)
        vel = {sd: side(num, sd, win)[1] for sd in 'LR'}
        ht = num['host_t']
        for i0, i1, lab in segments(num):
            if not lab.startswith('v='):
                continue
            v_cmd, w_cmd = _vw(lab)
            if abs(v_cmd) < 1e-9 and abs(w_cmd) < 1e-9:
                continue
            if ht[i1 - 1] - ht[i0] < min_seg:
                continue
            b = trk * mult
            tgt = {'L': (v_cmd - w_cmd * b / 2) * 60 / mpr, 'R': (v_cmd + w_cmd * b / 2) * 60 / mpr}
            sel = np.arange(i0, i1)[ht[i0:i1] >= ht[i1 - 1] - window]
            for sd in 'LR':
                if abs(tgt[sd]) < 1:
                    continue
                x = vel[sd][sel]
                rows.append(dict(run=os.path.basename(d), seg=lab, v=v_cmd, w=w_cmd, side=sd, target=tgt[sd], mpr=mpr,
                                 mean=float(np.nanmean(x)), sd=float(np.nanstd(x)),
                                 zero=float(np.mean(np.abs(x) < 0.2 * abs(tgt[sd])))))
    return rows


def stop_rows(runs, win, m_per_rev, track):
    """After the last moving segment (command back to 0): rollback distance (mm) = how far the track returns
    from its furthest point in the direction of motion, plus the settling time, per side."""
    rows = []
    for d in runs:
        num = load_teensy(d)
        mpr, _, _, _ = _run_geom(d, m_per_rev, track)
        ht = num['host_t']
        vel = {sd: side(num, sd, win)[1] for sd in 'LR'}
        segs = segments(num)
        idx = [k for k, sg in enumerate(segs) if sg[2].startswith('v=')]
        for k in range(len(idx) - 1, 0, -1):
            i0, i1, lab = segs[idx[k]]
            pi0, pi1, plab = segs[idx[k - 1]]
            vc, wc = _vw(lab)
            pvc, pwc = _vw(plab)
            if abs(vc) < 1e-9 and abs(wc) < 1e-9 and (abs(pvc) > 1e-9 or abs(pwc) > 1e-9):
                for sd in 'LR':
                    pre = np.arange(pi0, pi1)[ht[pi0:pi1] >= ht[pi1 - 1] - 1.0]
                    before = float(np.nanmean(vel[sd][pre]))
                    if abs(before) < 30:
                        continue
                    dirn = np.sign(before)
                    pos = num[f'{sd}_pos_rot'][i0:i1]
                    prog = dirn * (pos - pos[0])
                    rollback = float(max(0.0, np.max(prog) - prog[-1]) * mpr * 1000)
                    x = vel[sd][i0:i1]
                    tt = ht[i0:i1] - ht[i0]
                    t_still = float('nan')
                    for j in range(len(x)):
                        w2 = (tt >= tt[j]) & (tt <= tt[j] + 0.2)
                        if np.all(np.abs(x[w2]) < 30) and w2.sum() >= 5:
                            t_still = float(tt[j])
                            break
                    rows.append(dict(run=os.path.basename(d), side=sd, before=before, prev=plab, rollback=rollback, t_still=t_still))
                break
    return rows


def _fmt_mps(rpm, mpr):
    return abs(rpm) * mpr / 60


def cmd_delivery(args):
    import csv as _csv
    rows = delivery_rows(args.runs, args.win, args.window, args.min_seg, args.m_per_rev, args.track)
    if not rows:
        print('no steady segments found (labels "v=.. w=..", at least --min-seg s long)'); return
    print(f'steady speed from encoder position over the last {args.window} s of each segment (segments >= {args.min_seg} s)\n')
    print('mode | side | command RPM (m/s) | n | mean error % | 95 % band +/- | worst run % | ripple sd RPM | stuck % | limit % | result')
    groups = {}
    for r in rows:
        mode = 'straight' if abs(r['w']) < 1e-9 else 'spin'
        groups.setdefault((mode, r['side'], round(r['target'])), []).append(r)
    bad = 0
    out_rows = []
    mpr = rows[0]['mpr']
    for (mode, sd, tg), g in sorted(groups.items(), key=lambda kv: (kv[0][0] != 'straight', kv[0][1], kv[0][2])):
        err = np.array([100 * (r['mean'] / r['target'] - 1) for r in g])
        n = len(err)
        band = 2 * np.std(err, ddof=1) / math.sqrt(n) if n > 1 else float('nan')
        ms = _fmt_mps(tg, mpr)
        lim = args.limit_cruise if ms >= args.cruise_from else args.limit_slow
        ripple = float(np.mean([r['sd'] for r in g]))
        stuck = 100 * float(np.mean([r['zero'] for r in g]))
        ok = abs(err.mean()) <= lim and all(r['zero'] < 0.05 for r in g)
        bad += not ok
        worst = float(err[np.argmax(np.abs(err))])
        print(f'{mode:8s} | {sd} | {tg:6d} ({ms:5.2f}) | {n:2d} | {err.mean():+6.2f} | {band:5.2f} | {worst:+6.2f} | '
              f'{ripple:5.1f} | {stuck:4.1f} | {lim:4.1f} | {"PASS" if ok else "FAIL"}')
        out_rows.append(dict(kind=mode, side=sd, target_rpm=tg, target_mps=round(ms, 3), n=n, mean_err_pct=round(float(err.mean()), 3),
                             band_pct=round(float(band), 3) if n > 1 else '', worst_pct=round(worst, 3), ripple_rpm=round(ripple, 2),
                             stuck_pct=round(stuck, 1), rollback_max_mm='', t_still_s=''))
    print('\nleft/right match (same segment, right/left delivered ratio):')
    seg = {}
    for r in rows:
        seg.setdefault((r['run'], r['seg']), {})[r['side']] = r
    mm = {}
    for (run, lab), lr in seg.items():
        if 'L' in lr and 'R' in lr and abs(lr['L']['target'] - lr['R']['target']) < 1e-6:
            mm.setdefault(round(lr['L']['target']), []).append(100 * ((lr['R']['mean'] / lr['L']['mean']) - 1))
    for tg, v in sorted(mm.items()):
        print(f'  command {tg:6d} RPM: R vs L {np.mean(v):+5.2f} % (n={len(v)})')
    srows = stop_rows(args.runs, args.win, args.m_per_rev, args.track)
    if srows:
        print('\nstop behaviour (command back to 0 through the speed ramp; rollback = distance the track returns from its furthest point, limit 5 mm; 1 encoder count = 0.5 mm):')
        print('previous command | side | n | rollback mean / max mm | settle time mean s | result')
        sg = {}
        for r in srows:
            sg.setdefault((r['prev'], r['side']), []).append(r)
        for (prev, sd), g in sorted(sg.items()):
            rb = [r['rollback'] for r in g]
            ts = [r['t_still'] for r in g if r['t_still'] == r['t_still']]
            okr = max(rb) <= 5.0
            bad += not okr
            print(f'{prev:14s} | {sd} | {len(g):2d} | {np.mean(rb):5.1f} / {max(rb):5.1f} | {np.mean(ts) if ts else float("nan"):4.2f} | {"PASS" if okr else "FAIL"}')
            out_rows.append(dict(kind='stop', side=sd, target_rpm=prev, target_mps='', n=len(g), mean_err_pct='', band_pct='',
                                 worst_pct='', ripple_rpm='', stuck_pct='', rollback_max_mm=round(max(rb), 2),
                                 t_still_s=round(float(np.mean(ts)), 2) if ts else ''))
    print(f'\n{"ALL PASS" if not bad else str(bad) + " condition(s) FAIL"}  (limits: {args.limit_cruise} % at >= {args.cruise_from} m/s, {args.limit_slow} % below; stuck = share of 100 ms windows under 20 % of the command)')
    print('   (ruler = motor encoder position: exact for motor speed, blind to track slip on the ground -> distance scale is test 4.1)')
    if args.csv:
        new = not os.path.exists(args.csv)
        cols = ['label', 'gains', 'kind', 'side', 'target_rpm', 'target_mps', 'n', 'mean_err_pct', 'band_pct', 'worst_pct',
                'ripple_rpm', 'stuck_pct', 'rollback_max_mm', 't_still_s']
        with open(args.csv, 'a', newline='') as f:
            w = _csv.DictWriter(f, fieldnames=cols)
            if new:
                w.writeheader()
            for r in out_rows:
                w.writerow(dict(label=args.label or '', gains=args.gains or '', **r))
        print(f'appended {len(out_rows)} rows to {args.csv}')


def cmd_ffid(args):
    """Closed-form feedforward identification from FF-only runs (kP = kI = 0): the steady delivered speed m obeys
    kV_set*|t| + kS_set = kS_true + kV_true*|m|  ->  |m| = a*|t| + b,  kV_true = kV_set/a,  kS_true = kS_set - b*kV_true."""
    rows = [r for r in delivery_rows(args.runs, args.win, args.window, args.min_seg, args.m_per_rev, args.track) if abs(r['w']) < 1e-9]
    ks_set = {'L': args.ks_l, 'R': args.ks_r}
    print(f'feedforward identification (straight segments only; set gains kV {args.kv_set}, kS L {args.ks_l} / R {args.ks_r}; valid only if kP = kI = 0)')
    print('side | dir | points | slope a | intercept b RPM | r2 | kV true V/RPM | kS true V')
    fit = {}
    for sd in 'LR':
        for sgn, name in ((1, 'fwd'), (-1, 'rev')):
            pts = [(abs(r['target']), abs(r['mean'])) for r in rows if r['side'] == sd and np.sign(r['target']) == sgn
                   and abs(r['mean']) >= args.min_moving * abs(r['target'])]
            if len(pts) < 4:
                print(f'{sd} | {name} | {len(pts)} | too few moving points'); continue
            t = np.array([p[0] for p in pts]); m = np.array([p[1] for p in pts])
            a, b = np.polyfit(t, m, 1)
            r2 = 1 - np.sum((m - (a * t + b)) ** 2) / np.sum((m - m.mean()) ** 2)
            kvt = args.kv_set / a
            kst = ks_set[sd] - b * kvt
            fit[(sd, name)] = (kvt, kst)
            print(f'{sd} | {name} | {len(pts):3d} | {a:.4f} | {b:+7.1f} | {r2:.4f} | {kvt:.5f} | {kst:+.3f}')
    if len(fit) < 4:
        return
    kv_rec = float(np.mean([v[0] for v in fit.values()]))
    ks_rec = {sd: float(np.mean([fit[(sd, 'fwd')][1], fit[(sd, 'rev')][1]])) for sd in 'LR'}
    print(f'\nrecommended (one kV for both tracks and directions, one kS per track = mean of forward and reverse):')
    print(f'  kV = {kv_rec:.5f} V/RPM   kS_left = {ks_rec["L"]:.3f} V   kS_right = {ks_rec["R"]:.3f} V')
    print('\npredicted FF-only delivery error with the recommended values (model):')
    print('command RPM (m/s) | L fwd | L rev | R fwd | R rev   (% error)')
    for tr in (150, 301, 602, 903, 1505, 2106):
        cells = []
        for sd in 'LR':
            for name in ('fwd', 'rev'):
                kvt, kst = fit[(sd, name)]
                mpred = (kv_rec * tr + ks_rec[sd] - kst) / kvt
                cells.append(f'{100 * (mpred / tr - 1):+6.1f}')
        print(f'{tr:6d} ({tr * rows[0]["mpr"] / 60:4.2f}) | ' + ' | '.join(cells))


def cmd_guard(args):
    """Safety check of one finished run: prints 'ok ...' or 'BAD <reason> ...' (exit status 1 for BAD)."""
    import json
    d = args.runs[0]
    num = load_teensy(d)
    _, _, _, meta = _run_geom(d, args.m_per_rev, args.track)
    import re
    m = re.search(r'sf=(0x\w+/0x\w+)', meta.get('diag_end') or '')
    sf = m.group(1) if m else '?'
    bus = np.minimum(num['L_bus_V'], num['R_bus_V'])
    bus = bus[bus > 1]
    peak_a = float(max(np.max(num['L_current_A']), np.max(num['R_current_A'])))
    peak_rpm = float(max(np.max(np.abs(num['L_vel_rpm'])), np.max(np.abs(num['R_vel_rpm']))))
    rows = delivery_rows([d], args.win, args.window, args.min_seg, args.m_per_rev, args.track)
    ripple = max([r['sd'] for r in rows], default=0.0)
    dl = (num['L_pos_rot'][-1] - num['L_pos_rot'][0]) * args.m_per_rev
    dr = (num['R_pos_rot'][-1] - num['R_pos_rot'][0]) * args.m_per_rev
    jump = float(max(np.max(np.abs(np.diff(num['L_pos_rot']))), np.max(np.abs(np.diff(num['R_pos_rot'])))))
    why = []
    if jump > args.max_jump:
        why.append(f'counter reset: position jumped {jump:.0f} rotations in one sample (motor drivers power-cycled, e-stop?)')
    if sf not in ('0x00/0x00', '?'):
        why.append(f'fault flags {sf}')
    if bus.size and bus.min() < args.min_bus:
        why.append(f'bus {bus.min():.1f} V')
    if peak_a > args.max_amps:
        why.append(f'current {peak_a:.0f} A')
    if ripple > args.max_ripple:
        why.append(f'speed ripple {ripple:.0f} RPM (oscillation?)')
    info = (f'travel L {dl:+.2f} R {dr:+.2f} m | peak {peak_rpm:.0f} RPM | min bus {bus.min() if bus.size else float("nan"):.1f} V | '
            f'peak {peak_a:.0f} A | max ripple {ripple:.0f} RPM | faults {sf}')
    print(('BAD ' + '; '.join(why) + ' | ' if why else 'ok ') + os.path.basename(d) + ' ' + info)
    sys.exit(1 if why else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(required=True)
    for name, fn in (('ff', cmd_ff), ('lag', cmd_lag), ('steps', cmd_steps),
                     ('distance', cmd_distance), ('turn', cmd_turn), ('circle', cmd_circle),
                     ('gyroscale', cmd_gyroscale), ('still', cmd_still), ('tape', cmd_tape), ('stops', cmd_stops), ('delivery', cmd_delivery), ('ffid', cmd_ffid), ('guard', cmd_guard)):
        p = sub.add_parser(name)
        p.add_argument('runs', nargs='+')
        p.add_argument('--m-per-rev', dest='m_per_rev', type=float, default=0.01994)
        p.add_argument('--track', type=float, default=TRACK_W)
        p.add_argument('--vmin', type=float, default=30.0, help='ff: ignore |v| below this RPM')
        p.add_argument('--win', type=float, default=0.06 if name == 'ff' else 0.1,
                       help='velocity-from-position window, s (ff 0.06, steps 0.1)')
        p.add_argument('--all-segments', action='store_true',
                       help='ff: also fit rest/stop segments (default: only ramp/step test segments)')
        p.add_argument('--trim', type=float, default=1.0, help='distance: seconds trimmed at start/end')
        p.add_argument('--settle', type=float, default=1.5, help='turn/circle: seconds skipped at segment start')
        p.add_argument('--antenna-x', dest='antenna_x', type=float, default=ANTENNA_X,
                       help='circle: GNSS antenna distance ahead of base_link, m')
        p.add_argument('--turns', type=float, default=0, help='gyroscale: whole turns counted on a ground mark (optional)')
        p.add_argument('--residual-deg', dest='residual_deg', type=float, default=0.0,
                       help='gyroscale: extra angle past the counted turns, from the chalk heading lines (deg, same sign as the turn)')
        p.add_argument('--window', type=float, default=1.5, help='delivery: seconds at the end of each segment used as steady state')
        p.add_argument('--min-seg', dest='min_seg', type=float, default=3.0, help='delivery: ignore segments shorter than this (s)')
        p.add_argument('--limit-cruise', dest='limit_cruise', type=float, default=2.0, help='delivery: pass limit %% at >= --cruise-from m/s')
        p.add_argument('--limit-slow', dest='limit_slow', type=float, default=10.0, help='delivery: pass limit %% below --cruise-from m/s')
        p.add_argument('--cruise-from', dest='cruise_from', type=float, default=0.3, help='delivery: m/s where the tight limit starts')
        p.add_argument('--csv', default=None, help='delivery: append the aggregated rows to this CSV')
        p.add_argument('--label', default=None, help='delivery: experiment label for the CSV')
        p.add_argument('--gains', default=None, help='delivery: gain string recorded in the CSV')
        p.add_argument('--kv-set', dest='kv_set', type=float, default=0.0023, help='ffid: kV used in the runs (V/RPM)')
        p.add_argument('--ks-l', dest='ks_l', type=float, default=0.18, help='ffid: kS left used in the runs (V)')
        p.add_argument('--ks-r', dest='ks_r', type=float, default=0.18, help='ffid: kS right used in the runs (V)')
        p.add_argument('--min-moving', dest='min_moving', type=float, default=0.3, help='ffid: use points with mean >= this share of the command')
        p.add_argument('--max-jump', dest='max_jump', type=float, default=20.0, help='guard: position step (rotations per sample) that means a counter reset')
        p.add_argument('--max-ripple', dest='max_ripple', type=float, default=60.0, help='guard: speed ripple limit (RPM)')
        p.add_argument('--max-amps', dest='max_amps', type=float, default=45.0, help='guard: motor current limit (A)')
        p.add_argument('--min-bus', dest='min_bus', type=float, default=9.5, help='guard: lowest bus voltage (V)')
        p.add_argument('--along', default=None, help='tape: distance along the start direction per run, m (comma list)')
        p.add_argument('--side', default=None, help='tape: sideways end offset per run, m, + = left (comma list)')
        p.add_argument('--still', type=float, default=4.0, help='gyroscale: stationary seconds used before/after the spin')
        p.set_defaults(func=fn)
    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
