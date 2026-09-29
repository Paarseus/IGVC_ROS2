#!/usr/bin/env python3
"""Analysis for drive_tuner.py runs (docs/drive_tuning_2026_09_28/STRATEGY.md).

  analyze.py ff       RUN_DIR [RUN_DIR ...]   feedforward fit per side and direction: V = kS*sgn(v) + kV*v + kA*a   (step B)
  analyze.py lag      RUN_DIR [...]           speed-reading lag: reported STATUS_2 velocity vs d(position)/dt  (step B.5)
  analyze.py steps    RUN_DIR [...]           closed-loop step metrics per segment: 95% time, overshoot, steady error, L/R mismatch (steps C, D)
  analyze.py distance RUN_DIR [...]           m per motor revolution from RTK (/gnss) vs encoder revolutions (step E)
  analyze.py turn     RUN_DIR [...]           effective track width / multiplier from gyro vs track speeds (step F)

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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(required=True)
    for name, fn in (('ff', cmd_ff), ('lag', cmd_lag), ('steps', cmd_steps),
                     ('distance', cmd_distance), ('turn', cmd_turn)):
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
        p.add_argument('--settle', type=float, default=1.5, help='turn: seconds skipped at segment start')
        p.set_defaults(func=fn)
    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
