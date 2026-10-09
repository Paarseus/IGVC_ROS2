#!/usr/bin/env python3
"""Isolated bench scopes (tracks OFF the ground) from MPPI_READINESS_TEST_PLAN.md. RAM-only settings, restored after.
usage: scopes.py s0 | s1 | s2 | s3ff | s3loop | s3depth | s3low | s3sat | s5
Each prints a summary and saves ~/bench_scopes_2026_09_28/<scope>.json"""
import statistics as st, sys, time
from tio import Teensy, side_series, pos_speed, pct, save, standard, restore

RPM_PER_MPS = 60 / 0.01994
t = Teensy()


def steady_stats(rows, s, t0, t1):
    ser = [x for x in side_series(rows, s) if t0 <= x[0] <= t1]
    ps = [v for _, v in pos_speed(ser)] if len(ser) > 4 else []
    rep = [x[2] for x in ser]
    return (st.mean(ps) if ps else float('nan'), (max(ps) - min(ps)) / 2 if ps else float('nan'),
            st.pstdev(ps) if len(ps) > 1 else float('nan'), st.mean(rep) if rep else float('nan'),
            st.mean([x[3] for x in ser]) if ser else float('nan'))


def hold(cmd, secs, settle):
    """Stream cmd; return Teensy-us window [settle, secs] for steady stats."""
    t.clear_x(); t.stream(cmd, secs)
    if not t.x:
        return None
    us0 = t.x[0]['us']
    return us0 + settle * 1e6, us0 + secs * 1e6


def s0():
    t.send('X1'); t.clear_x(); t.stream('S', 10.0); t.send('X0')
    xs = t.x
    n = len(xs); us = [r['us'] for r in xs]; h = [r['h'] for r in xs]
    mu_u, mu_h = st.mean(us), st.mean(h)
    b = sum((u - mu_u) * (hh - mu_h) for u, hh in zip(us, h)) / sum((u - mu_u) ** 2 for u in us)
    a = mu_h - b * mu_u
    res = [(hh - (a + b * u)) * 1000 for u, hh in zip(us, h)]
    lat = sorted(res)
    out = dict(samples=n, drift_ppm=(b * 1e6 - 1) * 1e6, residual_ms_p50=pct(res, 0.5), residual_ms_p99=pct(res, 0.99),
               residual_ms_min=lat[0], residual_ms_max=lat[-1], pass_p99_le_2ms=(pct(res, 0.99) - lat[0]) <= 2.0)
    print('S0 clock map:', out); save('s0_clock_map', out)


def s1():
    standard(t); t.send('X1')
    w = hold('L1000 R1000', 8.0, 1.0)
    rows = list(t.x)
    res = {}
    for s in 'LR':
        rx = sorted(set(r[s + 'rx'] for r in rows))
        d = [(b - a) / 1000 for a, b in zip(rx, rx[1:])]
        res[f'status2_{s}_ms'] = dict(mean=st.mean(d), p99=pct(d, 0.99), max=max(d), gaps_over_40ms=sum(1 for x in d if x > 40))
        sp = sorted(set(r['sp' + s + 'us'] for r in rows))
        d2 = [(b - a) / 1000 for a, b in zip(sp, sp[1:])]
        res[f'setpoint_tx_{s}_ms'] = dict(mean=st.mean(d2), p99=pct(d2, 0.99), max=max(d2))
    t.send('S', 0.5)
    # serial round trip: command -> reply
    rtt = []
    for _ in range(50):
        t0 = time.time(); t.s.write(b'D\n')
        while True:
            t.pump(0.001)
            if t.lines and t.lines[-1][1].startswith('DIAG') and t.lines[-1][0] >= t0:
                rtt.append((t.lines[-1][0] - t0) * 1000); break
            if time.time() - t0 > 0.5:
                break
    res['serial_rtt_ms'] = dict(p50=pct(rtt, 0.5), p99=pct(rtt, 0.99), max=max(rtt) if rtt else None)
    # CAN tx rate from DIAG counters
    import re
    d0 = t.diag(); t0d = time.time(); t.pump(5.0); d1 = t.diag(); dt = time.time() - t0d
    m0, m1 = re.search(r'tx=(\d+) rx=(\d+)', d0), re.search(r'tx=(\d+) rx=(\d+)', d1)
    if m0 and m1:
        res['can_tx_frames_per_s'] = round((int(m1.group(1)) - int(m0.group(1))) / dt, 1)
        res['can_rx_frames_per_s'] = round((int(m1.group(2)) - int(m0.group(2))) / dt, 1)
        res['txq_txfail_sdrop'] = re.search(r'txq=\S+ txfail=\S+ foreign=\S+ sdrop=\S+', d1).group(0)
    # watchdog: stream, then silence; time from last command to setpoint forced to idle
    t.clear_x(); t.stream('L800 R800', 2.0); t_last = time.time(); t.pump(1.0)
    after = [r for r in t.x if r['h'] >= t_last]
    idle = [r for r in after if r['mode'] == 'D' or (r['spL'] == 0 and r['spR'] == 0)]
    res['watchdog_ms_host'] = (idle[0]['h'] - t_last) * 1000 if idle else None
    res['watchdog_expected_ms'] = '300 (+ up to 20 ms tick + USB)'
    restore(t)
    print('S1 timing:'); [print('  ', k, v) for k, v in res.items()]; save('s1_timing', res)


def s2():
    names = [l.split()[3] for l in t.send('PT', 1.0) if l.startswith('PT ')]
    man = {n: t.pr(n) for n in names}
    man['_firmware'] = [l for l in t.send('FV', 0.8) if l.startswith('FV ')]
    man['_diag'] = t.diag()
    print('S2 manifest:', len(names), 'params'); [print('  ', n, v) for n, v in man.items() if not n.startswith('_')]
    print('  ', man['_firmware']); save('s2_manifest_baseline', man)


GRID = [300, 1000, 2000, 3500, -300, -1000, -2000, -3500]


def s3ff():
    """Feedforward alone (P = I = D = 0): delivered/commanded per side and direction, reported vs position speed."""
    standard(t); t.send('KP0'); t.send('X1')
    res = []
    for rpm in GRID:
        w = hold(f'L{rpm} R{rpm}', 3.5, 2.0)
        for s in 'LR':
            m, rip, sd, rep, app = steady_stats(t.x, s, *w)
            res.append(dict(cmd=rpm, side=s, pos_rpm=round(m), ratio=round(m / rpm, 4), applied=round(app, 4),
                            duty_per_rpm=round(app / m, 7) if m else None, reported=round(rep)))
        t.send('S', 0.8)
    restore(t)
    for r in res: print('  S3.1 FF-only', r)
    save('s3_1_ff_only', res)


def s3loop(p=0.0003):
    """Steady error + ripple over the command grid, and steps (M10000), with the standard setup at P."""
    standard(t, p); t.send('X1')
    grid = []
    for rpm in GRID + ['1000,-1000', '2000,1000']:
        l, r = (rpm, rpm) if isinstance(rpm, int) else map(int, rpm.split(','))
        w = hold(f'L{l} R{r}', 3.5, 2.0)
        for s, c in (('L', l), ('R', r)):
            m, rip, sd, rep, app = steady_stats(t.x, s, *w)
            grid.append(dict(cmd=c, side=s, pos_rpm=round(m), err_pct=round(100 * (m - c) / c, 2), ripple=round(rip), sd=round(sd, 1)))
        t.send('S', 0.8)
    steps = []
    for a, b in ((0, 1000), (1000, 2000), (2000, 1000), (0, -1000), (-1000, -2000)):
        t.clear_x()
        if a:
            t.stream(f'L{a} R{a}', 2.5)
        t.stream(f'L{b} R{b}', 3.0)
        rows = t.x
        for s in 'LR':
            ser = side_series(rows, s)
            # step instant = first setpoint tx carrying the new value
            t_step = next((r['sp' + s + 'us'] for r in rows if abs(r['sp' + s] - b) < 1), None)
            ps = [(tt - t_step, v) for tt, v in pos_speed(ser) if t_step and tt >= t_step]
            span = b - a
            t95 = next(((tt / 1e6) for tt, v in ps if abs(v - a) >= 0.95 * abs(span)), None)
            t_start = next(((tt / 1e6) for tt, v in ps if abs(v - a) >= 0.05 * abs(span)), None)
            peak = (max(v for _, v in ps) if span > 0 else min(v for _, v in ps)) if ps else None
            over = 100 * (peak - b) / abs(span) * (1 if span > 0 else -1) if ps else None
            steps.append(dict(step=f'{a}->{b}', side=s, delay_to_5pct_s=t_start, t95_s=t95, overshoot_pct=round(over, 1) if over is not None else None))
        t.send('S', 1.0)
    restore(t)
    print(f'S3 loop P={p}:')
    for r in grid: print('  grid', r)
    for r in steps: print('  step', r)
    save(f's3_loop_P{p}', dict(grid=grid, steps=steps))


def s3depth():
    """Hall filter depth 3/2/1 x P: ripple at 1000 RPM, step 1000->2000 overshoot, stability, lag (reported vs position)."""
    res = []
    for depth in (3, 2, 1):
        for p in (0.0001, 0.0002, 0.0003, 0.0004):
            standard(t, p); t.pw('hallAvgDepth', depth); t.send('X1'); t.send('CF B', 0.4)
            w = hold('L1000 R1000', 3.0, 1.5)
            m, rip, sd, _, _ = steady_stats(t.x, 'L', *w)
            mr, ripr, _, _, _ = steady_stats(t.x, 'R', *w)
            t.clear_x(); t.stream('L2000 R2000', 2.5); rows = t.x
            ser = side_series(rows, 'L'); ps = pos_speed(ser)
            peak = max(v for _, v in ps) if ps else float('nan')
            # lag: best shift of reported onto position speed
            rep = [(x[0], x[2]) for x in ser]
            best = None
            for k_ms in range(0, 300, 4):
                e = []
                for (tt, v) in ps:
                    tr = tt + k_ms * 1000
                    near = min(rep, key=lambda q: abs(q[0] - tr))
                    if abs(near[0] - tr) < 15000:
                        e.append((near[1] - v) ** 2)
                if e and (best is None or st.mean(e) < best[1]):
                    best = (k_ms, st.mean(e))
            t.send('S', 1.0)
            sf = t.sticky()
            r = dict(depth=depth, P=p, ripple_L=round(rip), ripple_R=round(ripr), overshoot_1000to2000_pct=round(100 * (peak - 2000) / 1000, 1),
                     lag_ms=best[0] if best else None, stable=bool(rip < 60 and ripr < 60), drv=sf)
            print('  S3.7', r); res.append(r)
    restore(t); save('s3_7_depth_sweep', res)


def s3low(p=0.0003, ks=0.0):
    standard(t, p); t.send(f'KS{ks}'); t.send('X1'); res = []
    for rpm in (50, 100, 150, 300, -100):
        w = hold(f'L{rpm} R{rpm}', 5.0, 2.0)
        for s in 'LR':
            ser = [x for x in side_series(t.x, s) if w[0] <= x[0] <= w[1]]
            ps = [v for _, v in pos_speed(ser, 100000)]
            stuck = sum(1 for v in ps if abs(v) < 0.2 * abs(rpm)) / max(1, len(ps))
            res.append(dict(ks_V=ks, cmd=rpm, side=s, mean=round(st.mean(ps)) if ps else None, sd=round(st.pstdev(ps), 1) if len(ps) > 1 else None,
                            stuck_fraction=round(stuck, 3)))
        t.send('S', 0.8)
    t.send('KS0'); restore(t)
    for r in res: print('  S3.8 low speed', r)
    save(f's3_8_low_speed_ks{ks}', res)


def s3sat(p=0.0003):
    standard(t, p); t.send('X1')
    w = hold('L4600 R4600', 5.0, 3.0)
    res = {}
    for s in 'LR':
        m, rip, sd, rep, app = steady_stats(t.x, s, *w)
        res[s] = dict(cmd=4600, pos_rpm=round(m), applied=round(app, 3), err_pct=round(100 * (m - 4600) / 4600, 2))
    t.send('S', 1.5); res['drv'] = t.sticky(); restore(t)
    print('S3.9 saturation', res); save('s3_9_saturation', res)


def s5():
    """Per-hop latency, serial side: host->Teensy ack (H1), Teensy command->CAN setpoint (H2, from X),
    setpoint->motion onset (H3), reported-speed lag (H5)."""
    standard(t); t.send('X1')
    h1 = []
    for _ in range(40):
        t0 = time.time(); t.s.write(b'L0 R0\n')
        while time.time() - t0 < 0.3:
            t.pump(0.001)
            if t.lines and t.lines[-1][1].startswith('OK L=') and t.lines[-1][0] >= t0:
                h1.append((t.lines[-1][0] - t0) * 1000); break
    onset = []
    for _ in range(5):
        t.stream('S', 1.0); t.clear_x()
        t_cmd = time.time(); t.stream('L1500 R1500', 1.5)
        rows = t.x
        for s in 'LR':
            sp_us = next((r['sp' + s + 'us'] for r in rows if r['sp' + s] > 1), None)
            ser = side_series(rows, s)
            ps = pos_speed(ser, 40000)
            mv = next((tt for tt, v in ps if sp_us and tt >= sp_us and v > 75), None)
            host_first = next((r['h'] for r in rows if r['sp' + s] > 1), None)
            onset.append(dict(side=s, host_to_first_setpoint_ms=round((host_first - t_cmd) * 1000, 1) if host_first else None,
                              setpoint_to_motion_ms=round((mv - sp_us) / 1000, 1) if (mv and sp_us) else None))
    t.send('S', 0.5); restore(t)
    res = dict(H1_host_to_teensy_ack_ms=dict(p50=pct(h1, 0.5), p99=pct(h1, 0.99)), onset=onset)
    print('S5 hops:', res['H1_host_to_teensy_ack_ms']); [print('  ', o) for o in onset]; save('s5_hops_serial', res)


if __name__ == '__main__':
    fn = {'s0': s0, 's1': s1, 's2': s2, 's3ff': s3ff, 's3loop': s3loop, 's3depth': s3depth, 's3low': s3low, 's3sat': s3sat, 's5': s5}[sys.argv[1]]
    if len(sys.argv) > 3:
        fn(float(sys.argv[2]), float(sys.argv[3]))
    elif len(sys.argv) > 2:
        fn(float(sys.argv[2]))
    else:
        fn()
