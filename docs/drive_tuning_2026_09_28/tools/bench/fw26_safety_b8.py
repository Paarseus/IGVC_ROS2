#!/usr/bin/env python3
"""FW 26: T1.2 disabled-heartbeat (HB0) proof, T1.4 clamps, T3.4/B8 hall period (param 136) effect.
Tracks off the ground. RAM only; param 136 restored to its read value afterwards."""
import statistics as st, sys
from tio import Teensy, side_series, pos_speed, save

t = Teensy(); res = {}


def hb0():
    t.send('X1', 0.3); t.clear_x()
    import time
    t0 = time.time(); sent = False
    while time.time() - t0 < 5.0:
        t.s.write(b'UL0.06 UR0.06\n'); t.pump(0.05)
        if not sent and time.time() - t0 > 2.0:
            t.s.write(b'HB0\n'); sent = True; t_hb = time.time()
    t.send('HB1', 0.3); t.send('S', 0.6)
    before = [r for r in t.x if t_hb - 0.6 < r['h'] < t_hb]
    after = [r for r in t.x if r['h'] > t_hb + 0.5]
    stop_t = next((r['h'] - t_hb for r in t.x if r['h'] > t_hb and abs(r['Lv']) < 20 and abs(r['Rv']) < 20), None)
    out = dict(rpm_before=round(st.mean(r['Lv'] for r in before)), applied_after=max(abs(r['La']) for r in after),
               rpm_after=round(st.mean(abs(r['Lv']) for r in after)), stop_s=round(stop_t, 2) if stop_t else None)
    print('T1.2 HB0 on FW26:', out, ' pass: applied_after 0, rpm_after ~0, stop <= 0.3-0.5 s'); return out


def clamps():
    out = {}
    for c, key in (('L9000 R9000', 'OK L='), ('UL0.9 UR0.9', 'OK UL='), ('UVL20 UVR20', 'OK UVL='), ('Lnan Rnan', 'OK L=')):
        ack = [l for l in t.send(c, 0.3) if l.startswith(key)]
        out[c] = ack[0] if ack else None
        t.send('S', 0.4)
    print('T1.4 clamps:', out); return out


def lag_for(period):
    t.pw('hallSamplePeriod', period); t.send('M100', 0.3); t.send('X1', 0.3)
    t.clear_x(); t.stream('L800 R800', 1.5); t.stream('L2000 R2000', 1.5); t.stream('L800 R800', 1.5)
    ser = side_series(t.x, 'L'); ps = pos_speed(ser, 40000); rep = [(x[0], x[2]) for x in ser]
    best = None
    for k in range(0, 400, 4):
        e = []
        for tt, v in ps:
            tr = tt + k * 1000
            n = min(rep, key=lambda q: abs(q[0] - tr))
            if abs(n[0] - tr) < 15000:
                e.append((n[1] - v) ** 2)
        if e and (best is None or st.mean(e) < best[1]):
            best = (k, st.mean(e))
    steady = [x[2] for x in ser[-40:]]
    t.send('S', 0.8)
    return dict(period=period, lag_ms=best[0] if best else None, reported_sd_rpm=round(st.pstdev(steady), 1))


def b8():
    orig = t.pr('hallSamplePeriod')
    out = [lag_for(p) for p in ('0.3125', '0.03125', '0.016', '0.3125')]
    t.pw('hallSamplePeriod', orig.get('L', '0.3125'))
    for o in out:
        print('T3.4/B8 depth 2:', o)
    print('   restored hallSamplePeriod', t.pr('hallSamplePeriod'))
    return out


res['hb0'] = hb0(); res['clamps'] = clamps(); res['b8'] = b8()
t.send('X0', 0.3); save('fw26_safety_b8', res)
