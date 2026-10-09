#!/usr/bin/env python3
"""Bench acceptance, firmware and motor layer (tracks off the ground; actuator_node stopped).
Each check has a fixed pass limit. Final configuration is restored at the end.
Writes ~/bench_scopes_2026_09_28/acceptance_serial.json and prints a pass/fail table."""
import json, re, statistics as st, time
from tio import Teensy, side_series, pos_speed, pct, save

t = Teensy()
R = []   # (id, check, measured, limit, pass)


def add(i, check, measured, limit, ok):
    R.append(dict(id=i, check=check, measured=measured, limit=limit, passed=bool(ok)))
    print(f"{i:4s} {'PASS' if ok else 'FAIL'}  {check}: {measured}  (limit: {limit})")


def final_config():
    for c in ('KF0.0023', 'KP0.0002', 'KI0', 'KD0', 'KZ0', 'KS0', 'M100', 'S'):
        t.send(c, 0.3)


def sticky():
    m = re.search(r' f=(\S+) sf=(\S+)', t.diag())
    return m.group(2) if m else '?'


final_config(); t.send('CF B', 0.5)

# A1 configuration check
out = t.send('CHK', 3.5)
add('A1', 'configuration check (CHK)', 'CHK OK' if any(l.startswith('CHK OK') for l in out) else
    next((l for l in out if l.startswith('CHK FAIL')), 'no result'), 'CHK OK', any(l.startswith('CHK OK') for l in out))

# A2 timing
t.send('X1', 0.3); t.clear_x(); t.stream('L1000 R1000', 6.0); t.send('S', 0.6)
d2, dsp = [], []
for s in 'LR':
    rx = sorted(set(r[s + 'rx'] for r in t.x)); d2 += [(b - a) / 1000 for a, b in zip(rx, rx[1:])]
    sp = sorted(set(r['sp' + s + 'us'] for r in t.x)); dsp += [(b - a) / 1000 for a, b in zip(sp, sp[1:])]
gaps = sum(1 for x in d2 if x > 30)
add('A2a', 'speed/position report interval', f'mean {st.mean(d2):.1f} ms, {gaps} gaps > 30 ms', '20 +/- 1 ms, 0 gaps',
    abs(st.mean(d2) - 20) <= 1 and gaps == 0)
add('A2b', 'speed command interval', f'mean {st.mean(dsp):.1f} ms, max {max(dsp):.1f} ms', '20 ms, max <= 25 ms',
    abs(st.mean(dsp) - 20) <= 1 and max(dsp) <= 25)
t.clear_x(); t.stream('L800 R800', 2.0); t_last = time.time(); t.pump(1.0)
idle = [r for r in t.x if r['h'] >= t_last and r['mode'] == 'D']
wd = (idle[0]['h'] - t_last) * 1000 if idle else None
add('A2c', 'watchdog stop after commands stop', f'{wd:.0f} ms' if wd else 'no stop', '250-350 ms', wd is not None and 250 <= wd <= 350)
t.send('S', 0.5)

# A3 disable (HB0)
t.clear_x(); t0 = time.time(); hb = None
while time.time() - t0 < 5.0:
    t.s.write(b'UL0.06 UR0.06\n'); t.pump(0.05)
    if hb is None and time.time() - t0 > 2.0:
        t.s.write(b'HB0\n'); hb = time.time()
t.send('HB1', 0.3); t.send('S', 0.5)
w = [r for r in t.x if hb + 0.3 < r['h'] < hb + 2.8]
spd = max(max(abs(r['Lv']), abs(r['Rv'])) for r in w); cur = max(max(r['LI'], r['RI']) for r in w)
add('A3', 'disable test: power streamed, heartbeat disabled', f'max speed {spd:.0f} RPM, max current {cur:.1f} A',
    'speed <= 10 RPM, current <= 0.5 A', spd <= 10 and cur <= 0.5)

# A4 input limits
acks = {c: next((l for l in t.send(c, 0.3) if l.startswith('OK')), '') for c in ('L9000 R9000', 'UL0.9 UR0.9', 'UVL20 UVR20', 'Lnan Rnan')}
t.send('S', 0.5)
ok = ('4600' in acks['L9000 R9000'] and '0.300' in acks['UL0.9 UR0.9'] and '3.600' in acks['UVL20 UVR20'] and 'L=0 R=0' in acks['Lnan Rnan'])
add('A4', 'input limits (speed, power, voltage, invalid number)', '; '.join(acks.values()), '4600 RPM / 0.30 / 3.6 V / 0', ok)

# A5 speed accuracy at cruise speeds (final gains, ramped)
worst = 0; rip = 0
for rpm in (1000, 2000, 3500, -1000, -2000, -3500):
    t.clear_x(); t.stream(f'L{rpm} R{rpm}', 3.5 + abs(rpm) / 5000)
    us1 = t.x[-1]['us']
    for s in 'LR':
        ser = [x for x in side_series(t.x, s) if x[0] >= us1 - 1.5e6]
        ps = [v for _, v in pos_speed(ser)]
        worst = max(worst, abs(100 * (st.mean(ps) - rpm) / rpm)); rip = max(rip, st.pstdev(ps))
    t.send('S', 1.0)
add('A5', 'speed accuracy 1000-3500 RPM, both directions (off the ground)', f'worst error {worst:.1f} %, worst noise sd {rip:.0f} RPM',
    'error <= 5 % off the ground (ground target 2 %), noise sd <= 40 RPM', worst <= 5 and rip <= 40)

# A6 low speed with kS (bench value 0.18 V). Requirement T7: slowest controlled speed 0.05 m/s = 150 RPM.
# A6a: 100-300 RPM within 10 %; A6b: 50 RPM (a third of T7) must not stick; its error is reported, not limited.
t.send('KS0.18', 0.3); worst, stuck, e50 = 0, 0, 0
for rpm in (50, 100, 150, 300, -100):
    t.clear_x(); t.stream(f'L{rpm} R{rpm}', 5.0)
    us1 = t.x[-1]['us']
    for s in 'LR':
        ser = [x for x in side_series(t.x, s) if x[0] >= us1 - 3.0e6]
        ps = [v for _, v in pos_speed(ser, 100000)]
        err = abs(100 * (st.mean(ps) - rpm) / rpm)
        if abs(rpm) == 50:
            e50 = max(e50, err)
        else:
            worst = max(worst, err)
        stuck = max(stuck, sum(1 for v in ps if abs(v) < 0.2 * abs(rpm)) / len(ps))
    t.send('S', 0.8)
t.send('KS0', 0.3)
add('A6a', 'slow speed 100-300 RPM (0.033-0.1 m/s) with kS 0.18 V', f'worst error {worst:.1f} %', 'error <= 10 %', worst <= 10)
add('A6b', 'crawl 50 RPM (0.017 m/s) with kS 0.18 V', f'never stuck: {stuck == 0}; error {e50:.0f} % (reported)', 'never stuck', stuck == 0)

# A7 braked stop from speed, A8 ramped start: faults and supply
t.send('CF B', 0.5); res = []
for spd_ in (2000, 3500):
    for _ in range(2):
        t.stream(f'L{spd_} R{spd_}', 2.0 + spd_ / 5000); t.clear_x(); t.stream('S', 1.5)
        h0 = t.x[0]['h']; mov = [r['h'] - h0 for r in t.x if abs(r['Lv']) > 50 or abs(r['Rv']) > 50]
        res.append((mov[-1] if mov else 0.0, min(min(r['LV'], r['RV']) for r in t.x)))
sf = sticky()
add('A7', 'braked stop from 2000/3500 RPM (4 stops)', f'slowest {max(x[0] for x in res):.2f} s, lowest supply {min(x[1] for x in res):.1f} V, faults {sf}',
    '<= 0.40 s, supply >= 10 V, no faults', max(x[0] for x in res) <= 0.4 and min(x[1] for x in res) >= 10 and sf == '0x00/0x00')
t.send('CF B', 0.5); t.clear_x(); t.stream('L3500 R3500', 2.0)
minv = min(min(r['LV'], r['RV']) for r in t.x); pk = max(max(r['LI'], r['RI']) for r in t.x)
t.stream('L0 R0', 1.0); t.send('S', 0.8); sf = sticky()
add('A8', 'ramped start 0 -> 3500 RPM', f'lowest supply {minv:.1f} V, peak {pk:.0f} A, faults {sf}', 'supply >= 9.5 V, no faults',
    minv >= 9.5 and sf == '0x00/0x00')

final_config(); t.send('X0', 0.3)
save('acceptance_serial', R)
print(f"\n{sum(r['passed'] for r in R)}/{len(R)} passed")
