#!/usr/bin/env python3
"""T4.2 / T4.4 on FW 26: which stop method trips DRV / SENSOR sticky faults, and how clean is each stop.
Tracks off the ground; baseline gains (yaml) + standard RAM; faults cleared before every trial; 3 repeats.
Methods from speed V: 'brake' = S (duty 0, Brake idle); 'coast' = S with Coast idle; 'ramp_brake' = ramp the
velocity target to 0 through the Teensy slew (M100 = 5000 RPM/s) then S (Brake) = the actuator_node pattern."""
import re, statistics as st
from tio import Teensy, standard, restore, save

t = Teensy()


def faults():
    d = t.diag()
    m = re.search(r' f=(\S+) sf=(\S+)', d)
    return m.group(2) if m else '?'


def trial(speed, method):
    standard(t); t.send('M100', 0.3)
    t.pw('idleMode', 0 if method == 'coast' else 1)
    t.send('CF B', 0.5); t.send('X1', 0.3)
    t.stream(f'L{speed} R{speed}', 2.5 + speed / 5000)       # reach speed through the 5000 RPM/s ramp
    t.clear_x(); t0 = None
    if method == 'ramp_brake':
        t.stream('L0 R0', speed / 5000 + 0.1)
    t.stream('S', 2.0)
    rows = t.x
    peakI = max(max(r['LI'], r['RI']) for r in rows) if rows else None
    minV = min(min(r['LV'], r['RV']) for r in rows) if rows else None
    # time until both |speed| < 50 RPM (reported speed; relative ordering between methods only)
    h0 = rows[0]['h'] if rows else 0
    moving = [r['h'] - h0 for r in rows if abs(r['Lv']) > 50 or abs(r['Rv']) > 50]
    rev = min(min(r['Lv'], r['Rv']) for r in rows) if rows else None
    sf = faults()
    return dict(speed=speed, method=method, stop_s=round(moving[-1], 2) if moving else 0.0, peak_A=round(peakI, 1),
                min_busV=round(minV, 2), min_rpm=round(rev), sticky=sf)


res = []
for speed in (2000, 3500):
    for method in ('ramp_brake', 'coast', 'brake'):
        for rep in range(3):
            r = trial(speed, method); r['rep'] = rep; res.append(r); print(r)
t.send('CF B', 0.5); restore(t)
save('fw26_stop_faults', res)
print('\nSUMMARY (fault trials / 3):')
for speed in (2000, 3500):
    for method in ('ramp_brake', 'coast', 'brake'):
        rr = [r for r in res if r['speed'] == speed and r['method'] == method]
        bad = sum(1 for r in rr if r['sticky'] not in ('0x00/0x00',))
        print(f'  {speed} RPM {method:10s}: faults in {bad}/3, stop {st.mean(r["stop_s"] for r in rr):.2f}s, '
              f'peak {max(r["peak_A"] for r in rr)} A, min bus {min(r["min_busV"] for r in rr)} V')
