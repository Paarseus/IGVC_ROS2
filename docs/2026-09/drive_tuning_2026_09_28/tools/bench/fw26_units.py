#!/usr/bin/env python3
"""FW 26 feedforward-units test (tracks off the ground, RAM only, restored after).
P = I = D = 0, arbFF kS 0, native kS 0; kV at 0.000197 (FW 25 duty/RPM value) and 0.00236 (= 12 x) and 0.00211 (12/5676).
Hypothesis volts/RPM: 0.000197 -> 0.197 V at 1000 RPM (~1.6 % duty, below friction) -> ~0 RPM; 0.00236 -> ~1000 RPM.
Hypothesis duty/RPM (unchanged from FW 25): 0.000197 -> ~1000 RPM; 0.00236 -> saturates. Hence the 1000 RPM command and a 1 s cap."""
import statistics as st
from tio import Teensy, side_series, pos_speed, save

t = Teensy()
for c in ('KP0', 'KI0', 'KD0', 'KZ0', 'KS0', 'M100', 'X1'):
    t.send(c, 0.35)
t.pw('kS', 0)
res = []
for kv, secs in (('0.000197', 3.0), ('0.00211', 3.0), ('0.00236', 3.0)):
    if kv != '0.000197' and res and res[0]['rpm'] is not None and res[0]['rpm'] > 500:
        print('SKIP', kv, ': 0.000197 already gives ~1000 RPM (duty/RPM units) - a 12x kV would saturate'); continue
    t.send(f'KF{kv}', 0.45)
    t.clear_x(); t.stream('L1000 R1000', secs)
    rows = t.x; us1 = rows[-1]['us'] if rows else 0
    for s in 'LR':
        ser = [x for x in side_series(rows, s) if x[0] >= us1 - 1.0e6]
        ps = [v for _, v in pos_speed(ser)]
        app = [x[3] for x in ser]; bus = [x[5] for x in ser]
        r = dict(kV=kv, side=s, rpm=round(st.mean(ps)) if ps else None, applied_duty=round(st.mean(app), 4) if app else None,
                 bus_V=round(st.mean(bus), 2) if bus else None)
        if app and bus and ps and abs(st.mean(ps)) > 50:
            r['volts_applied'] = round(st.mean(app) * st.mean(bus), 3)
            r['volts_per_rpm_applied'] = round(st.mean(app) * st.mean(bus) / st.mean(ps), 6)
        res.append(r); print(r)
    t.send('S', 1.0)
for c in ('KF0.000197', 'KP0.0007', 'KI2.5e-07', 'KD0.0', 'KZ600', 'X0', 'S'):
    t.send(c, 0.35)
save('fw26_units', res)
