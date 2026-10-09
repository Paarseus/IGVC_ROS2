#!/usr/bin/env python3
"""Native kS (param 204) at a zero velocity setpoint: controlled variants to rule out test mistakes.
Tracks off the ground, RAM only; final config (yaml gains, 204 = 0) restored at the end.
Logs STATUS_0 applied, STATUS_2 speed and STATUS_8 (the setpoint the SPARK says it received)."""
import statistics as st, struct, time
from tio import Teensy, save

t = Teensy()
t.send('S', 0.5); t.send('X1', 0.3)


def gains(kv, kp):
    for c in (f'KF{kv}', f'KP{kp}', 'KI0', 'KD0', 'KZ0', 'KS0', 'M100'):
        t.send(c, 0.3)


def run(cmd, secs=2.0, pre=None):
    if pre:
        t.stream(pre, 1.5)
    t.clear_x(); t.stream(cmd, secs)
    tail = [r for r in t.x if r['h'] > t.x[0]['h'] + 0.8] if t.x else []
    def m(k): return round(st.mean(r[k] for r in tail if k in r), 4) if tail else None
    out = dict(cmd=cmd, pre=pre, applied_L=m('La'), applied_R=m('Ra'), rpm_L=round(m('Lv')), rpm_R=round(m('Rv')),
               sp8_L=m('Lsp8'), sp8_R=m('Rsp8'))
    t.send('S', 0.8)
    return out


res = []
# kS only
gains(0, 0); t.pw('kS', 0.30)
for cmd, pre in (('L0 R0', None),                 # V1 fresh (after S / duty idle)
                 ('L0 R0', 'L100 R100'),           # V2 after +100
                 ('L0 R0', 'L-100 R-100'),         # V3 after -100
                 ('L-0 R-0', None),                # V4 negative zero float (0x80000000)
                 ('L0.001 R0.001', None),          # V5 tiny +
                 ('L-0.001 R-0.001', None)):       # V5 tiny -
    r = run(cmd, pre=pre); r['case'] = 'kS only (204=0.30, kV=P=0)'; res.append(r); print(r)
# V6: realistic gains + native kS
gains(0.0023, 0.0002)
for cmd, pre in (('L0 R0', None), ('L0 R0', 'L300 R300')):
    r = run(cmd, pre=pre); r['case'] = 'kS 0.30 + kV 0.0023 + P 0.0002'; res.append(r); print(r)
# control: same, native kS = 0
t.pw('kS', 0)
r = run('L0 R0', pre='L300 R300'); r['case'] = 'CONTROL kS=0, kV 0.0023, P 0.0002'; res.append(r); print(r)
# restore final config
gains(0.0023, 0.0002); t.pw('kS', 0); t.send('X0', 0.3); t.send('S', 0.3)
print('restored kS=0:', t.pr('kS'))
print('bytes sent for L0: ', struct.pack('<f', 0.0).hex(), ' for L-0:', struct.pack('<f', -0.0).hex(), '(if the firmware passes -0.0 through)')
save('ks_zero_variants', res)
