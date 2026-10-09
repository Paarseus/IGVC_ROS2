#!/usr/bin/env python3
"""FW 26 isolated feedforward / telemetry tests (FW26_TEST_STRATEGY.md T2.3-T2.5, T3.5). Tracks off the ground,
RAM only, baseline restored after each test. Each test turns ONE term on from an all-zero controller.
usage: fw26_tests.py b3 | b5 | b4 | t35"""
import statistics as st, sys
from tio import Teensy, save, restore, KV_FW26

t = Teensy()


def zero_all():
    for c in ('KP0', 'KI0', 'KD0', 'KZ0', 'KF0', 'KS0', 'M100', 'X1'):
        t.send(c, 0.3)
    t.pw('kS', 0); t.pw('kA', 0)


def applied_during(cmd, secs, from_s=0.6):
    t.clear_x(); t.stream(cmd, secs)
    rows = [r for r in t.x if r['h'] >= t.x[0]['h'] + from_s] if t.x else []
    out = {}
    for s in 'LR':
        a = [r[s + 'a'] for r in rows]; v = [r[s + 'v'] for r in rows]; b = [r[s + 'V'] for r in rows]
        out[s] = dict(applied=round(st.mean(a), 4) if a else None, rpm=round(st.mean(v)) if v else None,
                      bus=round(st.mean(b), 2) if b else None)
    return out


def b3():
    """kS sign source and zero-setpoint behaviour: only param 204 = 0.30 V (below breakaway), all else 0."""
    zero_all(); t.pw('kS', 0.30)
    res = {}
    for cmd in ('L100 R100', 'L-100 R-100', 'L0 R0'):
        res[cmd] = applied_during(cmd, 1.5); t.send('S', 0.5)
    t.pw('kS', 0); restore(t)
    for k, v in res.items():
        print('B3', k.ljust(12), v, ' expected applied = +-0.30/Vbus = +-0.025; at L0: +0.025 (SIM) or 0')
    save('fw26_b3_ks_sign', res)


def b5():
    """arbFF kS + param kS: additive or override? 204 = 0.30 and Teensy KS 0.30 together, L100."""
    zero_all()
    res = {}
    t.pw('kS', 0.30); res['param204_only'] = applied_during('L100 R100', 1.5); t.send('S', 0.5)
    t.pw('kS', 0); t.send('KS0.30', 0.3); res['arbFF_only'] = applied_during('L100 R100', 1.5); t.send('S', 0.5)
    t.pw('kS', 0.30); res['both'] = applied_during('L100 R100', 1.5); t.send('S', 0.5)
    t.pw('kS', 0); t.send('KS0', 0.3); restore(t)
    for k, v in res.items():
        print('B5', k.ljust(14), v)
    print('   additive if "both" applied ~= sum of the two singles (~0.05)')
    save('fw26_b5_ks_additive', res)


def b4():
    """kA in plain velocity mode: only param 205 = 0.001 V/(RPM/s), setpoint ramped 1000 RPM/s (M20)."""
    zero_all(); t.pw('kA', 0.001); t.send('M20', 0.3)
    t.clear_x(); t.stream('L1000 R1000', 2.0)
    ramp = [r for r in t.x if 50 < r['spL'] < 950]
    plateau = [r for r in t.x if r['spL'] >= 999]
    res = dict(during_ramp_applied=round(st.mean(r['La'] for r in ramp), 4) if ramp else None,
               plateau_applied=round(st.mean(r['La'] for r in plateau), 4) if plateau else None,
               ramp_samples=len(ramp))
    t.send('S', 0.6); t.pw('kA', 0); restore(t)
    print('B4 kA only:', res, ' ignored -> ~0 during the ramp; applied -> ~1 V/Vbus = ~0.08 during the ramp')
    save('fw26_b4_ka', res)


def t35():
    """STATUS_7 / STATUS_8 decode: setpoint and at-setpoint follow the command; I accumulator 0 with kI 0, grows with kI."""
    res = {}
    for c in (f'KF{KV_FW26}', 'KP0.0003', 'KI0', 'KD0', 'KZ0', 'M100', 'X1'):
        t.send(c, 0.3)
    for label, ki in (('kI=0', '0'), ('kI=1e-7', '1e-7')):
        t.send(f'KI{ki}', 0.4)
        t.clear_x(); t.stream('L800 R1200', 2.5)
        tail = [r for r in t.x[-20:] if 'LI7' in r]
        res[label] = dict(L_sp8=tail[-1]['Lsp8'] if tail else None, R_sp8=tail[-1]['Rsp8'] if tail else None,
                          L_at=tail[-1]['Lat'] if tail else None, R_at=tail[-1]['Rat'] if tail else None,
                          L_iacc=round(tail[-1]['LI7'], 6) if tail else None, R_iacc=round(tail[-1]['RI7'], 6) if tail else None,
                          L_rpm=round(st.mean(r['Lv'] for r in tail)) if tail else None)
        t.send('S', 0.8)
    restore(t)
    for k, v in res.items():
        print('T3.5', k, v)
    print('   pass: sp8 = 800 / 1200 (after the ramp); iacc 0 with kI 0 and nonzero with kI 1e-7')
    save('fw26_t35_status78', res)


if __name__ == '__main__':
    {'b3': b3, 'b5': b5, 'b4': b4, 't35': t35}[sys.argv[1]]()
