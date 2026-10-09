#!/usr/bin/env python3
"""Bench test of the reference-standard drive setup (RAM only; restores the original settings at the end).

Standard per research: feedforward first, P below oscillation, no I (C1 practice 18); Brake idle (C3-S48);
REV smart current limit 40-60 A (C1 practice 13); stop = controlled ramp to zero, then idle/brake at
standstill (IEC 61800-5-2 SS1 pattern; Nav2 smoother / diff_drive ramp to zero; C3 section 7).

For each P value: speed steps 0 -> 1000 -> 2000 RPM (overshoot, steady error, ringing) and three stops from
2000 RPM: (1) instant S, (2) SS1 = ramp L0 R0 through the Teensy slew then duty 0 (brake), (3) immediate duty 0 + brake.
"""
import glob, re, time
import serial

port = glob.glob('/dev/serial/by-id/usb-Teensyduino*')[0]
s = serial.Serial(port, 115200, timeout=0.01)


def pump(d, rows=None):
    t = time.time(); buf = ''; out = []
    while time.time() - t < d:
        buf += s.read(8192).decode(errors='replace')
        while '\n' in buf:
            l, buf = buf.split('\n', 1); l = l.strip(); p = l.split()
            if rows is not None and p and p[0] == 'X' and len(p) >= 24:
                # host t, L vel, L current, L busV, R vel, R current, R busV
                rows.append((time.time(), float(p[3]), float(p[7]), float(p[8]), float(p[11]), float(p[15]), float(p[16])))
            elif l:
                out.append(l)
    return out


def cmd(c, w=0.35):
    s.write((c + '\n').encode()); return pump(w)


def sticky():
    d = [l for l in cmd('D', 0.4) if l.startswith('DIAG')]
    m = re.search(r' f=(\S+) sf=(\S+)', d[-1]) if d else None
    return m.group(2) if m else '?'


def run(segments):
    """segments: list of (line, secs, resend). Returns rows and segment start times."""
    rows, starts = [], []
    for line, secs, resend in segments:
        starts.append(time.time())
        t0 = time.time(); sent = False
        while time.time() - t0 < secs:
            if line and (resend or not sent):
                s.write((line + '\n').encode()); sent = True
            pump(0.05, rows)
    return rows, starts


def stats(rows, t0, t1, target):
    out = []
    for side, vi, ci, bi in (('L', 1, 2, 3), ('R', 4, 5, 6)):
        seg = [r for r in rows if t0 <= r[0] <= t1]
        if not seg:
            out.append(f'{side}: no data'); continue
        v = [r[vi] for r in seg]
        tail = [r[vi] for r in seg if r[0] >= t1 - 0.8]
        mean_tail = sum(tail) / len(tail)
        if target:
            over = 100 * (max(v) - target) / target
            out.append(f'{side}: steady {mean_tail:6.0f} ({100 * (mean_tail - target) / target:+5.1f}%) overshoot {over:+5.1f}% '
                       f'ripple ±{(max(tail) - min(tail)) / 2:4.0f}')
        else:
            rev = min(v)
            late = [r[0] - t0 for r in seg if abs(r[vi]) > 50]
            settle = late[-1] if late else 0.0
            zc = sum(1 for a, b in zip(v, v[1:]) if (a > 30 and b < -30) or (a < -30 and b > 30))
            out.append(f'{side}: overshoot {rev:5.0f} rev {zc:2d} settle {settle:4.2f}s peak {max(r[ci] for r in seg):3.0f}A '
                       f'minV {min(r[bi] for r in seg):5.2f}')
    return ' | '.join(out)


ORIG = ['KF0.000197', 'KP0.0007', 'KI2.5e-07', 'KD0.0', 'KZ600', 'M100',
        'PW B idleMode 0', 'PW B smartStallA 80', 'PW B smartFreeA 20', 'PW B hallAvgDepth 3', 'PW B closedLoopRamp 0']
STANDARD = ['KF0.000197', 'KI0', 'KD0', 'KZ0', 'M100', 'PW B idleMode 1', 'PW B smartStallA 50', 'PW B smartFreeA 50',
            'PW B hallAvgDepth 3', 'PW B closedLoopRamp 0']

print('=== setup, current (for comparison): P 0.0007, I 2.5e-7, coast, 80 A')
for c in ORIG:
    cmd(c, 0.45)
configs = [('CURRENT P=0.0007 I=2.5e-7 coast', None)] + [(f'STANDARD P={p} I=0 brake 50A', p)
                                                        for p in ('0.0001', '0.0002', '0.0004', '0.0007')]
cmd('X1')
for name, p in configs:
    if p is not None:
        for c in STANDARD + [f'KP{p}']:
            cmd(c, 0.45)
    cmd('CF B', 0.5)
    rows, st = run([('L1000 R1000', 2.5, True), ('L2000 R2000', 2.5, True)])
    print(f'\n### {name}')
    print('  step 1000 :', stats(rows, st[0], st[0] + 2.5, 1000))
    print('  step 2000 :', stats(rows, st[1], st[1] + 2.5, 2000))
    for stop_name, segs in (
            ('stop S (instant)      ', [('L2000 R2000', 2.0, True), ('S', 2.0, False)]),
            ('stop SS1 ramp+brake   ', [('L2000 R2000', 2.0, True), ('L0 R0', 0.6, True), ('UL0 UR0', 1.4, True)]),
            ('stop duty0 + idle     ', [('L2000 R2000', 2.0, True), ('UL0 UR0', 2.0, True)])):
        rows, st = run(segs + [('S', 0.3, False)])
        print(f'  {stop_name}:', stats(rows, st[1], st[1] + 2.0, 0))
    print('  sticky faults after:', sticky())
cmd('X0'); cmd('S')
for c in ORIG:
    cmd(c, 0.45)
print('\nrestored original settings:', [l for l in cmd('PR B idleMode', 0.5) if l.startswith('PRD')][:1])
