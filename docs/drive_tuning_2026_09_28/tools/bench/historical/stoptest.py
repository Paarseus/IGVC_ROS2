import glob, time, serial, re
port = glob.glob('/dev/serial/by-id/usb-Teensyduino*')[0]
s = serial.Serial(port, 115200, timeout=0.01)
def pump(d, rows=None):
    t = time.time(); buf = ''; out = []
    while time.time()-t < d:
        buf += s.read(8192).decode(errors='replace')
        while '\n' in buf:
            l, buf = buf.split('\n', 1); l = l.strip(); p = l.split()
            if rows is not None and p and p[0] == 'X' and len(p) >= 24:
                rows.append((time.time(), float(p[3]), float(p[7]), float(p[8]), float(p[11]), float(p[15]), float(p[16])))
            elif l: out.append(l)
    return out
def cmd(c, w=0.3): s.write((c+'\n').encode()); return pump(w)
def faults():
    d = [l for l in cmd('D', 0.4) if l.startswith('DIAG')]
    m = re.search(r' f=(\S+) sf=(\S+)', d[-1]) if d else None
    return m.group(2) if m else '?'
def trial(name, setup, stop_line, stop_repeat, teardown):
    for c in setup: cmd(c, 0.45)
    cmd('CF B', 0.5); cmd('X1')
    rows = []
    t0 = time.time()
    while time.time()-t0 < 2.5: s.write(b'L2000 R2000\n'); pump(0.05, rows)
    tS = time.time()
    while time.time()-tS < 2.0:
        s.write((stop_line+'\n').encode()); pump(0.05, rows)
        if not stop_repeat: stop_line = ''  # send once
    cmd('S', 0.3); cmd('X0')
    post = [r for r in rows if r[0] >= tS]
    res = []
    for side, (vi, ci, bi) in (('L', (1, 2, 3)), ('R', (4, 5, 6))):
        v = [(r[0]-tS, r[vi]) for r in post]
        rev = min(x[1] for x in v)                        # most negative speed (overshoot past zero)
        # settle: last time |v| > 50 RPM
        late = [t for t, x in v if abs(x) > 50]
        settle = late[-1] if late else 0.0
        zc = sum(1 for a, b in zip(v, v[1:]) if (a[1] > 30 and b[1] < -30) or (a[1] < -30 and b[1] > 30))
        res.append(f'{side}: overshoot {rev:5.0f} RPM, settle {settle:4.2f}s, reversals {zc}, peak {max(r[ci] for r in post):4.0f}A, minV {min(r[bi] for r in post):5.2f}')
    fl = faults()
    for c in teardown: cmd(c, 0.45)
    print(f'{name:38s} | ' + ' | '.join(res) + f' | stickyDRV {fl}')
BASE = ['KF0.000197', 'KP0.0007', 'KI2.5e-07', 'KD0.0', 'KZ600', 'M100', 'PW B idleMode 0', 'PW B hallAvgDepth 3', 'PW B closedLoopRamp 0']
trial('A  S (instant vel 0)  [current]', BASE, 'S', False, [])
trial('B  ramped L0 R0 via Teensy slew', BASE, 'L0 R0', True, [])
trial('C  S + hall filter depth 1', BASE + ['PW B hallAvgDepth 1'], 'S', False, ['PW B hallAvgDepth 3'])
trial('D  ramped L0 + hall depth 1', BASE + ['PW B hallAvgDepth 1'], 'L0 R0', True, ['PW B hallAvgDepth 3'])
trial('E  S + closedLoopRamp 2 (0.5 s)', BASE + ['PW B closedLoopRamp 2'], 'S', False, ['PW B closedLoopRamp 0'])
trial('F  duty 0 + BRAKE idle', BASE + ['PW B idleMode 1'], 'UL0 UR0', True, ['PW B idleMode 0'])
trial('G  duty 0 + COAST idle', BASE, 'UL0 UR0', True, [])
for c in BASE: cmd(c, 0.45)
print('restored:', [l for l in cmd('PR B hallAvgDepth', 0.5) if l.startswith('PRD')][:1])
