"""Shared Teensy v2/v2b serial harness for the isolated bench scopes (MPPI_READINESS_TEST_PLAN.md)."""
import glob, json, os, re, statistics as st, time
import serial

OUT = os.path.expanduser('~/bench_scopes_2026_09_28')
os.makedirs(OUT, exist_ok=True)
X_KEYS = ['us', 'Lv', 'Lp', 'Lrx', 'La', 'LI', 'LV', 'LT', 'Rv', 'Rp', 'Rrx', 'Ra', 'RI', 'RV', 'RT', 'spL', 'spR', 'spLus', 'spRus']


class Teensy:
    def __init__(self):
        self.s = serial.Serial(glob.glob('/dev/serial/by-id/usb-Teensyduino*')[0], 115200, timeout=0.005)
        self.buf = ''
        self.x = []        # dicts with host 'h' + X fields
        self.lines = []    # (host t, line)

    def pump(self, d):
        t_end = time.time() + d
        while True:
            self.buf += self.s.read(16384).decode(errors='replace')
            while '\n' in self.buf:
                l, self.buf = self.buf.split('\n', 1)
                h = time.time(); l = l.strip()
                p = l.split()
                if p and p[0] == 'X' and len(p) >= 24:
                    try:
                        v = [float(p[1])] + [float(q) for q in p[3:10]] + [float(q) for q in p[11:18]] + [float(q) for q in p[19:23]]
                        r = dict(zip(X_KEYS, v)); r['h'] = h; r['mode'] = p[23]
                        # v2d appended fields: I7 <L> <R> SP8 <L_sp> <R_sp> <L_atsp> <R_atsp>
                        if len(p) >= 32 and p[24] == 'I7' and p[27] == 'SP8':
                            try:
                                r.update(LI7=float(p[25]), RI7=float(p[26]), Lsp8=float(p[28]), Rsp8=float(p[29]),
                                         Lat=int(p[30]), Rat=int(p[31]))
                            except ValueError:
                                pass
                        self.x.append(r)
                    except ValueError:
                        pass
                elif l and not l.startswith('E L'):
                    self.lines.append((h, l))
            if time.time() >= t_end:
                return

    def send(self, c, w=0.3):
        self.s.write((c + '\n').encode()); t0 = time.time(); n0 = len(self.lines)
        self.pump(w)
        return [l for (h, l) in self.lines[n0:]]

    def stream(self, c, secs, period=0.05):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.s.write((c + '\n').encode()); self.pump(period)

    def pw(self, name, val, side='B'):
        out = self.send(f'PW {side} {name} {val}', 0.45)
        ok = [l for l in out if l.startswith('PWR') and ' res=0 ' in l]
        return len(ok) == (2 if side == 'B' else 1)

    def pr(self, name):
        out = self.send(f'PR B {name}', 0.4)
        vals = {}
        for l in out:
            m = re.match(r'PRD ([LR]) id=\d+ type=\w val=(\S+)', l)
            if m:
                vals[m.group(1)] = m.group(2)
        return vals

    def diag(self):
        for _ in range(4):
            d = [l for l in self.send('D', 0.6) if 'DIAG' in l]
            if d:
                return d[-1][d[-1].index('DIAG'):]
        return ''

    def sticky(self):
        m = re.search(r' f=(\S+) sf=(\S+)', self.diag())
        return m.group(2) if m else '?'

    def clear_x(self):
        self.x = []


def side_series(rows, s):
    """Unique STATUS_2 samples for one side: (teensy rx us, pos rot, reported rpm, applied, current, busV)."""
    seen, out = set(), []
    for r in rows:
        k = r[s + 'rx']
        if k in seen:
            continue
        seen.add(k); out.append((k, r[s + 'p'], r[s + 'v'], r[s + 'a'], r[s + 'I'], r[s + 'V']))
    return out


def pos_speed(series, win_us=60000):
    """Central-difference speed (RPM) from position over +/- win/2, at each sample."""
    out = []
    for i, (t, p, *_rest) in enumerate(series):
        j0 = i
        while j0 > 0 and t - series[j0 - 1][0] <= win_us / 2:
            j0 -= 1
        j1 = i
        while j1 + 1 < len(series) and series[j1 + 1][0] - t <= win_us / 2:
            j1 += 1
        if series[j1][0] > series[j0][0]:
            out.append((t, (series[j1][1] - series[j0][1]) / ((series[j1][0] - series[j0][0]) / 1e6) * 60))
    return out


def pct(a, q):
    a = sorted(a)
    return a[min(len(a) - 1, int(q * (len(a) - 1) + 0.5))] if a else float('nan')


def save(name, obj):
    with open(os.path.join(OUT, name + '.json'), 'w') as f:
        json.dump(obj, f, indent=1, default=str)


# = the final BURNed configuration (2026-09-28)
STANDARD_RAM = [('idleMode', 1), ('smartStallA', 50), ('smartFreeA', 50), ('hallAvgDepth', 2), ('hallSamplePeriod', 0.016), ('closedLoopRamp', 0)]


# FW 26.1.5 final configuration = actuator_params.yaml + controller flash (2026-09-28)
KV_FW26 = 0.0023


def standard(t, p=0.0002):
    for c in (f'KF{KV_FW26}', f'KP{p}', 'KI0', 'KD0', 'KZ0', 'KS0', 'M10000'):
        t.send(c, 0.35)
    for n, v in STANDARD_RAM:
        t.pw(n, v)


def restore(t):
    for c in (f'KF{KV_FW26}', 'KP0.0002', 'KI0', 'KD0', 'KZ0', 'KS0', 'M100', 'X0', 'S'):
        t.send(c, 0.35)
    # final BURNed configuration (2026-09-28)
    for n, v in (('idleMode', 1), ('smartStallA', 50), ('smartFreeA', 50), ('hallAvgDepth', 2), ('hallSamplePeriod', 0.016),
                 ('closedLoopRamp', 0), ('status2Period', 20), ('kS', 0), ('kA', 0)):
        t.pw(n, v)
