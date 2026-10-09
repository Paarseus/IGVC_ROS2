import glob, re, time, serial, sys
sys.path.insert(0, '/tmp')
port = glob.glob('/dev/serial/by-id/usb-Teensyduino*')[0]
s = serial.Serial(port, 115200, timeout=0.01)
def pump(d):
    t = time.time(); buf = ''
    while time.time() - t < d: buf += s.read(8192).decode(errors='replace')
    return [l.strip() for l in buf.split('\n') if l.strip()]
def cmd(c, w=0.3): s.write((c+'\n').encode()); return pump(w)
def stream(line, secs, then=None, tsecs=0):
    out = []; t0 = time.time()
    while time.time()-t0 < secs: s.write((line+'\n').encode()); out += pump(0.05)
    t1 = time.time()
    while then and time.time()-t1 < tsecs: s.write((then+'\n').encode()); out += pump(0.05)
    s.write(b'S\n'); out += pump(0.8); return out
def faults():
    d = [l for l in cmd('D', 0.4) if l.startswith('DIAG')]
    m = re.search(r'f=(\S+) sf=(\S+)', d[-1]) if d else None
    return m.groups() if m else ('?', '?')
def pw(n, v): cmd(f'PW B {n} {v}', 0.45)
cmd('CF B', 0.6); print('after CF:', faults())
tests = [
 ('duty 0.25 then duty 0 (coast)',  lambda: (pw('idleMode',0), stream('UL0.25 UR0.25',2.5,'UL0 UR0',2))),
 ('duty 0.25 then duty 0 (brake)',  lambda: (pw('idleMode',1), stream('UL0.25 UR0.25',2.5,'UL0 UR0',2), pw('idleMode',0))),
 ('duty 0.25 then S (vel 0, coast)', lambda: stream('UL0.25 UR0.25',2.5)),
 ('vel 2000 slew off then S',       lambda: (cmd('M10000'), stream('L2000 R2000',2.5), cmd('M100'))),
 ('vel 3000 then S (M100)',         lambda: stream('L3000 R3000',3.0)),
 ('vel +1500 -> -1500 reversal',    lambda: (stream('L1500 R1500',2.0,'L-1500 R-1500',2.0))),
 ('outputMax 0.1 at L3000',         lambda: (pw('outputMax',0.1), stream('L3000 R3000',2.0), pw('outputMax',1))),
 ('HB0 during duty 0.06',           lambda: stream('UL0.06 UR0.06',2.0,'HB0',0.05)),
]
for name, fn in tests:
    out = fn()
    flines = [l for l in (out if isinstance(out, list) else sum([o for o in out if isinstance(o, list)], [])) if l.startswith('F ')]
    time.sleep(0.3)
    print(f'{name:36s} -> f/sf = {faults()} {flines}')
    if faults()[1] not in ('0x00/0x00',):
        cmd('CF B', 0.6)
cmd('HB1'); cmd('S')
