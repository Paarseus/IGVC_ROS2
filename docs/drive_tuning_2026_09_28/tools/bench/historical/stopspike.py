import glob, time, serial
port = glob.glob('/dev/serial/by-id/usb-Teensyduino*')[0]
s = serial.Serial(port, 115200, timeout=0.01)
def pump(d, rows):
    t = time.time(); buf = ''
    while time.time()-t < d:
        buf += s.read(8192).decode(errors='replace')
        while '\n' in buf:
            l, buf = buf.split('\n', 1); p = l.split()
            if p and p[0] == 'X' and len(p) >= 24:
                rows.append((time.time(), float(p[3]), float(p[6]), float(p[7]), float(p[8]), float(p[11]), float(p[14]), float(p[15])))
            elif l.startswith('F '): rows.append(('F', l.strip()))
for c in ('CF B', 'X1', 'M100'): s.write((c+'\n').encode()); time.sleep(0.3)
rows = []; pump(0.3, [])
t0 = time.time()
while time.time()-t0 < 2.5: s.write(b'L2000 R2000\n'); pump(0.05, rows)
tS = time.time(); s.write(b'S\n'); pump(1.2, rows)
s.write(b'X0\n')
print('t_rel  L_rpm  L_app  L_A   L_busV | R_rpm  R_A  R_busV')
for r in rows:
    if r[0] == 'F': print('   ', r[1]); continue
    t = r[0]-tS
    if -0.2 <= t <= 0.8: print(f'{t:5.2f} {r[1]:6.0f} {r[2]:6.3f} {r[3]:5.1f} {r[4]:6.2f} | {r[5]:6.0f} {r[6]:5.1f} {r[7]:6.2f}')
