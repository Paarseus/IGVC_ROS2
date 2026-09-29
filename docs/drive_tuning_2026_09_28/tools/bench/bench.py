#!/usr/bin/env python3
"""Send serial commands to the Teensy v2 bridge and print replies (E lines hidden). '#sleep N' pauses."""
import glob, sys, time
import serial
port = glob.glob('/dev/serial/by-id/usb-Teensyduino*')[0]
s = serial.Serial(port, 115200, timeout=0.05)
for c in sys.argv[1:]:
    if c.startswith('#sleep'):
        time.sleep(float(c.split()[1])); continue
    s.write((c + '\n').encode())
    t = time.time(); buf = ''
    while time.time() - t < 0.8:
        buf += s.read(8192).decode(errors='replace')
    print(f'>>> {c}')
    print('\n'.join(l for l in buf.splitlines() if l and not l.startswith(('E L', 'X '))))
