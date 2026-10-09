#!/usr/bin/env python3
"""Read one Teensy status line (the 'D' command). Read-only: no motion commands.

Only run while actuator_node is stopped (it owns the serial port).
Usage: python3 teensy_status.py [port]
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else \
    '/dev/serial/by-id/usb-Teensyduino_USB_Serial_20383890-if00'

s = serial.Serial(PORT, 115200, timeout=0.3)
time.sleep(0.5)
s.reset_input_buffer()
s.write(b'D\n')
t = time.time()
found = False
while time.time() - t < 2.0:
    line = s.readline().decode(errors='replace').strip()
    if line.startswith('DIAG'):
        print(line)
        found = True
        break
s.close()
if not found:
    print('ERROR: no DIAG line received')
    sys.exit(1)
