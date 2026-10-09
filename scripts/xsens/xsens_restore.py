"""Restore the MTi configuration to the 2026-09-30 pre-update snapshot (FW 1.12.0), then 921600 baud."""
import sys, time, xsensdeviceapi as xda
PORT = '/dev/ttyUSB0'
OUTPUTS = [(0x1020, 65535), (0x1060, 65535), (0x1010, 65535), (0x4020, 100), (0x8020, 100), (0xC020, 100),
           (0x2010, 100), (0x4030, 100), (0x5042, 100), (0x5022, 100), (0xD012, 100), (0x7010, 4), (0xE020, 65535)]
OPTION_FLAGS = 0x1A20
c = xda.XsControl.construct()
if not c.openPort(PORT, xda.XBR_115k2): sys.exit('open failed')
d = c.device(c.deviceIds()[0])
assert d.gotoConfig(), 'gotoConfig failed'
def step(name, ok):
    print(f'{name:34s} {"OK" if ok else "FAILED"}'); 
    if not ok: sys.exit(f'stopped at: {name}')
arr = xda.XsOutputConfigurationArray()
for did, hz in OUTPUTS:
    arr.push_back(xda.XsOutputConfiguration(did, hz))
step('output configuration (13 items)', d.setOutputConfiguration(arr))
cur = d.deviceOptionFlags()
step('option flags -> 0x1A20', d.setDeviceOptionFlags(OPTION_FLAGS & ~cur, cur & ~OPTION_FLAGS))
v = xda.XsVector(3); v[0], v[1], v[2] = 0.74, 0.0, 0.0
step('GNSS lever arm -> [0.74, 0, 0]', d.setGnssLeverArm(v))
step('u-blox platform -> 4 (automotive)', d.setUbloxGnssPlatform(4))
g = xda.XsIntArray(); [g.push_back(x) for x in (3, 1, 4, 4)]
step('GNSS receiver settings -> [3,1,4,4]', d.setGnssReceiverSettings(g))
step('serial baud -> 921600', d.setSerialBaudRate(xda.XBR_921k6))
step('reset device', d.reset())
c.closePort(PORT); c.destruct()
print('written; reopen at 921600 to verify')
