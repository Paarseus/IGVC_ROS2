"""Read-only: live GNSS PVT (and sat info if enabled) + status from the MTi for N seconds."""
import sys, time
from threading import Lock
import xsensdeviceapi as xda
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30

class CB(xda.XsCallback):
    def __init__(s):
        xda.XsCallback.__init__(s); s.buf = []; s.lock = Lock()
    def onLiveDataAvailable(s, dev, packet):
        with s.lock:
            s.buf.append(xda.XsDataPacket(packet)); s.buf = s.buf[-400:]

c = xda.XsControl.construct()
p = next(p for p in xda.XsScanner.scanPorts() if p.deviceId().isMti() or p.deviceId().isMtig())
c.openPort(p.portName(), p.baudrate()); d = c.device(p.deviceId())
cb = CB(); d.addCallbackHandler(cb); d.gotoMeasurement()
FIX = {0: 'none', 1: 'DR', 2: '2D', 3: '3D', 4: 'GNSS+DR', 5: 'time'}
t0 = time.time(); lastp = None; sat = None; n = 0
while time.time() - t0 < secs:
    time.sleep(0.05)
    with cb.lock:
        pk, cb.buf = cb.buf, []
    for q in pk:
        n += 1
        if q.containsRawGnssPvtData():
            lastp = q.rawGnssPvtData()
            f = lastp.m_flags
            carr = {0: 'none', 1: 'FLOAT', 2: 'FIXED'}.get((f >> 6) & 3, '?')
            print(f'{time.time()-t0:5.1f}s fix={FIX.get(lastp.m_fixType, lastp.m_fixType)} numSV={lastp.m_numSv} '
                  f'diffSoln={(f >> 1) & 1} carrier={carr} hAcc={lastp.m_hAcc/1000:.2f}m vAcc={lastp.m_vAcc/1000:.2f}m '
                  f'pDOP={lastp.m_pdop/100:.2f}', flush=True)
        if q.containsRawGnssSatInfo():
            sat = q.rawGnssSatInfo()
print('packets', n)
if sat is not None:
    print('satellites:', sat.m_numSvs)
d.gotoConfig(); c.closePort(p.portName()); c.destruct()
