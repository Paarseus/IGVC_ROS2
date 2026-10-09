"""Read-only inspection of the Xsens MTi: identity, firmware, filter, GNSS settings, output configuration."""
import sys, xsensdeviceapi as xda
c = xda.XsControl.construct()
port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
baud = {115200: xda.XBR_115k2, 921600: xda.XBR_921k6}[int(sys.argv[2]) if len(sys.argv) > 2 else 115200]
if not c.openPort(port, baud):
    sys.exit(f'could not open {port}')
ids = c.deviceIds()
if len(ids) == 0:
    sys.exit('port open but no device answered')
d = c.device(ids[0])
print('port', port, 'id', ids[0].toXsString())
d.gotoConfig()
def show(name, f):
    try:
        v = f()
        if hasattr(v, 'toXsString'): v = v.toXsString()
        print(f'{name:28s} {v}')
    except Exception as e:
        print(f'{name:28s} ERROR {e}')
show('product code', d.productCode)
show('firmware', d.firmwareVersion)
show('hardware', d.hardwareVersion)
show('onboard filter profile', lambda: d.onboardFilterProfile().label())
show('available filter profiles', lambda: [x.label() for x in d.availableOnboardFilterProfiles()])
show('ublox gnss platform', d.ubloxGnssPlatform)
show('gnss lever arm', lambda: [d.gnssLeverArm()[i] for i in range(3)])
show('gnss receiver settings', lambda: [d.gnssReceiverSettings()[i] for i in range(len(d.gnssReceiverSettings()))])
show('device option flags', lambda: hex(d.deviceOptionFlags()))
show('serial baud', d.serialBaudRate)
cfg = d.outputConfiguration()
print('output configuration:')
for i in range(cfg.size()):
    q = cfg[i]
    print(f'   0x{q.m_dataIdentifier:04x}  {q.m_frequency} Hz')
c.closePort(port); c.destruct()
