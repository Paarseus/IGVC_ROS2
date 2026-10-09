"""Read-only Xsens GNSS diagnostic. Writes NOTHING to the device.

Prints device info and GNSS-related settings, then streams 60 s of
measurement data and summarises GNSS PVT (satellites used, fix type) and,
if the device already outputs it, per-satellite info.
"""
import sys
import time
from collections import Counter, defaultdict

import xsensdeviceapi as xda

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0


class Cb(xda.XsCallback):
    def __init__(self):
        super().__init__()
        self.packets = []

    def onLiveDataAvailable(self, dev, packet):
        self.packets.append(xda.XsDataPacket(packet))


def show(label, fn):
    try:
        print(f'{label}: {fn()}')
    except Exception as e:  # noqa: BLE001 - diagnostic only
        print(f'{label}: <unavailable: {e}>')


ctrl = xda.XsControl_construct()
try:
    ports = xda.XsScanner_scanPorts()
    port = next((p for p in ports if p.portName() == PORT), None)
    if port is None or port.empty():
        port = xda.XsScanner_scanPort(PORT, xda.XBR_921k6)
    if port.empty():
        sys.exit(f'No Xsens device found on {PORT}')
    print(f'Found {port.deviceId().toXsString()} on {port.portName()} @ {port.baudrate()}')
    if not ctrl.openPort(port.portName(), port.baudrate()):
        sys.exit('Could not open port')
    dev = ctrl.device(port.deviceId())

    print('\n== Device and GNSS settings (read only) ==')
    show('Product code', lambda: dev.productCode())
    show('Firmware', lambda: dev.firmwareVersion().toXsString())
    show('Onboard filter profile', lambda: dev.onboardFilterProfile().label())
    flags = dev.deviceOptionFlags()
    names = [n for n in dir(xda) if n.startswith('XDOF_') and n not in ('XDOF_None', 'XDOF_All')]
    show('Option flags (raw)', lambda: hex(int(flags)))
    show('Option flags set', lambda: [n for n in names if int(getattr(xda, n)) and (int(flags) & int(getattr(xda, n))) == int(getattr(xda, n))])
    show('u-blox platform', lambda: int(dev.ubloxGnssPlatform()))
    show('GNSS lever arm', lambda: [dev.gnssLeverArm()[i] for i in range(3)])
    try:
        rs = dev.gnssReceiverSettings()
        print('GNSS receiver settings:', {k: getattr(rs, k) for k in dir(rs) if k.startswith('m_')})
    except Exception as e:  # noqa: BLE001
        print(f'GNSS receiver settings: <unavailable: {e}>')
    oc = dev.outputConfiguration()
    print('Output configuration:')
    has_satinfo = False
    for i in range(oc.size()):
        item = oc[i]
        did = item.m_dataIdentifier
        print(f'   0x{int(did):04X} @ {item.m_frequency} Hz')
        if (int(did) & 0xFFF0) == (int(xda.XDI_GnssSatInfo) & 0xFFF0):
            has_satinfo = True
    print('Per-satellite output (GnssSatInfo) enabled:', has_satinfo)

    print(f'\n== Streaming {SECS:.0f} s (measurement mode, no settings changed) ==')
    cb = Cb()
    dev.addCallbackHandler(cb)
    if not dev.gotoMeasurement():
        sys.exit('Could not enter measurement mode')
    time.sleep(SECS)
    dev.removeCallbackHandler(cb)

    numsv, fixtype, flag_bits = [], Counter(), Counter()
    sat_seen = defaultdict(list)   # (gnssId, svId) -> [cno]
    sat_used = defaultdict(int)
    gnss_names = {0: 'GPS', 1: 'SBAS', 2: 'Galileo', 3: 'BeiDou', 4: 'IMES', 5: 'QZSS', 6: 'GLONASS'}
    for p in cb.packets:
        if p.containsRawGnssPvtData():
            pvt = p.rawGnssPvtData()
            numsv.append(pvt.m_numSv)
            fixtype[pvt.m_fixType] += 1
            flag_bits[pvt.m_flags] += 1
        if p.containsRawGnssSatInfo():
            si = p.rawGnssSatInfo()
            for k in range(si.m_numSvs):
                s = si.m_satInfos[k]
                sat_seen[(s.m_gnssId, s.m_svId)].append(s.m_cno)
                if s.m_flags & 0x08:  # svUsed
                    sat_used[(s.m_gnssId, s.m_svId)] += 1
    print(f'Packets: {len(cb.packets)}')
    if numsv:
        print(f'Satellites used (PVT numSv): min {min(numsv)}, max {max(numsv)}, mean {sum(numsv)/len(numsv):.1f}')
        print('Fix type counts (3 = 3D fix):', dict(fixtype))
        print('PVT flag values (bits 6-7 = RTK: 0x40 float, 0x80 fixed):', {hex(k): v for k, v in flag_bits.items()})
    if sat_seen:
        by_sys = defaultdict(list)
        for (g, sv), cnos in sat_seen.items():
            by_sys[gnss_names.get(g, g)].append((sv, sum(cnos) / len(cnos), sat_used.get((g, sv), 0) > 0))
        print('\nPer-satellite (tracked, mean signal dB-Hz, used in fix):')
        for name, lst in sorted(by_sys.items()):
            lst.sort()
            used = sum(1 for _, _, u in lst if u)
            print(f'  {name:8s}: tracked {len(lst):2d}, used {used:2d}, mean signal {sum(c for _, c, _ in lst)/len(lst):.1f} dB-Hz')
            print('     ' + ', '.join(f'{sv}:{c:.0f}{"*" if u else ""}' for sv, c, u in lst))
    else:
        print('\nNo per-satellite data in the stream (GnssSatInfo output not enabled on the device).')
finally:
    ctrl.close()
