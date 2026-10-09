"""Temporarily enable per-satellite GNSS output, stream, then restore the exact
original output configuration and verify it. Only the output list is touched.
"""
import sys
import time
from collections import defaultdict

import xsensdeviceapi as xda

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
NAMES = {0: 'GPS', 1: 'SBAS', 2: 'Galileo', 3: 'BeiDou', 4: 'IMES', 5: 'QZSS', 6: 'GLONASS'}


class Cb(xda.XsCallback):
    def __init__(self):
        super().__init__()
        self.packets = []
        self.sat_epochs = []   # list of [(gnssId, svId, cno, flags)]

    def onLiveDataAvailable(self, dev, packet):
        self.packets.append(xda.XsDataPacket(packet))

    def onMessageReceivedFromDevice(self, dev, msg):
        # MTData2 (0x36): repeated [id(2) size(1) data]
        if msg.getMessageId() != 0x36:
            return
        n = msg.getDataSize()
        b = bytes(msg.getDataByte(i) for i in range(n))
        i = 0
        while i + 3 <= n:
            did = (b[i] << 8) | b[i + 1]
            size = b[i + 2]
            data = b[i + 3:i + 3 + size]
            if (did & 0xFFF0) == 0x7020 and len(data) >= 8:
                num = data[4]
                sats = []
                for k in range(num):
                    o = 8 + 4 * k
                    if o + 4 <= len(data):
                        sats.append((data[o], data[o + 1], data[o + 2], data[o + 3]))
                self.sat_epochs.append(sats)
            i += 3 + size


def cfg_list(oc):
    return [(int(oc[i].m_dataIdentifier), int(oc[i].m_frequency)) for i in range(oc.size())]


ctrl = xda.XsControl_construct()
dev = None
original = None
try:
    port = xda.XsScanner_scanPort(PORT, xda.XBR_921k6)
    if port.empty():
        sys.exit(f'No device on {PORT}')
    ctrl.openPort(port.portName(), port.baudrate())
    dev = ctrl.device(port.deviceId())
    dev.gotoConfig()
    original = xda.XsOutputConfigurationArray(dev.outputConfiguration())
    orig_list = cfg_list(original)
    print('Saved original output list:', [f'0x{d:04X}@{f}' for d, f in orig_list])

    temp = xda.XsOutputConfigurationArray(original)
    temp.push_back(xda.XsOutputConfiguration(xda.XDI_GnssSatInfo, 4))
    if not dev.setOutputConfiguration(temp):
        raise RuntimeError('could not add GnssSatInfo output')
    print('Added per-satellite output (GnssSatInfo @ 4 Hz)')

    cb = Cb()
    dev.addCallbackHandler(cb)
    dev.gotoMeasurement()
    time.sleep(SECS)
    dev.removeCallbackHandler(cb)

    seen = defaultdict(list)
    used = defaultdict(int)
    numsv = []
    for p in cb.packets:
        if p.containsRawGnssPvtData():
            numsv.append(p.rawGnssPvtData().m_numSv)
    flagcount = defaultdict(lambda: defaultdict(int))
    for sats in cb.sat_epochs:
        for g, sv, cno, fl in sats:
            flagcount[(g, sv)][fl] += 1
            seen[(g, sv)].append(cno)
            if fl & 0x08:
                used[(g, sv)] += 1
    print(f'Satellite-info epochs decoded: {len(cb.sat_epochs)}; tracked per epoch: '
          f'{min(len(e) for e in cb.sat_epochs) if cb.sat_epochs else 0}-{max(len(e) for e in cb.sat_epochs) if cb.sat_epochs else 0}')
    print(f'\nPackets {len(cb.packets)}; satellites used (PVT): '
          f'{min(numsv) if numsv else "-"}-{max(numsv) if numsv else "-"}')
    by = defaultdict(list)
    for (g, sv), c in seen.items():
        by[NAMES.get(g, g)].append((sv, sum(c) / len(c), max(c), used.get((g, sv), 0) > 0))
    print("Per system: tracked / used / mean and best signal (dB-Hz). Healthy open sky: 35-50 dB-Hz.")
    for name, lst in sorted(by.items()):
        lst.sort()
        print(f'  {name:8s}: tracked {len(lst):2d}, used {sum(u for *_, u in lst):2d}, '
              f'mean {sum(m for _, m, _, _ in lst)/len(lst):4.1f}, best {max(b for _, _, b, _ in lst):4.1f}')
        for sv, m, b, u in lst:
            g = [k for k, v in NAMES.items() if v == name]
            fc = flagcount[(g[0] if g else name, sv)]
            top = max(fc, key=fc.get) if fc else 0
            q = top & 0x07
            qn = {0: 'no signal', 1: 'searching', 2: 'acquired', 3: 'unusable', 4: 'code locked'}.get(q, 'code+carrier locked')
            h = (top >> 4) & 0x03
            print(f'      sv {sv:3d}: mean {m:4.1f}  best {b:4.1f}  used {"yes" if u else "no ":3s}  '
                  f'quality {qn:20s} health {["unknown", "healthy", "UNHEALTHY", "?"][h]}')
    if not by:
        print('  (no satellites tracked)')
finally:
    if dev is not None and original is not None:
        dev.gotoConfig()
        ok = dev.setOutputConfiguration(original)
        now = cfg_list(dev.outputConfiguration())
        print('\nRestore:', 'OK' if ok else 'FAILED',
              '| read back matches original:', now == cfg_list(original))
    ctrl.close()
