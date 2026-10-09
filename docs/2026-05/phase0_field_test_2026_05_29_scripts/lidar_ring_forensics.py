#!/usr/bin/env python3
"""Forensic: does the local-costmap phantom 'ring' correlate with rotation rate?

Reads /imu/data (|angular_velocity.z|) and /local_costmap/costmap_raw (count of
lethal cells, total and in a near-robot box) from a rosbag2, bins to 1 s, and
reports the correlation + a high-rotation vs low-rotation comparison. If the
near-robot lethal count is much higher during high |omega|, the ring is
rotation-triggered (out-of-frustum decay-persistence / TF-lag / motion-distortion).

Usage: lidar_ring_forensics.py <bagdir>
"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bagdir = sys.argv[1]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bagdir, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
reader.set_filter(rosbag2_py.StorageFilter(
    topics=['/imu/data', '/local_costmap/costmap_raw']))
tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
Imu = get_message(tmap['/imu/data'])
Cm = get_message(tmap['/local_costmap/costmap_raw'])

NEAR = 15          # cells from center (~3.0 m at 0.2 m res)
LETHAL = 253       # >=253 = inscribed/lethal

imu = []           # (t, |wz|)
cm = []            # (t, total_lethal, near_lethal)
n = 0
while reader.has_next():
    topic, data, t = reader.read_next()
    ts = t / 1e9
    if topic == '/imu/data':
        m = deserialize_message(data, Imu)
        imu.append((ts, abs(m.angular_velocity.z)))
    else:
        m = deserialize_message(data, Cm)
        sx = m.metadata.size_x
        sy = m.metadata.size_y
        arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(sy, sx)
        total = int((arr >= LETHAL).sum())
        cy, cx = sy // 2, sx // 2
        sub = arr[max(0, cy - NEAR):cy + NEAR, max(0, cx - NEAR):cx + NEAR]
        near = int((sub >= LETHAL).sum())
        cm.append((ts, total, near))
    n += 1

print(f"read {n} msgs: imu={len(imu)} costmap={len(cm)}")
if not imu or not cm:
    print("MISSING DATA"); sys.exit(0)

t0 = min(imu[0][0], cm[0][0])


def binize(rows, idx):
    d = {}
    for r in rows:
        d.setdefault(int(r[0] - t0), []).append(r[idx])
    return d


wz = binize(imu, 1)
nearb = binize(cm, 2)
totb = binize(cm, 1)
bins = sorted(set(wz) & set(nearb) & set(totb))
wzm = np.array([np.mean(wz[b]) for b in bins])
nearm = np.array([np.mean(nearb[b]) for b in bins])
totm = np.array([np.mean(totb[b]) for b in bins])
degs = np.degrees(wzm)

print(f"\nbins(1s)={len(bins)}")
print(f"corr(|wz|, near_lethal)   = {np.corrcoef(wzm, nearm)[0,1]:.3f}")
print(f"corr(|wz|, total_lethal)  = {np.corrcoef(wzm, totm)[0,1]:.3f}")

hi = degs > 10.0     # rotating fast
mid = (degs >= 2.0) & (degs <= 10.0)
lo = degs < 2.0      # ~stationary/straight
for name, mask in [("HIGH-rot >10deg/s", hi), ("MID 2-10deg/s", mid), ("LOW <2deg/s", lo)]:
    if mask.sum():
        print(f"{name:20s}: n={int(mask.sum()):4d}  near_lethal={nearm[mask].mean():6.0f}  "
              f"total_lethal={totm[mask].mean():6.0f}")
if lo.sum() and hi.sum():
    print(f"\nnear_lethal HIGH/LOW ratio = {nearm[hi].mean()/max(1.0, nearm[lo].mean()):.2f}x")
    print(f"total_lethal HIGH/LOW ratio = {totm[hi].mean()/max(1.0, totm[lo].mean()):.2f}x")

# top phantom windows
order = np.argsort(nearm)[::-1][:8]
print("\nTop near-robot-lethal 1s windows (t_rel, near_lethal, total_lethal, deg/s):")
for i in order:
    print(f"  t+{bins[i]:5d}s  near={nearm[i]:5.0f}  total={totm[i]:6.0f}  wz={degs[i]:5.1f} deg/s")
