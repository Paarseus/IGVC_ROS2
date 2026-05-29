#!/usr/bin/env python3
"""Localize the phantoms: radial lethal-cell profile (costmap) + Velodyne near-range
point counts, comparing HIGH-rotation vs STATIONARY windows.

- Builds a per-second |wz| index from /imu/data.
- For /local_costmap/costmap_raw: radial histogram of lethal cells vs distance from
  robot center, averaged over high-rot vs low-rot seconds.
- For /velodyne_points (sampled): point-range histogram + near-range (1-2.5 m) count,
  high-rot vs low-rot. Tests proximity + motion-distortion (distorted scans during
  rotation tend to add spurious points at varied ranges).

Usage: lidar_ring_radial.py <bagdir>
"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

bagdir = sys.argv[1]


def reader_for(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bagdir, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


# pass 1: |wz| per second
r = reader_for(['/imu/data'])
tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
Imu = get_message(tmap['/imu/data'])
wz = {}
t0 = None
while r.has_next():
    _, data, t = r.read_next()
    ts = t / 1e9
    if t0 is None:
        t0 = ts
    m = deserialize_message(data, Imu)
    wz.setdefault(int(ts - t0), []).append(abs(m.angular_velocity.z))
wzbin = {b: np.degrees(np.mean(v)) for b, v in wz.items()}
print(f"imu seconds={len(wzbin)}  t0={t0:.1f}")


def rot_class(ts):
    b = int(ts - t0)
    d = wzbin.get(b, 0.0)
    return 'HI' if d > 10 else ('LO' if d < 2 else 'MID')


# pass 2: costmap radial profile
RES = 0.2
RINGS = np.arange(0, 16, 1.0)  # 0..15 m in 1 m rings
prof = {'HI': np.zeros(len(RINGS) - 1), 'LO': np.zeros(len(RINGS) - 1)}
cnt = {'HI': 0, 'LO': 0}
r = reader_for(['/local_costmap/costmap_raw'])
tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
Cm = get_message(tmap['/local_costmap/costmap_raw'])
while r.has_next():
    _, data, t = r.read_next()
    c = rot_class(t / 1e9)
    if c == 'MID':
        continue
    m = deserialize_message(data, Cm)
    sx, sy = m.metadata.size_x, m.metadata.size_y
    arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(sy, sx)
    ys, xs = np.where(arr >= 253)
    if len(xs) == 0:
        cnt[c] += 1
        continue
    d = np.hypot((xs - sx / 2) * RES, (ys - sy / 2) * RES)
    h, _ = np.histogram(d, bins=RINGS)
    prof[c] += h
    cnt[c] += 1
print(f"\ncostmap frames: HI={cnt['HI']} LO={cnt['LO']}")
print("radial lethal-cell profile (avg cells per frame in each 1 m ring):")
print(f"{'ring(m)':>8} {'HI-rot':>8} {'LO-rot':>8} {'HI/LO':>6}")
for i in range(len(RINGS) - 1):
    hi = prof['HI'][i] / max(1, cnt['HI'])
    lo = prof['LO'][i] / max(1, cnt['LO'])
    print(f"{RINGS[i]:3.0f}-{RINGS[i+1]:<3.0f} {hi:8.1f} {lo:8.1f} {hi/max(0.1,lo):6.2f}")

# pass 3: velodyne near-range, sampled (every Nth cloud) high vs low rot
r = reader_for(['/velodyne_points'])
tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
Pc = get_message(tmap['/velodyne_points'])
RB = np.arange(0, 16, 1.0)
vprof = {'HI': np.zeros(len(RB) - 1), 'LO': np.zeros(len(RB) - 1)}
vcnt = {'HI': 0, 'LO': 0}
i = 0
while r.has_next():
    _, data, t = r.read_next()
    i += 1
    if i % 5 != 0:           # sample 1 in 5 clouds
        continue
    c = rot_class(t / 1e9)
    if c == 'MID':
        continue
    m = deserialize_message(data, Pc)
    pts = point_cloud2.read_points_numpy(m, field_names=['x', 'y', 'z'], skip_nans=True)
    if pts.shape[0] == 0:
        continue
    rng = np.hypot(pts[:, 0], pts[:, 1])
    h, _ = np.histogram(rng, bins=RB)
    vprof[c] += h
    vcnt[c] += 1
print(f"\nvelodyne clouds sampled: HI={vcnt['HI']} LO={vcnt['LO']}")
print("avg LiDAR points per cloud in each 1 m range ring (planar):")
print(f"{'rng(m)':>8} {'HI-rot':>9} {'LO-rot':>9} {'HI/LO':>6}")
for i in range(len(RB) - 1):
    hi = vprof['HI'][i] / max(1, vcnt['HI'])
    lo = vprof['LO'][i] / max(1, vcnt['LO'])
    print(f"{RB[i]:3.0f}-{RB[i+1]:<3.0f} {hi:9.0f} {lo:9.0f} {hi/max(0.1,lo):6.2f}")
