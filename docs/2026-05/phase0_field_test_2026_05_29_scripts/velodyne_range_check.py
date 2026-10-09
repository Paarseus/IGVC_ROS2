#!/usr/bin/env python3
"""Sensor-level check: do RAW Velodyne points gain near-range/extra returns during
rotation, or are the raw scans clean (=> the ring is a costmap/odom-frame artifact)?

Compares per-cloud point-range histograms in HIGH-rotation vs LOW-rotation seconds.
Uses read_points (handles VLP-16 mixed field dtypes). Usage: <bagdir>
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


def rot_class(ts):
    d = wzbin.get(int(ts - t0), 0.0)
    return 'HI' if d > 10 else ('LO' if d < 2 else 'MID')


RB = np.arange(0, 16, 1.0)
vprof = {'HI': np.zeros(len(RB) - 1), 'LO': np.zeros(len(RB) - 1)}
vcnt = {'HI': 0, 'LO': 0}
tot = {'HI': 0, 'LO': 0}
r = reader_for(['/velodyne_points'])
tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
Pc = get_message(tmap['/velodyne_points'])
i = 0
while r.has_next():
    _, data, t = r.read_next()
    i += 1
    if i % 8 != 0:
        continue
    c = rot_class(t / 1e9)
    if c == 'MID':
        continue
    m = deserialize_message(data, Pc)
    xy = np.array([(p[0], p[1]) for p in point_cloud2.read_points(
        m, field_names=['x', 'y'], skip_nans=True)], dtype=np.float32)
    if xy.shape[0] == 0:
        continue
    rng = np.hypot(xy[:, 0], xy[:, 1])
    h, _ = np.histogram(rng, bins=RB)
    vprof[c] += h
    vcnt[c] += 1
    tot[c] += xy.shape[0]

print(f"velodyne clouds sampled: HI={vcnt['HI']} LO={vcnt['LO']}")
print(f"avg total points/cloud: HI={tot['HI']/max(1,vcnt['HI']):.0f} "
      f"LO={tot['LO']/max(1,vcnt['LO']):.0f}")
print("avg LiDAR points per cloud in each 1 m range ring (planar):")
print(f"{'rng(m)':>8} {'HI-rot':>9} {'LO-rot':>9} {'HI/LO':>6}")
for i in range(len(RB) - 1):
    hi = vprof['HI'][i] / max(1, vcnt['HI'])
    lo = vprof['LO'][i] / max(1, vcnt['LO'])
    print(f"{RB[i]:3.0f}-{RB[i+1]:<3.0f} {hi:9.0f} {lo:9.0f} {hi/max(0.1,lo):6.2f}")
