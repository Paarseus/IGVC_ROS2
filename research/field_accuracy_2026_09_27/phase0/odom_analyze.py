#!/usr/bin/env python3
"""Odometry accuracy: GPS antenna track (truth) vs encoders, /wheel_odom and /odometry/filtered.
Usage: odom_analyze.py <bag_dir>"""
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
T = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
gps, wd, wo, fo, rtk = [], [], [], [], []
while r.has_next():
    tp, d, t = r.read_next()
    t *= 1e-9
    m = deserialize_message(d, T[tp])
    if tp == '/gnss':
        gps.append((t, m.latitude, m.longitude))
    elif tp == '/avros/wheel_debug':
        wd.append((t, m.data[4], m.data[5], m.data[2], m.data[3]))
    elif tp == '/wheel_odom':
        wo.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
    elif tp == '/odometry/filtered':
        fo.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
    elif tp == '/status':
        rtk.append((t, m.rtk_status))

lat0, lon0 = gps[0][1], gps[0][2]
kn = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
ke = 111412.84 * math.cos(math.radians(lat0))
xy = [(t, (lo - lon0) * ke, (la - lat0) * kn) for t, la, lo in gps]

# motion window from the encoders (reliable start/stop)
m_rev = 0.01994
moving = [t for t, _, _, lr, rr in wd if abs(lr) > 30 or abs(rr) > 30]
if not moving:
    sys.exit('No motion found')
t_start, t_end = moving[0], moving[-1]
# compare from standstill before to standstill after (whole move), so no speed-up trimming
ta, tb = t_start - 0.5, t_end + 1.0


def at(series, t):
    return min(series, key=lambda s: abs(s[0] - t))


g0, g1 = at(xy, ta), at(xy, tb)
gps_d = math.hypot(g1[1] - g0[1], g1[2] - g0[2])
course = math.degrees(math.atan2(g1[2] - g0[2], g1[1] - g0[1]))
w0, w1 = at(wd, ta), at(wd, tb)
dl, dr = (w1[1] - w0[1]) * m_rev, (w1[2] - w0[2]) * m_rev
enc_d = abs(dl + dr) / 2
o0, o1 = at(wo, ta), at(wo, tb)
wo_d = math.hypot(o1[1] - o0[1], o1[2] - o0[2])
f0, f1 = at(fo, ta), at(fo, tb)
fo_d = math.hypot(f1[1] - f0[1], f1[2] - f0[2])
seg = [(x, y) for t, x, y in xy if t_start + 1.0 <= t <= t_end]
lat = []
if len(seg) > 2:
    ux, uy = (g1[1] - g0[1]) / gps_d, (g1[2] - g0[2]) / gps_d
    lat = [(-(x - g0[1]) * uy + (y - g0[2]) * ux) for x, y in seg]
rs = [s for t, s in rtk if ta <= t <= tb]

print(f'Move: {t_end - t_start:.1f} s, RTK FIXED {100 * rs.count(2) / max(1, len(rs)):.0f} % of the time')
print(f'GPS antenna (truth): {gps_d:.3f} m, direction {(90 - course) % 360:.1f} deg compass')
print(f'Encoders:            left {dl:+.3f} m, right {dr:+.3f} m, average {enc_d:.3f} m  -> {100 * (enc_d / gps_d - 1):+.1f} % vs GPS')
print(f'Left vs right:       {100 * (abs(dl) - abs(dr)) / max(abs(dl), 1e-6):+.2f} %')
print(f'/wheel_odom:         {wo_d:.3f} m -> {100 * (wo_d / gps_d - 1):+.1f} % vs GPS')
print(f'/odometry/filtered:  {fo_d:.3f} m -> {100 * (fo_d / gps_d - 1):+.1f} % vs GPS')
if lat:
    print(f'Sideways deviation from a straight line (GPS): max {100 * max(abs(v) for v in lat):.1f} cm')
print(f'Implied m_per_motor_rev: {m_rev * gps_d / enc_d:.5f} (current {m_rev})')
