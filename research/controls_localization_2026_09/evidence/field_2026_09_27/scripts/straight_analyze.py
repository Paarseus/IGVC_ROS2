#!/usr/bin/env python3
"""Straight-drive analysis: RTK track straightness, true heading error, track balance.
Usage: straight_analyze.py <bag_dir>"""
import math
import statistics as st
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
T = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
pl, imu, wd, st_ = [], [], [], []
while r.has_next():
    tp, d, t = r.read_next()
    t *= 1e-9
    if tp == '/gnss':  # raw antenna position (independent of the Xsens heading)
        m = deserialize_message(d, T[tp]); pl.append((t, m.latitude, m.longitude))
    elif tp == '/imu/data':
        m = deserialize_message(d, T[tp]); q = m.orientation
        imu.append((t, math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))))
    elif tp == '/avros/wheel_debug':
        m = deserialize_message(d, T[tp]); wd.append((t, m.data[4], m.data[5]))
    elif tp == '/status':
        m = deserialize_message(d, T[tp]); st_.append((t, m.rtk_status))

lat0, lon0 = pl[0][1], pl[0][2]
kn = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
ke = 111412.84 * math.cos(math.radians(lat0))
xy = [(t, (lo - lon0) * ke, (la - lat0) * kn) for t, la, lo in pl]
# moving segment: speed > 0.2 m/s over 0.5 s windows
spd = []
for i in range(2, len(xy)):
    t0, x0, y0 = xy[i - 2]; t1, x1, y1 = xy[i]
    spd.append((t1, math.hypot(x1 - x0, y1 - y0) / (t1 - t0)))
moving = [t for t, v in spd if v > 0.2]
if not moving:
    sys.exit('No motion found in the recording')
ta, tb = moving[0] + 1.0, moving[-1] - 0.5          # skip the ramp-up second
seg = [(t, x, y) for t, x, y in xy if ta <= t <= tb]
x0, y0, x1, y1 = seg[0][1], seg[0][2], seg[-1][1], seg[-1][2]
course = math.degrees(math.atan2(y1 - y0, x1 - x0))  # ENU, true direction of travel
length = math.hypot(x1 - x0, y1 - y0)
ux, uy = (x1 - x0) / length, (y1 - y0) / length
lat = [(-(x - x0) * uy + (y - y0) * ux) for _, x, y in seg]
yaw_seg = [y for t, y in imu if ta <= t <= tb]
yaw_mean = st.mean(yaw_seg)
err = (yaw_mean - course + 180) % 360 - 180
rtk_seg = [s for t, s in st_ if ta <= t <= tb]
L = [(t, l, r_) for t, l, r_ in wd if ta <= t <= tb]
m_rev = 0.01994
dl, dr = (L[-1][1] - L[0][1]) * m_rev, (L[-1][2] - L[0][2]) * m_rev
print(f'Straight segment analysed: {tb - ta:.1f} s, {length:.2f} m (RTK), RTK FIXED {100 * rtk_seg.count(2) / max(1, len(rtk_seg)):.0f} % of the time')
print(f'True direction of travel (raw GPS antenna track): {course:.1f} deg ENU = compass bearing {(90 - course) % 360:.1f} deg')
print(f'Xsens heading during the drive: {yaw_mean:.1f} deg ENU = compass bearing {(90 - yaw_mean) % 360:.1f} deg')
print(f'Xsens heading error vs RTK track: {err:+.1f} deg')
print(f'Xsens heading change during the drive: {yaw_seg[-1] - yaw_seg[0]:+.2f} deg')
print(f'Sideways deviation from a straight line: max {100 * max(abs(v) for v in lat):.1f} cm, end {100 * lat[-1]:+.1f} cm (+ = left)')
print(f'Track distances (encoders): left {dl:.3f} m, right {dr:.3f} m, difference {100 * (dl - dr) / max(abs(dl), 1e-6):+.2f} %')
print(f'Encoder distance vs RTK distance: {100 * ((dl + dr) / 2 / length - 1):+.1f} %')
