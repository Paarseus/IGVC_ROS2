#!/usr/bin/env python3
"""Locate the phantom odom during the e-stopped run:
- /cmd_vel linear.x       (what MPPI commanded)
- /avros/actuator_state throttle/brake (derived from MEASURED wheel RPM)
- /odometry/filtered position.x + twist.linear.x (the odom EKF)
If throttle stays ~0 but odom moves -> phantom is downstream of the measured wheel
(EKF/wheel_odom-source). If throttle goes forward -> measured RPM itself was phantom.
Usage: estop_phantom_check.py <bagdir>
"""
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/cmd_vel', '/avros/actuator_state', '/odometry/filtered']))
tm = {t.name: t.type for t in r.get_all_topics_and_types()}
M = {n: get_message(tm[n]) for n in tm}

cmd_vx = []
thr = []
brk = []
odx = []
otw = []
while r.has_next():
    topic, data, t = r.read_next()
    m = deserialize_message(data, M[topic])
    if topic == '/cmd_vel':
        cmd_vx.append(m.linear.x)
    elif topic == '/avros/actuator_state':
        thr.append(m.throttle); brk.append(m.brake)
    elif topic == '/odometry/filtered':
        odx.append(m.pose.pose.position.x); otw.append(m.twist.twist.linear.x)


def rng(a, name):
    if not a:
        print(f"{name}: (none)"); return
    print(f"{name}: n={len(a)} min={min(a):.3f} max={max(a):.3f}")


rng(cmd_vx, "cmd_vel.linear.x (MPPI commanded)")
rng(thr, "actuator throttle (from MEASURED rpm)")
rng(brk, "actuator brake (from MEASURED rpm)")
if odx:
    print(f"odom/filtered x: start={odx[0]:.3f} end={odx[-1]:.3f} disp={odx[-1]-odx[0]:.3f} m")
rng(otw, "odom/filtered twist.linear.x (EKF fwd vel)")
