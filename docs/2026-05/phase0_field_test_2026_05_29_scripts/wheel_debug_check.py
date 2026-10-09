#!/usr/bin/env python3
"""From a bag, compare what the actuator COMMANDS the SparkMAX (L/R_rpm_sent) vs what
the SparkMAX REPORTS back (L/R_meas) — to find the source of the phantom feedback.
wheel_debug.data = [L_sent,R_sent,L_meas,R_meas,L_pos,R_pos,v_req,w_req,
                    v_slewed,w_slewed,v,w,yaw,yaw_rate,heading_locked,estop]
Usage: wheel_debug_check.py <bagdir>"""
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/avros/wheel_debug', '/cmd_vel', '/wheel_odom']))
tm = {t.name: t.type for t in r.get_all_topics_and_types()}
M = {n: get_message(tm[n]) for n in tm if n in ('/avros/wheel_debug', '/cmd_vel', '/wheel_odom')}

rows = []   # (L_sent, R_sent, L_meas, R_meas, v)
cmd = []
wodom = []
for n in ('/avros/wheel_debug', '/cmd_vel', '/wheel_odom'):
    pass
while r.has_next():
    topic, data, t = r.read_next()
    if topic == '/avros/wheel_debug':
        d = list(deserialize_message(data, M[topic]).data)
        if len(d) >= 11:
            rows.append((d[0], d[1], d[2], d[3], d[10]))
    elif topic == '/cmd_vel':
        cmd.append(deserialize_message(data, M[topic]).linear.x)
    elif topic == '/wheel_odom':
        wodom.append(deserialize_message(data, M[topic]).twist.twist.linear.x)


def rng(vals, name):
    if not vals:
        print(f"{name}: (none)"); return
    lo, hi = min(vals), max(vals)
    print(f"{name:24s} n={len(vals):4d}  min={lo:8.2f}  max={hi:8.2f}  {'CONSTANT' if abs(hi-lo)<1e-3 else ''}")


print(f"wheel_debug msgs: {len(rows)}")
if rows:
    rng([x[0] for x in rows], "L_rpm_SENT->sparkmax")
    rng([x[1] for x in rows], "R_rpm_SENT->sparkmax")
    rng([x[2] for x in rows], "L_meas (sparkmax reports)")
    rng([x[3] for x in rows], "R_meas (sparkmax reports)")
    rng([x[4] for x in rows], "odom v (m/s)")
rng(cmd, "/cmd_vel linear.x")
rng(wodom, "/wheel_odom twist.x")
# show a few interleaved samples
print("\nsample [L_sent R_sent | L_meas R_meas | odom_v]:")
for x in rows[::max(1, len(rows)//12)][:12]:
    print(f"   {x[0]:7.1f} {x[1]:7.1f}  | {x[2]:7.1f} {x[3]:7.1f} | {x[4]:.3f}")
