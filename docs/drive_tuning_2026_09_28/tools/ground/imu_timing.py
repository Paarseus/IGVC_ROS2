#!/usr/bin/env python3
"""IMU arrival timing: inter-arrival spread and age (receive time - header stamp) of /imu/data for N seconds."""
import sys, time, statistics as st
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20
rclpy.init(); n = Node('imu_timing'); rx = []; age = []
def cb(m):
    t = time.time(); rx.append(t); age.append((t - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)) * 1000)
n.create_subscription(Imu, '/imu/data', cb, qos_profile_sensor_data)
t_end = time.time() + secs
while time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.05)
d = [(b - a) * 1000 for a, b in zip(rx, rx[1:])]
q = lambda a, p: sorted(a)[int(p * (len(a) - 1))]
print(f'samples {len(rx)}  rate {len(rx)/secs:.1f} Hz')
print(f'inter-arrival ms: mean {st.mean(d):.2f}  sd {st.pstdev(d):.2f}  p1 {q(d,.01):.2f}  p99 {q(d,.99):.2f}  max {max(d):.2f}  <2ms {100*sum(x<2 for x in d)/len(d):.0f}%')
print(f'age ms (receive - stamp): median {st.median(age):.1f}  p95 {q(age,.95):.1f}  max {max(age):.1f}')
