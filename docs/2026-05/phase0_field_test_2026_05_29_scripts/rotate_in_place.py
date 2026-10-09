#!/usr/bin/env python3
"""Publish a pure in-place yaw to /cmd_vel for a fixed duration, then stop.
Usage: rotate_in_place.py [wz_rad_s=0.5] [duration_s=13.0]
"""
import sys, time
import rclpy
from geometry_msgs.msg import Twist

wz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 13.0
rclpy.init()
n = rclpy.create_node('rotate_in_place')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
time.sleep(0.5)
t = Twist(); t.angular.z = wz
print(f"rotating wz={wz} rad/s for {dur}s...")
t0 = time.time()
while time.time() - t0 < dur:
    pub.publish(t)
    time.sleep(0.05)          # 20 Hz
z = Twist()
for _ in range(15):           # ensure stop
    pub.publish(z)
    time.sleep(0.05)
print("stopped")
n.destroy_node(); rclpy.shutdown()
