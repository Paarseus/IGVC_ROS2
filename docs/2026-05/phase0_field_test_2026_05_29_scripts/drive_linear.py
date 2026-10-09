#!/usr/bin/env python3
"""Publish a steady forward /cmd_vel for a duration, then stop.
Usage: drive_linear.py [vx_m_s=0.3] [duration_s=8]"""
import sys, time
import rclpy
from geometry_msgs.msg import Twist
vx = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
rclpy.init()
n = rclpy.create_node('drive_linear')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
time.sleep(0.4)
t = Twist(); t.linear.x = vx
t0 = time.time()
while time.time() - t0 < dur:
    pub.publish(t); time.sleep(0.05)
z = Twist()
for _ in range(15):
    pub.publish(z); time.sleep(0.05)
n.destroy_node(); rclpy.shutdown()
