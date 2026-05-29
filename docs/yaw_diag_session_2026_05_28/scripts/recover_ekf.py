#!/usr/bin/env python3
"""Recover the EKF from Mahalanobis lockout by publishing /set_pose with current GPS.

When the EKF/GPS gap grows past the Mahalanobis rejection gate (default ~4σ²),
the gate rejects all GPS updates and the EKF stays divergent indefinitely. This
script reads the latest /odometry/gps message, then publishes one /set_pose
message at that location with an inflated covariance (σ=5 m), which forces the
EKF to re-anchor and clear its measurement queues.

Diagnostic threshold: anything over ~3 m gap is suspect. Run this before
sending any nav goal if you've moved the chassis far from where the EKF
last saw GPS, or if symptoms appear after a stuck-IMU restart.

Usage:
  python3 recover_ekf.py
"""
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class R(Node):
    def __init__(self):
        super().__init__("recover_ekf")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, "/odometry/gps", self.cb_gps, qos)
        self.create_subscription(Odometry, "/odometry/global", self.cb_ekf, qos)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/set_pose", 10)
        self.gps = None
        self.ekf = None
    def cb_gps(self, m): self.gps = (m.pose.pose.position.x, m.pose.pose.position.y)
    def cb_ekf(self, m): self.ekf = (m.pose.pose.position.x, m.pose.pose.position.y)

rclpy.init()
n = R()
print("Reading current GPS + EKF positions ...")
for _ in range(50):
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.gps and n.ekf: break
if not n.gps:
    print("  ERROR: no /odometry/gps within 5 s — nav stack not running?")
    rclpy.shutdown(); raise SystemExit(1)

print(f"  GPS:  ({n.gps[0]:+.3f}, {n.gps[1]:+.3f})")
print(f"  EKF:  ({n.ekf[0]:+.3f}, {n.ekf[1]:+.3f})")
import math
gap = math.hypot(n.ekf[0] - n.gps[0], n.ekf[1] - n.gps[1])
print(f"  gap:  {gap:.3f} m")
print()
print(f"Publishing /set_pose at GPS coords with σ=5 m covariance ...")

msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.header.stamp = n.get_clock().now().to_msg()
msg.pose.pose.position.x = n.gps[0]
msg.pose.pose.position.y = n.gps[1]
msg.pose.pose.position.z = 0.0
msg.pose.pose.orientation.w = 1.0
cov = [0.0] * 36
cov[0] = 25.0    # x var
cov[7] = 25.0    # y var
cov[14] = 25.0   # z var
cov[21] = 0.1    # roll var
cov[28] = 0.1    # pitch var
cov[35] = 0.5    # yaw var
msg.pose.covariance = cov

for _ in range(5):
    n.pub.publish(msg)
    time.sleep(0.1)
print("  ✓ /set_pose published 5x")
print()
print("Waiting 3 s, then re-checking ...")
time.sleep(3.0)
for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.1)
gap_after = math.hypot(n.ekf[0] - n.gps[0], n.ekf[1] - n.gps[1])
print(f"  EKF after: ({n.ekf[0]:+.3f}, {n.ekf[1]:+.3f})  gap = {gap_after:.3f} m")
print(f"  {'✓ RECOVERED' if gap_after < 1.0 else '✗ STILL DIVERGENT'}")

n.destroy_node()
rclpy.shutdown()
