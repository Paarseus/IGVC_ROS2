#!/usr/bin/env python3
"""Phase 2 Test 2.1 — 60 s stationary baseline.

Measures map→odom drift rate, EKF/GPS gap, IMU bias, while chassis is motionless.

Pass criteria (from strategy doc, M4 baseline):
  - map→odom drift rate < 0.5 cm/s
  - EKF and GPS positions agree within 1 m
  - Gyro bias |μ| < 0.05 °/s, σ < 0.15 °/s
"""
import math, statistics, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

DUR = 60.0  # seconds

class M(Node):
    def __init__(self):
        super().__init__("test_2_1")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, "/odometry/global", self.cb_g, qos)
        self.create_subscription(Odometry, "/odometry/gps", self.cb_gps, qos)
        self.create_subscription(Imu, "/imu/data", self.cb_imu, qos_be)
        self.ekf = []
        self.gps = []
        self.gyro_z = []
    def cb_g(self, m):
        t = self.get_clock().now().nanoseconds / 1e9
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.ekf.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
    def cb_gps(self, m):
        t = self.get_clock().now().nanoseconds / 1e9
        self.gps.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
    def cb_imu(self, m):
        self.gyro_z.append(m.angular_velocity.z)

rclpy.init()
n = M()
print(f"=== Phase 2 Test 2.1 — 60 s stationary baseline ===")
print(f"  Recording /odometry/global + /odometry/gps + /imu/data ...")
print(f"  KEEP CHASSIS MOTIONLESS for the full {DUR:.0f} s.")
print()
t_start = time.time()
t_end = t_start + DUR
while time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.05)

print(f"=== RESULTS ===")
print(f"  samples: ekf={len(n.ekf)} gps={len(n.gps)} imu={len(n.gyro_z)}")
print()
if len(n.ekf) < 100 or len(n.gps) < 10:
    print("  NOT ENOUGH DATA — bag may not be capturing topics")
    rclpy.shutdown(); raise SystemExit(1)

ekf_x = [e[1] for e in n.ekf]; ekf_y = [e[2] for e in n.ekf]
ekf_x0, ekf_y0 = ekf_x[0], ekf_y[0]
ekf_xf, ekf_yf = ekf_x[-1], ekf_y[-1]
ekf_drift = math.hypot(ekf_xf - ekf_x0, ekf_yf - ekf_y0)
ekf_drift_rate = ekf_drift / DUR * 100
print(f"  EKF drift over {DUR:.0f}s: {ekf_drift*100:.1f} cm  ({ekf_drift_rate:.3f} cm/s avg)")

gps_x = [g[1] for g in n.gps]; gps_y = [g[2] for g in n.gps]
gps_x_std = statistics.pstdev(gps_x) * 100
gps_y_std = statistics.pstdev(gps_y) * 100
print(f"  GPS noise (1σ): x={gps_x_std:.1f} cm  y={gps_y_std:.1f} cm")

gap_x = ekf_xf - gps_x[-1]
gap_y = ekf_yf - gps_y[-1]
gap = math.hypot(gap_x, gap_y)
print(f"  EKF vs GPS gap at end: dx={gap_x:+.3f} dy={gap_y:+.3f}  total={gap:.3f} m")

mean_wz = math.degrees(statistics.mean(n.gyro_z))
std_wz = math.degrees(statistics.pstdev(n.gyro_z))
print(f"  Gyro bias: μ={mean_wz:+.4f}°/s  σ={std_wz:.4f}°/s")
print()
print(f"  PASS CRITERIA:")
pass1 = ekf_drift_rate < 0.5
print(f"    EKF drift < 0.5 cm/s:    {ekf_drift_rate:.3f} -> {'PASS' if pass1 else 'FAIL'}")
pass2 = gap < 1.0
print(f"    EKF/GPS gap < 1 m:       {gap:.3f} -> {'PASS' if pass2 else 'FAIL'}")
pass3 = abs(mean_wz) < 0.05 and std_wz < 0.15
print(f"    Gyro healthy:            μ={mean_wz:+.4f} σ={std_wz:.4f} -> {'PASS' if pass3 else 'FAIL'}")
print()
print(f"  Overall: {'PASS' if (pass1 and pass2 and pass3) else 'NOT PASS'}")
n.destroy_node()
rclpy.shutdown()
