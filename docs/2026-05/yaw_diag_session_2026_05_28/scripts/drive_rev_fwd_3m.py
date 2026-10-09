#!/usr/bin/env python3
"""Phase 1 Test 1.1+1.2: reverse 3 m, settle 3 s, forward 3 m.

Captures EKF poses at each leg boundary for analysis.
Final closure = end-vs-start distance; ideal = 0 m (chassis returns to start).
"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

VX = 0.35           # m/s magnitude
LEG_DUR = 8.57      # s — 8.57 × 0.35 = 3.00 m commanded per leg
SETTLE = 3.0
RATE = 20.0

def yaw_from(q):
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)

class D(Node):
    def __init__(self):
        super().__init__("drive_rev_fwd_3m")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, "/odometry/filtered", self.cb, qos)
        self.last_pose = None
    def cb(self, m):
        self.last_pose = (m.pose.pose.position.x,
                          m.pose.pose.position.y,
                          yaw_from(m.pose.pose.orientation))
    def snapshot(self, label):
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        p = self.last_pose
        print(f"  [{label}] EKF pose: ({p[0]:+.3f}, {p[1]:+.3f}) yaw={math.degrees(p[2]):+.2f}°")
        return p
    def drive(self, vx, dur):
        tw = Twist(); tw.linear.x = vx
        t_end = time.time() + dur
        while time.time() < t_end:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0/RATE)
        tw.linear.x = 0.0
        for _ in range(8):
            self.pub.publish(tw); time.sleep(0.1)

rclpy.init()
n = D()

for _ in range(40):
    rclpy.spin_once(n, timeout_sec=0.05)
while n.last_pose is None:
    rclpy.spin_once(n, timeout_sec=0.1)

print(f"=== Phase 1: reverse 3 m + forward 3 m ===")
print(f"  vx={VX} m/s magnitude, {LEG_DUR}s per leg = {VX*LEG_DUR:.2f}m commanded")
print()
p0 = n.snapshot("INITIAL")
print()
print(f"--- REVERSE 3 m ({LEG_DUR} s @ -{VX} m/s) ---")
n.drive(-VX, LEG_DUR)
p1 = n.snapshot("AFTER_REV")
dx1, dy1 = p1[0]-p0[0], p1[1]-p0[1]
dist1 = math.hypot(dx1, dy1)
print(f"  Δ leg 1: dx={dx1:+.3f} dy={dy1:+.3f} dist={dist1:.3f} m (cmd {VX*LEG_DUR:.1f})")
print()
print(f"--- SETTLE {SETTLE} s ---")
time.sleep(SETTLE)
for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.05)
print()
print(f"--- FORWARD 3 m ({LEG_DUR} s @ +{VX} m/s) ---")
n.drive(VX, LEG_DUR)
p2 = n.snapshot("AFTER_FWD")
dx2, dy2 = p2[0]-p1[0], p2[1]-p1[1]
dist2 = math.hypot(dx2, dy2)
print(f"  Δ leg 2: dx={dx2:+.3f} dy={dy2:+.3f} dist={dist2:.3f} m (cmd {VX*LEG_DUR:.1f})")
print()
print(f"=== CLOSURE ===")
cx, cy = p2[0]-p0[0], p2[1]-p0[1]
print(f"  end vs start: dx={cx:+.3f} dy={cy:+.3f} closure_dist={math.hypot(cx,cy):.3f} m")
print(f"  end yaw vs start: Δ={math.degrees(p2[2]-p0[2]):+.2f}°")
print()
print(f"  Now measure with TAPE the chassis ground position.")
print(f"  Ideal closure: 0 m (chassis returns to start after rev+fwd).")
n.destroy_node()
rclpy.shutdown()
