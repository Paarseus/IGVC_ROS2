#!/usr/bin/env python3
"""Drive the chassis via direct cmd_vel publishing at 20 Hz.

Usage:
  python3 drive_cmd_vel.py <linear_x_mps> <angular_z_rps> <duration_s>

Examples:
  Forward 5 m at 0.35 m/s for 14.3 s:  python3 drive_cmd_vel.py  0.35  0.0  14.3
  Reverse 5 m at 0.35 m/s for 14.3 s:  python3 drive_cmd_vel.py -0.35  0.0  14.3
  In-place 360 CCW at 0.5 rad/s:       python3 drive_cmd_vel.py  0.0   0.5  12.6

NOTE: yaw wraparound in (-180, 180] — 360 deg rotation reports as a small
delta. Cross-check against ground / visual observation for full rotations.
"""
import sys, time, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

if len(sys.argv) != 4:
    print(__doc__); sys.exit(1)
VX = float(sys.argv[1])
WZ = float(sys.argv[2])
DUR = float(sys.argv[3])
RATE = 20.0  # Hz

class D(Node):
    def __init__(self):
        super().__init__("drive_cmd_vel")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, "/odometry/filtered", self.cb, qos)
        self.start_pose = None
        self.last_pose = None
    def cb(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        p = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        if self.start_pose is None:
            self.start_pose = p
        self.last_pose = p

rclpy.init()
n = D()
print(f"Drive: vx={VX:+.3f} m/s  wz={WZ:+.3f} rad/s  duration={DUR:.2f} s")

for _ in range(40):
    rclpy.spin_once(n, timeout_sec=0.05)
while n.start_pose is None:
    rclpy.spin_once(n, timeout_sec=0.1)
sx, sy, syaw = n.start_pose
print(f"  start: ({sx:+.3f}, {sy:+.3f}) yaw={math.degrees(syaw):+.2f} deg")

t_start = time.time()
t_end = t_start + DUR
tw = Twist()
tw.linear.x = VX
tw.angular.z = WZ
while time.time() < t_end:
    n.pub.publish(tw)
    rclpy.spin_once(n, timeout_sec=0.0)
    time.sleep(1.0/RATE)

tw.linear.x = 0.0; tw.angular.z = 0.0
for _ in range(8):
    n.pub.publish(tw); time.sleep(0.1)

for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.05)
ex, ey, eyaw = n.last_pose
dx, dy = ex - sx, ey - sy
dist = math.hypot(dx, dy)
dyaw = math.degrees(eyaw - syaw)
while dyaw > 180: dyaw -= 360
while dyaw <= -180: dyaw += 360
print(f"  end:   ({ex:+.3f}, {ey:+.3f}) yaw={math.degrees(eyaw):+.2f} deg")
print(f"  EKF Δ: dx={dx:+.3f} dy={dy:+.3f} dist={dist:.3f} m  Δyaw={dyaw:+.2f} deg")
print(f"")
print(f"  Now measure with TAPE the actual ground distance.")
n.destroy_node()
rclpy.shutdown()
