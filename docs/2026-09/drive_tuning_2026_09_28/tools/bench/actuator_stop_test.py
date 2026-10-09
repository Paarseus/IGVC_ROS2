#!/usr/bin/env python3
"""Stop behaviour through the real pipeline: /cmd_vel -> actuator_node (slew) -> Teensy -> SPARK.
Records /avros/wheel_debug and reports, per stop: time to standstill, overshoot past zero, reversals.
Speed is computed from wheel position (pos_rev), not the lagged reported speed."""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class T(Node):
    def __init__(self):
        super().__init__('actuator_stop_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rows = []
        self.create_subscription(Float32MultiArray, '/avros/wheel_debug', self.cb, 50)

    def cb(self, m):
        d = list(m.data)
        # L_cmd, R_cmd, L_meas, R_meas, L_pos, R_pos, v_target, w_target, v_slewed, w_slewed, ...
        self.rows.append((time.time(), d[0], d[1], d[2], d[3], d[4], d[5], d[8], d[9]))

    def drive(self, v, w, secs, publish=True):
        t0 = time.time()
        while time.time() - t0 < secs:
            if publish:
                m = Twist(); m.linear.x = float(v); m.angular.z = float(w); self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)


def pos_speed(rows, i_pos, t0, t1, win=0.1):
    pts = [(r[0], r[i_pos]) for r in rows if t0 - win <= r[0] <= t1 + win]
    out = []
    for i in range(len(pts)):
        j = i
        while j + 1 < len(pts) and pts[j + 1][0] - pts[i][0] < win:
            j += 1
        if pts[j][0] > pts[i][0] and t0 <= pts[i][0] <= t1:
            out.append((pts[i][0] - t0, (pts[j][1] - pts[i][1]) / (pts[j][0] - pts[i][0]) * 60))
    return out


def report(n, name, t_stop):
    res = []
    for side, ip in (('L', 5), ('R', 6)):
        sp = pos_speed(n.rows, ip, t_stop, t_stop + 3.0)
        if not sp:
            res.append(f'{side}: no data'); continue
        v0 = sp[0][1]
        sign = 1 if v0 >= 0 else -1
        past = min(sign * v for _, v in sp)                   # most negative in the travel direction
        still = [t for t, v in sp if abs(v) > 30]
        settle = still[-1] if still else 0.0
        vs = [v for _, v in sp]
        rev = sum(1 for a, b in zip(vs, vs[1:]) if (a > 40 and b < -40) or (a < -40 and b > 40))
        res.append(f'{side}: from {v0:5.0f} RPM, overshoot past 0 {min(0, past):5.0f} RPM, reversals {rev}, standstill after {settle:4.2f}s')
    print(f'{name:40s} | ' + ' | '.join(res))


rclpy.init()
n = T()
n.drive(0, 0, 1.0)
tests = [('0.6 m/s, then cmd_vel stops (timeout)', 0.6, 0.0, False),
         ('0.6 m/s, then cmd_vel = 0', 0.6, 0.0, True),
         ('turn 0.8 rad/s, then cmd_vel = 0', 0.0, 0.8, True),
         ('1.0 m/s, then cmd_vel = 0', 1.0, 0.0, True)]
for name, v, w, explicit_zero in tests:
    n.drive(v, w, 4.0)
    t_stop = time.time()
    n.drive(0.0, 0.0, 3.5, publish=explicit_zero)
    report(n, name, t_stop)
n.destroy_node(); rclpy.shutdown()
