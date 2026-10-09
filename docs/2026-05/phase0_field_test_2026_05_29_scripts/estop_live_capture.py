#!/usr/bin/env python3
"""With the hardware e-stop ON (motors off): publish forward /cmd_vel at two
magnitudes while subscribing (BEST_EFFORT) to /avros/wheel_debug, then report
what the actuator COMMANDS the SparkMAX (L/R_rpm_sent) vs what it REPORTS back
(L/R_meas). Answers: is the phantom feedback a fixed value or does it track cmd?

wheel_debug.data = [L_sent,R_sent,L_meas,R_meas,L_pos,R_pos,v_req,w_req,
                    v_slewed,w_slewed,v,w,yaw,yaw_rate,heading_locked,estop]
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class Cap(Node):
    def __init__(self):
        super().__init__('estop_live_capture')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.rows = []          # (t, L_sent,R_sent,L_meas,R_meas,v)
        self.create_subscription(Float32MultiArray, '/avros/wheel_debug', self.cb, qos)

    def cb(self, m):
        d = m.data
        if len(d) >= 11:
            self.rows.append((time.time(), d[0], d[1], d[2], d[3], d[10]))

    def drive(self, vx, dur):
        t = Twist(); t.linear.x = float(vx)
        t0 = time.time()
        while time.time() - t0 < dur:
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def stop(self):
        z = Twist()
        for _ in range(15):
            self.pub.publish(z); rclpy.spin_once(self, timeout_sec=0.02); time.sleep(0.03)


def seg(rows, name):
    if not rows:
        print(f"  {name}: (no wheel_debug)"); return
    import statistics as s
    for i, lbl in [(1, 'L_sent'), (3, 'L_meas'), (5, 'odom_v')]:
        v = [r[i] for r in rows]
        print(f"  {name} {lbl:7s}: mean={s.mean(v):8.1f} min={min(v):8.1f} max={max(v):8.1f}")


def main():
    rclpy.init()
    n = Cap()
    time.sleep(1.0)
    print(">>> commanding 0.3 m/s (motors should be OFF)")
    s0 = len(n.rows); n.drive(0.3, 6); s1 = len(n.rows)
    print(">>> commanding 0.5 m/s")
    n.drive(0.5, 6); s2 = len(n.rows)
    n.stop()
    print(f"\nwheel_debug samples: {len(n.rows)}")
    seg(n.rows[s0:s1], "@cmd0.3")
    seg(n.rows[s1:s2], "@cmd0.5")
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
