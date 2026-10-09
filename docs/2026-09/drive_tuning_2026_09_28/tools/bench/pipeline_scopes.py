#!/usr/bin/env python3
"""Pipeline scopes through actuator_node (tracks OFF the ground). Needs actuator.launch.py running.
S5-H6: /cmd_vel publish -> actuator setpoint change (wheel_debug L_cmd) latency.
S4b:   omega flips +w -> -w through the pipeline (reversals, time to new direction).
S9:    /wheel_odom rate, gaps, standstill noise, twist lag vs position-derived speed."""
import json, os, statistics as st, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

M_PER_REV = 0.01994
OUT = os.path.expanduser('~/bench_scopes_2026_09_28')


class N(Node):
    def __init__(self):
        super().__init__('pipeline_scopes')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg, self.odom = [], []
        self.create_subscription(Float32MultiArray, '/avros/wheel_debug', lambda m: self.dbg.append((time.time(), list(m.data))), 100)
        self.create_subscription(Odometry, '/wheel_odom', lambda m: self.odom.append(
            (time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.twist.twist.linear.x, m.twist.twist.angular.z)), 100)

    def run(self, v, w, secs, publish=True):
        # callbacks are serviced by a background executor thread (every message), so this
        # loop only publishes at 50 Hz and never throttles reception
        t0 = time.time(); first = None
        while time.time() - t0 < secs:
            if publish:
                m = Twist(); m.linear.x = float(v); m.angular.z = float(w); self.pub.publish(m)
                first = first or time.time()
            time.sleep(0.02)
        return first


def pct(a, q):
    a = sorted(a); return a[min(len(a) - 1, int(q * (len(a) - 1) + 0.5))] if a else None


import threading
from rclpy.executors import MultiThreadedExecutor
rclpy.init(); n = N(); res = {}
ex = MultiThreadedExecutor(num_threads=2); ex.add_node(n)
threading.Thread(target=ex.spin, daemon=True).start()
n.run(0, 0, 2.0)

# S9 standstill noise + rate
n.odom.clear(); n.run(0, 0, 5.0)
t = [o[0] for o in n.odom]; d = [(b - a) * 1000 for a, b in zip(t, t[1:])]
res['S9_standstill'] = dict(rate_hz=round(len(n.odom) / 5.0, 1), gap_p99_ms=round(pct(d, 0.99), 1), gap_max_ms=round(max(d), 1),
                            v_sd=st.pstdev([o[2] for o in n.odom]), w_sd=st.pstdev([o[3] for o in n.odom]),
                            stamp_age_ms_p50=round(pct([(o[0] - o[1]) * 1000 for o in n.odom], 0.5), 1))

# S5-H6: cmd_vel -> actuator L setpoint change (the actuator slews, so measure first nonzero setpoint)
lat = []
for _ in range(6):
    n.run(0, 0, 1.5); n.dbg.clear()
    t_pub = n.run(0.3, 0.0, 0.6)
    ch = next((tt for tt, dd in n.dbg if tt >= t_pub and abs(dd[0]) > 1), None)
    if ch:
        lat.append((ch - t_pub) * 1000)
n.run(0, 0, 1.5)
res['S5_H6_cmdvel_to_setpoint_ms'] = dict(samples=[round(x, 1) for x in lat], p50=pct(lat, 0.5), max=max(lat) if lat else None)

# S9 twist lag during a speed change: compare /wheel_odom v with position-derived v from wheel_debug
n.dbg.clear(); n.odom.clear()
n.run(0.4, 0, 3.0); n.run(0.1, 0, 2.0); n.run(0.4, 0, 2.0); n.run(0, 0, 2.0)
pos = [(tt, (dd[4] + dd[5]) / 2) for tt, dd in n.dbg]
pv = []
for i in range(len(pos)):
    j = i
    while j + 1 < len(pos) and pos[j + 1][0] - pos[i][0] < 0.08:
        j += 1
    if pos[j][0] > pos[i][0]:
        pv.append(((pos[i][0] + pos[j][0]) / 2, (pos[j][1] - pos[i][1]) / (pos[j][0] - pos[i][0]) * M_PER_REV))
best = None
for k in range(0, 400, 5):
    e = []
    for tt, v in pv:
        tr = tt + k / 1000
        near = min(n.odom, key=lambda o: abs(o[0] - tr))
        if abs(near[0] - tr) < 0.03:
            e.append((near[2] - v) ** 2)
    if e and (best is None or st.mean(e) < best[1]):
        best = (k, st.mean(e))
res['S9_twist_lag_vs_position_ms'] = best[0] if best else None
res['S9_twist_rms_err_mps_at_best_lag'] = round(best[1] ** 0.5, 4) if best else None

# S4b: omega flips
flips = []
for w in (0.8, -0.8):
    n.dbg.clear()
    n.run(0, w, 3.0); t_flip = time.time(); n.run(0, -w, 3.0); n.run(0, 0, 1.5)
    pl = [(tt, dd[4]) for tt, dd in n.dbg if tt >= t_flip]
    sp = []
    for i in range(len(pl)):
        j = i
        while j + 1 < len(pl) and pl[j + 1][0] - pl[i][0] < 0.08:
            j += 1
        if pl[j][0] > pl[i][0]:
            sp.append((pl[i][0] - t_flip, (pl[j][1] - pl[i][1]) / (pl[j][0] - pl[i][0]) * 60))
    target = next((dd[0] for tt, dd in reversed(n.dbg) if tt < t_flip + 2.5 and abs(dd[0]) > 1), None)
    t_reach = next((tt for tt, v in sp if target and abs(v - target) <= 0.1 * abs(target)), None)
    signs = [1 if v > 40 else -1 if v < -40 else 0 for _, v in sp[:int(len(sp) * 0.9)]]
    rev = sum(1 for a, b in zip(signs, signs[1:]) if a and b and a != b)
    flips.append(dict(from_w=w, to_w=-w, left_target_rpm=round(target) if target else None,
                      time_to_within_10pct_s=round(t_reach, 2) if t_reach else None, sign_changes=rev))
res['S4b_omega_flips'] = flips

n.run(0, 0, 1.0)
for k, v in res.items():
    print(k, v)
json.dump(res, open(os.path.join(OUT, 'pipeline_scopes.json'), 'w'), indent=1)
ex.shutdown(); n.destroy_node(); rclpy.shutdown()
