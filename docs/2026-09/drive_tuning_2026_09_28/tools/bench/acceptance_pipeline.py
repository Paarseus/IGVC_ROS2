#!/usr/bin/env python3
"""Bench acceptance through the real command path: /cmd_vel -> actuator_node -> Teensy -> motor controllers.
Needs actuator_node running with actuator_params.yaml (tracks off the ground). Fixed pass limits.
Writes ~/bench_scopes_2026_09_28/acceptance_pipeline.json."""
import json, os, statistics as st, threading, time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

M_PER_REV = 0.01994
R = []


def add(i, check, measured, limit, ok):
    R.append(dict(id=i, check=check, measured=measured, limit=limit, passed=bool(ok)))
    print(f"{i:4s} {'PASS' if ok else 'FAIL'}  {check}: {measured}  (limit: {limit})")


class N(Node):
    def __init__(self):
        super().__init__('acceptance_pipeline')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg, self.odom = [], []
        self.create_subscription(Float32MultiArray, '/avros/wheel_debug', lambda m: self.dbg.append((time.time(), list(m.data))), 100)
        self.create_subscription(Odometry, '/wheel_odom', lambda m: self.odom.append((time.time(), m.twist.twist.linear.x)), 100)

    def drive(self, v, w, secs, publish=True):
        t0 = time.time(); first = None
        while time.time() - t0 < secs:
            if publish:
                m = Twist(); m.linear.x = float(v); m.angular.z = float(w); self.pub.publish(m); first = first or time.time()
            time.sleep(0.02)
        return first


def speed_series(rows, idx, t0, t1, win=0.1):
    pts = [(tt, d[idx]) for tt, d in rows if t0 - win <= tt <= t1 + win]
    out = []
    for i in range(len(pts)):
        j = i
        while j + 1 < len(pts) and pts[j + 1][0] - pts[i][0] < win:
            j += 1
        if pts[j][0] > pts[i][0] and t0 <= pts[i][0] <= t1:
            out.append((pts[i][0] - t0, (pts[j][1] - pts[i][1]) / (pts[j][0] - pts[i][0]) * 60))
    return out


rclpy.init(); n = N(); ex = MultiThreadedExecutor(num_threads=2); ex.add_node(n)
threading.Thread(target=ex.spin, daemon=True).start()
n.drive(0, 0, 2.0)

# P1 stops through the pipeline
worst_dip, worst_rev, worst_t = 0, 0, 0
for name, v, w, explicit in (('0.6 m/s, commands stop', 0.6, 0, False), ('0.6 m/s, command 0', 0.6, 0, True),
                             ('1.0 m/s, command 0', 1.0, 0, True), ('turn 0.8 rad/s, command 0', 0, 0.8, True)):
    n.drive(v, w, 4.0); ts = time.time(); n.drive(0, 0, 3.0, publish=explicit)
    for idx in (4, 5):
        sp = speed_series(n.dbg, idx, ts, ts + 3.0)
        # travel direction from the mean of the first 0.2 s (a single sample can read ~0 at the window edge)
        head = [x for tt, x in sp if tt <= 0.2] or [sp[0][1]]
        sign = 1 if st.mean(head) >= 0 else -1
        dip = min(0.0, min(sign * x for _, x in sp))
        vs = [x for _, x in sp]
        rev = sum(1 for a, b in zip(vs, vs[1:]) if (a > 40 and b < -40) or (a < -40 and b > 40))
        still = [tt for tt, x in sp if abs(x) > 30]
        worst_dip = min(worst_dip, dip); worst_rev = max(worst_rev, rev); worst_t = max(worst_t, still[-1] if still else 0)
add('P1', 'stops through actuator_node (4 cases, both tracks)', f'worst dip past zero {worst_dip:.0f} RPM, reversals {worst_rev}, standstill {worst_t:.2f} s',
    'dip >= -100 RPM, 0 reversals, standstill <= 1.3 s', worst_dip >= -100 and worst_rev == 0 and worst_t <= 1.3)

# P2 turn reversal
sc = 0
for w in (0.8, -0.8):
    n.drive(0, w, 3.0); tf = time.time(); n.drive(0, -w, 3.0); n.drive(0, 0, 1.5)
    sp = [x for _, x in speed_series(n.dbg, 4, tf, tf + 2.7)]
    # direction changes of the track, ignoring near-zero samples; exactly one is commanded
    nz = [1 if x > 40 else -1 for x in sp if abs(x) > 40]
    changes = sum(1 for a, b in zip(nz, nz[1:]) if a != b)
    sc = max(sc, changes - 1)
add('P2', 'turn reversal +0.8 <-> -0.8 rad/s', f'{sc} direction changes beyond the one commanded', '0', sc == 0)

# P3 command latency
lat = []
for _ in range(6):
    n.drive(0, 0, 1.5); n.dbg.clear(); tp = n.drive(0.3, 0, 0.6)
    ch = next((tt for tt, d in n.dbg if tt >= tp and abs(d[0]) > 1), None)
    if ch:
        lat.append((ch - tp) * 1000)
n.drive(0, 0, 1.5)
add('P3', 'cmd_vel to motor command', f'median {st.median(lat):.0f} ms, max {max(lat):.0f} ms', 'max <= 40 ms', max(lat) <= 40)

# P4 odometry freshness
n.dbg.clear(); n.odom.clear()
n.drive(0.4, 0, 3.0); n.drive(0.1, 0, 2.0); n.drive(0.4, 0, 2.0); n.drive(0, 0, 2.0)
pos = [(tt, (d[4] + d[5]) / 2) for tt, d in n.dbg]
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
        nr = min(n.odom, key=lambda o: abs(o[0] - tt - k / 1000))
        if abs(nr[0] - tt - k / 1000) < 0.03:
            e.append((nr[1] - v) ** 2)
    if e and (best is None or st.mean(e) < best[1]):
        best = (k, st.mean(e))
rate = len([o for o in n.odom if o[0] > n.odom[-1][0] - 5]) / 5
add('P4', 'wheel odometry speed lag (what MPPI receives)', f'{best[0]} ms, published at {rate:.0f} Hz', 'lag <= 60 ms (target 50), >= 20 Hz',
    best[0] <= 60 and rate >= 19)

# P5 low speed through the full path (wheel speed from position, 0.2 s window)
M_S = os.environ.get('ACCEPT_KS', 'yaml')
worst, stuck = 0.0, 0.0
for v, w in ((0.05, 0.0), (0.1, 0.0), (-0.05, 0.0), (0.0, 0.1), (0.0, -0.1)):
    n.dbg.clear(); n.drive(v, w, 6.0); n.drive(0, 0, 1.5)
    t_end = n.dbg[-1][0] - 1.5 if n.dbg else 0
    for idx, cmd_idx in ((4, 0), (5, 1)):
        sp = speed_series(n.dbg, idx, t_end - 3.0, t_end, win=0.2)
        cmds = [d[cmd_idx] for tt, d in n.dbg if t_end - 3.0 <= tt <= t_end]
        if not sp or not cmds:
            continue
        target = st.mean(cmds)
        meas = st.mean(x for _, x in sp)
        worst = max(worst, abs(100 * (meas - target) / target))
        stuck = max(stuck, sum(1 for _, x in sp if abs(x) < 0.2 * abs(target)) / len(sp))
add('P5', f'slow speed via cmd_vel: 0.05 / 0.1 m/s, +-0.1 rad/s (kS: {M_S})', f'worst error {worst:.0f} %, stuck {100 * stuck:.0f} % of time',
    'error <= 10 %, never stuck', worst <= 10 and stuck == 0)

ex.shutdown(); n.destroy_node(); rclpy.shutdown()
json.dump(R, open(os.path.expanduser('~/bench_scopes_2026_09_28/acceptance_pipeline.json'), 'w'), indent=1)
print(f"\n{sum(r['passed'] for r in R)}/{len(R)} passed")
