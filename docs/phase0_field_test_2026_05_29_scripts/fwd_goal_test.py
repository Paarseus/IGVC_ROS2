#!/usr/bin/env python3
"""Phase 0 forward-goal instrumented test.

Snapshots the current map pose, sends a NavigateToPose goal `dist` metres straight
ahead (map frame, current stamp), and while the goal runs collects:
  - /cmd_vel timing  -> mean Hz + MAX GAP (the MPPI-starvation indicator)
  - /odometry/filtered (odom = truth) -> travel distance, max lateral deviation
  - /odometry/global (map) -> final position error vs target

Usage:  fwd_goal_test.py [dist_m]   (default 5.0)
Prints a human line + a JSON blob (parseable for the results doc).
"""
import sys, math, time, json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped


def yaw_of(q):
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class FwdTest(Node):
    def __init__(self):
        super().__init__('fwd_goal_test')
        self.map_pose = None
        self.cmd_t = []
        self.mpath = []   # map-frame path
        self.opath = []   # odom-frame (truth) path
        self.create_subscription(Odometry, '/odometry/global', self._map_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _map_cb(self, m):
        self.map_pose = m.pose.pose
        self.mpath.append((m.pose.pose.position.x, m.pose.pose.position.y))

    def _odom_cb(self, m):
        self.opath.append((m.pose.pose.position.x, m.pose.pose.position.y))

    def _cmd_cb(self, m):
        self.cmd_t.append(time.time())


def line_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy) or 1e-9
    return abs((px - ax) * dy - (py - ay) * dx) / L


def path_len(p):
    return sum(math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1]) for i in range(len(p) - 1))


def main():
    dist = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    n = FwdTest()
    t0 = time.time()
    while n.map_pose is None and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.map_pose is None:
        print("NO POSE on /odometry/global"); return
    p = n.map_pose
    yaw = yaw_of(p.orientation)
    sx, sy = p.position.x, p.position.y
    tx, ty = sx + dist * math.cos(yaw), sy + dist * math.sin(yaw)

    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = n.get_clock().now().to_msg()
    ps.pose.position.x, ps.pose.position.y = tx, ty
    ps.pose.orientation.z, ps.pose.orientation.w = p.orientation.z, p.orientation.w
    goal.pose = ps

    if not n.ac.wait_for_server(timeout_sec=10):
        print("NO ACTION SERVER"); return
    print(f"START map=({sx:.2f},{sy:.2f}) yaw={math.degrees(yaw):.1f}deg "
          f"target=({tx:.2f},{ty:.2f}) dist={dist}")

    # reset collectors at the moment of sending
    n.cmd_t, n.mpath, n.opath = [], [], []
    osx, osy = (n.opath[0] if n.opath else (0.0, 0.0))
    send = n.ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, send)
    gh = send.result()
    if not gh.accepted:
        print("GOAL REJECTED"); return
    t_start = time.time()
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=90)
    dur = time.time() - t_start
    status = rf.result().status if rf.done() else -1  # 4 = SUCCEEDED

    ct = sorted(n.cmd_t)
    if len(ct) > 2:
        mean_hz = (len(ct) - 1) / (ct[-1] - ct[0])
        gaps = [ct[i + 1] - ct[i] for i in range(len(ct) - 1)]
        max_gap = max(gaps)
        # fraction of inter-msg gaps slower than 1/18 s (the >=18 Hz criterion)
        slow = sum(1 for g in gaps if g > 1.0 / 18) / len(gaps)
    else:
        mean_hz, max_gap, slow = 0.0, 0.0, 1.0

    op = n.opath
    mp = n.mpath
    odom_travel = math.hypot(op[-1][0] - op[0][0], op[-1][1] - op[0][1]) if len(op) > 1 else 0.0
    odom_pathlen = path_len(op)
    max_lat = max((line_dist(px, py, op[0][0], op[0][1],
                             op[0][0] + dist * math.cos(yaw), op[0][1] + dist * math.sin(yaw))
                   for px, py in op), default=0.0)
    final_err = math.hypot(mp[-1][0] - tx, mp[-1][1] - ty) if mp else -1

    out = {
        "dist_cmd_m": dist, "status": status, "duration_s": round(dur, 1),
        "cmd_vel_mean_hz": round(mean_hz, 1), "cmd_vel_max_gap_s": round(max_gap, 3),
        "cmd_vel_frac_gaps_over_18hz": round(slow, 3),
        "odom_travel_m": round(odom_travel, 2), "odom_pathlen_m": round(odom_pathlen, 2),
        "max_lateral_dev_m": round(max_lat, 2), "final_pos_err_map_m": round(final_err, 2),
    }
    print("RESULT " + json.dumps(out))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
