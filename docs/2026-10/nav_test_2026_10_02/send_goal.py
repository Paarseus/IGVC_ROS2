#!/usr/bin/env python3
"""Send one NavigateToPose goal relative to the robot's current pose, and record what happens.

  send_goal.py --ahead 8 [--lateral 0] [--timeout 120] [--outdir DIR] [--label NAME]

The goal is in the MAP frame, ahead of the robot's current heading (vx_min is 0: the robot never drives in reverse,
so goals must be ahead). Run on the Jetson with the navigation stack up. Prints one status line per second and a
summary at the end; writes <outdir>/<label>.csv (t, x, y, speed, dist_to_goal, clearance, cmd_vel rate).
Clearance = distance from base_link to the nearest lethal/inscribed cell of the LOCAL costmap (what MPPI sees);
measure the true closest approach with a tape as well.  Ctrl-C cancels the goal.
"""
import argparse, math, os, sys, time

import numpy as np


def goal_from_pose(x, y, yaw, ahead, lateral=0.0):
    """Goal position ahead (along yaw) and to the left (lateral) of the pose; heading unchanged."""
    gx = x + ahead * math.cos(yaw) - lateral * math.sin(yaw)
    gy = y + ahead * math.sin(yaw) + lateral * math.cos(yaw)
    return gx, gy, yaw


def nearest_lethal(data, width, height, res, ox, oy, rx, ry, thresh=99):
    """Distance from (rx, ry) to the nearest cell with value >= thresh in a row-major occupancy grid; inf if none."""
    g = np.asarray(data, dtype=np.int16).reshape(height, width)
    iy, ix = np.nonzero(g >= thresh)
    if iy.size == 0:
        return float('inf')
    cx = ox + (ix + 0.5) * res
    cy = oy + (iy + 0.5) * res
    return float(np.min(np.hypot(cx - rx, cy - ry)))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ahead', type=float, required=True)
    ap.add_argument('--lateral', type=float, default=0.0)
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--outdir', default=os.path.expanduser('~/nav_tests'))
    ap.add_argument('--label', default=time.strftime('goal_%H%M%S'))
    a = ap.parse_args()

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry, OccupancyGrid
    from tf2_ros import Buffer, TransformListener
    from action_msgs.msg import GoalStatus

    rclpy.init()
    node = Node('send_goal_metrics')
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    st = dict(odom=None, grid=None, cmd_n=0, cmd_t0=None)
    node.create_subscription(Odometry, '/odometry/filtered', lambda m: st.update(odom=m), 10)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: st.update(grid=m),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))

    def on_cmd(_):
        if st['cmd_t0'] is None:
            st['cmd_t0'] = time.time()
        st['cmd_n'] += 1
    node.create_subscription(Twist, '/cmd_vel', on_cmd, 10)

    def spin(sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(node, timeout_sec=0.05)

    def pose_map():
        t = tfbuf.lookup_transform('map', 'base_link', rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    spin(2.0)
    try:
        x, y, yaw = pose_map()
    except Exception as e:
        sys.exit(f'no map->base_link transform: {e}')
    gx, gy, gyaw = goal_from_pose(x, y, yaw, a.ahead, a.lateral)
    print(f'start (map): {x:.2f}, {y:.2f}, heading {math.degrees(yaw):.0f} deg   goal: {gx:.2f}, {gy:.2f}  ({a.ahead} m ahead, {a.lateral} m left)')

    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=15.0):
        sys.exit('navigate_to_pose action server not available')
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    fut = client.send_goal_async(goal)
    while not fut.done():
        spin(0.05)
    handle = fut.result()
    if not handle.accepted:
        sys.exit('goal rejected')
    res_fut = handle.get_result_async()

    os.makedirs(a.outdir, exist_ok=True)
    csv = open(os.path.join(a.outdir, a.label + '.csv'), 'w')
    csv.write('t,x,y,speed,dist_to_goal,clearance_m,cmd_rate_hz\n')
    t0 = time.time(); last_print = 0; path = 0.0; prev = (x, y); min_clear = float('inf'); cancelled = False
    try:
        while not res_fut.done():
            spin(0.1)
            now = time.time() - t0
            try:
                cx, cy, _ = pose_map()
            except Exception:
                continue
            path += math.hypot(cx - prev[0], cy - prev[1]); prev = (cx, cy)
            speed = st['odom'].twist.twist.linear.x if st['odom'] else float('nan')
            clear = float('inf')
            g = st['grid']
            if g is not None and st['odom'] is not None:
                o = st['odom'].pose.pose.position
                clear = nearest_lethal(g.data, g.info.width, g.info.height, g.info.resolution,
                                       g.info.origin.position.x, g.info.origin.position.y, o.x, o.y)
            min_clear = min(min_clear, clear)
            rate = st['cmd_n'] / (time.time() - st['cmd_t0']) if st['cmd_t0'] else 0.0
            d = math.hypot(gx - cx, gy - cy)
            csv.write(f'{now:.2f},{cx:.3f},{cy:.3f},{speed:.3f},{d:.3f},{clear:.2f},{rate:.1f}\n')
            if now - last_print >= 1.0:
                last_print = now
                print(f't {now:5.1f} s  to goal {d:5.2f} m  speed {speed:4.2f} m/s  nearest obstacle cell {clear:5.2f} m  cmd_vel {rate:4.1f} Hz', flush=True)
            if now > a.timeout:
                print('timeout: cancelling'); handle.cancel_goal_async(); cancelled = True; break
    except KeyboardInterrupt:
        print('Ctrl-C: cancelling'); handle.cancel_goal_async(); cancelled = True
    spin(1.0)
    status = res_fut.result().status if res_fut.done() else None
    names = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED', GoalStatus.STATUS_CANCELED: 'CANCELED'}
    try:
        cx, cy, _ = pose_map(); final_d = math.hypot(gx - cx, gy - cy)
    except Exception:
        final_d = float('nan')
    rate = st['cmd_n'] / (time.time() - st['cmd_t0']) if st['cmd_t0'] else 0.0
    print('\n== summary ==')
    print(f'result: {names.get(status, status)}{" (cancelled by us)" if cancelled else ""}')
    print(f'time {time.time() - t0:.1f} s, path {path:.2f} m (straight line {a.ahead:.2f} m), final distance to goal {final_d:.2f} m')
    print(f'closest obstacle cell seen by the local costmap: {min_clear:.2f} m from base_link (tape-measure the real closest approach)')
    print(f'mean /cmd_vel rate {rate:.1f} Hz (need >= 18)')
    csv.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
