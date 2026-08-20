#!/usr/bin/env python3
"""auto_drive — operator loop for point-and-go autonomous driving.

Workflow (one run = one destination):

    1. Pre-flight: refuse to run unless the whole stack is genuinely healthy
       AND the operator has cleared e-stop and engaged AUTO.
    2. Wait for the operator to click a destination in RViz (Publish Point).
    3. Plan a route over the campus road graph, draw it in RViz.
    4. Print a summary and wait for explicit confirmation.
    5. Drive it leg-by-leg, hands-off, aborting instantly on e-stop.

Companion launch file brings the stack up:  avros_bringup auto_drive.launch.py
Leave the stack running and re-run this node once per destination.

WHY LEGS: the global costmap is a 100 m rolling window centred on the robot
(~50 m usable radius). NavfnPlanner cannot plan to a goal outside it -- it
returns "goal off the global costmap" and aborts. So a long route is chopped
into LEG_SPACING_M hops, each comfortably inside the window, and driven in
sequence. This mirrors the "dumb BT, smart orchestrator" split the competition
behaviour tree documents.
"""

import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2, NavSatFix
from nav2_msgs.action import ComputeRoute, NavigateToPose
from avros_msgs.msg import ActuatorState
from lifecycle_msgs.srv import GetState

# --- tunables -------------------------------------------------------------
LEG_SPACING_M = 30.0      # < ~50 m global-costmap radius, with margin
LEG_TIMEOUT_S = 90.0      # > the BT's own 45 s internal timeout
LEG_RETRIES = 2           # outer retries; the BT already retries 4x internally
STALE_AFTER_S = 3.0       # a topic older than this counts as not publishing
MAX_LOAD_PER_CORE = 1.6   # load average per core; above this MPPI starts failing

REQUIRED_SERVERS = [
    'controller_server', 'planner_server', 'bt_navigator',
    'behavior_server', 'velocity_smoother', 'smoother_server', 'route_server',
]

GOAL_STATUS_SUCCEEDED = 4


class AutoDrive(Node):
    def __init__(self):
        super().__init__('auto_drive')

        self.pose = None
        self.pose_t = 0.0
        self.estop = True          # safe default until told otherwise
        self.estop_t = 0.0
        self.autonomous = False
        self.cloud_t = 0.0
        self.clicked = None
        self.gps = None
        self.gps_t = 0.0

        self.create_subscription(Odometry, '/odometry/global', self._on_odom, 10)
        self.create_subscription(ActuatorState, '/avros/actuator_state', self._on_state, 10)
        self.create_subscription(Bool, '/autonomous_mode', self._on_auto, 10)
        self.create_subscription(PointCloud2, '/perception/costmap_cloud', self._on_cloud, 1)
        self.create_subscription(PointStamped, '/clicked_point', self._on_click, 10)
        self.create_subscription(NavSatFix, '/gnss', self._on_gps, 10)

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.preview_pub = self.create_publisher(Path, '/auto_drive/route_preview', latched)

        self.route_client = ActionClient(self, ComputeRoute, 'compute_route')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    # ---------------------------------------------------------- subscriptions
    def _on_odom(self, msg):
        self.pose = msg.pose.pose
        self.pose_t = time.monotonic()

    def _on_state(self, msg):
        self.estop = msg.estop
        self.estop_t = time.monotonic()

    def _on_auto(self, msg):
        self.autonomous = msg.data

    def _on_cloud(self, _msg):
        self.cloud_t = time.monotonic()

    def _on_click(self, msg):
        self.clicked = (msg.point.x, msg.point.y)

    def _on_gps(self, msg):
        self.gps = msg
        self.gps_t = time.monotonic()

    # ------------------------------------------------------------- pre-flight
    def _server_active(self, name):
        """True only if the node reports lifecycle state 'active'.

        Checking that a PROCESS exists is not enough -- we were bitten by
        exactly that: controller_server had been killed while its name still
        appeared in a lifecycle_manager argument string, and separately the
        whole managed set sat in 'inactive' after a bond break. Both looked
        healthy to a ps-grep and neither could drive.
        """
        cli = self.create_client(GetState, f'/{name}/get_state')
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, 'no get_state service'
        fut = cli.call_async(GetState.Request())
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not fut.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not fut.done() or fut.result() is None:
            return False, 'get_state timed out'
        label = fut.result().current_state.label
        return (label == 'active'), label

    def preflight(self):
        print('\n=== PRE-FLIGHT ===')
        ok = True

        for name in REQUIRED_SERVERS:
            active, label = self._server_active(name)
            print(f'  {"OK  " if active else "FAIL"}  {name:<20} {label}')
            ok &= active

        now = time.monotonic()
        checks = [
            ('GPS/EKF pose  (/odometry/global)', self.pose is not None and (now - self.pose_t) < STALE_AFTER_S),
            ('vision costmap (/perception/costmap_cloud)', self.cloud_t > 0 and (now - self.cloud_t) < STALE_AFTER_S),
            ('actuator state (/avros/actuator_state)', self.estop_t > 0 and (now - self.estop_t) < STALE_AFTER_S),
        ]
        for label, good in checks:
            print(f'  {"OK  " if good else "FAIL"}  {label}')
            ok &= good

        # GPS VALIDITY -- not just liveness.
        #
        # /odometry/global keeps publishing at 20 Hz even with no satellite
        # fix: navsat_transform happily projects lat/lon 0,0 ("null island")
        # against the campus datum and emits a position ~900 km away. The
        # freshness check above passes on that, so without this check
        # auto_drive would plan a route from a garbage start position.
        # Observed live on 2026-08-20 -- this check exists because of it.
        gps_fresh = self.gps is not None and (now - self.gps_t) < STALE_AFTER_S
        if not gps_fresh:
            print('  FAIL  GPS fix        (/gnss not publishing)')
            ok = False
        else:
            status = self.gps.status.status          # -1 = NO_FIX
            null_island = abs(self.gps.latitude) < 1e-6 and abs(self.gps.longitude) < 1e-6
            gps_ok = status >= 0 and not null_island
            detail = (f'status={status} lat={self.gps.latitude:.6f} '
                      f'lon={self.gps.longitude:.6f}')
            print(f'  {"OK  " if gps_ok else "FAIL"}  GPS fix        {detail}')
            if not gps_ok:
                print('        -> NO satellite fix. Everything downstream (map pose,')
                print('           routing, waypoints) is meaningless without it.')
                print('           Check the antenna and that the vehicle has sky view.')
            ok &= gps_ok

        # CPU headroom. Sustained overload starved the 20 Hz MPPI control loop
        # and produced "Optimizer fail to compute path" plus erratic motion.
        try:
            import os
            load1 = os.getloadavg()[0]
            ncpu = os.cpu_count() or 1
            per_core = load1 / ncpu
            good = per_core < MAX_LOAD_PER_CORE
            print(f'  {"OK  " if good else "WARN"}  CPU load {load1:.1f} over {ncpu} cores '
                  f'({per_core:.2f}/core, limit {MAX_LOAD_PER_CORE})')
            if not good:
                print('        -> close extra RViz windows / dashboards before driving')
                ok = False
        except OSError:
            print('  WARN  could not read load average')

        # Operator safety interlock. Deliberately last so it reads as the
        # final gate, and deliberately fatal: this node never clears e-stop
        # or engages AUTO itself -- that stays a human action at the joystick.
        print(f'  {"OK  " if not self.estop else "FAIL"}  e-stop cleared')
        print(f'  {"OK  " if self.autonomous else "FAIL"}  AUTO engaged')
        ok &= (not self.estop) and self.autonomous

        print('=== PRE-FLIGHT ' + ('PASSED ===' if ok else 'FAILED ===') + '\n')
        if not ok:
            print('Fix the FAIL lines above, then re-run. If e-stop/AUTO are the')
            print('only failures: open the joystick page, then press AUTO.')
        return ok

    # ------------------------------------------------------------------ route
    def plan_route(self, goal_xy):
        """Route from the CURRENT live pose to goal_xy over the road graph."""
        if not self.route_client.wait_for_server(timeout_sec=5.0):
            print('  route_server action unavailable')
            return None

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose = self.pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x, goal_pose.pose.position.y = goal_xy
        goal_pose.pose.orientation.w = 1.0

        goal = ComputeRoute.Goal()
        goal.use_start = True
        goal.use_poses = True
        goal.start = start
        goal.goal = goal_pose

        fut = self.route_client.send_goal_async(goal)
        gh = self._await(fut, 10.0)
        if gh is None or not gh.accepted:
            print('  route goal rejected')
            return None
        res = self._await(gh.get_result_async(), 20.0)
        if res is None or not res.result.path.poses:
            print('  route_server returned an empty path')
            return None
        return res.result.path

    def _await(self, fut, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not fut.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    @staticmethod
    def path_length(path):
        total = 0.0
        pts = path.poses
        for a, b in zip(pts[:-1], pts[1:]):
            dx = a.pose.position.x - b.pose.position.x
            dy = a.pose.position.y - b.pose.position.y
            total += (dx * dx + dy * dy) ** 0.5
        return total

    @staticmethod
    def to_legs(path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        if not pts:
            return []
        legs, acc, last = [], 0.0, pts[0]
        for x, y in pts[1:]:
            acc += ((x - last[0]) ** 2 + (y - last[1]) ** 2) ** 0.5
            last = (x, y)
            if acc >= LEG_SPACING_M:
                legs.append((x, y))
                acc = 0.0
        if not legs or legs[-1] != pts[-1]:
            legs.append(pts[-1])
        return legs

    # ------------------------------------------------------------------ drive
    def drive_leg(self, xy, idx, total):
        """Drive one leg. Returns True on SUCCEEDED. Aborts on e-stop."""
        if self.estop:
            print(f'  leg {idx}/{total}: ABORT — e-stop engaged')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = xy
        goal.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            print('  navigate_to_pose unavailable')
            return False

        gh = self._await(self.nav_client.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            print(f'  leg {idx}/{total}: goal REJECTED')
            return False

        res_fut = gh.get_result_async()
        deadline = time.monotonic() + LEG_TIMEOUT_S
        while rclpy.ok() and not res_fut.done() and time.monotonic() < deadline:
            if self.estop:
                print(f'  leg {idx}/{total}: E-STOP mid-leg — cancelling')
                self._await(gh.cancel_goal_async(), 5.0)
                return False
            time.sleep(0.1)

        if not res_fut.done():
            print(f'  leg {idx}/{total}: TIMEOUT after {LEG_TIMEOUT_S:.0f}s — cancelling')
            self._await(gh.cancel_goal_async(), 5.0)
            return False

        status = res_fut.result().status
        if status == GOAL_STATUS_SUCCEEDED:
            print(f'  leg {idx}/{total}: reached')
            return True
        print(f'  leg {idx}/{total}: FAILED (status={status})')
        return False

    def run_to(self, goal_xy):
        """Plan and drive to goal_xy, re-planning fresh on each retry."""
        for attempt in range(1, LEG_RETRIES + 2):
            if attempt > 1:
                print(f'\n-- retry {attempt - 1}/{LEG_RETRIES}: re-planning from current position --')
                time.sleep(2.0)
                if self.estop:
                    print('   e-stop engaged; not retrying')
                    return False

            path = self.plan_route(goal_xy)
            if path is None:
                continue
            legs = self.to_legs(path)
            print(f'   {self.path_length(path):.0f} m over {len(legs)} legs')

            for i, xy in enumerate(legs, 1):
                if not self.drive_leg(xy, i, len(legs)):
                    break
            else:
                return True     # every leg succeeded
        return False


def main():
    rclpy.init()
    node = AutoDrive()

    # Spin in the background so callbacks (crucially e-stop) keep flowing
    # while the main thread blocks on operator input.
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    time.sleep(2.0)   # let subscriptions latch before we judge freshness

    def shutdown(code):
        """Tear down in order. Calling rclpy.shutdown() while the executor is
        still spinning aborts the process ('terminate called without an active
        exception'), which would mask any real error we were trying to report.
        """
        ex.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        return code

    if not node.preflight():
        return shutdown(1)

    print('Click a destination in RViz using the "Publish Point" tool...')
    node.clicked = None
    while rclpy.ok() and node.clicked is None:
        time.sleep(0.1)
    goal_xy = node.clicked
    print(f'Destination: ({goal_xy[0]:.1f}, {goal_xy[1]:.1f})')

    path = node.plan_route(goal_xy)
    if path is None:
        print('Could not plan a route there. Pick a point on a road.')
        return shutdown(1)

    node.preview_pub.publish(path)
    legs = node.to_legs(path)
    print(f'\nRoute: {node.path_length(path):.0f} m, {len(legs)} legs '
          f'(shown in RViz as /auto_drive/route_preview)')
    print('Review the route in RViz. Keep your hand on the e-stop.')

    try:
        answer = input('Drive it? [y/N] ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        answer = ''
    if answer not in ('y', 'yes'):
        print('Cancelled. Nothing was sent to the vehicle.')
        return shutdown(0)

    # Re-check the interlock: time passed while the operator was reading.
    if node.estop or not node.autonomous:
        print('ABORT — e-stop re-engaged or AUTO dropped while confirming.')
        return shutdown(1)

    print('\n=== DRIVING ===')
    ok = node.run_to(goal_xy)
    print('\n=== ARRIVED ===' if ok else '\n=== STOPPED — did not reach destination ===')
    if not ok:
        print('Vehicle is stopped. Re-run to try again, or drive manually.')

    return shutdown(0 if ok else 1)


if __name__ == '__main__':
    sys.exit(main())
