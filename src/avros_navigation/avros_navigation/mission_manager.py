"""mission_manager — IGVC AutoNav waypoint orchestrator.

Drives the navigate_igvc_autonav_humble.xml BT by feeding it sequential
NavigateToPose goals from a YAML waypoint list. The BT is "dumb" — it
plans, follows, and recovers; this node owns the waypoint cursor and
advances it on proximity.

Phase M6 scope: skip-on-failure for ABORTED/CANCELED of the current
waypoint, and a 10 s "mission complete" heartbeat after the list is
exhausted (so an operator can see the node is alive and idling).
Launch-file integration lives in navigation.launch.py.
"""
import math
import sys
from typing import List, Optional, Tuple

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from robot_localization.srv import FromLL
from std_msgs.msg import Bool


# Latched QoS for /autonomous_mode so actuator_node picks up the current state
# even if it (re)starts mid-mission. Must match the subscriber in actuator_node.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

FROMLL_WAIT_TIMEOUT_S = 30.0
FROMLL_CALL_TIMEOUT_S = 5.0
NAV2_WAIT_TIMEOUT_S = 30.0
PROXIMITY_TICK_HZ = 5.0
DONE_HEARTBEAT_S = 10.0
GOAL_FRAME_ID = 'map'


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('acceptance_radius_m', 2.0)
        self.declare_parameter('odom_topic', '/odometry/global')

        self._waypoints_file = self.get_parameter('waypoints_file').value
        self._radius = self.get_parameter('acceptance_radius_m').value
        self._odom_topic = self.get_parameter('odom_topic').value

        self.get_logger().info(
            f'mission_manager started '
            f'(waypoints_file={self._waypoints_file or "<unset>"}, '
            f'acceptance_radius_m={self._radius}, '
            f'odom_topic={self._odom_topic})'
        )

        self._ll_waypoints = self._load_waypoints(self._waypoints_file)
        self._fromll_client = self.create_client(FromLL, '/fromLL')
        self._map_waypoints: List[Point] = self._convert_to_map_frame(self._ll_waypoints)

        # IGVC §I.2 safety light: flag the vehicle as autonomous for the whole
        # mission (first goal accepted -> mission complete), latched so a
        # restarted actuator_node re-syncs. Published once on each transition,
        # not per-waypoint, so the light never flickers between goals.
        self._auto_pub = self.create_publisher(Bool, '/autonomous_mode', LATCHED_QOS)
        self._autonomous = None
        self._set_autonomous(False)

        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._goal_handle = None
        self._cursor = 0
        self._latest_robot_xy: Optional[Tuple[float, float]] = None
        if not self._nav_client.wait_for_server(timeout_sec=NAV2_WAIT_TIMEOUT_S):
            self.get_logger().error(
                f'/navigate_to_pose action server not available after '
                f'{NAV2_WAIT_TIMEOUT_S}s — is bt_navigator running?'
            )
            raise SystemExit(1)

        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, 10
        )
        self._proximity_timer = self.create_timer(
            1.0 / PROXIMITY_TICK_HZ, self._on_proximity_tick
        )
        self._done_timer = None
        self._send_goal(self._cursor)

    def _load_waypoints(self, path: str) -> List[Tuple[float, float]]:
        if not path:
            self.get_logger().error('waypoints_file param is empty — pass with -p waypoints_file:=<path>')
            raise SystemExit(1)
        try:
            with open(path, 'r') as f:
                doc = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f'waypoints_file not found: {path}')
            raise SystemExit(1)

        entries = doc.get('waypoints') if isinstance(doc, dict) else None
        if not entries:
            self.get_logger().error(f'waypoints_file has no `waypoints:` list: {path}')
            raise SystemExit(1)

        out: List[Tuple[float, float]] = []
        for i, entry in enumerate(entries):
            if not isinstance(entry, dict) or 'lat' not in entry or 'lon' not in entry:
                self.get_logger().error(f'waypoint[{i}] missing lat/lon: {entry!r}')
                raise SystemExit(1)
            out.append((float(entry['lat']), float(entry['lon'])))

        self.get_logger().info(
            f'Loaded {len(out)} waypoints from {path}: ' +
            ', '.join(f'({lat:.6f}, {lon:.6f})' for lat, lon in out)
        )
        return out

    def _convert_to_map_frame(self, ll_waypoints: List[Tuple[float, float]]) -> List[Point]:
        if not self._fromll_client.wait_for_service(timeout_sec=FROMLL_WAIT_TIMEOUT_S):
            self.get_logger().error(
                f'/fromLL service not available after {FROMLL_WAIT_TIMEOUT_S}s — '
                f'is navsat_transform_node running?'
            )
            raise SystemExit(1)

        out: List[Point] = []
        for i, (lat, lon) in enumerate(ll_waypoints):
            request = FromLL.Request()
            request.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
            future = self._fromll_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=FROMLL_CALL_TIMEOUT_S)
            if not future.done() or future.result() is None:
                self.get_logger().error(
                    f'/fromLL call timed out for waypoint[{i}] ({lat:.6f}, {lon:.6f})'
                )
                raise SystemExit(1)
            point = future.result().map_point
            out.append(point)
            self.get_logger().info(
                f'Converted wp{i} ({lat:.6f}, {lon:.6f}) -> map frame ({point.x:.3f}, {point.y:.3f})'
            )
        return out

    def _set_autonomous(self, val: bool) -> None:
        """Publish the §I.2 autonomy flag, but only on an actual change."""
        if val == self._autonomous:
            return
        self._autonomous = val
        self._auto_pub.publish(Bool(data=val))
        self.get_logger().info(
            f'autonomous_mode -> {val} (safety light {"FLASH" if val else "SOLID"})'
        )

    def _send_goal(self, index: int) -> None:
        point = self._map_waypoints[index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = GOAL_FRAME_ID
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = point
        goal_msg.pose.pose.orientation.w = 1.0
        self.get_logger().info(
            f'Sending wp{index} -> NavigateToPose ({point.x:.3f}, {point.y:.3f}) in {GOAL_FRAME_ID}'
        )
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._on_goal_response(f, index))

    def _on_goal_response(self, future, index: int) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'wp{index} REJECTED by /navigate_to_pose')
            self._goal_handle = None
            return
        self.get_logger().info(f'wp{index} accepted by /navigate_to_pose')
        self._set_autonomous(True)   # navigating -> flash the safety light
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._on_result(f, index)
        )

    def _on_result(self, future, index: int) -> None:
        status = future.result().status
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(status, f'UNKNOWN({status})')
        self.get_logger().info(f'wp{index} result: {status_str}')
        # Only act on results for the CURRENT cursor. An ABORTED/CANCELED
        # result for a previously-canceled goal can arrive AFTER the proximity
        # tick has already advanced to the next waypoint and accepted a new
        # goal — touching _goal_handle or _cursor here would wipe the new
        # state and stall the orchestrator.
        if index != self._cursor:
            return
        self._goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            # BT got there before our proximity tick fired — most common when
            # the robot is already inside the radius at goal send time, e.g.
            # starting at the datum for wp0.
            self._advance_cursor(reason=f'wp{index} SUCCEEDED')
        elif status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            # BT gave up on this waypoint before we reached its acceptance
            # radius. IGVC scores by waypoint count, not by no-failures, so
            # skip-on-failure: log and advance to the next waypoint.
            self.get_logger().warn(
                f'wp{index} did not complete ({status_str}) — skip-on-failure, advancing'
            )
            self._advance_cursor(reason=f'wp{index} {status_str}')

    def _advance_cursor(self, reason: str) -> None:
        self._cursor += 1
        if self._cursor >= len(self._map_waypoints):
            self._enter_done_state(reason)
            return
        self._send_goal(self._cursor)

    def _enter_done_state(self, reason: str) -> None:
        self.get_logger().info(
            f'all {len(self._map_waypoints)} waypoints reached — mission complete ({reason})'
        )
        self._set_autonomous(False)   # out of autonomous -> safety light solid
        if self._proximity_timer is not None:
            self._proximity_timer.cancel()
            self._proximity_timer = None
        if self._done_timer is None:
            self._done_timer = self.create_timer(DONE_HEARTBEAT_S, self._on_done_heartbeat)

    def _on_done_heartbeat(self) -> None:
        self.get_logger().info('mission complete — idling')

    def destroy_node(self) -> None:
        # Best-effort: dropping out of autonomous on shutdown -> light solid.
        # (Judges' e-stop also forces A0 at the actuator, so this is belt-and-braces.)
        try:
            self._set_autonomous(False)
        except Exception:
            pass
        super().destroy_node()

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_proximity_tick(self) -> None:
        if self._latest_robot_xy is None:
            return
        if self._cursor >= len(self._map_waypoints):
            return
        if self._goal_handle is None:
            return

        target = self._map_waypoints[self._cursor]
        rx, ry = self._latest_robot_xy
        dist = math.hypot(target.x - rx, target.y - ry)
        if dist >= self._radius:
            return

        self.get_logger().info(
            f'wp{self._cursor} reached (dist={dist:.2f} m < radius={self._radius:.2f} m), advancing'
        )
        # Cancel the in-flight goal; its eventual result lands in _on_result
        # with index < self._cursor and is filtered out by the cursor check.
        self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._advance_cursor(reason=f'wp{self._cursor} proximity')


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = MissionManager()
    except SystemExit as exc:
        rclpy.shutdown()
        sys.exit(exc.code)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
