"""mission_manager — IGVC AutoNav waypoint orchestrator.

Drives the navigate_igvc_autonav_humble.xml BT by feeding it sequential
NavigateToPose goals from a YAML waypoint list. The BT is "dumb" — it
plans, follows, and recovers; this node owns the waypoint cursor and
advances it on proximity.

Phase M4 scope: also wire a NavigateToPose ActionClient and send the
first waypoint as a single-shot goal at startup. Cursor advance on
proximity lands in M5.
"""
import sys
from typing import List, Optional, Tuple

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_localization.srv import FromLL


FROMLL_WAIT_TIMEOUT_S = 30.0
FROMLL_CALL_TIMEOUT_S = 5.0
NAV2_WAIT_TIMEOUT_S = 30.0
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

        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._goal_handle = None
        if not self._nav_client.wait_for_server(timeout_sec=NAV2_WAIT_TIMEOUT_S):
            self.get_logger().error(
                f'/navigate_to_pose action server not available after '
                f'{NAV2_WAIT_TIMEOUT_S}s — is bt_navigator running?'
            )
            raise SystemExit(1)

        self._send_goal(0)

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
        self._goal_handle = None


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
