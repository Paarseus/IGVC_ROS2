"""mission_manager — IGVC AutoNav waypoint orchestrator.

Drives the navigate_igvc_autonav_humble.xml BT by feeding it sequential
NavigateToPose goals from a YAML waypoint list. The BT is "dumb" — it
plans, follows, and recovers; this node owns the waypoint cursor and
advances it on proximity.

Phase M2 scope: skeleton only. Declares parameters and logs startup.
Loading, conversion, action client, and advance logic land in M3-M6.
"""
import rclpy
from rclpy.node import Node


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('acceptance_radius_m', 2.0)
        self.declare_parameter('odom_topic', '/odometry/global')

        waypoints_file = self.get_parameter('waypoints_file').value
        radius = self.get_parameter('acceptance_radius_m').value
        odom_topic = self.get_parameter('odom_topic').value

        self.get_logger().info(
            f'mission_manager started '
            f'(waypoints_file={waypoints_file or "<unset>"}, '
            f'acceptance_radius_m={radius}, odom_topic={odom_topic})'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
