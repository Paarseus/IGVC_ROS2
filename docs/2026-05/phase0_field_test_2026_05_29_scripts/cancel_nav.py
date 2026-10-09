#!/usr/bin/env python3
"""Cancel ALL active /navigate_to_pose goals (empty goal_info = cancel-all)."""
import rclpy
from action_msgs.srv import CancelGoal
rclpy.init()
n = rclpy.create_node('cancel_nav')
cli = n.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
if not cli.wait_for_service(timeout_sec=5):
    print('cancel service not available'); raise SystemExit(0)
req = CancelGoal.Request()  # empty goal id + zero stamp -> cancel all
fut = cli.call_async(req)
rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
r = fut.result()
print('cancel return_code:', getattr(r, 'return_code', '?'),
      'cancelling:', len(getattr(r, 'goals_canceling', [])))
n.destroy_node(); rclpy.shutdown()
