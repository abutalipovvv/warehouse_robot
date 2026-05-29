#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("goal_pose_bridge")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._subscription = self.create_subscription(
            PoseStamped,
            "goal_pose",
            self._handle_goal_pose,
            10,
        )

    def _handle_goal_pose(self, msg: PoseStamped) -> None:
        if not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("navigate_to_pose action server is not available")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = msg.header.frame_id or "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = msg.pose

        self.get_logger().info(
            f"Forwarding /goal_pose to navigate_to_pose at sim time for "
            f"({goal.pose.pose.position.x:.2f}, {goal.pose.pose.position.y:.2f})"
        )
        self._client.send_goal_async(goal)


def main() -> None:
    rclpy.init()
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
