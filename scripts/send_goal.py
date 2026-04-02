#!/usr/bin/env python3
"""
scripts/send_goal.py
────────────────────
Send a single NavigateToPose goal to Nav2 and wait for the result.

Usage (inside ROS2 container, after Nav2 is active):
    python3 /workspace/scripts/send_goal.py [--x X] [--y Y] [--yaw YAW]

Defaults: x=3.0  y=2.0  yaw=0.0 (matches a clear area on simple_map)
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def quaternion_from_yaw(yaw: float):
    """Return (x, y, z, w) quaternion for a pure Z rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalSender(Node):
    def __init__(self, x: float, y: float, yaw: float):
        super().__init__("goal_sender")
        self._x   = x
        self._y   = y
        self._yaw = yaw
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def send(self):
        self.get_logger().info("Waiting for navigate_to_pose action server …")
        self._client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self._x
        pose.pose.position.y = self._y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(self._yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending goal: x={self._x:.2f}  y={self._y:.2f}  yaw={math.degrees(self._yaw):.1f}°"
        )
        future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb,
        )
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal REJECTED by Nav2")
            return False

        self.get_logger().info("Goal ACCEPTED — navigating …")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        status = result_future.result().status
        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("Navigation SUCCEEDED!")
            return True
        else:
            self.get_logger().warn(f"Navigation finished with status {status}")
            return False

    def _feedback_cb(self, msg):
        remaining = msg.feedback.distance_remaining
        self.get_logger().info(f"  distance remaining: {remaining:.2f} m", throttle_duration_sec=1.0)


def main():
    parser = argparse.ArgumentParser(description="Send a Nav2 goal pose")
    parser.add_argument("--x",   type=float, default=3.0)
    parser.add_argument("--y",   type=float, default=2.0)
    parser.add_argument("--yaw", type=float, default=0.0,
                        help="Goal yaw in radians")
    args = parser.parse_args()

    rclpy.init()
    node = GoalSender(args.x, args.y, args.yaw)
    try:
        success = node.send()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
