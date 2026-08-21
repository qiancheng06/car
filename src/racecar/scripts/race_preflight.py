#!/usr/bin/env python3
"""Preflight checker for the outdoor ROS racecar competition stack."""

from __future__ import annotations

import os
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class RacePreflight(Node):
    def __init__(self) -> None:
        super().__init__("race_preflight")
        self.declare_parameter("required_topics", [
            "/scan",
            "/odom_combined",
            "/image_raw",
            "/racecar/follow_cmd_vel",
            "/teleop_cmd_vel",
            "/racecar/race_state",
            "/racecar/red_light_detected",
            "/racecar/vision/status",
        ])
        self.declare_parameter("optional_topics", [
            "/racecar/parking_sign",
            "/map",
            "/tf",
            "/tf_static",
        ])
        self.timer = self.create_timer(2.0, self.check_once)
        self.done = False
        self.result_pub = self.create_publisher(Bool, "/racecar/preflight_ok", 1)

    def check_topics(self, topics: Iterable[str]) -> list[str]:
        active = {name for name, _types in self.get_topic_names_and_types()}
        return [topic for topic in topics if topic not in active]

    def check_once(self) -> None:
        if self.done:
            return
        required = list(self.get_parameter("required_topics").value)
        optional = list(self.get_parameter("optional_topics").value)
        missing_required = self.check_topics(required)
        missing_optional = self.check_topics(optional)

        self.get_logger().info(f"ROS_DISTRO={os.environ.get('ROS_DISTRO', 'unknown')}")
        if missing_required:
            self.get_logger().error("Missing required topics: " + ", ".join(missing_required))
            ok = False
        else:
            self.get_logger().info("All required competition topics are present.")
            ok = True

        if missing_optional:
            self.get_logger().warn("Missing optional topics: " + ", ".join(missing_optional))
        else:
            self.get_logger().info("All optional topics are present.")

        self.done = True
        result = Bool()
        result.data = ok
        self.result_pub.publish(result)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RacePreflight()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
