#!/usr/bin/env python3
"""Competition task manager for the outdoor ROS racecar event.

This node is intentionally conservative: it does not replace the existing
tracking, mapping, navigation, or driver nodes.  It arbitrates their velocity
commands and adds the race-specific state machine:

1. Lap 1: follow the track while gmapping builds the map.
2. Lap transition: optionally save the map and raise the cruise speed.
3. Lap 2: keep following/navigating, stop for red lights, detect A/B sign,
   execute a tunable parking maneuver, and hold the vehicle stopped.

Default command mode is "pwm" because the existing lidar_tracking node publishes
PWM-style Twist values to /teleop_cmd_vel.
"""

from __future__ import annotations

import math
import os
import subprocess
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class RaceState(str, Enum):
    WAITING_FOR_ODOM = "WAITING_FOR_ODOM"
    LAP1_MAPPING = "LAP1_MAPPING"
    LAP2_FAST = "LAP2_FAST"
    TRAFFIC_STOP = "TRAFFIC_STOP"
    PARKING_APPROACH = "PARKING_APPROACH"
    PARKING_TURN = "PARKING_TURN"
    FINISHED = "FINISHED"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass
class Pose2D:
    x: float
    y: float

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


class RaceManager(Node):
    def __init__(self) -> None:
        super().__init__("race_manager")

        # Topics and command interpretation.
        self.declare_parameter("input_cmd_topic", "/racecar/follow_cmd_vel")
        self.declare_parameter("output_cmd_topic", "/teleop_cmd_vel")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("command_mode", "pwm")  # pwm or normalized

        # Lap detection.
        self.declare_parameter("lap_radius_m", 0.75)
        self.declare_parameter("lap_min_leave_distance_m", 4.0)
        self.declare_parameter("lap1_min_time_s", 20.0)
        self.declare_parameter("lap2_min_time_s", 20.0)

        # PWM defaults; used by existing racecar_driver teleop path.
        self.declare_parameter("pwm_stop_speed", 1500.0)
        self.declare_parameter("pwm_neutral_steer", 90.0)
        self.declare_parameter("pwm_lap1_speed", 1538.0)
        self.declare_parameter("pwm_lap2_speed", 1548.0)
        self.declare_parameter("pwm_max_speed", 1565.0)
        self.declare_parameter("pwm_min_steer", 35.0)
        self.declare_parameter("pwm_max_steer", 145.0)

        # Normalized mode defaults; useful if the upstream command is /cmd_vel.
        self.declare_parameter("norm_stop_speed", 0.0)
        self.declare_parameter("norm_lap1_speed", 0.25)
        self.declare_parameter("norm_lap2_speed", 0.35)
        self.declare_parameter("norm_max_speed", 0.45)
        self.declare_parameter("norm_max_steer", 1.0)

        # Red light detection / response.
        self.declare_parameter("enable_vision", True)
        self.declare_parameter("use_external_vision", True)
        self.declare_parameter("red_light_topic", "/racecar/red_light_detected")
        self.declare_parameter("red_light_stop_s", 3.2)
        self.declare_parameter("red_light_cooldown_s", 8.0)
        self.declare_parameter("red_light_min_area_ratio", 0.003)
        self.declare_parameter("red_light_min_frames", 3)
        self.declare_parameter("red_light_roi_top_ratio", 0.0)
        self.declare_parameter("red_light_roi_bottom_ratio", 0.70)

        # Parking sign and timed final parking maneuver.
        self.declare_parameter("parking_sign_topic", "/racecar/parking_sign")
        self.declare_parameter("parking_sign_min_frames", 3)
        self.declare_parameter("parking_approach_s", 1.0)
        self.declare_parameter("parking_turn_s", 1.5)
        self.declare_parameter("parking_speed_pwm", 1530.0)
        self.declare_parameter("parking_left_steer_pwm", 62.0)
        self.declare_parameter("parking_right_steer_pwm", 118.0)
        self.declare_parameter("parking_speed_norm", 0.16)
        self.declare_parameter("parking_left_steer_norm", 0.65)
        self.declare_parameter("parking_right_steer_norm", -0.65)
        self.declare_parameter("template_a_path", "")
        self.declare_parameter("template_b_path", "")

        # Map saving at the lap transition.
        self.declare_parameter("save_map_on_lap1_complete", True)
        self.declare_parameter("map_save_path", "/home/davinci-mini/racecar/src/racecar/map/competition_map")

        self.command_mode = str(self.get_parameter("command_mode").value).lower()
        self.state = RaceState.WAITING_FOR_ODOM
        self.previous_drive_state = RaceState.LAP1_MAPPING
        self.state_started = self.get_clock().now()
        self.race_started = self.get_clock().now()
        self.lap_started = self.get_clock().now()
        self.start_pose: Optional[Pose2D] = None
        self.current_pose: Optional[Pose2D] = None
        self.left_start_area = False
        self.lap_number = 1
        self.latest_cmd = self.stop_cmd()
        self.last_red_stop_time: Optional[float] = None
        self.red_frame_count = 0
        self.parking_vote: Optional[str] = None
        self.parking_vote_count = 0
        self.template_a = self.load_template("template_a_path")
        self.template_b = self.load_template("template_b_path")

        self.bridge = CvBridge()

        self.cmd_sub = self.create_subscription(
            Twist, str(self.get_parameter("input_cmd_topic").value), self.cmd_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 20
        )
        self.force_stop_sub = self.create_subscription(
            Bool, "/racecar/force_stop", self.force_stop_callback, 5
        )
        self.sign_sub = self.create_subscription(
            String, str(self.get_parameter("parking_sign_topic").value), self.parking_sign_callback, 5
        )
        self.image_sub = self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self.image_callback, 5
        )
        self.red_light_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("red_light_topic").value),
            self.red_light_callback,
            5,
        )

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("output_cmd_topic").value), 10
        )
        self.state_pub = self.create_publisher(String, "/racecar/race_state", 5)
        self.light_pub = self.create_publisher(String, "/light", 5)

        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f"Race manager started: input={self.get_parameter('input_cmd_topic').value}, "
            f"output={self.get_parameter('output_cmd_topic').value}, mode={self.command_mode}"
        )

    def load_template(self, parameter_name: str) -> Optional[np.ndarray]:
        path = str(self.get_parameter(parameter_name).value)
        if not path:
            return None
        expanded = os.path.expanduser(path)
        if not os.path.exists(expanded):
            self.get_logger().warn(f"Template path does not exist: {expanded}")
            return None
        img = cv2.imread(expanded, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().warn(f"Failed to load template: {expanded}")
        return img

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed_in_state(self) -> float:
        return (self.get_clock().now() - self.state_started).nanoseconds / 1e9

    def elapsed_in_lap(self) -> float:
        return (self.get_clock().now() - self.lap_started).nanoseconds / 1e9

    def transition(self, state: RaceState) -> None:
        if self.state == state:
            return
        self.get_logger().info(f"Race state: {self.state.value} -> {state.value}")
        self.state = state
        self.state_started = self.get_clock().now()
        msg = String()
        msg.data = state.value
        self.state_pub.publish(msg)

    def stop_cmd(self) -> Twist:
        msg = Twist()
        if self.command_mode == "pwm":
            msg.linear.x = float(self.get_parameter("pwm_stop_speed").value)
            msg.angular.z = float(self.get_parameter("pwm_neutral_steer").value)
        else:
            msg.linear.x = float(self.get_parameter("norm_stop_speed").value)
            msg.angular.z = 0.0
        return msg

    def parking_cmd(self, sign: str) -> Twist:
        msg = Twist()
        if self.command_mode == "pwm":
            msg.linear.x = float(self.get_parameter("parking_speed_pwm").value)
            if sign == "A":
                msg.angular.z = float(self.get_parameter("parking_left_steer_pwm").value)
            else:
                msg.angular.z = float(self.get_parameter("parking_right_steer_pwm").value)
        else:
            msg.linear.x = float(self.get_parameter("parking_speed_norm").value)
            if sign == "A":
                msg.angular.z = float(self.get_parameter("parking_left_steer_norm").value)
            else:
                msg.angular.z = float(self.get_parameter("parking_right_steer_norm").value)
        return msg

    def speed_limited_cmd(self, source: Twist) -> Twist:
        msg = Twist()
        msg.linear.x = source.linear.x
        msg.linear.y = source.linear.y
        msg.linear.z = source.linear.z
        msg.angular.x = source.angular.x
        msg.angular.y = source.angular.y
        msg.angular.z = source.angular.z

        if self.command_mode == "pwm":
            desired = float(
                self.get_parameter("pwm_lap2_speed" if self.lap_number >= 2 else "pwm_lap1_speed").value
            )
            # Preserve the upstream steering but standardize speed per lap.
            if msg.linear.x > 1490.0:
                msg.linear.x = desired
            msg.linear.x = min(float(self.get_parameter("pwm_max_speed").value), max(1500.0, msg.linear.x))
            msg.angular.z = min(
                float(self.get_parameter("pwm_max_steer").value),
                max(float(self.get_parameter("pwm_min_steer").value), msg.angular.z),
            )
        else:
            desired = float(
                self.get_parameter("norm_lap2_speed" if self.lap_number >= 2 else "norm_lap1_speed").value
            )
            if abs(msg.linear.x) > 0.001:
                msg.linear.x = math.copysign(desired, msg.linear.x)
            msg.linear.x = min(float(self.get_parameter("norm_max_speed").value), max(-0.1, msg.linear.x))
            steer_max = float(self.get_parameter("norm_max_steer").value)
            msg.angular.z = min(steer_max, max(-steer_max, msg.angular.z))
        return msg

    def cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.current_pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.start_pose is None:
            self.start_pose = self.current_pose
            self.race_started = self.get_clock().now()
            self.lap_started = self.get_clock().now()
            self.transition(RaceState.LAP1_MAPPING)
            self.get_logger().info(
                f"Start pose locked at x={self.start_pose.x:.2f}, y={self.start_pose.y:.2f}"
            )
            return

        distance = self.current_pose.distance_to(self.start_pose)
        if distance > float(self.get_parameter("lap_min_leave_distance_m").value):
            self.left_start_area = True

        lap_min_time = float(
            self.get_parameter("lap1_min_time_s" if self.lap_number == 1 else "lap2_min_time_s").value
        )
        if (
            self.left_start_area
            and distance < float(self.get_parameter("lap_radius_m").value)
            and self.elapsed_in_lap() > lap_min_time
        ):
            self.handle_lap_complete()

    def handle_lap_complete(self) -> None:
        self.left_start_area = False
        self.lap_started = self.get_clock().now()

        if self.lap_number == 1:
            self.lap_number = 2
            self.save_map_if_requested()
            self.transition(RaceState.LAP2_FAST)
        elif self.state not in (RaceState.PARKING_TURN, RaceState.FINISHED):
            self.get_logger().info("Second lap return detected; waiting for parking sign or manual stop.")

    def save_map_if_requested(self) -> None:
        if not bool(self.get_parameter("save_map_on_lap1_complete").value):
            return
        map_path = str(self.get_parameter("map_save_path").value)
        Path(map_path).parent.mkdir(parents=True, exist_ok=True)
        cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path]
        try:
            self.get_logger().info(f"Saving map with: {' '.join(cmd)}")
            subprocess.Popen(cmd)
        except Exception as exc:  # noqa: BLE001 - ROS node should stay alive.
            self.get_logger().warn(f"Could not start map_saver_cli: {exc}")

    def force_stop_callback(self, msg: Bool) -> None:
        if msg.data:
            self.transition(RaceState.EMERGENCY_STOP)
        elif self.state == RaceState.EMERGENCY_STOP:
            self.transition(RaceState.LAP2_FAST if self.lap_number >= 2 else RaceState.LAP1_MAPPING)

    def parking_sign_callback(self, msg: String) -> None:
        value = msg.data.strip().upper()
        if value not in ("A", "B"):
            return
        self.accept_parking_vote(value)

    def red_light_callback(self, msg: Bool) -> None:
        if not bool(self.get_parameter("use_external_vision").value):
            return
        if msg.data:
            self.red_frame_count = int(self.get_parameter("red_light_min_frames").value)
        else:
            self.red_frame_count = 0

    def image_callback(self, msg: Image) -> None:
        if not bool(self.get_parameter("enable_vision").value):
            return
        # The YOLO node already performs class-specific confidence and temporal
        # filtering. Do not run the legacy HSV/template detectors a second time.
        if bool(self.get_parameter("use_external_vision").value):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Image conversion failed: {exc}")
            return

        if self.detect_red_light(frame):
            self.red_frame_count += 1
        else:
            self.red_frame_count = 0

        if self.lap_number >= 2:
            sign = self.detect_parking_sign(frame)
            if sign:
                self.accept_parking_vote(sign)

    def detect_red_light(self, frame: np.ndarray) -> bool:
        h, w = frame.shape[:2]
        top = int(h * float(self.get_parameter("red_light_roi_top_ratio").value))
        bottom = int(h * float(self.get_parameter("red_light_roi_bottom_ratio").value))
        roi = frame[max(0, top) : min(h, bottom), :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 80, 80]), np.array([180, 255, 255]))
        mask = cv2.morphologyEx(mask1 | mask2, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        red_area = float(cv2.countNonZero(mask))
        return red_area / max(1.0, float(roi.shape[0] * w)) >= float(
            self.get_parameter("red_light_min_area_ratio").value
        )

    def detect_parking_sign(self, frame: np.ndarray) -> Optional[str]:
        if self.template_a is None or self.template_b is None:
            return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        score_a = self.match_template(gray, self.template_a)
        score_b = self.match_template(gray, self.template_b)
        if max(score_a, score_b) < 0.45:
            return None
        return "A" if score_a > score_b else "B"

    @staticmethod
    def match_template(gray: np.ndarray, template: np.ndarray) -> float:
        if template.shape[0] >= gray.shape[0] or template.shape[1] >= gray.shape[1]:
            return 0.0
        result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
        return float(result.max())

    def accept_parking_vote(self, sign: str) -> None:
        if self.parking_vote == sign:
            self.parking_vote_count += 1
        else:
            self.parking_vote = sign
            self.parking_vote_count = 1
        if (
            self.lap_number >= 2
            and self.state == RaceState.LAP2_FAST
            and self.parking_vote_count >= int(self.get_parameter("parking_sign_min_frames").value)
        ):
            self.transition(RaceState.PARKING_APPROACH)

    def red_light_should_stop(self) -> bool:
        if self.red_frame_count < int(self.get_parameter("red_light_min_frames").value):
            return False
        now = self.now_s()
        if self.last_red_stop_time is None:
            return True
        return now - self.last_red_stop_time > float(self.get_parameter("red_light_cooldown_s").value)

    def tick(self) -> None:
        if self.state == RaceState.WAITING_FOR_ODOM:
            self.cmd_pub.publish(self.stop_cmd())
            return

        if self.state in (RaceState.LAP1_MAPPING, RaceState.LAP2_FAST) and self.red_light_should_stop():
            self.previous_drive_state = self.state
            self.last_red_stop_time = self.now_s()
            light = String()
            light.data = "red"
            self.light_pub.publish(light)
            self.transition(RaceState.TRAFFIC_STOP)

        if self.state == RaceState.TRAFFIC_STOP:
            self.cmd_pub.publish(self.stop_cmd())
            if self.elapsed_in_state() >= float(self.get_parameter("red_light_stop_s").value):
                light = String()
                light.data = "green"
                self.light_pub.publish(light)
                self.red_frame_count = 0
                self.transition(self.previous_drive_state)
            return

        if self.state == RaceState.PARKING_APPROACH:
            self.cmd_pub.publish(self.speed_limited_cmd(self.latest_cmd))
            if self.elapsed_in_state() >= float(self.get_parameter("parking_approach_s").value):
                self.transition(RaceState.PARKING_TURN)
            return

        if self.state == RaceState.PARKING_TURN:
            self.cmd_pub.publish(self.parking_cmd(self.parking_vote or "A"))
            if self.elapsed_in_state() >= float(self.get_parameter("parking_turn_s").value):
                self.transition(RaceState.FINISHED)
            return

        if self.state in (RaceState.FINISHED, RaceState.EMERGENCY_STOP):
            self.cmd_pub.publish(self.stop_cmd())
            return

        self.cmd_pub.publish(self.speed_limited_cmd(self.latest_cmd))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaceManager()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(node.stop_cmd())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
