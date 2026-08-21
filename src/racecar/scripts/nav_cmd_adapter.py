#!/usr/bin/env python3
"""Safely adapt Nav2 velocity commands to the racecar PWM interface.

Nav2/TEB publishes SI-unit ``geometry_msgs/Twist`` commands.  The protected
driver input ``/racecar_driver/cmd_pwm`` uses a ``Twist`` envelope whose fields mean:

* ``linear.x``: motor PWM (1500 is neutral)
* ``angular.z``: steering angle in degrees (90 is nominally centered)

This node is the only bridge between those two conventions.  It also adds an
arm switch, a software stop, finite-value checks, command clamping and a
command timeout.  The driver has a second watchdog as the final safety layer.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class NavCmdAdapter(Node):
    """Convert Nav2 commands to the PWM-style command used by this car."""

    def __init__(self) -> None:
        super().__init__("nav_cmd_adapter")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/racecar_driver/cmd_pwm")
        self.declare_parameter("arm_topic", "/nav/arm")
        self.declare_parameter("estop_topic", "/nav/estop")
        self.declare_parameter("state_topic", "/nav/adapter_state")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("cmd_timeout_s", 0.40)
        self.declare_parameter("angular_input_mode", "yaw_rate")

        # Geometry.  Wheelbase is taken from the existing vehicle controller;
        # minimum turning radius is the conservative value already used by Nav2.
        self.declare_parameter("wheelbase_m", 0.305)
        self.declare_parameter("min_turning_radius_m", 0.60)
        self.declare_parameter("max_steering_angle_rad", 0.50)

        # Safe commissioning limits.  The theoretical limits in the vehicle
        # document are not suitable as first-run controller limits.
        self.declare_parameter("max_linear_speed_mps", 0.45)
        self.declare_parameter("max_reverse_speed_mps", 0.0)
        self.declare_parameter("min_motion_speed_mps", 0.02)
        self.declare_parameter("allow_reverse", False)

        # Existing firmware/PWM convention.
        self.declare_parameter("neutral_throttle_pwm", 1500.0)
        self.declare_parameter("min_throttle_pwm", 1500.0)
        self.declare_parameter("max_throttle_pwm", 1550.0)
        self.declare_parameter("pwm_per_mps", 100.0)
        self.declare_parameter("steering_center_deg", 90.0)
        self.declare_parameter("steering_min_deg", 55.0)
        self.declare_parameter("steering_max_deg", 125.0)
        self.declare_parameter("steering_direction", 1.0)
        self.declare_parameter("steering_gain", 1.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.angular_input_mode = str(
            self.get_parameter("angular_input_mode").value
        ).strip().lower()
        if self.angular_input_mode not in ("yaw_rate", "steering_angle"):
            raise ValueError(
                "angular_input_mode must be 'yaw_rate' or 'steering_angle'"
            )

        # Deliberately no auto-arm parameter: every process start requires a
        # new, explicit /nav/arm message from the operator.
        self.armed = False
        self.estop = False
        self.latest_cmd: Optional[Twist] = None
        self.latest_cmd_time: Optional[float] = None
        self.last_reported_state = ""
        self.last_state_publish_time = 0.0

        self.output_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), 10
        )
        self.create_subscription(Twist, self.input_topic, self._cmd_callback, 10)
        self.create_subscription(
            Bool, str(self.get_parameter("arm_topic").value), self._arm_callback, 10
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("estop_topic").value),
            self._estop_callback,
            10,
        )

        publish_rate = max(5.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / publish_rate, self._timer_callback)

        self._publish_stop()
        self._publish_state(force=True)
        self.get_logger().info(
            "Nav command adapter ready: %s -> %s; explicit arm required; mode=%s"
            % (self.input_topic, self.output_topic, self.angular_input_mode)
        )

    @staticmethod
    def _finite_twist(msg: Twist) -> bool:
        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        return all(math.isfinite(value) for value in values)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(upper, max(lower, value))

    def _cmd_callback(self, msg: Twist) -> None:
        if not self._finite_twist(msg):
            self.get_logger().error("Rejected non-finite Nav2 velocity command")
            self.latest_cmd = None
            self.latest_cmd_time = None
            return
        self.latest_cmd = msg
        self.latest_cmd_time = time.monotonic()

    def _arm_callback(self, msg: Bool) -> None:
        if msg.data and self.estop:
            self.get_logger().warning("Arm request ignored while software stop is active")
            return
        requested = bool(msg.data)
        if requested and not self.armed:
            # A command received before the arm edge must never move the car.
            self.latest_cmd = None
            self.latest_cmd_time = None
        if not requested:
            self.latest_cmd = None
            self.latest_cmd_time = None
        self.armed = requested
        self.get_logger().info("Navigation output %s" % ("ARMED" if self.armed else "DISARMED"))
        if not self.armed:
            self._publish_stop()
        self._publish_state(force=True)

    def _estop_callback(self, msg: Bool) -> None:
        if msg.data:
            self.estop = True
            self.armed = False
            self.latest_cmd = None
            self.latest_cmd_time = None
            self.get_logger().error("SOFTWARE STOP asserted; re-arm is required after release")
        else:
            self.estop = False
            self.get_logger().warning("Software stop released; output remains disarmed")
        self._publish_stop()
        self._publish_state(force=True)

    def _command_is_fresh(self) -> bool:
        if self.latest_cmd is None or self.latest_cmd_time is None:
            return False
        age = time.monotonic() - self.latest_cmd_time
        return age <= float(self.get_parameter("cmd_timeout_s").value)

    def _stop_command(self) -> Twist:
        msg = Twist()
        msg.linear.x = float(self.get_parameter("neutral_throttle_pwm").value)
        msg.angular.z = float(self.get_parameter("steering_center_deg").value)
        return msg

    def _publish_stop(self) -> None:
        self.output_pub.publish(self._stop_command())

    def _convert(self, source: Twist) -> Twist:
        max_forward = max(0.0, float(self.get_parameter("max_linear_speed_mps").value))
        max_reverse = max(0.0, float(self.get_parameter("max_reverse_speed_mps").value))
        allow_reverse = bool(self.get_parameter("allow_reverse").value)

        min_speed = -max_reverse if allow_reverse else 0.0
        velocity = self._clamp(float(source.linear.x), min_speed, max_forward)
        motion_deadband = max(0.0, float(self.get_parameter("min_motion_speed_mps").value))

        # Ackermann steering cannot rotate the vehicle in place.  Stop instead
        # of translating a Nav2 recovery-spin command into a hard steering lock.
        if abs(velocity) < motion_deadband:
            return self._stop_command()

        wheelbase = max(0.001, float(self.get_parameter("wheelbase_m").value))
        turning_radius = max(
            0.001, float(self.get_parameter("min_turning_radius_m").value)
        )
        geometry_limit = math.atan(wheelbase / turning_radius)
        configured_limit = max(
            0.01, float(self.get_parameter("max_steering_angle_rad").value)
        )
        steering_limit = min(geometry_limit, configured_limit)

        if self.angular_input_mode == "steering_angle":
            steering = float(source.angular.z)
        else:
            steering = math.atan(wheelbase * float(source.angular.z) / velocity)
        steering = self._clamp(steering, -steering_limit, steering_limit)

        neutral_pwm = float(self.get_parameter("neutral_throttle_pwm").value)
        throttle = neutral_pwm + velocity * float(self.get_parameter("pwm_per_mps").value)
        throttle = self._clamp(
            throttle,
            float(self.get_parameter("min_throttle_pwm").value),
            float(self.get_parameter("max_throttle_pwm").value),
        )

        center = float(self.get_parameter("steering_center_deg").value)
        direction = float(self.get_parameter("steering_direction").value)
        gain = float(self.get_parameter("steering_gain").value)
        steering_deg = center + direction * gain * math.degrees(steering)
        steering_deg = self._clamp(
            steering_deg,
            float(self.get_parameter("steering_min_deg").value),
            float(self.get_parameter("steering_max_deg").value),
        )

        output = Twist()
        output.linear.x = throttle
        output.angular.z = steering_deg
        return output

    def _current_state(self) -> str:
        if self.estop:
            return "ESTOP"
        if not self.armed:
            return "DISARMED"
        if not self._command_is_fresh():
            return "COMMAND_TIMEOUT"
        return "ARMED"

    def _publish_state(self, force: bool = False) -> None:
        state = self._current_state()
        now = time.monotonic()
        if (
            not force
            and state == self.last_reported_state
            and now - self.last_state_publish_time < 1.0
        ):
            return
        self.last_reported_state = state
        self.last_state_publish_time = now
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _timer_callback(self) -> None:
        if self._current_state() != "ARMED":
            self._publish_stop()
        else:
            self.output_pub.publish(self._convert(self.latest_cmd))
        self._publish_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavCmdAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
