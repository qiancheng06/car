#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pure lidar corridor-following node for the Davinci-mini racecar.

Important:
  * This node does not use the yellow line or a hard-coded track path.
  * It estimates a drivable corridor from LaserScan point clusters.
  * IMU yaw rate and odometry speed are used for damping and safety only.
  * The default is dry-run (enable_motion=false); set that parameter true only
    after a supervised, wheels-off-ground test and a low-speed track test.

The existing racecar driver uses a legacy PWM convention on /teleop_cmd_vel:
  linear.x  = 1500 + throttle offset (1500 is neutral)
  angular.z = steering servo angle in degrees (90 is center)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan


@dataclass
class Cluster:
    x: float
    y: float
    radius: float
    points: int


@dataclass
class Corridor:
    valid: bool
    confidence: float
    left_x: Optional[float]
    right_x: Optional[float]
    near_center_x: float
    far_center_x: float
    heading_error: float
    width: Optional[float]
    cluster_count: int
    left_count: int
    right_count: int


class PureLidarNode(Node):
    """LaserScan-only track corridor estimator with IMU/odom assistance."""

    def __init__(self) -> None:
        super().__init__("pure_lidar_node")

        # Topics and runtime mode.
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/IMU_data")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("cmd_topic", "/teleop_cmd_vel")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("control_hz", 20.0)

        # Scan geometry. The installed code uses x = range*sin(theta),
        # y = range*cos(theta), i.e. y is forward and x is lateral.
        self.declare_parameter("min_range", 0.15)
        self.declare_parameter("max_range", 8.0)
        self.declare_parameter("max_lateral", 2.6)
        self.declare_parameter("max_forward", 5.0)
        self.declare_parameter("cluster_gap", 0.32)
        self.declare_parameter("min_cluster_points", 2)
        self.declare_parameter("max_cluster_points", 80)
        self.declare_parameter("max_cluster_radius", 0.75)
        self.declare_parameter("side_deadband", 0.12)
        self.declare_parameter("track_width", 1.50)
        self.declare_parameter("track_width_tolerance", 0.85)
        self.declare_parameter("near_lookahead", 0.90)
        self.declare_parameter("far_lookahead", 2.30)
        self.declare_parameter("side_min_y", 0.30)
        self.declare_parameter("side_max_y", 3.80)

        # Control and safety.
        self.declare_parameter("pwm_neutral", 1500.0)
        self.declare_parameter("pwm_fast", 1528.0)
        self.declare_parameter("pwm_slow", 1518.0)
        self.declare_parameter("steer_center", 90.0)
        self.declare_parameter("steer_min", 35.0)
        self.declare_parameter("steer_max", 145.0)
        self.declare_parameter("cte_gain", 34.0)
        self.declare_parameter("heading_gain", 42.0)
        self.declare_parameter("gyro_damping", 1.2)
        self.declare_parameter("steer_rate_limit", 90.0)
        self.declare_parameter("steer_smoothing", 0.65)
        self.declare_parameter("front_width", 0.38)
        self.declare_parameter("front_stop_distance", 0.42)
        self.declare_parameter("front_slow_distance", 0.85)
        self.declare_parameter("scan_timeout", 0.35)
        self.declare_parameter("imu_timeout", 0.60)
        self.declare_parameter("odom_timeout", 0.80)
        self.declare_parameter("allow_one_side", True)
        self.declare_parameter("one_side_speed_scale", 0.55)
        self.declare_parameter("no_boundary_stop", True)

        get = self.get_parameter
        self.scan_topic = str(get("scan_topic").value)
        self.imu_topic = str(get("imu_topic").value)
        self.odom_topic = str(get("odom_topic").value)
        self.cmd_topic = str(get("cmd_topic").value)
        self.enable_motion = bool(get("enable_motion").value)
        self.control_hz = max(1.0, float(get("control_hz").value))

        self.min_range = float(get("min_range").value)
        self.max_range = float(get("max_range").value)
        self.max_lateral = float(get("max_lateral").value)
        self.max_forward = float(get("max_forward").value)
        self.cluster_gap = float(get("cluster_gap").value)
        self.min_cluster_points = int(get("min_cluster_points").value)
        self.max_cluster_points = int(get("max_cluster_points").value)
        self.max_cluster_radius = float(get("max_cluster_radius").value)
        self.side_deadband = float(get("side_deadband").value)
        self.track_width = float(get("track_width").value)
        self.track_width_tolerance = float(get("track_width_tolerance").value)
        self.near_lookahead = float(get("near_lookahead").value)
        self.far_lookahead = float(get("far_lookahead").value)
        self.side_min_y = float(get("side_min_y").value)
        self.side_max_y = float(get("side_max_y").value)

        self.pwm_neutral = float(get("pwm_neutral").value)
        self.pwm_fast = float(get("pwm_fast").value)
        self.pwm_slow = float(get("pwm_slow").value)
        self.steer_center = float(get("steer_center").value)
        self.steer_min = float(get("steer_min").value)
        self.steer_max = float(get("steer_max").value)
        self.cte_gain = float(get("cte_gain").value)
        self.heading_gain = float(get("heading_gain").value)
        self.gyro_damping = float(get("gyro_damping").value)
        self.steer_rate_limit = float(get("steer_rate_limit").value)
        self.steer_smoothing = min(0.98, max(0.0, float(get("steer_smoothing").value)))
        self.front_width = float(get("front_width").value)
        self.front_stop_distance = float(get("front_stop_distance").value)
        self.front_slow_distance = float(get("front_slow_distance").value)
        self.scan_timeout = float(get("scan_timeout").value)
        self.imu_timeout = float(get("imu_timeout").value)
        self.odom_timeout = float(get("odom_timeout").value)
        self.allow_one_side = bool(get("allow_one_side").value)
        self.one_side_speed_scale = float(get("one_side_speed_scale").value)
        self.no_boundary_stop = bool(get("no_boundary_stop").value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, sensor_qos
        )
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._imu_callback, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, sensor_qos
        )
        self.timer = self.create_timer(1.0 / self.control_hz, self._control_callback)

        self.last_scan: Optional[LaserScan] = None
        self.last_scan_time = 0.0
        self.last_imu_time = 0.0
        self.last_odom_time = 0.0
        self.gyro_z = 0.0
        self.odom_speed = 0.0
        self.last_steer = self.steer_center
        self.last_log_time = 0.0
        self.last_reason = "startup"
        self.shutdown_sent = False

        mode = "MOTION ENABLED" if self.enable_motion else "DRY RUN / NEUTRAL ONLY"
        self.get_logger().warning(f"pure_lidar_node started: {mode}")
        self.get_logger().info(
            f"scan={self.scan_topic} imu={self.imu_topic} "
            f"odom={self.odom_topic} cmd={self.cmd_topic}"
        )
        self.get_logger().warning(
            "This is a geometry-following prototype: it cannot identify traffic-light "
            "colors, A/B signs, or yellow-line semantics."
        )

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.last_scan_time = self._now()

    def _imu_callback(self, msg: Imu) -> None:
        value = float(msg.angular_velocity.z)
        if math.isfinite(value):
            self.gyro_z = value
            self.last_imu_time = self._now()

    def _odom_callback(self, msg: Odometry) -> None:
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        speed = math.hypot(vx, vy)
        if math.isfinite(speed):
            self.odom_speed = speed
            self.last_odom_time = self._now()

    def _control_callback(self) -> None:
        now = self._now()
        if self.last_scan is None or now - self.last_scan_time > self.scan_timeout:
            self._safe_stop("scan timeout")
            return

        corridor, front_min = self._estimate_corridor(self.last_scan)
        imu_ok = now - self.last_imu_time <= self.imu_timeout
        odom_ok = now - self.last_odom_time <= self.odom_timeout

        if front_min <= self.front_stop_distance:
            self._safe_stop(f"front obstacle {front_min:.2f} m")
            return
        if not corridor.valid and self.no_boundary_stop:
            self._safe_stop("no reliable corridor")
            return

        # A one-sided estimate is deliberately limited to slow speed. It is useful
        # at the entrance/exit of a bend but should never be treated as full confidence.
        speed_scale = 1.0
        if corridor.left_count == 0 or corridor.right_count == 0:
            if not self.allow_one_side:
                self._safe_stop("only one boundary")
                return
            speed_scale *= self.one_side_speed_scale
            corridor.confidence = min(corridor.confidence, 0.55)

        steer_delta = self.cte_gain * corridor.near_center_x
        steer_delta += self.heading_gain * corridor.heading_error
        if imu_ok:
            steer_delta -= self.gyro_damping * self.gyro_z
        steer_delta = max(-50.0, min(50.0, steer_delta))

        desired_steer = self.steer_center + steer_delta
        # First-order smoothing plus a hard rate limit protects against a bad scan.
        alpha = 1.0 - self.steer_smoothing
        filtered = self.last_steer * self.steer_smoothing + desired_steer * alpha
        max_step = self.steer_rate_limit / self.control_hz
        filtered = max(self.last_steer - max_step, min(self.last_steer + max_step, filtered))
        steer = max(self.steer_min, min(self.steer_max, filtered))
        self.last_steer = steer

        speed = self.pwm_fast
        if abs(steer - self.steer_center) > 18.0 or abs(corridor.heading_error) > 0.22:
            speed = self.pwm_slow
        if front_min < self.front_slow_distance:
            speed = self.pwm_slow
        if corridor.confidence < 0.70:
            speed = self.pwm_slow
        speed = self.pwm_neutral + (speed - self.pwm_neutral) * speed_scale
        if odom_ok and self.odom_speed > 1.20:
            speed = min(speed, self.pwm_slow)

        self._publish_command(speed, steer, "corridor")
        self._periodic_log(
            f"corridor conf={corridor.confidence:.2f} "
            f"L/R={corridor.left_count}/{corridor.right_count} "
            f"center={corridor.near_center_x:.2f} heading={corridor.heading_error:.2f} "
            f"front={front_min:.2f} cmd=({speed:.1f},{steer:.1f})"
        )

    def _estimate_corridor(self, scan: LaserScan) -> Tuple[Corridor, float]:
        points: List[Tuple[float, float, float]] = []
        front_min = float("inf")
        angle = float(scan.angle_min)
        for raw_range in scan.ranges:
            r = float(raw_range)
            if not math.isfinite(r) or r < self.min_range or r > self.max_range:
                angle += scan.angle_increment
                continue
            x = r * math.sin(angle)
            y = r * math.cos(angle)
            angle += scan.angle_increment
            if y > 0.10 and abs(x) <= self.front_width:
                front_min = min(front_min, y)
            if -self.max_lateral <= x <= self.max_lateral and -0.1 <= y <= self.max_forward:
                points.append((x, y, r))

        clusters = self._cluster(points)
        left = [c for c in clusters if c.x < -self.side_deadband and self.side_min_y <= c.y <= self.side_max_y]
        right = [c for c in clusters if c.x > self.side_deadband and self.side_min_y <= c.y <= self.side_max_y]
        left_fit = self._fit_side(left)
        right_fit = self._fit_side(right)

        near_y = self.near_lookahead
        far_y = self.far_lookahead
        left_near = self._eval_fit(left_fit, near_y)
        left_far = self._eval_fit(left_fit, far_y)
        right_near = self._eval_fit(right_fit, near_y)
        right_far = self._eval_fit(right_fit, far_y)

        if left_near is not None and right_near is not None:
            near_center = 0.5 * (left_near + right_near)
            far_center = 0.5 * (left_far + right_far)
            width = right_near - left_near
        elif left_near is not None:
            near_center = left_near + 0.5 * self.track_width
            far_center = (left_far if left_far is not None else left_near) + 0.5 * self.track_width
            width = None
        elif right_near is not None:
            near_center = right_near - 0.5 * self.track_width
            far_center = (right_far if right_far is not None else right_near) - 0.5 * self.track_width
            width = None
        else:
            return Corridor(False, 0.0, None, None, 0.0, 0.0, 0.0, None, len(clusters), len(left), len(right)), front_min

        heading_error = math.atan2(far_center - near_center, max(0.1, far_y - near_y))
        confidence = self._confidence(left, right, width)
        valid = (left_near is not None or right_near is not None) and confidence >= 0.25
        return Corridor(
            valid, confidence, left_near, right_near, near_center, far_center,
            heading_error, width, len(clusters), len(left), len(right)
        ), front_min

    def _cluster(self, points: Sequence[Tuple[float, float, float]]) -> List[Cluster]:
        if not points:
            return []
        clusters: List[List[Tuple[float, float, float]]] = []
        current: List[Tuple[float, float, float]] = [points[0]]
        for p in points[1:]:
            prev = current[-1]
            gap = math.hypot(p[0] - prev[0], p[1] - prev[1])
            if gap <= self.cluster_gap:
                current.append(p)
            else:
                clusters.append(current)
                current = [p]
        clusters.append(current)

        result: List[Cluster] = []
        for group in clusters:
            if not self.min_cluster_points <= len(group) <= self.max_cluster_points:
                continue
            xs = [p[0] for p in group]
            ys = [p[1] for p in group]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            radius = max(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys))
            if radius <= self.max_cluster_radius:
                result.append(Cluster(cx, cy, radius, len(group)))
        return result

    @staticmethod
    def _fit_side(clusters: Sequence[Cluster]) -> Optional[Tuple[float, float]]:
        """Fit x = a*y+b. Returns None if there are no clusters."""
        if not clusters:
            return None
        if len(clusters) == 1:
            return 0.0, clusters[0].x
        # Give nearer observations more weight, but keep all points represented.
        weights = [1.0 / max(0.35, c.y) for c in clusters]
        sw = sum(weights)
        my = sum(w * c.y for w, c in zip(weights, clusters)) / sw
        mx = sum(w * c.x for w, c in zip(weights, clusters)) / sw
        denom = sum(w * (c.y - my) ** 2 for w, c in zip(weights, clusters))
        if denom < 1e-6:
            return 0.0, mx
        a = sum(w * (c.y - my) * (c.x - mx) for w, c in zip(weights, clusters)) / denom
        b = mx - a * my
        return a, b

    @staticmethod
    def _eval_fit(fit: Optional[Tuple[float, float]], y: float) -> Optional[float]:
        if fit is None:
            return None
        return fit[0] * y + fit[1]

    def _confidence(self, left: Sequence[Cluster], right: Sequence[Cluster], width: Optional[float]) -> float:
        if not left and not right:
            return 0.0
        count_score = min(1.0, (len(left) + len(right)) / 4.0)
        side_score = 1.0 if left and right else 0.45
        width_score = 1.0
        if width is not None:
            width_score = max(0.0, 1.0 - abs(width - self.track_width) / self.track_width_tolerance)
        return max(0.0, min(1.0, 0.45 * count_score + 0.35 * side_score + 0.20 * width_score))

    def _publish_command(self, throttle: float, steer: float, reason: str) -> None:
        msg = Twist()
        if self.enable_motion:
            msg.linear.x = float(throttle)
            msg.angular.z = float(steer)
        else:
            msg.linear.x = float(self.pwm_neutral)
            msg.angular.z = float(self.steer_center)
        self.cmd_pub.publish(msg)
        self.last_reason = reason

    def _safe_stop(self, reason: str) -> None:
        self.last_steer = self.steer_center
        self._publish_command(self.pwm_neutral, self.steer_center, reason)
        self._periodic_log(f"SAFE STOP: {reason}", force=True)

    def _periodic_log(self, text: str, force: bool = False) -> None:
        now = self._now()
        if force or now - self.last_log_time >= 1.0:
            self.get_logger().info(text)
            self.last_log_time = now

    def destroy_node(self) -> None:
        # Publish neutral several times so the driver receives a final stop even
        # if the process is interrupted between timer ticks.
        if not self.shutdown_sent:
            self.shutdown_sent = True
            for _ in range(3):
                self._publish_command(self.pwm_neutral, self.steer_center, "shutdown")
                time.sleep(0.02)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PureLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


