#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Track-B lidar corridor-following node for the Davinci-mini racecar.

Important:
  * This node does not use the yellow line or a hard-coded track path.
  * It estimates a drivable corridor from LaserScan point clusters.
  * IMU yaw and odometry distance bridge Track B cone-free yellow section.
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
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


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


class PureLidarBNode(Node):
    """Track-B corridor estimator with an IMU/odometry bridge across the yellow gap."""

    def __init__(self) -> None:
        super().__init__("pure_lidar_b_node")

        # Topics and runtime mode.
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/IMU_data")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("cmd_topic", "/teleop_cmd_vel")
        self.declare_parameter("filtered_scan_topic", "/pure_lidar/scan_filtered")
        self.declare_parameter("publish_filtered_scan", True)
        self.declare_parameter("min_valid_scan_points", 30)
        self.declare_parameter("min_valid_scan_ratio", 0.03)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("control_hz", 20.0)

        # Scan geometry. The installed code uses x = range*sin(theta),
        # y = range*cos(theta), i.e. y is forward and x is lateral.
        self.declare_parameter("min_range", 0.15)
        self.declare_parameter("max_range", 8.0)
        self.declare_parameter("max_lateral", 2.6)
        self.declare_parameter("boundary_max_lateral", 1.65)
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
        # node1-inspired cone-edge extraction plus scan-index-aware clustering.
        self.declare_parameter("max_index_gap", 2)
        self.declare_parameter("wall_segment_points", 12)
        self.declare_parameter("use_bucket_edges", True)
        self.declare_parameter("bucket_window", 10)
        self.declare_parameter("bucket_min_support", 6)
        self.declare_parameter("bucket_range_tolerance", 0.20)
        self.declare_parameter("bucket_edge_jump", 0.55)
        self.declare_parameter("candidate_merge_distance", 0.24)
        self.declare_parameter("max_boundary_points_per_side", 5)
        self.declare_parameter("boundary_fit_residual", 0.36)
        self.declare_parameter("pair_y_tolerance", 0.85)
        self.declare_parameter("min_corridor_width", 0.65)
        self.declare_parameter("max_corridor_width", 2.35)
        self.declare_parameter("perception_smoothing", 0.42)

        # Control and safety.
        self.declare_parameter("pwm_neutral", 1500.0)
        self.declare_parameter("pwm_fast", 1534.0)
        self.declare_parameter("pwm_slow", 1524.0)
        self.declare_parameter("steer_center", 90.0)
        self.declare_parameter("steer_min", 35.0)
        self.declare_parameter("steer_max", 145.0)
        self.declare_parameter("cte_gain", 34.0)
        self.declare_parameter("heading_gain", 42.0)
        self.declare_parameter("cte_d_gain", 4.5)
        self.declare_parameter("cte_d_limit", 1.2)
        self.declare_parameter("gyro_damping", 1.2)
        self.declare_parameter("steer_rate_limit", 105.0)
        self.declare_parameter("steer_smoothing", 0.58)
        self.declare_parameter("pwm_smoothing", 0.55)
        self.declare_parameter("pwm_accel_rate", 28.0)
        self.declare_parameter("pwm_decel_rate", 45.0)
        self.declare_parameter("front_width", 0.38)
        self.declare_parameter("front_stop_distance", 0.38)
        self.declare_parameter("front_slow_distance", 0.75)
        self.declare_parameter("front_obstacle_min_points", 5)
        self.declare_parameter("scan_timeout", 0.80)
        self.declare_parameter("imu_timeout", 0.60)
        self.declare_parameter("odom_timeout", 0.80)
        self.declare_parameter("allow_one_side", True)
        self.declare_parameter("one_side_speed_scale", 0.62)
        self.declare_parameter("no_boundary_stop", True)
        self.declare_parameter("boundary_loss_grace_scans", 3)
        self.declare_parameter("boundary_hold_time", 0.48)
        self.declare_parameter("boundary_loss_speed_scale", 0.62)
        self.declare_parameter("odom_speed_limit", 1.80)

        # Track B yellow-line bridge. Track B has a 2 x 3.06 m cone-free V section.
        # The assumed direction is: start in the lower-left box, drive right, round
        # the right U-turn, then cross the yellow V from right to left.
        self.declare_parameter("track_mode", "B")
        self.declare_parameter("b_gap_enabled", True)
        self.declare_parameter("b_gap_min_total_distance", 12.0)
        self.declare_parameter("b_gap_max_total_distance", 28.0)
        self.declare_parameter("b_gap_min_heading_change_deg", 120.0)
        self.declare_parameter("b_gap_loss_scans", 3)
        self.declare_parameter("b_gap_first_length", 3.06)
        self.declare_parameter("b_gap_second_length", 3.06)
        self.declare_parameter("b_gap_first_yaw_deg", 22.5)
        self.declare_parameter("b_gap_direction", 1.0)
        self.declare_parameter("b_gap_reacquire_distance", 5.25)
        self.declare_parameter("b_gap_max_distance", 7.20)
        self.declare_parameter("b_gap_pwm", 1516.0)
        self.declare_parameter("b_gap_yaw_gain", 58.0)
        self.declare_parameter("b_gap_gyro_gain", 4.0)
        self.declare_parameter("b_gap_steer_limit", 34.0)

        get = self.get_parameter
        self.scan_topic = str(get("scan_topic").value)
        self.imu_topic = str(get("imu_topic").value)
        self.odom_topic = str(get("odom_topic").value)
        self.cmd_topic = str(get("cmd_topic").value)
        self.filtered_scan_topic = str(get("filtered_scan_topic").value)
        self.publish_filtered_scan = bool(get("publish_filtered_scan").value)
        self.min_valid_scan_points = max(1, int(get("min_valid_scan_points").value))
        self.min_valid_scan_ratio = max(0.0, min(1.0, float(get("min_valid_scan_ratio").value)))
        self.enable_motion = bool(get("enable_motion").value)
        self.control_hz = max(1.0, float(get("control_hz").value))

        self.min_range = float(get("min_range").value)
        self.max_range = float(get("max_range").value)
        self.max_lateral = float(get("max_lateral").value)
        self.boundary_max_lateral = min(
            self.max_lateral, max(0.30, float(get("boundary_max_lateral").value))
        )
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
        self.max_index_gap = max(1, int(get("max_index_gap").value))
        self.wall_segment_points = max(3, int(get("wall_segment_points").value))
        self.use_bucket_edges = bool(get("use_bucket_edges").value)
        self.bucket_window = max(3, int(get("bucket_window").value))
        self.bucket_min_support = max(2, int(get("bucket_min_support").value))
        self.bucket_range_tolerance = max(0.02, float(get("bucket_range_tolerance").value))
        self.bucket_edge_jump = max(0.05, float(get("bucket_edge_jump").value))
        self.candidate_merge_distance = max(0.05, float(get("candidate_merge_distance").value))
        self.max_boundary_points_per_side = max(2, int(get("max_boundary_points_per_side").value))
        self.boundary_fit_residual = max(0.10, float(get("boundary_fit_residual").value))
        self.pair_y_tolerance = max(0.10, float(get("pair_y_tolerance").value))
        self.min_corridor_width = max(0.20, float(get("min_corridor_width").value))
        self.max_corridor_width = max(
            self.min_corridor_width + 0.10, float(get("max_corridor_width").value)
        )
        self.perception_smoothing = min(
            0.95, max(0.0, float(get("perception_smoothing").value))
        )

        self.pwm_neutral = float(get("pwm_neutral").value)
        self.pwm_fast = float(get("pwm_fast").value)
        self.pwm_slow = float(get("pwm_slow").value)
        self.steer_center = float(get("steer_center").value)
        self.steer_min = float(get("steer_min").value)
        self.steer_max = float(get("steer_max").value)
        self.cte_gain = float(get("cte_gain").value)
        self.heading_gain = float(get("heading_gain").value)
        self.cte_d_gain = float(get("cte_d_gain").value)
        self.cte_d_limit = max(0.05, float(get("cte_d_limit").value))
        self.gyro_damping = float(get("gyro_damping").value)
        self.steer_rate_limit = float(get("steer_rate_limit").value)
        self.steer_smoothing = min(0.98, max(0.0, float(get("steer_smoothing").value)))
        self.pwm_smoothing = min(0.95, max(0.0, float(get("pwm_smoothing").value)))
        self.pwm_accel_rate = max(1.0, float(get("pwm_accel_rate").value))
        self.pwm_decel_rate = max(1.0, float(get("pwm_decel_rate").value))
        self.front_width = float(get("front_width").value)
        self.front_stop_distance = float(get("front_stop_distance").value)
        self.front_slow_distance = float(get("front_slow_distance").value)
        self.front_obstacle_min_points = max(
            1, int(get("front_obstacle_min_points").value)
        )
        self.scan_timeout = float(get("scan_timeout").value)
        self.imu_timeout = float(get("imu_timeout").value)
        self.odom_timeout = float(get("odom_timeout").value)
        self.allow_one_side = bool(get("allow_one_side").value)
        self.one_side_speed_scale = float(get("one_side_speed_scale").value)
        self.no_boundary_stop = bool(get("no_boundary_stop").value)
        self.boundary_loss_grace_scans = max(0, int(get("boundary_loss_grace_scans").value))
        self.boundary_hold_time = max(0.0, float(get("boundary_hold_time").value))
        self.boundary_loss_speed_scale = max(0.0, min(1.0, float(get("boundary_loss_speed_scale").value)))
        self.odom_speed_limit = max(0.1, float(get("odom_speed_limit").value))

        self.track_mode = str(get("track_mode").value).upper()
        self.b_gap_enabled = bool(get("b_gap_enabled").value)
        self.b_gap_min_total_distance = float(get("b_gap_min_total_distance").value)
        self.b_gap_max_total_distance = float(get("b_gap_max_total_distance").value)
        self.b_gap_min_heading_change = math.radians(float(get("b_gap_min_heading_change_deg").value))
        self.b_gap_loss_scans = int(get("b_gap_loss_scans").value)
        self.b_gap_first_length = float(get("b_gap_first_length").value)
        self.b_gap_second_length = float(get("b_gap_second_length").value)
        self.b_gap_first_yaw = math.radians(float(get("b_gap_first_yaw_deg").value))
        self.b_gap_direction = 1.0 if float(get("b_gap_direction").value) >= 0.0 else -1.0
        self.b_gap_reacquire_distance = float(get("b_gap_reacquire_distance").value)
        self.b_gap_max_distance = float(get("b_gap_max_distance").value)
        self.b_gap_pwm = float(get("b_gap_pwm").value)
        self.b_gap_yaw_gain = float(get("b_gap_yaw_gain").value)
        self.b_gap_gyro_gain = float(get("b_gap_gyro_gain").value)
        self.b_gap_steer_limit = float(get("b_gap_steer_limit").value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.filtered_scan_pub = self.create_publisher(LaserScan, self.filtered_scan_topic, sensor_qos)
        self.center_error_pub = self.create_publisher(Float32, "/pure_lidar/center_error", 10)
        self.heading_error_pub = self.create_publisher(Float32, "/pure_lidar/heading_error", 10)
        self.confidence_pub = self.create_publisher(Float32, "/pure_lidar/confidence", 10)
        self.boundary_points_pub = self.create_publisher(Float32MultiArray, "/pure_lidar/boundary_points", 10)
        self.obstacle_pub = self.create_publisher(Bool, "/pure_lidar/obstacle", 10)
        self.fault_pub = self.create_publisher(Bool, "/pure_lidar/fault", 10)
        self.status_pub = self.create_publisher(String, "/pure_lidar/status", 10)
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
        self.last_throttle = self.pwm_neutral
        self.last_center_error: Optional[float] = None
        self.last_center_error_time = 0.0
        self.filtered_center_error: Optional[float] = None
        self.filtered_heading_error: Optional[float] = None
        self.last_log_time = 0.0
        self.last_reason = "startup"
        self.shutdown_sent = False
        self.latest_scan_valid = False
        self.latest_valid_points = 0
        self.latest_valid_ratio = 0.0
        self.latest_boundary_points: List[float] = []
        self.last_corridor = Corridor(False, 0.0, None, None, 0.0, 0.0, 0.0, None, 0, 0, 0)

        # Track-B state and dead-reckoning state.
        self.odom_distance = 0.0
        self.last_odom_xy: Optional[Tuple[float, float]] = None
        self.current_yaw: Optional[float] = None
        self.unwrapped_yaw: Optional[float] = None
        self.initial_unwrapped_yaw: Optional[float] = None
        self.last_yaw_sample: Optional[float] = None
        self.last_imu_orientation_time = 0.0
        self.corridor_loss_count = 0
        self.last_processed_scan_time = 0.0
        self.last_valid_corridor_time = 0.0
        self.last_good_corridor: Optional[Corridor] = None
        self.last_good_corridor_time = 0.0
        self.b_gap_active = False
        self.b_gap_completed = False
        self.b_gap_start_distance = 0.0
        self.b_gap_start_yaw = 0.0
        self.b_gap_phase = "normal"

        mode = "MOTION ENABLED" if self.enable_motion else "DRY RUN / NEUTRAL ONLY"
        self.get_logger().warning(f"pure_lidar_b_node started: {mode}")
        self.get_logger().info(
            f"scan={self.scan_topic} imu={self.imu_topic} "
            f"odom={self.odom_topic} cmd={self.cmd_topic}"
        )
        self.get_logger().warning(
            "Track B mode is active. Yellow V gap is bridged with IMU/odometry. "
            "It still cannot identify traffic-light "
            "colors, A/B signs, or yellow-line semantics."
        )

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.last_scan_time = self._now()

        sensor_min = max(self.min_range, float(msg.range_min))
        sensor_max = min(self.max_range, float(msg.range_max))
        valid_ranges = [
            float(value) for value in msg.ranges
            if math.isfinite(float(value)) and sensor_min <= float(value) <= sensor_max
        ]
        self.latest_valid_points = len(valid_ranges)
        total = max(1, len(msg.ranges))
        self.latest_valid_ratio = self.latest_valid_points / total
        self.latest_scan_valid = (
            self.latest_valid_points >= self.min_valid_scan_points
            and self.latest_valid_ratio >= self.min_valid_scan_ratio
        )

        if self.publish_filtered_scan:
            filtered = LaserScan()
            filtered.header = msg.header
            filtered.angle_min = msg.angle_min
            filtered.angle_max = msg.angle_max
            filtered.angle_increment = msg.angle_increment
            filtered.time_increment = msg.time_increment
            filtered.scan_time = msg.scan_time
            filtered.range_min = msg.range_min
            filtered.range_max = msg.range_max
            filtered.ranges = [
                float(value) if math.isfinite(float(value)) and sensor_min <= float(value) <= sensor_max
                else float("inf")
                for value in msg.ranges
            ]
            filtered.intensities = list(msg.intensities)
            self.filtered_scan_pub.publish(filtered)

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> Optional[float]:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 0.5:
            return None
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def _set_yaw(self, yaw: float) -> None:
        if not math.isfinite(yaw):
            return
        if self.last_yaw_sample is None:
            self.current_yaw = yaw
            self.unwrapped_yaw = yaw
            self.initial_unwrapped_yaw = yaw
            self.last_yaw_sample = yaw
            return
        delta = self._wrap_angle(yaw - self.last_yaw_sample)
        self.unwrapped_yaw = float(self.unwrapped_yaw) + delta
        self.current_yaw = yaw
        self.last_yaw_sample = yaw

    def _imu_callback(self, msg: Imu) -> None:
        now = self._now()
        value = float(msg.angular_velocity.z)
        if math.isfinite(value):
            self.gyro_z = value
            self.last_imu_time = now
        q = msg.orientation
        yaw = self._yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        orientation_valid = (
            len(msg.orientation_covariance) == 0
            or float(msg.orientation_covariance[0]) != -1.0
        )
        if yaw is not None and orientation_valid:
            self._set_yaw(yaw)
            self.last_imu_orientation_time = now

    def _odom_callback(self, msg: Odometry) -> None:
        now = self._now()
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        speed = math.hypot(vx, vy)
        if math.isfinite(speed):
            self.odom_speed = speed
            self.last_odom_time = now

        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        if math.isfinite(px) and math.isfinite(py):
            if self.last_odom_xy is not None:
                step = math.hypot(px - self.last_odom_xy[0], py - self.last_odom_xy[1])
                if 0.0001 <= step <= 0.75:
                    self.odom_distance += step
            self.last_odom_xy = (px, py)

        # Prefer IMU orientation. Odom yaw is the fallback if the IMU message has
        # no valid quaternion.
        if now - self.last_imu_orientation_time > 0.50:
            q = msg.pose.pose.orientation
            yaw = self._yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
            if yaw is not None:
                self._set_yaw(yaw)

    def _control_callback(self) -> None:
        now = self._now()
        if self.last_scan is None or now - self.last_scan_time > self.scan_timeout:
            self._safe_stop("scan timeout", "SCAN_TIMEOUT")
            return
        if not self.latest_scan_valid:
            self._safe_stop(
                f"invalid scan: {self.latest_valid_points} valid points "
                f"({100.0 * self.latest_valid_ratio:.1f}%)",
                "INVALID_SCAN",
            )
            return

        corridor, front_min = self._estimate_corridor(self.last_scan)
        self.last_corridor = corridor
        new_scan = self.last_scan_time > self.last_processed_scan_time + 1e-6
        if new_scan:
            self.last_processed_scan_time = self.last_scan_time
        imu_ok = now - self.last_imu_time <= self.imu_timeout
        odom_ok = now - self.last_odom_time <= self.odom_timeout

        if front_min <= self.front_stop_distance:
            self._safe_stop(f"front obstacle {front_min:.2f} m", "FRONT_OBSTACLE", obstacle=True)
            return

        if self.b_gap_active:
            self._control_b_gap(corridor, imu_ok, odom_ok)
            return

        # Count boundary losses once per new lidar frame, not at the 20 Hz
        # control rate. This prevents a single weak 7-10 Hz scan from being
        # counted two or three times and causing a stop/start cycle.
        speed_scale = 1.0
        status_override: Optional[str] = None
        if corridor.valid:
            if new_scan:
                self.corridor_loss_count = 0
                self.last_valid_corridor_time = now
                self.last_good_corridor_time = now
                self.last_good_corridor = Corridor(**corridor.__dict__)
        else:
            if new_scan:
                self.corridor_loss_count += 1
            if self._try_start_b_gap(imu_ok, odom_ok):
                self._control_b_gap(corridor, imu_ok, odom_ok)
                return

            # A live scan may occasionally lose the cones for one or two frames.
            # Briefly reuse only the last reliable corridor, at reduced speed.
            # Scan timeout and invalid-scan faults still stop immediately.
            can_hold = (
                self.last_good_corridor is not None
                and self.corridor_loss_count <= self.boundary_loss_grace_scans
                and now - self.last_good_corridor_time <= self.boundary_hold_time
            )
            if can_hold:
                corridor = Corridor(**self.last_good_corridor.__dict__)
                corridor.confidence = min(corridor.confidence, 0.48)
                speed_scale *= self.boundary_loss_speed_scale
                status_override = "BOUNDARY_HOLD"
            elif self.no_boundary_stop:
                self._safe_stop(
                    f"no reliable corridor for {self.corridor_loss_count} lidar frames",
                    "NO_BOUNDARY",
                )
                return

        # A one-sided estimate is deliberately limited to slow speed.
        if corridor.left_count == 0 or corridor.right_count == 0:
            if not self.allow_one_side:
                self._safe_stop("only one boundary", "ONE_SIDE_REJECTED")
                return
            speed_scale *= self.one_side_speed_scale
            corridor.confidence = min(corridor.confidence, 0.55)

        # node1 uses a PD steering controller. Here the same useful idea is
        # applied to the corridor centre error, while retaining heading preview
        # and IMU yaw-rate damping. Derivative state is updated once per scan,
        # not once per timer tick, so a 10 Hz lidar is stable with a 20 Hz loop.
        center_derivative = 0.0
        if self.last_scan_time > self.last_center_error_time + 1e-6:
            if self.last_center_error is not None and self.last_center_error_time > 0.0:
                dt = max(0.02, self.last_scan_time - self.last_center_error_time)
                center_derivative = (
                    corridor.near_center_x - self.last_center_error
                ) / dt
                center_derivative = max(
                    -self.cte_d_limit, min(self.cte_d_limit, center_derivative)
                )
            self.last_center_error = corridor.near_center_x
            self.last_center_error_time = self.last_scan_time

        steer_delta = self.cte_gain * corridor.near_center_x
        steer_delta += self.heading_gain * corridor.heading_error
        steer_delta += self.cte_d_gain * center_derivative
        if imu_ok:
            steer_delta -= self.gyro_damping * self.gyro_z
        steer_delta = max(-50.0, min(50.0, steer_delta))

        desired_steer = self.steer_center + steer_delta
        alpha = 1.0 - self.steer_smoothing
        filtered = self.last_steer * self.steer_smoothing + desired_steer * alpha
        max_step = self.steer_rate_limit / self.control_hz
        filtered = max(self.last_steer - max_step, min(self.last_steer + max_step, filtered))
        steer = max(self.steer_min, min(self.steer_max, filtered))
        self.last_steer = steer

        # Continuous speed scaling avoids the abrupt fast/slow toggling in
        # node1. Steering demand, preview curvature, confidence, one-side mode,
        # and obstacle clearance all reduce the throttle span smoothly.
        turn_ratio = max(
            abs(steer_delta) / 45.0,
            abs(corridor.heading_error) / 0.50,
        )
        turn_ratio = max(0.0, min(1.0, turn_ratio))
        # Keep normal cornering above pwm_slow instead of repeatedly dropping
        # close to neutral. Confidence and special fallback modes still reduce
        # speed, but all changes pass through a throttle low-pass/rate limiter.
        speed = self.pwm_slow + (self.pwm_fast - self.pwm_slow) * (1.0 - 0.78 * turn_ratio)
        speed -= 8.0 * (1.0 - corridor.confidence)
        speed = self.pwm_neutral + max(0.0, speed - self.pwm_neutral) * speed_scale
        if front_min < self.front_slow_distance:
            clearance = (front_min - self.front_stop_distance) / max(
                0.05, self.front_slow_distance - self.front_stop_distance
            )
            obstacle_scale = 0.45 + 0.55 * max(0.0, min(1.0, clearance))
            speed = self.pwm_neutral + (speed - self.pwm_neutral) * obstacle_scale
        if odom_ok and self.odom_speed > self.odom_speed_limit:
            speed = min(speed, self.pwm_slow)
        speed = self._smooth_throttle(speed)

        status = status_override or (
            "ONE_SIDE" if corridor.left_count == 0 or corridor.right_count == 0 else "CORRIDOR"
        )
        self._publish_perception(status, False, front_min < self.front_slow_distance, corridor)
        self._publish_command(speed, steer, "corridor")
        self._periodic_log(
            f"B corridor conf={corridor.confidence:.2f} "
            f"L/R={corridor.left_count}/{corridor.right_count} "
            f"center={corridor.near_center_x:.2f} heading={corridor.heading_error:.2f} "
            f"distance={self.odom_distance:.2f} cmd=({speed:.1f},{steer:.1f})"
        )

    def _try_start_b_gap(self, imu_ok: bool, odom_ok: bool) -> bool:
        if self.track_mode != "B" or not self.b_gap_enabled or self.b_gap_completed:
            return False
        if self.corridor_loss_count < self.b_gap_loss_scans:
            return False
        if not imu_ok or not odom_ok or self.unwrapped_yaw is None or self.initial_unwrapped_yaw is None:
            return False
        if not (self.b_gap_min_total_distance <= self.odom_distance <= self.b_gap_max_total_distance):
            return False
        heading_change = abs(self.unwrapped_yaw - self.initial_unwrapped_yaw)
        if heading_change < self.b_gap_min_heading_change:
            return False
        if self._now() - self.last_valid_corridor_time > 1.2:
            return False

        self.b_gap_active = True
        self.b_gap_phase = "first_leg"
        self.b_gap_start_distance = self.odom_distance
        self.b_gap_start_yaw = float(self.unwrapped_yaw)
        self.last_steer = self.steer_center
        self.get_logger().warning(
            f"TRACK B YELLOW GAP ENTER: distance={self.odom_distance:.2f} "
            f"heading_change={math.degrees(heading_change):.1f} deg"
        )
        return True

    def _control_b_gap(self, corridor: Corridor, imu_ok: bool, odom_ok: bool) -> None:
        if not imu_ok or not odom_ok or self.unwrapped_yaw is None:
            self._safe_stop("B gap requires live IMU and odometry", "B_GAP_SENSOR_FAULT")
            return

        travelled = max(0.0, self.odom_distance - self.b_gap_start_distance)
        total_nominal = self.b_gap_first_length + self.b_gap_second_length

        # At the right entrance the car is approximately heading west. The first
        # 3.06 m segment is 22.5 degrees toward the V point; the second 3.06 m
        # segment is 22.5 degrees on the opposite side of the entrance heading.
        if travelled < self.b_gap_first_length:
            self.b_gap_phase = "first_leg"
            yaw_offset = self.b_gap_direction * self.b_gap_first_yaw
        elif travelled < total_nominal:
            self.b_gap_phase = "second_leg"
            yaw_offset = -self.b_gap_direction * self.b_gap_first_yaw
        else:
            self.b_gap_phase = "exit_align"
            yaw_offset = 0.0

        target_yaw = self.b_gap_start_yaw + yaw_offset
        yaw_error = self._wrap_angle(target_yaw - float(self.unwrapped_yaw))
        steer_delta = self.b_gap_yaw_gain * yaw_error - self.b_gap_gyro_gain * self.gyro_z
        steer_delta = max(-self.b_gap_steer_limit, min(self.b_gap_steer_limit, steer_delta))
        desired_steer = self.steer_center + steer_delta

        max_step = self.steer_rate_limit / self.control_hz
        steer = max(self.last_steer - max_step, min(self.last_steer + max_step, desired_steer))
        steer = max(self.steer_min, min(self.steer_max, steer))
        self.last_steer = steer

        # Near the left endpoint, immediately hand control back to lidar once both
        # boundaries are visible again.
        if travelled >= self.b_gap_reacquire_distance and corridor.valid and corridor.left_count > 0 and corridor.right_count > 0:
            self.b_gap_active = False
            self.b_gap_completed = True
            self.b_gap_phase = "completed"
            self.corridor_loss_count = 0
            self._publish_perception("LIDAR_REACQUIRE", False, False, corridor)
            self.get_logger().warning(
                f"TRACK B YELLOW GAP EXIT: travelled={travelled:.2f} m; lidar reacquired"
            )
            return

        if travelled > self.b_gap_max_distance:
            self.b_gap_active = False
            self._safe_stop(
                f"B gap exceeded {self.b_gap_max_distance:.2f} m without lidar reacquisition",
                "B_GAP_NO_REACQUIRE",
            )
            return

        gap_status = {
            "first_leg": "B_GAP_FIRST",
            "second_leg": "B_GAP_SECOND",
            "exit_align": "B_GAP_EXIT_ALIGN",
        }.get(self.b_gap_phase, "B_GAP")
        self._publish_perception(gap_status, False, False, corridor)
        gap_speed = self._smooth_throttle(self.b_gap_pwm)
        self._publish_command(gap_speed, steer, f"B_gap_{self.b_gap_phase}")
        self._periodic_log(
            f"B GAP phase={self.b_gap_phase} travelled={travelled:.2f}/{total_nominal:.2f} "
            f"yaw_error={math.degrees(yaw_error):.1f}deg cmd=({gap_speed:.1f},{steer:.1f})"
        )

    def _estimate_corridor(self, scan: LaserScan) -> Tuple[Corridor, float]:
        # Keep the original LaserScan index with every point. This is important:
        # inf/NaN samples represent a real break between objects and must not be
        # silently removed before clustering.
        points: List[Tuple[float, float, float, int]] = []
        front_returns: List[float] = []
        angle = float(scan.angle_min)
        for index, raw_range in enumerate(scan.ranges):
            r = float(raw_range)
            if math.isfinite(r) and self.min_range <= r <= self.max_range:
                x = r * math.sin(angle)
                y = r * math.cos(angle)
                if y > 0.10 and abs(x) <= self.front_width:
                    front_returns.append(y)
                if -self.max_lateral <= x <= self.max_lateral and -0.1 <= y <= self.max_forward:
                    points.append((x, y, r, index))
            angle += float(scan.angle_increment)

        # Require several neighbouring returns before declaring a frontal
        # obstacle. This removes single-ray dust/glare stops without hiding a
        # real board, cone, or wall.
        front_returns.sort()
        if len(front_returns) >= self.front_obstacle_min_points:
            front_min = front_returns[self.front_obstacle_min_points - 1]
        else:
            front_min = float("inf")

        cluster_candidates = self._cluster(points)
        bucket_candidates = self._extract_bucket_edges(scan) if self.use_bucket_edges else []
        candidates = self._merge_candidates(bucket_candidates + cluster_candidates)

        left_all = [
            c for c in candidates
            if (
                -self.boundary_max_lateral <= c.x < -self.side_deadband
                and self.side_min_y <= c.y <= self.side_max_y
            )
        ]
        right_all = [
            c for c in candidates
            if (
                self.side_deadband < c.x <= self.boundary_max_lateral
                and self.side_min_y <= c.y <= self.side_max_y
            )
        ]
        left = self._select_side(left_all)
        right = self._select_side(right_all)

        # Flat groups of five values: x, y, radius, point_count, side.
        # side=-1 means vehicle-left; side=+1 means vehicle-right.
        self.latest_boundary_points = []
        for item, side in [(c, -1.0) for c in left] + [(c, 1.0) for c in right]:
            self.latest_boundary_points.extend(
                [float(item.x), float(item.y), float(item.radius), float(item.points), side]
            )

        left_fit = self._fit_side(left)
        right_fit = self._fit_side(right)
        center_points, pair_widths = self._pair_boundaries(left, right)
        center_fit = self._fit_side(center_points) if len(center_points) >= 2 else None

        near_y = self.near_lookahead
        far_y = self.far_lookahead
        left_near = self._eval_fit(left_fit, near_y)
        left_far = self._eval_fit(left_fit, far_y)
        right_near = self._eval_fit(right_fit, near_y)
        right_far = self._eval_fit(right_fit, far_y)

        width: Optional[float] = None
        if pair_widths:
            ordered_widths = sorted(pair_widths)
            width = ordered_widths[len(ordered_widths) // 2]

        # Paired left/right observations form centre points directly and are
        # preferred over averaging two independent fits. This is much harder
        # for a single outlier or an inside-corner cone to pull off course.
        if center_fit is not None:
            near_center = float(self._eval_fit(center_fit, near_y))
            far_center = float(self._eval_fit(center_fit, far_y))
        elif left_near is not None and right_near is not None:
            near_center = 0.5 * (left_near + right_near)
            far_center = 0.5 * (
                (left_far if left_far is not None else left_near)
                + (right_far if right_far is not None else right_near)
            )
            if width is None:
                width = right_near - left_near
        elif left_near is not None:
            near_center = left_near + 0.5 * self.track_width
            far_center = (left_far if left_far is not None else left_near) + 0.5 * self.track_width
        elif right_near is not None:
            near_center = right_near - 0.5 * self.track_width
            far_center = (right_far if right_far is not None else right_near) - 0.5 * self.track_width
        else:
            self.filtered_center_error = None
            self.filtered_heading_error = None
            return Corridor(
                False, 0.0, None, None, 0.0, 0.0, 0.0, None,
                len(candidates), len(left), len(right)
            ), front_min

        heading_error = math.atan2(
            far_center - near_center, max(0.1, far_y - near_y)
        )
        confidence = self._confidence(left, right, width, len(center_points))
        valid = (left_near is not None or right_near is not None) and confidence >= 0.25

        # Filter only consecutive valid estimates. Fault paths still publish a
        # fresh neutral command and never continue driving on an old estimate.
        if valid:
            alpha = 1.0 - self.perception_smoothing
            if self.filtered_center_error is None:
                self.filtered_center_error = near_center
                self.filtered_heading_error = heading_error
            else:
                self.filtered_center_error = (
                    self.perception_smoothing * self.filtered_center_error
                    + alpha * near_center
                )
                self.filtered_heading_error = (
                    self.perception_smoothing * float(self.filtered_heading_error)
                    + alpha * heading_error
                )
            center_shift = float(self.filtered_center_error) - near_center
            near_center = float(self.filtered_center_error)
            far_center += center_shift
            heading_error = float(self.filtered_heading_error)
        else:
            self.filtered_center_error = None
            self.filtered_heading_error = None

        return Corridor(
            valid, confidence, left_near, right_near, near_center, far_center,
            heading_error, width, len(candidates), len(left), len(right)
        ), front_min

    def _extract_bucket_edges(self, scan: LaserScan) -> List[Cluster]:
        """Extract cone/object leading edges using the useful idea from node1."""
        ranges = scan.ranges
        count = len(ranges)
        result: List[Cluster] = []
        if count < self.bucket_window + 2:
            return result

        sensor_min = max(self.min_range, float(scan.range_min))
        sensor_max = min(self.max_range, float(scan.range_max))
        for i in range(1, count - self.bucket_window):
            current = float(ranges[i])
            if not math.isfinite(current) or not sensor_min <= current <= sensor_max:
                continue

            previous = float(ranges[i - 1])
            # A leading edge is either preceded by no return, or by a return
            # substantially farther away. node1 used 2.0 m; 0.55 m is a more
            # tolerant default for nearby cones and can be tuned in YAML.
            leading_edge = (
                not math.isfinite(previous)
                or previous < sensor_min
                or previous > sensor_max
                or previous - current >= self.bucket_edge_jump
            )
            if not leading_edge:
                continue

            support_ranges: List[float] = []
            for offset in range(self.bucket_window):
                value = float(ranges[i + offset])
                if math.isfinite(value) and abs(value - current) <= self.bucket_range_tolerance:
                    support_ranges.append(value)
            if len(support_ranges) < self.bucket_min_support:
                continue

            representative = sum(support_ranges) / len(support_ranges)
            middle_index = i + (self.bucket_window - 1) // 2
            theta = float(scan.angle_min) + middle_index * float(scan.angle_increment)
            x = representative * math.sin(theta)
            y = representative * math.cos(theta)
            if (
                abs(x) <= self.max_lateral
                and self.side_min_y <= y <= self.side_max_y
            ):
                result.append(Cluster(x, y, 0.08, len(support_ranges)))
        return result

    def _cluster(self, points: Sequence[Tuple[float, float, float, int]]) -> List[Cluster]:
        if not points:
            return []
        groups: List[List[Tuple[float, float, float, int]]] = []
        current: List[Tuple[float, float, float, int]] = [points[0]]
        for point in points[1:]:
            previous = current[-1]
            index_gap = point[3] - previous[3]
            spatial_gap = math.hypot(point[0] - previous[0], point[1] - previous[1])
            if index_gap <= self.max_index_gap and spatial_gap <= self.cluster_gap:
                current.append(point)
            else:
                groups.append(current)
                current = [point]
        groups.append(current)

        result: List[Cluster] = []
        for group in groups:
            if len(group) < self.min_cluster_points:
                continue
            whole = self._cluster_from_group(group)
            if (
                whole is not None
                and len(group) <= self.max_cluster_points
                and whole.radius <= self.max_cluster_radius
            ):
                result.append(whole)
                continue

            # A long continuous wall/board is still a useful boundary. Split it
            # into local segments instead of rejecting the complete large group.
            step = self.wall_segment_points
            for offset in range(0, len(group), step):
                segment = group[offset:offset + step]
                if len(segment) < self.min_cluster_points:
                    continue
                item = self._cluster_from_group(segment)
                if item is not None:
                    result.append(item)
        return result

    @staticmethod
    def _cluster_from_group(
        group: Sequence[Tuple[float, float, float, int]]
    ) -> Optional[Cluster]:
        if not group:
            return None
        xs = [point[0] for point in group]
        ys = [point[1] for point in group]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        radius = max(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys))
        return Cluster(cx, cy, radius, len(group))

    def _merge_candidates(self, candidates: Sequence[Cluster]) -> List[Cluster]:
        merged: List[Cluster] = []
        # Nearest observations first, following node1's preference for the few
        # cones that matter most to immediate steering.
        for candidate in sorted(candidates, key=lambda item: item.y):
            match_index: Optional[int] = None
            for index, existing in enumerate(merged):
                if math.hypot(candidate.x - existing.x, candidate.y - existing.y) <= self.candidate_merge_distance:
                    match_index = index
                    break
            if match_index is None:
                merged.append(candidate)
                continue
            existing = merged[match_index]
            old_weight = max(1, existing.points)
            new_weight = max(1, candidate.points)
            total = old_weight + new_weight
            merged[match_index] = Cluster(
                (existing.x * old_weight + candidate.x * new_weight) / total,
                (existing.y * old_weight + candidate.y * new_weight) / total,
                max(existing.radius, candidate.radius),
                total,
            )
        return merged

    def _select_side(self, candidates: Sequence[Cluster]) -> List[Cluster]:
        if not candidates:
            return []
        # Keep a small local horizon as node1 does (it uses three points), but
        # allow up to five so a continuous wall and wider cone spacing also work.
        selected = sorted(candidates, key=lambda item: item.y)[
            : max(self.max_boundary_points_per_side * 2, self.max_boundary_points_per_side)
        ]
        while len(selected) > 2:
            fit = self._weighted_fit(selected)
            if fit is None:
                break
            residuals = [abs(item.x - (fit[0] * item.y + fit[1])) for item in selected]
            worst = max(range(len(residuals)), key=residuals.__getitem__)
            if residuals[worst] <= self.boundary_fit_residual:
                break
            selected.pop(worst)
        return sorted(selected, key=lambda item: item.y)[:self.max_boundary_points_per_side]

    def _pair_boundaries(
        self, left: Sequence[Cluster], right: Sequence[Cluster]
    ) -> Tuple[List[Cluster], List[float]]:
        centers: List[Cluster] = []
        widths: List[float] = []
        available = set(range(len(right)))
        for left_item in sorted(left, key=lambda item: item.y):
            choices = sorted(
                available, key=lambda index: abs(right[index].y - left_item.y)
            )
            for index in choices:
                right_item = right[index]
                if abs(right_item.y - left_item.y) > self.pair_y_tolerance:
                    break
                width = right_item.x - left_item.x
                if self.min_corridor_width <= width <= self.max_corridor_width:
                    available.remove(index)
                    centers.append(Cluster(
                        0.5 * (left_item.x + right_item.x),
                        0.5 * (left_item.y + right_item.y),
                        0.5 * (left_item.radius + right_item.radius),
                        left_item.points + right_item.points,
                    ))
                    widths.append(width)
                    break
        return centers, widths

    def _weighted_fit(self, clusters: Sequence[Cluster]) -> Optional[Tuple[float, float]]:
        if not clusters:
            return None
        if len(clusters) == 1:
            return 0.0, clusters[0].x
        weights = [1.0 / max(0.35, item.y) for item in clusters]
        total_weight = sum(weights)
        mean_y = sum(w * item.y for w, item in zip(weights, clusters)) / total_weight
        mean_x = sum(w * item.x for w, item in zip(weights, clusters)) / total_weight
        denominator = sum(
            w * (item.y - mean_y) ** 2 for w, item in zip(weights, clusters)
        )
        if denominator < 1e-6:
            return 0.0, mean_x
        slope = sum(
            w * (item.y - mean_y) * (item.x - mean_x)
            for w, item in zip(weights, clusters)
        ) / denominator
        return slope, mean_x - slope * mean_y

    def _fit_side(self, clusters: Sequence[Cluster]) -> Optional[Tuple[float, float]]:
        """Robustly fit x=a*y+b, removing at most a few large outliers."""
        if not clusters:
            return None
        kept = list(clusters)
        while len(kept) > 2:
            fit = self._weighted_fit(kept)
            if fit is None:
                return None
            residuals = [abs(item.x - (fit[0] * item.y + fit[1])) for item in kept]
            worst = max(range(len(residuals)), key=residuals.__getitem__)
            if residuals[worst] <= self.boundary_fit_residual:
                return fit
            kept.pop(worst)
        return self._weighted_fit(kept)

    @staticmethod
    def _eval_fit(fit: Optional[Tuple[float, float]], y: float) -> Optional[float]:
        if fit is None:
            return None
        return fit[0] * y + fit[1]

    def _confidence(
        self,
        left: Sequence[Cluster],
        right: Sequence[Cluster],
        width: Optional[float],
        pair_count: int,
    ) -> float:
        if not left and not right:
            return 0.0
        count_score = min(1.0, (len(left) + len(right)) / 6.0)
        side_score = 1.0 if left and right else 0.42
        pair_score = min(1.0, pair_count / 2.0) if left and right else 0.0
        width_score = 0.65 if width is None else max(
            0.0, 1.0 - abs(width - self.track_width) / self.track_width_tolerance
        )
        confidence = (
            0.30 * count_score
            + 0.25 * side_score
            + 0.25 * pair_score
            + 0.20 * width_score
        )
        return max(0.0, min(1.0, confidence))

    def _publish_perception(
        self, status: str, fault: bool, obstacle: bool, corridor: Optional[Corridor] = None
    ) -> None:
        current = corridor if corridor is not None else self.last_corridor

        center_msg = Float32()
        center_msg.data = float(current.near_center_x)
        self.center_error_pub.publish(center_msg)

        heading_msg = Float32()
        heading_msg.data = float(current.heading_error)
        self.heading_error_pub.publish(heading_msg)

        confidence_msg = Float32()
        confidence_msg.data = float(current.confidence)
        self.confidence_pub.publish(confidence_msg)

        boundary_msg = Float32MultiArray()
        boundary_msg.data = list(self.latest_boundary_points)
        self.boundary_points_pub.publish(boundary_msg)

        obstacle_msg = Bool()
        obstacle_msg.data = bool(obstacle)
        self.obstacle_pub.publish(obstacle_msg)

        fault_msg = Bool()
        fault_msg.data = bool(fault)
        self.fault_pub.publish(fault_msg)

        status_msg = String()
        status_msg.data = str(status)
        self.status_pub.publish(status_msg)

    def _smooth_throttle(self, desired: float) -> float:
        desired = max(self.pwm_neutral, min(self.pwm_fast, float(desired)))
        filtered = (
            self.last_throttle * self.pwm_smoothing
            + desired * (1.0 - self.pwm_smoothing)
        )
        rate = self.pwm_accel_rate if filtered >= self.last_throttle else self.pwm_decel_rate
        max_step = rate / self.control_hz
        output = max(
            self.last_throttle - max_step,
            min(self.last_throttle + max_step, filtered),
        )
        self.last_throttle = output
        return output

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

    def _safe_stop(self, reason: str, status: str = "FAULT", obstacle: bool = False) -> None:
        # Always publish a fresh neutral command. Never retain the previous
        # steering/throttle output after a scan or sensor fault.
        self.last_steer = self.steer_center
        self.last_throttle = self.pwm_neutral
        self.last_center_error = None
        self.last_center_error_time = 0.0
        self.filtered_center_error = None
        self.filtered_heading_error = None
        self._publish_perception(status, True, obstacle)
        self._publish_command(self.pwm_neutral, self.steer_center, reason)
        self._periodic_log(f"SAFE STOP [{status}]: {reason}")

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
            # SIGINT/timeout can invalidate the ROS context before this method
            # runs. Publish the final neutral command only while the context is
            # live; the control loop already publishes neutral on every fault.
            if rclpy.ok():
                for _ in range(3):
                    try:
                        self._publish_command(
                            self.pwm_neutral, self.steer_center, "shutdown"
                        )
                    except Exception as exc:  # context may close asynchronously
                        self.get_logger().warning(
                            f"final neutral publish skipped: {exc}"
                        )
                        break
                    time.sleep(0.02)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PureLidarBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGINT/SIGTERM can invalidate the ROS context while a timer or scan
        # callback is publishing. Treat that shutdown race as a normal exit,
        # while still surfacing genuine runtime failures.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


