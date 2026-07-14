#!/usr/bin/env python3
"""Pure ROS 2 version of car_test lidar controller (two-lap radar follow)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


SCAN_RESOLUTION = 1440
BUCKET_THRESHOLD = 10
VALID_BUCKET_THRESHOLD = 6
MAX_BUCKET_POINTS = 3
BUCKET_CACHE_SIZE = 30


@dataclass
class BucketPoint:
    x: float = 0.0
    y: float = 0.0


class MissionPhase(Enum):
    # Ordered mission states used to multiplex CV和雷达控制
    CV_LAP1 = auto()
    LIDAR_LAP1 = auto()
    CV_LAP2 = auto()
    LIDAR_LAP2 = auto()


class PID:
    def __init__(self, kp: float = 3.5, ki: float = 0.0, kd: float = 3.0, integral_max: float = 8.0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.integral = 0.0
        self.last_error = 0.0

    def positional(self, error: float) -> float:
        self.integral += error
        self.integral = max(min(self.integral, self.integral_max), -self.integral_max)
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * (error - self.last_error))
        self.last_error = error
        return output


class LaserControlNode(Node):
    def __init__(self) -> None:
        super().__init__('car_test')
        # Parameters matching the original C++ defaults
        self.speed = self.declare_parameter('speed', 1525.0).value  # 基本PWM巡航速度
        self.max_left_distance = self.declare_parameter('max_left_distance', 1.8).value  # 桶点筛选范围
        self.min_right_distance = self.declare_parameter('min_right_distance', 1.5).value
        self.lap_exit_radius = self.declare_parameter('lap_exit_radius', 9.5).value  # 起圈半径阈值
        self.lap_entry_radius = self.declare_parameter('lap_entry_radius', 2.3).value  # 完成一圈的回收阈值
        self.traffic_stop_radius = self.declare_parameter('traffic_stop_radius', 8.7).value  # 红绿灯停车触发半径
        self.final_stop_radius = self.declare_parameter('final_stop_radius', 1.0).value  # 最终停车点半径
        self.forced_stop_duration = self.declare_parameter('forced_stop_duration', 3.0).value  # 红绿灯等待时间
        self.stop_linear_pwm = self.declare_parameter('stop_linear_pwm', 1350.0).value  # 停车保持PWM
        self.stop_angular_deg = self.declare_parameter('stop_angular_deg', 90.0).value
        self.final_stop_linear_pwm = self.declare_parameter('final_stop_linear_pwm', 1300.0).value
        self.total_laps = self.declare_parameter('total_laps', 2).value
        self.start_with_cv_phase = self.declare_parameter('start_with_cv_phase', True).value  # 是否先跑CV段
        self.use_origin_radius_thresholds = self.declare_parameter('use_origin_radius_thresholds', True).value  # 阈值依据
        # 三个核心阈值：A/B/C，用于切换 CV/雷达段
        self.distance_threshold_cv1_end = float(self.declare_parameter('distance_threshold_cv1_end', 2.63).value)
        self.distance_threshold_lidar1_end = float(self.declare_parameter('distance_threshold_lidar1_end', 0.0).value)
        self.distance_threshold_cv2_end = float(self.declare_parameter('distance_threshold_cv2_end', 2.63).value)
        self.cv_cmd_topic = self.declare_parameter('cv_cmd_topic', '/cv_cmd_vel').value  # 必须与CV节点的输出话题保持一致
        self.cv_cmd_timeout = float(self.declare_parameter('cv_cmd_timeout', 0.5).value)  # CV超时时间，秒
        self.cv_fallback_linear = float(
            self.declare_parameter('cv_fallback_linear', self.stop_linear_pwm).value
        )  # CV断联时的线速度回退值
        self.cv_fallback_angular = float(
            self.declare_parameter('cv_fallback_angular', self.stop_angular_deg).value
        )  # CV断联时的角度回退值

        qos = rclpy.qos.QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, '/teleop_cmd_vel', qos)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, qos)
        self.create_subscription(Odometry, '/encoder_imu_odom', self.odom_callback, qos)
        self.cv_cmd_sub = self.create_subscription(Twist, str(self.cv_cmd_topic), self._cv_cmd_callback, qos)
        # 以上订阅用于接收CV控制指令，在CV阶段直接透传

        self.pid = PID()
        self.twist = Twist()
        self.twist.angular.z = 90.0
        self.twist.linear.x = self.speed

        self.base_link_x = 0.0
        self.base_link_y = 0.0
        self.awaiting_lap_start = True
        self.current_lap = 0
        self.red_stop_lap = 0
        self.forced_stop_active = False
        self.final_stop_active = False
        self.mission_complete = False
        self.total_distance = 0.0
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.has_last_pose = False
        self.forced_stop_release_time = self.get_clock().now()
        self.phase = MissionPhase.CV_LAP1 if self.start_with_cv_phase else MissionPhase.LIDAR_LAP1
        self.latest_cv_cmd: Optional[Twist] = None
        self.last_cv_cmd_time: Optional[Time] = None
        self.cv_warned_stale = False
        self.phase_distance_metric = 0.0
        self.phase_metric_label = 'pose radius' if self.use_origin_radius_thresholds else 'path distance'

        self._validate_distance_thresholds()
        self._evaluate_distance_phase()
        self.get_logger().info('car_test Python node initialised')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry) -> None:
        self.base_link_x = msg.pose.pose.position.x
        self.base_link_y = msg.pose.pose.position.y
        pose_norm = math.hypot(self.base_link_x, self.base_link_y)

        self._update_traveled_distance(pose_norm)

        if self.awaiting_lap_start and pose_norm >= self.lap_exit_radius and self.current_lap < self.total_laps:
            self.awaiting_lap_start = False
            self.current_lap += 1
            self.get_logger().info(
                f'Lap {self.current_lap} started ({self.total_distance:.2f} m accumulated)'
            )
        elif (not self.awaiting_lap_start) and pose_norm <= self.lap_entry_radius:
            self.awaiting_lap_start = True
            self.get_logger().info(f'Lap {self.current_lap} completed, awaiting next start window')

        if self.current_lap > 0 and pose_norm <= self.traffic_stop_radius and self.red_stop_lap < self.current_lap:
            self.red_stop_lap = self.current_lap
            self._engage_timed_stop('traffic-light hold')

        if self.current_lap >= self.total_laps and pose_norm <= self.final_stop_radius:
            self._engage_final_stop()

    def laser_callback(self, msg: LaserScan) -> None:
        now_time = self.get_clock().now()

        if self.final_stop_active:
            self.twist.linear.x = self.final_stop_linear_pwm
            self.twist.angular.z = self.stop_angular_deg
            self._publish_twist()
            return

        if self.forced_stop_active:
            if now_time >= self.forced_stop_release_time:
                self.forced_stop_active = False
                self.get_logger().info(f'Stop window elapsed, resuming lap {self.current_lap}')
            else:
                self.twist.linear.x = self.stop_linear_pwm
                self.twist.angular.z = self.stop_angular_deg
                self._publish_twist()
                return

        if self._cv_phase_active():
            self._apply_cv_command(now_time)
            self._publish_twist()
            return

        red_points: List[BucketPoint] = [BucketPoint() for _ in range(BUCKET_CACHE_SIZE)]
        blue_points: List[BucketPoint] = [BucketPoint() for _ in range(BUCKET_CACHE_SIZE)]
        red_count = 0
        blue_count = 0

        scan_len = min(len(msg.ranges), SCAN_RESOLUTION)
        upper_bound = max(1, scan_len - BUCKET_THRESHOLD)
        for i in range(1, upper_bound):
            current = msg.ranges[i]
            if (not math.isfinite(current)) or current <= 0.0 or current > 10.0:
                continue
            prev = msg.ranges[i - 1]
            next_val = msg.ranges[i + 1] if i + 1 < scan_len else msg.ranges[i]
            if (not math.isfinite(prev)) or (prev - current < 2.0):
                continue
            if (not math.isfinite(next_val)) or (prev - next_val < 2.0):
                continue

            continue_ranges = 0
            for idx in range(1, BUCKET_THRESHOLD):
                if i + idx >= scan_len:
                    break
                neighbour = msg.ranges[i + idx]
                if not math.isfinite(neighbour):
                    break
                if abs(current - neighbour) < 0.2:
                    continue_ranges += 1
                    if continue_ranges >= VALID_BUCKET_THRESHOLD:
                        break

            if continue_ranges < VALID_BUCKET_THRESHOLD:
                continue

            theta = msg.angle_min + float(i) * msg.angle_increment
            x_val = current * math.sin(theta)
            y_val = current * math.cos(theta)

            if x_val <= self.max_left_distance and x_val >= -self.min_right_distance and -0.2 < y_val < 2.0:
                point = BucketPoint(x=x_val, y=y_val)
                if x_val > 0.0 and red_count < BUCKET_CACHE_SIZE:
                    red_points[red_count] = point
                    red_count += 1
                elif x_val <= 0.0 and blue_count < BUCKET_CACHE_SIZE:
                    blue_points[blue_count] = point
                    blue_count += 1

        red_count, blue_count = self._adjust_bucket_counts(red_points, red_count, blue_points, blue_count)

        error_count = min(red_count, blue_count)
        if error_count > 0:
            weighted_error = 0.0
            weight_sum = 0.0
            for idx in range(error_count):
                weight = (blue_points[idx].y + red_points[error_count - 1 - idx].y) / 2.0
                weighted_error += (blue_points[idx].x + red_points[error_count - 1 - idx].x) * weight
                weight_sum += weight

            error = (weighted_error / weight_sum) * 20.0 if weight_sum > 0.0 else 0.0
            if error > 0.0:
                error *= 1.1
            angle = self.pid.positional(error)
            self.twist.angular.z = 90.0 + angle
            self.twist.linear.x = self.speed

            if abs(angle) >= 20.0:
                self.twist.linear.x -= 3.0

            self.twist.angular.z = max(0.0, min(170.0, self.twist.angular.z))

        self._publish_twist()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _publish_twist(self) -> None:
        self.twist.linear.x = max(0.0, min(2000.0, self.twist.linear.x))
        self.twist.angular.z = max(0.0, min(180.0, self.twist.angular.z))
        self.cmd_pub.publish(self.twist)

    def _update_traveled_distance(self, pose_radius: float) -> None:
        if self.has_last_pose:
            self.total_distance += math.hypot(self.base_link_x - self.last_pose_x, self.base_link_y - self.last_pose_y)
        self.last_pose_x = self.base_link_x
        self.last_pose_y = self.base_link_y
        self.has_last_pose = True
        if self.use_origin_radius_thresholds:
            # 根据开关选择“原点半径”或“累计里程”作为阶段触发指标
            self.phase_distance_metric = pose_radius
        else:
            self.phase_distance_metric = self.total_distance
        self._evaluate_distance_phase()

    def _engage_timed_stop(self, reason: str) -> None:
        if self.final_stop_active:
            return
        self.forced_stop_active = True
        self.forced_stop_release_time = self.get_clock().now() + Duration(seconds=float(self.forced_stop_duration))
        self.twist.linear.x = self.stop_linear_pwm
        self.twist.angular.z = self.stop_angular_deg
        self._publish_twist()
        self.get_logger().info(
            f'Timed stop triggered ({reason}) on lap {self.current_lap} | hold {self.forced_stop_duration:.1f}s | '
            f'distance {self.total_distance:.2f}m'
        )

    def _engage_final_stop(self) -> None:
        if self.final_stop_active:
            return
        self.final_stop_active = True
        self.forced_stop_active = False
        self.mission_complete = True
        self.twist.linear.x = self.final_stop_linear_pwm
        self.twist.angular.z = self.stop_angular_deg
        self._publish_twist()
        self.get_logger().info(f'Final stop engaged after {self.total_distance:.2f} m. Mission complete.')

    def _cv_cmd_callback(self, msg: Twist) -> None:
        self.latest_cv_cmd = Twist()
        self.latest_cv_cmd.linear.x = msg.linear.x
        self.latest_cv_cmd.linear.y = msg.linear.y
        self.latest_cv_cmd.linear.z = msg.linear.z
        self.latest_cv_cmd.angular.x = msg.angular.x
        self.latest_cv_cmd.angular.y = msg.angular.y
        self.latest_cv_cmd.angular.z = msg.angular.z
        self.last_cv_cmd_time = self.get_clock().now()
        self.cv_warned_stale = False

    def _cv_phase_active(self) -> bool:
        return self.phase in (MissionPhase.CV_LAP1, MissionPhase.CV_LAP2)

    def _apply_cv_command(self, now_time: Time) -> None:
        if self.latest_cv_cmd is not None and self.last_cv_cmd_time is not None:
            elapsed = (now_time - self.last_cv_cmd_time).nanoseconds / 1e9
            if elapsed <= self.cv_cmd_timeout:
                self.twist.linear.x = self.latest_cv_cmd.linear.x
                self.twist.angular.z = self.latest_cv_cmd.angular.z
                return

        self.twist.linear.x = self.cv_fallback_linear
        self.twist.angular.z = self.cv_fallback_angular
        if not self.cv_warned_stale and self._cv_phase_active():
            self.cv_warned_stale = True
            self.get_logger().warn(
                'CV command timeout reached; pushing fallback teleop until fresh data arrives'
            )

    def _set_phase(self, new_phase: MissionPhase, reason: str) -> None:
        if self.phase == new_phase:
            return
        self.phase = new_phase
        if new_phase in (MissionPhase.LIDAR_LAP1, MissionPhase.LIDAR_LAP2):
            self.pid.integral = 0.0
            self.pid.last_error = 0.0
        self.cv_warned_stale = False
        self.get_logger().info(
            f'Phase switched to {new_phase.name} at metric {self.phase_distance_metric:.2f} ({self.phase_metric_label} | {reason}); '
            f'total distance {self.total_distance:.2f} m'
        )

    def _evaluate_distance_phase(self) -> None:
        transitions = (
            (MissionPhase.CV_LAP1, MissionPhase.LIDAR_LAP1, self.distance_threshold_cv1_end, '阈值A -> 第一圈雷达'),
            (MissionPhase.LIDAR_LAP1, MissionPhase.CV_LAP2, self.distance_threshold_lidar1_end, '阈值B -> 第二段CV'),
            (MissionPhase.CV_LAP2, MissionPhase.LIDAR_LAP2, self.distance_threshold_cv2_end, '阈值C -> 第二圈雷达'),
        )
        for current_phase, next_phase, threshold, label in transitions:
            if self.phase == current_phase and threshold > 0.0 and self.phase_distance_metric >= threshold:
                self._set_phase(next_phase, label)
                break

    def _validate_distance_thresholds(self) -> None:
        ordered = [
            ('A (第一圈CV结束)', self.distance_threshold_cv1_end),
            ('B (第一圈雷达结束)', self.distance_threshold_lidar1_end),
            ('C (第二段CV结束)', self.distance_threshold_cv2_end),
        ]
        last_val = -math.inf
        for label, value in ordered:
            if value <= 0.0:
                continue
            if value <= last_val:
                self.get_logger().warn(
                    f'Distance threshold {label}={value:.2f}m is not larger than previous ({last_val:.2f}m)'
                )
            last_val = value

    @staticmethod
    def _adjust_bucket_counts(
        red_points: List[BucketPoint],
        red_count: int,
        blue_points: List[BucketPoint],
        blue_count: int,
    ) -> Tuple[int, int]:
        def distance_sq(p1: BucketPoint, p2: BucketPoint) -> float:
            return (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2

        if blue_count >= red_count:
            split_index = 100
            for i in range(1, blue_count):
                if distance_sq(blue_points[i - 1], blue_points[i]) > 1.4 * 1.4:
                    split_index = i
                    break

            if split_index != 100:
                for i in range(red_count - 1, -1, -1):
                    target = i + blue_count - split_index
                    if target < BUCKET_CACHE_SIZE:
                        red_points[target] = BucketPoint(red_points[i].x, red_points[i].y)
                for i in range(blue_count - split_index):
                    red_points[i] = BucketPoint(
                        blue_points[split_index + i].x,
                        blue_points[split_index + i].y,
                    )
                red_count += blue_count - split_index
                blue_count = split_index
        else:
            split_index = 100
            for i in range(red_count - 2, -1, -1):
                if distance_sq(red_points[i], red_points[i + 1]) > 1.4 * 1.4:
                    split_index = i
                    break

            if split_index != 100:
                shift_len = split_index + 1
                for i in range(shift_len):
                    if blue_count + i < BUCKET_CACHE_SIZE:
                        blue_points[blue_count + i] = BucketPoint(red_points[i].x, red_points[i].y)
                for i in range(red_count - shift_len):
                    red_points[i] = BucketPoint(red_points[i + shift_len].x, red_points[i + shift_len].y)
                red_count -= shift_len
                blue_count += shift_len

        red_count = min(red_count, MAX_BUCKET_POINTS)
        blue_count = min(blue_count, MAX_BUCKET_POINTS)

        if red_count > blue_count and blue_count > 0:
            if blue_count == 1:
                if red_count > 2:
                    for i in range(2):
                        red_points[i] = BucketPoint(red_points[i + 1].x, red_points[i + 1].y)
                    red_count = 2
                blue_points[1] = BucketPoint(blue_points[0].x, blue_points[0].y)
                blue_count = 2
            elif blue_count == 2:
                blue_points[2] = BucketPoint(blue_points[1].x, blue_points[1].y)
                blue_count = 3
        elif blue_count > red_count and red_count > 0:
            if red_count == 1:
                blue_count = 2
                red_points[1] = BucketPoint(red_points[0].x, red_points[0].y)
                red_count = 2
            elif red_count == 2:
                red_points[2] = BucketPoint(red_points[1].x, red_points[1].y)
                red_points[1] = BucketPoint(red_points[0].x, red_points[0].y)
                red_count = 3

        return red_count, blue_count


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LaserControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
