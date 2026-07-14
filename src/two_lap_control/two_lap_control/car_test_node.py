#!/usr/bin/env python3
"""Two-lap controller orchestrating CV and LiDAR segments using origin-distance thresholds."""

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class PID:
    """Simple positional PID mirroring the legacy C++ implementation."""

    def __init__(self, kp: float = 3.0, ki: float = 0.0, kd: float = 1.0, integral_max: float = 8.0) -> None:
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


class CarTestNode(Node):
    def __init__(self) -> None:
        super().__init__('car_test_node')
        qos = 10

        self.cmd_pub = self.create_publisher(Twist, '/teleop_cmd_vel', qos)
        self.cv_start_pub = self.create_publisher(Bool, 'start_cv', qos)
        self.create_subscription(Bool, 'start_laser', self._start_laser_cb, qos)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.create_subscription(Odometry, '/encoder_imu_odom', self._odom_cb, qos)

        # Parameters
        self.declare_parameter('speed_pwm', 1530.0)
        self.declare_parameter('max_left_distance', 1.8)
        self.declare_parameter('min_right_distance', 1.5)
        self.declare_parameter('threshold_cv_to_lidar', 5.0)   # CV1 收到 start_laser 后，原点距离回落到此值即可开启雷达段1
        self.declare_parameter('threshold_lidar_to_cv', 0.0)   # 雷达段1 收缩到该距离触发第二段 CV
        self.declare_parameter('threshold_cv2_to_lidar', 5.0)  # CV2 完成后距离回落到该值开启雷达段2
        self.declare_parameter('final_stop_threshold', 1.8)    # 雷达段2 收缩至该距离即执行最终停车
        self.declare_parameter('threshold_tolerance', 0.2)     # 阈值比较允许的正向容差，避免噪声抖动
        self.declare_parameter('cv_start_retries', 5)
        self.declare_parameter('cv_start_period', 0.2)
        self.declare_parameter('stop_linear_pwm', 1500.0)
        self.declare_parameter('stop_angular_deg', 90.0)
        self.declare_parameter('ab_trigger_distance', 2.0)     # 原点距离达到该值触发 AB 段
        self.declare_parameter('ab_linear_pwm', 1520.0)        # AB 段固定线速度 PWM
        self.declare_parameter('ab_angular_deg', 110.0)        # AB 段固定舵角
        self.declare_parameter('ab_target_distance', 1.0)      # AB 段需行驶的距离（米）

        # Cached parameter values
        self.speed_pwm = float(self.get_parameter('speed_pwm').value)
        self.max_left_distance = float(self.get_parameter('max_left_distance').value)
        self.min_right_distance = float(self.get_parameter('min_right_distance').value)
        self.threshold_a = float(self.get_parameter('threshold_cv_to_lidar').value)   # CV1 -> LiDAR1 阶段阈值
        self.threshold_b = float(self.get_parameter('threshold_lidar_to_cv').value)   # LiDAR1 -> CV2 阶段阈值
        self.threshold_c = float(self.get_parameter('threshold_cv2_to_lidar').value)  # CV2 -> LiDAR2 阶段阈值
        self.final_stop_threshold = float(self.get_parameter('final_stop_threshold').value)  # LiDAR2 -> 停车阈值
        self.threshold_tolerance = float(self.get_parameter('threshold_tolerance').value)    # 阈值比较容差
        self.stop_linear_pwm = float(self.get_parameter('stop_linear_pwm').value)
        self.stop_angular_deg = float(self.get_parameter('stop_angular_deg').value)
        self.cv_start_retries = int(self.get_parameter('cv_start_retries').value)
        self.cv_start_period = float(self.get_parameter('cv_start_period').value)
        self.ab_trigger_distance = float(self.get_parameter('ab_trigger_distance').value)
        self.ab_linear_pwm = float(self.get_parameter('ab_linear_pwm').value)
        self.ab_angular_deg = float(self.get_parameter('ab_angular_deg').value)
        self.ab_target_distance = float(self.get_parameter('ab_target_distance').value)

        # Internal state
        self.pid = PID()
        self.pose_norm: Optional[float] = None
        self.current_phase = 'cv1'
        self.cv_runs_started = 0
        self.cv_runs_completed = 0
        self.pending_lidar: Optional[str] = None
        self.lidar_active = False
        self.lidar_segment = 0
        self.pending_cv_request: Optional[int] = None
        self.cv_start_retry_remaining = 0
        self.mission_complete = False
        self.ab_mode = False
        self.ab_distance = 0.0
        self.ab_last_position: Optional[Tuple[float, float]] = None

        self.bucket_threshold = 10
        self.valid_bucket_threshold = 6
        self.max_bucket_points = 30

        self.create_timer(self.cv_start_period, self._cv_command_tick)
        self._request_cv_segment()
        self.get_logger().info('car_test_node ready – waiting for CV completion to start LiDAR segment')

    # ------------------------------------------------------------------
    # CV orchestration helpers
    # ------------------------------------------------------------------

    def _request_cv_segment(self) -> None:
        if self.cv_runs_started >= 2:
            self.get_logger().warning('CV segments already scheduled twice; ignoring extra request')
            return
        self.cv_runs_started += 1
        self.pending_cv_request = self.cv_runs_started
        self.cv_start_retry_remaining = max(self.cv_start_retries, 1)
        self.get_logger().info('Requesting CV segment %d' % self.cv_runs_started)

    def _cv_command_tick(self) -> None:
        if self.pending_cv_request is None:
            return
        if self.cv_start_retry_remaining <= 0:
            self.pending_cv_request = None
            return
        msg = Bool()
        msg.data = True
        self.cv_start_pub.publish(msg)
        self.cv_start_retry_remaining -= 1

    def _start_laser_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.cv_runs_completed += 1
        segment = self.cv_runs_completed
        self.pending_cv_request = None
        self.cv_start_retry_remaining = 0
        origin = self.pose_norm if self.pose_norm is not None else -1.0
        self.get_logger().info('CV segment %d complete (origin %.2f m)' % (segment, origin))
        if segment == 1:
            self.pending_lidar = 'lidar1'
        elif segment == 2:
            self.pending_lidar = 'lidar2'
        else:
            self.get_logger().warning('Unexpected CV completion #%d' % segment)
            return
        self._maybe_start_pending_lidar()

    # ------------------------------------------------------------------
    # Odometry logic
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.pose_norm = math.hypot(position.x, position.y)
        self._maybe_start_pending_lidar()

        if not self.ab_mode and not self.mission_complete and self.pose_norm is not None:
            if self.pose_norm >= self.ab_trigger_distance:
                self._start_ab_mode(position)

        if self.ab_mode:
            self._update_ab_distance(position)
            return

        if not self.lidar_active or self.pose_norm is None:
            return

        if self.lidar_segment == 1 and self.pose_norm <= self.threshold_b:
            self._transition_to_second_cv()
        elif self.lidar_segment == 2 and self.pose_norm <= self.final_stop_threshold:
            self._final_stop('radial threshold')

    def _maybe_start_pending_lidar(self) -> None:
        if self.pose_norm is None or self.pending_lidar is None:
            return
        tol = self.threshold_tolerance
        if self.pending_lidar == 'lidar1' and self.pose_norm <= self.threshold_a + tol:
            self.pending_lidar = None
            self._activate_lidar_segment(1)
        elif self.pending_lidar == 'lidar2' and self.pose_norm <= self.threshold_c + tol:
            self.pending_lidar = None
            self._activate_lidar_segment(2)

    def _activate_lidar_segment(self, segment: int) -> None:
        self.lidar_active = True
        self.lidar_segment = segment
        self.current_phase = f'lidar{segment}'
        self.get_logger().info('Starting lidar segment %d (origin %.2f m)' % (segment, self.pose_norm or -1.0))

    def _transition_to_second_cv(self) -> None:
        if self.lidar_segment != 1:
            return
        self.lidar_active = False
        self.lidar_segment = 0
        self.current_phase = 'cv2'
        self.get_logger().info('First lidar segment finished; requesting second CV segment')
        self._publish_stop()
        self._request_cv_segment()

    def _final_stop(self, reason: str = 'radial threshold') -> None:
        if self.mission_complete:
            return
        self.ab_mode = False
        self.lidar_active = False
        self.mission_complete = True
        self.current_phase = 'stopped'
        self.get_logger().info('Final stop reached (%s, origin %.2f m)' % (reason, self.pose_norm or -1.0))
        self._publish_stop()

    def _publish_stop(self) -> None:
        stop = Twist()
        stop.linear.x = self.stop_linear_pwm
        stop.angular.z = self.stop_angular_deg
        self.cmd_pub.publish(stop)

    # ------------------------------------------------------------------
    # LiDAR control
    # ------------------------------------------------------------------

    def _scan_cb(self, scan: LaserScan) -> None:
        if self.mission_complete:
            return
        if self.ab_mode:
            self._publish_ab_motion()
            return
        if not self.lidar_active:
            return
        twist = self._compute_lidar_twist(scan)
        self.cmd_pub.publish(twist)

    def _compute_lidar_twist(self, scan: LaserScan) -> Twist:
        red_points, blue_points = self._extract_bucket_points(scan)
        twist = Twist()
        twist.linear.x = self.speed_pwm
        twist.angular.z = 90.0

        count = min(len(red_points), len(blue_points))
        if count == 0:
            return twist

        weighted_error = 0.0
        weight_sum = 0.0
        for idx in range(count):
            mirror = count - 1 - idx
            weight = (blue_points[idx][1] + red_points[mirror][1]) / 2.0
            weighted_error += (blue_points[idx][0] + red_points[mirror][0]) * weight
            weight_sum += weight
        if weight_sum > 0.0:
            error = (weighted_error / weight_sum) * 20.0
            if error > 0.0:
                error *= 0.9
            angle_offset = self.pid.positional(error)
            pwm_angle = 90.0 + angle_offset
            twist.angular.z = max(0.0, min(170.0, pwm_angle))
            if abs(angle_offset) >= 20.0:
                twist.linear.x = max(0.0, self.speed_pwm - 3.0)
        return twist

    def _extract_bucket_points(self, scan: LaserScan) -> Tuple[List[List[float]], List[List[float]]]:
        ranges = scan.ranges
        red_points: List[List[float]] = []
        blue_points: List[List[float]] = []
        max_index = max(1, len(ranges) - self.bucket_threshold)
        for i in range(1, max_index):
            current = ranges[i]
            if math.isinf(current) or math.isnan(current) or current <= 0.0 or current > 10.0:
                continue
            prev_val = ranges[i - 1]
            next_val = ranges[i + 1]
            if any(math.isnan(v) or math.isinf(v) for v in (prev_val, next_val)):
                continue
            if prev_val - current < 2.0 or prev_val - next_val < 2.0:
                continue

            continue_ranges = 0
            for idx in range(1, self.bucket_threshold):
                neighbor = ranges[i + idx]
                if math.isinf(neighbor) or math.isnan(neighbor):
                    continue
                if abs(current - neighbor) < 0.2:
                    continue_ranges += 1
                    if continue_ranges >= self.valid_bucket_threshold:
                        break
            if continue_ranges < self.valid_bucket_threshold:
                continue

            theta = scan.angle_min + float(i) * scan.angle_increment
            x = current * math.sin(theta)
            y = current * math.cos(theta)
            if x <= self.max_left_distance and x >= -self.min_right_distance and -0.2 < y < 2.0:
                bucket = red_points if x > 0.0 else blue_points
                if len(bucket) < self.max_bucket_points:
                    bucket.append([x, y])
        count = min(len(red_points), len(blue_points), 3)
        return red_points[:count], blue_points[:count]

    def _start_ab_mode(self, position) -> None:
        self.ab_mode = True
        self.ab_distance = 0.0
        self.ab_last_position = (position.x, position.y)
        self.lidar_active = False
        self.pending_lidar = None
        self.current_phase = 'ab_stop'
        self.get_logger().info('AB stop triggered (origin %.2f m)' % (self.pose_norm or -1.0))

    def _update_ab_distance(self, position) -> None:
        if self.ab_last_position is not None:
            step = math.hypot(position.x - self.ab_last_position[0], position.y - self.ab_last_position[1])
            if 0.0 < step < 5.0:
                self.ab_distance += step
        self.ab_last_position = (position.x, position.y)
        self._publish_ab_motion()
        if self.ab_distance >= self.ab_target_distance:
            self.get_logger().info('AB stop distance %.2f m satisfied' % self.ab_distance)
            self._final_stop('ab stop')

    def _publish_ab_motion(self) -> None:
        cmd = Twist()
        cmd.linear.x = self.ab_linear_pwm
        cmd.angular.z = self.ab_angular_deg
        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = CarTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
