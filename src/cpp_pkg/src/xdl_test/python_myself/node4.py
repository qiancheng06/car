#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import numpy as np


start_angle = 0
end_angle = 252
L_max_Angle = 63
R_max_Angle = 189
number_fitin = 2.8

lidar_forward = 1.22
error_range = 0.15
distance_range = [0.2, 1.8]
base_angle = 90.0

def python_angle_to_angular_z(python_angle):
    base_angle = 90.0
    max_angle_range = 45.0
    
    deviation = python_angle - base_angle
    normalized = deviation / max_angle_range
    normalized = max(min(normalized, 1.0), -1.0)
    return normalized


# ★ 驱动 legacy PWM 模式：throttle=linear.x∈[1500,1550]，steering=angular.z∈[30,145]（0~180度）
PWM_NEUTRAL = 1500.0     # 停车（中性）
PWM_SPEED   = 1545.0     # 直道速度（测试降速）
PWM_SLOW    = 1522.0     # 弯道减速
ANGLE_CENTER = 90.0      # 直行舵角
ANGLE_MIN   = 30.0
ANGLE_MAX   = 145.0

# 2026 B 赛道黄色折线区：每条斜边约 3.06m，中心线转角依次为左22.5°、
# 右45°、左22.5°。黄色胶带无法由激光雷达识别，因此按 IMU 转角和时间写死。
PWM_YELLOW = 1532.0
ANGLE_YELLOW_LEFT = 110.0
ANGLE_YELLOW_RIGHT = 70.0
YELLOW_ENTRY_DEG = 22.5
YELLOW_APEX_DEG = 45.0
YELLOW_EXIT_DEG = 22.5
YELLOW_LEG_SEC = 3.00
YELLOW_TURN_MAX_SEC = 2.00
YELLOW_REACQUIRE_SEC = 3.50
YELLOW_TRIGGER_HEADING_DEG = 145.0
YELLOW_REARM_HEADING_DEG = 35.0
YELLOW_ARM_FRAMES = 20
YELLOW_NO_PATH_FRAMES = 2
YELLOW_REACQUIRE_FRAMES = 3
YELLOW_EMERGENCY_DIST = 0.28
YELLOW_MAX_PASSES = 2

# 循迹舵角速率限制。黄色写死动作段不受此限制。
STEER_SLEW_PER_FRAME = 18.0


class BucketPoint:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


class ServoPID:
    def __init__(self):
        self.lidar_angle = 0.0
        self.P = 0.55
        self.D = 0.25
        self.I = 0.0
        self.angle_integral_error = 0.0
        self.integral_limit = 100.0
        self.angle_last_error = 0.0

    def servo_pid_control(self, new_angle_error):
        self.angle_integral_error += new_angle_error
        self.angle_integral_error = max(min(self.angle_integral_error, self.integral_limit), -self.integral_limit)
        output = self.P * new_angle_error + self.D * (new_angle_error - self.angle_last_error) + self.I * self.angle_integral_error
        output = max(min(output, 45.0), -45.0)
        self.angle_last_error = new_angle_error
        return float(output)


class LidarCoordinateControlNode(Node):
    def __init__(self):
        super().__init__('lidar_coordinate_control_node')
        self.servo = ServoPID()
        
        self.cmd_vel = Twist()
        #self.publisher_twist = self.create_publisher(Twist, 'car_cmd_vel', 10)
        self.publisher_twist = self.create_publisher(Twist, 'teleop_cmd_vel', 10)
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            100
        )

        # 远程小车 hipnuc_imu 驱动实际发布 /IMU_data。
        self.declare_parameter('imu_topic', '/IMU_data')
        self.declare_parameter('front_block_distance', 0.55)
        self.front_block_distance = float(self.get_parameter('front_block_distance').value)

        self.imu_yaw = None
        self.imu_last_ns = 0
        self.yellow_state = 'FOLLOW'
        self.yellow_state_start_ns = 0
        self.yellow_yaw_accumulated_rad = 0.0
        self.yellow_last_yaw = None
        self.yellow_pass_count = 0
        self.yellow_armed = True
        self.start_yaw_ref = None
        self.no_bucket_streak = 0
        self.valid_follow_streak = 0
        self.reacquire_streak = 0
        self.last_command_angle = ANGLE_CENTER
        self.imu_missing_reported = False

        imu_topic = str(self.get_parameter('imu_topic').value)
        self.subscription_imu = self.create_subscription(Imu, imu_topic, self.imu_callback, 50)

        self.default_linear_speed = 0.35

        # Ͱ�о��㷨�������� node2 ����һ�£�������ֲ���ԣ�
        self.bucket_threshold = 10            # �������ٸ�����һ��Ͱ
        self.valid_bucket_threshold = 6       # һ��Ͱ������Ҫ����Ч����
        self.lidar_filter_min_dist = 0.3      # ���˹������
        self.lidar_filter_max_dist = 2.5      # ��עǰ�� 2.5m ��
        self.roi_y_min = -0.2                 # ROI ���
        self.roi_y_max = 3.0                  # ROI ǰ��
        self.max_left_distance = 1.5          # ROI ���
        self.min_right_distance = 1.5         # ROI �ҽ�
        self.steer_gain = 60.0                # �������Ŵ���
        self.steer_threshold_slow_down = 20.0 # ��ת�Ǽ�����ֵ���ȣ�

        self.get_logger().info("B-track lidar follow + fixed yellow zigzag started")

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def imu_callback(self, data):
        q = data.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_yaw = yaw
        self.imu_last_ns = self.now_ns()

        if self.yellow_state in ('ENTRY_TURN', 'APEX_TURN', 'EXIT_TURN'):
            if self.yellow_last_yaw is not None:
                delta = self.normalize_angle(yaw - self.yellow_last_yaw)
                self.yellow_yaw_accumulated_rad += abs(delta)
        self.yellow_last_yaw = yaw

    def imu_is_fresh(self):
        return self.imu_yaw is not None and self.now_ns() - self.imu_last_ns < 500_000_000

    def stop_car(self):
        self.cmd_vel.linear.x = PWM_NEUTRAL
        self.cmd_vel.angular.z = ANGLE_CENTER
        self.last_command_angle = ANGLE_CENTER

    def heading_from_start_deg(self):
        if self.start_yaw_ref is None or self.imu_yaw is None:
            return 0.0
        delta = self.normalize_angle(self.imu_yaw - self.start_yaw_ref)
        return abs(math.degrees(delta))

    def set_yellow_phase(self, state):
        self.yellow_state = state
        self.yellow_state_start_ns = self.now_ns()
        self.yellow_yaw_accumulated_rad = 0.0
        self.yellow_last_yaw = self.imu_yaw
        self.get_logger().warn(f"Yellow zigzag phase: {state}")

    def maybe_begin_yellow(self, no_path, front_blocked):
        self.no_bucket_streak = self.no_bucket_streak + 1 if no_path else 0
        if (
            not self.yellow_armed
            or self.yellow_pass_count >= YELLOW_MAX_PASSES
            or self.valid_follow_streak < YELLOW_ARM_FRAMES
            or self.no_bucket_streak < YELLOW_NO_PATH_FRAMES
            or front_blocked
            or not self.imu_is_fresh()
            or self.start_yaw_ref is None
        ):
            return False

        heading_deg = self.heading_from_start_deg()
        if heading_deg < YELLOW_TRIGGER_HEADING_DEG:
            return False

        self.yellow_armed = False
        self.reacquire_streak = 0
        self.set_yellow_phase('ENTRY_TURN')
        self.get_logger().warn(
            f"Yellow zigzag {self.yellow_pass_count + 1}/{YELLOW_MAX_PASSES} "
            f"triggered: heading={heading_deg:.1f} deg"
        )
        return True

    def run_yellow(self, red_count, blue_count, emergency_blocked):
        if self.yellow_state == 'FAULT_STOP':
            self.stop_car()
            return

        if not self.imu_is_fresh():
            self.stop_car()
            if not self.imu_missing_reported:
                self.get_logger().error("Yellow zigzag stopped: IMU unavailable/stale")
                self.imu_missing_reported = True
            return
        self.imu_missing_reported = False

        if emergency_blocked:
            self.stop_car()
            return

        now = self.now_ns()
        elapsed = (now - self.yellow_state_start_ns) / 1e9
        turn_config = {
            'ENTRY_TURN': (YELLOW_ENTRY_DEG, ANGLE_YELLOW_LEFT, 'LEG1'),
            'APEX_TURN': (YELLOW_APEX_DEG, ANGLE_YELLOW_RIGHT, 'LEG2'),
            'EXIT_TURN': (YELLOW_EXIT_DEG, ANGLE_YELLOW_LEFT, 'REACQUIRE'),
        }

        if self.yellow_state in turn_config:
            target_deg, steering, next_state = turn_config[self.yellow_state]
            progress_deg = math.degrees(self.yellow_yaw_accumulated_rad)
            self.cmd_vel.linear.x = PWM_YELLOW
            self.cmd_vel.angular.z = steering
            self.last_command_angle = steering
            if progress_deg >= target_deg - 2.0 or elapsed >= YELLOW_TURN_MAX_SEC:
                if elapsed >= YELLOW_TURN_MAX_SEC and progress_deg < target_deg - 2.0:
                    self.get_logger().warn(
                        f"{self.yellow_state} timeout: "
                        f"{progress_deg:.1f}/{target_deg:.1f} deg"
                    )
                self.set_yellow_phase(next_state)
            return

        if self.yellow_state in ('LEG1', 'LEG2'):
            self.cmd_vel.linear.x = PWM_YELLOW
            self.cmd_vel.angular.z = ANGLE_CENTER
            self.last_command_angle = ANGLE_CENTER
            if elapsed >= YELLOW_LEG_SEC:
                next_state = (
                    'APEX_TURN' if self.yellow_state == 'LEG1' else 'EXIT_TURN'
                )
                self.set_yellow_phase(next_state)
            return

        if self.yellow_state == 'REACQUIRE':
            corridor_valid = red_count >= 2 and blue_count >= 2
            self.reacquire_streak = self.reacquire_streak + 1 if corridor_valid else 0
            self.cmd_vel.linear.x = PWM_SLOW
            self.cmd_vel.angular.z = ANGLE_CENTER
            self.last_command_angle = ANGLE_CENTER

            if self.reacquire_streak >= YELLOW_REACQUIRE_FRAMES:
                self.yellow_pass_count += 1
                self.yellow_state = 'FOLLOW'
                self.valid_follow_streak = 0
                self.no_bucket_streak = 0
                self.reacquire_streak = 0
                self.servo.angle_integral_error = 0.0
                self.servo.angle_last_error = 0.0
                self.get_logger().info(
                    f"Yellow zigzag complete "
                    f"({self.yellow_pass_count}/{YELLOW_MAX_PASSES}); "
                    "radar corridor reacquired"
                )
                return

            if elapsed >= YELLOW_REACQUIRE_SEC:
                self.yellow_state = 'FAULT_STOP'
                self.stop_car()
                self.get_logger().error(
                    "Yellow zigzag reacquire timeout; latched stop"
                )

    def lidar_callback(self, data):
        self.lidar_deal(data)
        self.publisher_twist.publish(self.cmd_vel)

    def lidar_deal(self, data):
        ranges = data.ranges
        scan_resolution = len(ranges)
        angle_increment = data.angle_increment
        angle_min = data.angle_min

        if scan_resolution == 0 or all(math.isnan(r) or math.isinf(r) for r in ranges):
            self.get_logger().warn("Radar ranges is empty or invalid! Force stop temporarily.")
            self.stop_car()
            return

        red_points = []
        blue_points = []
        front_clearance = float('inf')

        for i in range(1, scan_resolution - self.bucket_threshold):
            current = ranges[i]

            if math.isnan(current) or math.isinf(current):
                continue
            if current <= 0.0 or current > self.lidar_filter_max_dist:
                continue

            # 安全距离直接使用原始有效点，不能要求该点先通过锥桶形状检测。
            theta = angle_min + i * angle_increment
            raw_x = current * math.sin(theta)
            raw_y = current * math.cos(theta)
            if abs(raw_x) < 0.35 and 0.0 < raw_y < front_clearance:
                front_clearance = raw_y

            if current < self.lidar_filter_min_dist:
                continue

            if ranges[i-1] - current < 2.0:
                continue
            if ranges[i-1] - ranges[i+1] < 2.0:
                continue

            continue_ranges = 0
            for idx in range(1, self.bucket_threshold):
                if i + idx < scan_resolution:
                    if abs(current - ranges[i + idx]) < 0.2:
                        continue_ranges += 1
                        if continue_ranges >= self.valid_bucket_threshold:
                            break

            if continue_ranges < self.valid_bucket_threshold:
                continue

            x = current * math.sin(theta)
            y = current * math.cos(theta)

            if (x <= self.max_left_distance and x >= -self.min_right_distance and
                self.roi_y_min < y < self.roi_y_max):
                pt = BucketPoint(x, y)
                if x > 0.0:
                    red_points.append(pt)
                else:
                    blue_points.append(pt)

        red_points, blue_points = self.adjust_bucket_counts(red_points, len(red_points), blue_points, len(blue_points))

        red_count = len(red_points)
        blue_count = len(blue_points)
        error_count = min(red_count, blue_count)
        front_blocked = front_clearance < self.front_block_distance
        no_path = error_count == 0
        emergency_blocked = front_clearance < YELLOW_EMERGENCY_DIST

        if self.yellow_state != 'FOLLOW':
            self.run_yellow(red_count, blue_count, emergency_blocked)
            return

        if error_count >= 2:
            self.valid_follow_streak += 1
            self.no_bucket_streak = 0
            if (
                self.start_yaw_ref is None
                and self.valid_follow_streak >= YELLOW_ARM_FRAMES
                and self.imu_is_fresh()
            ):
                self.start_yaw_ref = self.imu_yaw
                self.get_logger().info("B-track start heading captured")

            if (
                not self.yellow_armed
                and self.yellow_pass_count < YELLOW_MAX_PASSES
                and self.heading_from_start_deg() <= YELLOW_REARM_HEADING_DEG
            ):
                self.yellow_armed = True
                self.get_logger().info("Yellow zigzag rearmed for next lap")

        if self.maybe_begin_yellow(no_path, front_blocked):
            self.run_yellow(red_count, blue_count, emergency_blocked)
            return

        angle_output = ANGLE_CENTER
        speed_output = PWM_SPEED

        if error_count > 0:
            weighted_error = 0.0
            weight_sum = 0.0

            for idx in range(error_count):
                b_pt = blue_points[idx]
                r_pt = red_points[error_count - 1 - idx]

                weight = (b_pt.y + r_pt.y) / 2.0
                weighted_error += (b_pt.x + r_pt.x) * weight
                weight_sum += weight

            error = (weighted_error / weight_sum) * self.steer_gain if weight_sum > 0.0 else 0.0
            angle_pid = self.servo.servo_pid_control(error)
            final_angle = ANGLE_CENTER + angle_pid

            if abs(angle_pid) >= self.steer_threshold_slow_down:
                speed_output = PWM_SLOW

            # ★ 驱动 legacy PWM：直接发布 0~180 度舵角
            final_angle = min(max(final_angle, ANGLE_MIN), ANGLE_MAX)
            angle_output = final_angle
        else:
            self.stop_car()
            reason = "FRONT_BLOCKED" if front_blocked else "NO_PATH"
            self.get_logger().info(f"NO BUCKET: {reason}")
            return

        # v2.3: 舵角速率限制——相对上一帧输出限幅，抑制稀疏段质心跳变
        # 引发的 45↔135° 病理性甩舵震荡。特殊弯动作段不经过此处。
        angle_output = max(min(angle_output,
            self.last_command_angle + STEER_SLEW_PER_FRAME),
            self.last_command_angle - STEER_SLEW_PER_FRAME)

        self.cmd_vel.linear.x = speed_output
        self.cmd_vel.angular.z = angle_output
        self.last_command_angle = angle_output
        self.get_logger().info(
            f"Bucket: Linear {self.cmd_vel.linear.x:.2f}, Angular_z {self.cmd_vel.angular.z:.2f}"
        )

    def adjust_bucket_counts(self, red_points, red_count, blue_points, blue_count):
        if blue_count >= red_count:
            split_index = 100
            for i in range(1, blue_count):
                distance_squared = (blue_points[i-1].x - blue_points[i].x) ** 2 + (
                    blue_points[i-1].y - blue_points[i].y
                ) ** 2
                if distance_squared > 1.4 * 1.4:
                    split_index = i
                    break

            if split_index != 100:
                points_to_move = blue_points[split_index:]
                blue_points = blue_points[:split_index]
                red_points = points_to_move + red_points

                red_count = len(red_points)
                blue_count = len(blue_points)
        else:
            split_index = 100
            for i in range(red_count - 2, -1, -1):
                distance_squared = (red_points[i].x - red_points[i+1].x) ** 2 + (
                    red_points[i].y - red_points[i+1].y
                ) ** 2
                if distance_squared > 1.4 * 1.4:
                    split_index = i
                    break

            if split_index != 100:
                points_to_move = red_points[: split_index + 1]
                red_points = red_points[split_index + 1 :]
                blue_points = blue_points + points_to_move

                red_count = len(red_points)
                blue_count = len(blue_points)

        red_points = red_points[:3]
        blue_points = blue_points[:3]
        red_count = len(red_points)
        blue_count = len(blue_points)

        if red_count > blue_count and blue_count > 0:
            if blue_count == 1:
                if red_count > 2:
                    red_points = red_points[1:3]
                    red_count = 2
                blue_points.append(blue_points[0])
                blue_count = 2
            elif blue_count == 2:
                blue_points.append(blue_points[1])
                blue_count = 3
        elif blue_count > red_count and red_count > 0:
            if red_count == 1:
                blue_count = 2
                red_points.append(red_points[0])
                red_count = 2
            elif red_count == 2:
                red_points = [red_points[0]] + red_points
                red_count = 3

        return red_points, blue_points


def main(args=None):
    rclpy.init(args=args)
    node = LidarCoordinateControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_cmd = Twist()
        stop_cmd.linear.x = PWM_NEUTRAL
        stop_cmd.angular.z = ANGLE_CENTER
        node.publisher_twist.publish(stop_cmd)
        node.get_logger().info("Node stopped by user (KeyboardInterrupt) - Robot forced stop.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
