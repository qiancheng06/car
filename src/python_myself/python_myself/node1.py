#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
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


class BucketPoint:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


class ServoPID:
    def __init__(self):
        self.lidar_angle = 0.0
        self.P = 0.6
        self.D = 0.12
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
        self.publisher_twist = self.create_publisher(Twist, 'car_cmd_vel', 10)
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            100
        )

        self.default_linear_speed = 0.4

        # 桶判据算法参数（与 node2 保持一致，便于移植调试）
        self.bucket_threshold = 10            # 连续多少个点算一个桶
        self.valid_bucket_threshold = 6       # 一个桶最少需要的有效点数
        self.lidar_filter_min_dist = 0.3      # 过滤过近噪点
        self.lidar_filter_max_dist = 2.5      # 关注前方 2.5m 内
        self.roi_y_min = -0.2                 # ROI 后界
        self.roi_y_max = 3.0                  # ROI 前界
        self.max_left_distance = 1.5          # ROI 左界
        self.min_right_distance = 1.5         # ROI 右界
        self.steer_gain = 20.0                # 横向误差放大倍数
        self.steer_threshold_slow_down = 20.0 # 大转角减速阈值（度）

        self.get_logger().info("Lidar coordinate control node started (Bucket algorithm, pure lidar)")
    
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
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
            return

        red_points = []
        blue_points = []

        for i in range(1, scan_resolution - self.bucket_threshold):
            current = ranges[i]

            if math.isnan(current) or math.isinf(current):
                continue
            if current <= 0.0 or current > self.lidar_filter_max_dist:
                continue
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

            theta = angle_min + i * angle_increment
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

        angle_output = python_angle_to_angular_z(base_angle)
        speed_output = self.default_linear_speed

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
            final_angle = 90.0 + angle_pid

            if abs(angle_pid) >= self.steer_threshold_slow_down:
                speed_output = max(speed_output - 0.05, 0.1)

            final_angle = min(max(final_angle, 10.0), 170.0)
            angle_output = python_angle_to_angular_z(final_angle)
        else:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
            self.get_logger().info("NO BUCKET")
            return

        self.cmd_vel.linear.x = speed_output
        self.cmd_vel.angular.z = angle_output
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
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = python_angle_to_angular_z(base_angle)
        node.publisher_twist.publish(stop_cmd)
        node.get_logger().info("Node stopped by user (KeyboardInterrupt) - Robot forced stop.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()