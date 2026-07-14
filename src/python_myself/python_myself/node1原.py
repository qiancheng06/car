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


class ServoPID:
    def __init__(self):
        self.lidar_angle = 0.0
        self.P = 0.3
        self.D = 0.1
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
        
        self.default_linear_speed = 0.20
        
        self.get_logger().info("Lidar coordinate control node started (Angle Normalized Angular_z)")
    
    def lidar_callback(self, data):
        self.lidar_deal(data)
        self.publisher_twist.publish(self.cmd_vel)

    def lidar_deal(self, data):
        ranges = data.ranges
        ranges_len = len(ranges)
        Left_edge = []
        Right_edge = []

        if ranges_len == 0:
            self.get_logger().warn("Radar ranges is empty! Force stop temporarily.")
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
            return

        L_points = self.lidar_find_point(ranges, ranges_len, start_angle, L_max_Angle, 'L')
        R_points = self.lidar_find_point(ranges, ranges_len, R_max_Angle, end_angle, 'R')
        
        if L_points and R_points:
            i = 0
            while i < len(R_points):
                if abs(L_points[0][0] - R_points[i][0] - 360) < 35:
                    R_points.pop(i)
                else:
                    i += 1

        if L_points:
            min_y = L_points[0][1]
            i = 0
            while i < len(L_points):
                if L_points[i][1] <= min_y:
                    min_y = L_points[i][1]
                elif len(Left_edge) > 1:
                    L_points.pop(i)
                    continue
                i += 1
            if L_points:
                Lower, Left_edge, Higher = self.lidar_find_2edge(L_points)
            else:
                Left_edge = [L_max_Angle, lidar_forward]
            self.get_logger().debug(f"Left edge: {Lower}, {Left_edge}, {Higher}")
        else:
            Left_edge = [L_max_Angle, lidar_forward]

        if R_points:
            min_y = R_points[0][1]
            i = 0
            while i < len(R_points):
                if R_points[i][1] <= min_y:
                    min_y = R_points[i][1]
                elif len(Left_edge) > 1:
                    R_points.pop(i)
                    continue
                i += 1
            if R_points:
                Lower, Right_edge, Higher = self.lidar_find_2edge(R_points)
            else:
                Right_edge = [R_max_Angle, lidar_forward]
            self.get_logger().debug(f"Right edge: {Lower}, {Right_edge}, {Higher}")
        else:
            Right_edge = [R_max_Angle, lidar_forward]

        mid_point = int((Left_edge[0] + (Right_edge[0] - 360)))
        self.servo.lidar_angle = mid_point
        control_angle = self.servo.servo_pid_control(mid_point) + base_angle
        normalized_angular_z = python_angle_to_angular_z(control_angle)
        self.get_logger().debug(f"Raw angle: {mid_point}, Control angle: {control_angle:.1f}, Normalized angular_z: {normalized_angular_z:.2f}")

        self.cmd_vel.linear.x = self.default_linear_speed
        self.cmd_vel.angular.z = normalized_angular_z
        self.get_logger().info(f"Linear: {self.cmd_vel.linear.x:.2f}, Angular_z: {self.cmd_vel.angular.z:.2f}")

    def lidar_find_2edge(self, points):
        if points[0][1] < lidar_forward or len(points) == 1:
            return points[0], points[0], points[0]
        else:
            for i in range(1, len(points)):
                if points[i][1] < lidar_forward:
                    denominator = (points[i-1][0] - points[i][0])
                    if denominator == 0:
                        return points[i-1], points[i-1], points[i]
                    
                    k = (points[i-1][1] - points[i][1]) / denominator
                    b = points[i-1][1] - k * points[i-1][0]
                    self.get_logger().debug(f"Edge k: {k:.2f}, b: {b:.2f}")
                    edge_angle = int(lidar_forward / k - b / k)
                    return points[i-1], [edge_angle, lidar_forward], points[i]
        
        return points[0], points[0], points[0]

    def lidar_find_point(self, ranges, ranges_len, start, end, lor_r):
        points = []
        if lor_r == 'L':
            max_rgi = min(63 * 4 - 1, ranges_len - 1)
            rgi = 0
            while rgi <= max_rgi:
                rgi_true = rgi % ranges_len
                if not (math.isnan(ranges[rgi_true]) or math.isinf(ranges[rgi_true])):
                    if distance_range[0] < ranges[rgi_true] < distance_range[1]:
                        start_i = rgi
                        while (rgi <= max_rgi and 
                               abs(ranges[rgi_true] - ranges[(rgi_true + 1) % ranges_len]) < error_range):
                            rgi_true = (rgi_true + 1) % ranges_len
                            rgi += 1
                        if abs(rgi - start_i) > number_fitin:
                            points.append([int(rgi / 2.8), ranges[rgi_true]])
                        rgi += 1
                        continue
                rgi += 1

        elif lor_r == 'R':
            min_rgi = max(189 * 4 + 1, 0)
            max_rgi = min(251 * 4 - 1, ranges_len - 1)
            rgi = max_rgi
            while rgi >= min_rgi:
                rgi_true = rgi % ranges_len
                if not (math.isnan(ranges[rgi_true]) or math.isinf(ranges[rgi_true])):
                    if distance_range[0] < ranges[rgi_true] < distance_range[1]:
                        start_i = rgi
                        while (rgi >= min_rgi and 
                               abs(ranges[rgi_true] - ranges[(rgi_true - 1) % ranges_len]) < error_range):
                            rgi_true = (rgi_true - 1) % ranges_len
                            rgi -= 1
                        if abs(rgi - start_i) > number_fitin:
                            points.append([int(rgi / 2.8), ranges[rgi_true]])
                        rgi -= 1
                        continue
                rgi -= 1

        return points


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