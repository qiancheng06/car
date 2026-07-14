#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 
import math
import numpy as np
from nav_msgs.msg import Odometry 

# 激光雷达基础参数
start_angle = 0
end_angle = 252
L_max_Angle = 63
R_max_Angle = 189
number_fitin = 2.8
lidar_forward = 1.22
error_range = 0.15
distance_range = [0.2, 1.8]
base_angle = 90.0  # 基准角度（对应归一化后0）

# 距离触发阈值配置（单位：米）
DISTANCE_THRESHOLDS = {
    "first": 2.8,        # 第一模式触发阈值
    #"second": 6.0,       # 第二模式触发阈值
    "second": 7.8,       # 第二模式触发阈值
    #"third": 15.0,       # 第三模式触发阈值
    "third": 21.0,       # 第三模式触发阈值
    #"fourth": 17.0,      # 第四模式触发阈值
    "fourth": 22.0,      # 第四模式触发阈值
    #"fifth": 18.5,       # 第五模式触发阈值
    "fifth": 23.7,       # 第五模式触发阈值
    #"sixth": 20.0,       # 第六模式触发阈值
    "sixth": 25.5,       # 第六模式触发阈值
    #"lap_total": 35.0,   # 单圈总距离（判断第二圈）
    "lap_total": 35.23,   # 单圈总距离（判断第二圈）
    "traffic_light": 27.3, # 红绿灯距离触发阈值
    "ab_stop": 32.0      # 第二圈AB停车触发距离（可根据实际调整）
}

# 角度→归一化角速度（[-1,1]）转换
def python_angle_to_angular_z(python_angle):
    max_angle_range = 45.0
    deviation = python_angle - base_angle
    normalized = deviation / max_angle_range
    return max(min(normalized, 1.0), -1.0)

class ServoPID:
    def __init__(self):
        self.lidar_angle = 0.0  
        self.P = 0.36         
        self.D = 0.1       
        self.I = 0.0
        self.angle_integral_error = 0.0  # PID积分项
        self.integral_limit = 100.0      # 积分限幅
        self.angle_last_error = 0.0  
        self.first_circle_end_flag = 0
        self.walk_distance = 0.0  # 累计行走距离（核心切换依据）
        self.ab_mode_distance = 0.0  # AB模式下已行驶距离

    def servo_pid_control(self, new_angle_error):
        self.angle_integral_error += new_angle_error
        self.angle_integral_error = max(min(self.angle_integral_error, self.integral_limit), -self.integral_limit)
        output = self.P * new_angle_error + self.D * (new_angle_error - self.angle_last_error) + self.I * self.angle_integral_error
        output = max(min(output, 45.0), -45.0)
        self.angle_last_error = new_angle_error
        return float(output)

class LidarCoordinateControlNode(Node):
    def __init__(self):
        super().__init__('lidar_distance_control_node')
        self.servo = ServoPID()
        
        self.cmd_vel = Twist()
        self.publisher_twist = self.create_publisher(Twist, 'car_cmd_vel', 10)
        # 激光雷达数据订阅
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 100)
        # 里程计订阅（用于计算累计距离和AB模式距离）
        self.subscription_odom = self.create_subscription(Odometry, '/odom_combined', self.odometry_callback, 10)
        
        # 速度参数配置
        self.default_linear_speed = 0.30
        self.special_linear_speed1 = 0.30
        self.special_angular_speed1 = 95.1
        self.special_angular_speed3 = 96.3
        self.special_linear_speed5 = 0.30
        self.special_angular_speed5 = 98.5
        self.special_linear_speed4 = 0.30
        self.special_linear_speed7 = -3.0
        self.special_angular_speed7 = 90.0
        self.special_angular_speed8 = 120.0
        # AB模式参数（新增）
        self.ab_linear_speed = 0.2  # AB模式行驶速度
        #self.ab_fixed_angle = 100.0  # AB模式固定角度（可根据需求调整）
        self.ab_fixed_angle = 100.0  # AB模式固定角度（可根据需求调整）
        self.ab_target_distance = 0.5  # AB模式需行驶的距离（1米）
        #self.ab_target_distance = 0.8  # AB模式需行驶的距离（1米）
        
        # 模式状态标记
        self.reached_first = False
        self.reached_second = False
        self.reached_third = False
        self.reached_fourth = False
        self.reached_fifth = False
        self.reached_sixth = False
        self.reached_stop = False
        self.reached_traffic_light = False
        self.first_lap_completed = False
        self.traffic_light_passed = False
        self.in_ab_mode = False  # 是否处于AB模式
        self.ab_mode_triggered = False  # AB模式是否已触发过
        
        # 时间戳记录（用于距离计算）
        self.last_odom_timestamp = None
        # 红绿灯停车定时器
        self.stop_timer = None
        self.stop_start_time = None
        self.stop_duration = 3.0  # 停车时长（3秒）

        self.get_logger().info("激光雷达距离控制节点已启动（距离触发模式）")
    
    def log_distance_state(self, name, threshold, current_distance):
        self.get_logger().info(
            f"到达{name}：阈值 {threshold:.2f} 米 | 当前距离 {current_distance:.2f} 米"
        )

    def odometry_callback(self, msg):
        current_timestamp = self.get_clock().now().nanoseconds / 1e9  
        linear_speed = abs(msg.twist.twist.linear.x)
        
        # 计算累计行走距离
        if self.last_odom_timestamp is not None:
            dt = current_timestamp - self.last_odom_timestamp  
            if dt > 0:
                self.servo.walk_distance += linear_speed * dt
                # 若处于AB模式，单独累计该模式下的行驶距离
                if self.in_ab_mode:
                    self.servo.ab_mode_distance += linear_speed * dt
        
        self.last_odom_timestamp = current_timestamp
        # 日志：累计距离与圈数
        self.get_logger().info(
            f"累计里程：{self.servo.walk_distance:.2f} 米 | "
            f"第一圈完成：{self.first_lap_completed} | "
            f"AB 模式：{self.in_ab_mode}（已行驶 {self.servo.ab_mode_distance:.2f} 米）"
        )
    
    def lidar_callback(self, data):
        self.lidar_deal(data)
        self.publisher_twist.publish(self.cmd_vel)

    # 重置模式状态（第一圈结束后）
    def reset_mode_states(self):
        self.reached_first = False
        self.reached_second = False
        self.reached_third = False
        self.reached_fourth = False
        self.reached_fifth = False  
        self.reached_sixth = False
        self.reached_traffic_light = False
        self.traffic_light_passed = False
        # 重置AB模式状态（进入第二圈时）
        self.in_ab_mode = False
        self.ab_mode_triggered = False
        self.servo.ab_mode_distance = 0.0

    # 启动红绿灯停车（非阻塞）
    def start_traffic_light_stop(self):
        if self.stop_timer is None:
            self.stop_start_time = self.get_clock().now().nanoseconds / 1e9
            self.stop_timer = self.create_timer(0.1, self.traffic_light_stop_callback)
            self.get_logger().info("Traffic light stop started (3s)")

    # 红绿灯停车回调
    def traffic_light_stop_callback(self):
        self.cmd_vel.linear.x = -3.0
        self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
        
        # 达到停车时长后恢复
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.stop_start_time >= self.stop_duration:
            self.stop_timer.cancel()
            self.stop_timer = None
            self.reached_traffic_light = False
            self.traffic_light_passed = True
            self.servo.angle_integral_error = -3.0
            self.get_logger().info("Traffic light stop finished - Resuming movement")

    # 距离触发逻辑判断
    def check_distance_trigger(self):
        current_dist = self.servo.walk_distance
        lap_total = DISTANCE_THRESHOLDS["lap_total"]

        # 判断第一圈完成
        if not self.first_lap_completed and current_dist >= lap_total:
            self.first_lap_completed = True
            self.servo.walk_distance -= lap_total
            current_dist = self.servo.walk_distance  # 使用清零后的距离参与后续判断
            self.reset_mode_states()
            self.get_logger().info(
                f"第一圈完成（距离清零后：{self.servo.walk_distance:.2f} 米）"
            )

        # 模式触发（按距离递增）
        if not self.reached_first and current_dist >= DISTANCE_THRESHOLDS["first"]:
            self.reached_first = True
            self.log_distance_state("第一段", DISTANCE_THRESHOLDS["first"], current_dist)
        if not self.reached_second and current_dist >= DISTANCE_THRESHOLDS["second"]:
            self.reached_second = True
            self.log_distance_state("第二段", DISTANCE_THRESHOLDS["second"], current_dist)
        if not self.reached_third and current_dist >= DISTANCE_THRESHOLDS["third"]:
            self.reached_third = True
            self.log_distance_state("第三段", DISTANCE_THRESHOLDS["third"], current_dist)
        if not self.reached_fourth and current_dist >= DISTANCE_THRESHOLDS["fourth"]:
            self.reached_fourth = True
            self.log_distance_state("第四段", DISTANCE_THRESHOLDS["fourth"], current_dist)
        if not self.reached_fifth and current_dist >= DISTANCE_THRESHOLDS["fifth"]:
            self.reached_fifth = True
            self.log_distance_state("第五段", DISTANCE_THRESHOLDS["fifth"], current_dist)
        if not self.reached_sixth and current_dist >= DISTANCE_THRESHOLDS["sixth"]:
            self.reached_sixth = True
            self.log_distance_state("第六段", DISTANCE_THRESHOLDS["sixth"], current_dist)

        # 红绿灯距离触发
        traffic_light_dist = DISTANCE_THRESHOLDS["traffic_light"]
        if (not self.reached_traffic_light and not self.traffic_light_passed and 
            current_dist >= traffic_light_dist and 
            current_dist < traffic_light_dist + 5.0):
            self.reached_traffic_light = True
            self.start_traffic_light_stop()
            self.get_logger().info(f"到达红绿灯区域（距离：{current_dist:.2f} 米），开始定时停车")

        # 第二圈AB模式触发（新增逻辑）
        ab_stop_dist = DISTANCE_THRESHOLDS["ab_stop"]
        if (self.first_lap_completed and  # 仅在第二圈触发
            not self.ab_mode_triggered and  # 未触发过
            not self.in_ab_mode and  # 不在AB模式中
            current_dist >= ab_stop_dist):  # 达到AB触发距离
            self.in_ab_mode = True
            self.ab_mode_triggered = True
            self.servo.ab_mode_distance = 0.0  # 重置AB模式内行驶距离
            self.get_logger().info(f"第二圈 AB 模式触发（距离：{current_dist:.2f} 米）")

    def lidar_deal(self, data):
        # 最终停车状态保持
        if self.reached_stop:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed8)
            return
        
        # AB模式处理（新增）：固定角度行驶1米后退出
        if self.in_ab_mode:
            # 检查是否已行驶够1米
            if self.servo.ab_mode_distance >= self.ab_target_distance:
                self.in_ab_mode = False
                self.get_logger().info(f"AB 模式完成（模式内里程：{self.servo.ab_mode_distance:.2f} 米）")
                
                self.reached_stop = True
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
                return
            else:
                # 以固定角度和速度行驶
                self.cmd_vel.linear.x = self.ab_linear_speed
                self.cmd_vel.angular.z = python_angle_to_angular_z(self.ab_fixed_angle)
                self.get_logger().info(
                    f"AB 模式：线速度 {self.cmd_vel.linear.x:.2f} m/s，"
                    f"固定角度 {self.ab_fixed_angle}°，"
                    f"进度 {self.servo.ab_mode_distance:.2f}/{self.ab_target_distance:.2f} 米"
                )
                return  # 直接返回，不执行其他模式逻辑
        
        ranges = data.ranges
        ranges_len = len(ranges)

        # 激光雷达数据无效时强制停车
        if ranges_len == 0 or all(math.isnan(r) or math.isinf(r) for r in ranges):
            self.get_logger().warn("激光雷达数据无效，强制停车")
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
            return

        # 查找左右边缘点
        L_points = self.lidar_find_point(ranges, ranges_len, start_angle, L_max_Angle, 'L')
        R_points = self.lidar_find_point(ranges, ranges_len, R_max_Angle, end_angle, 'R')
        
        # 过滤异常边缘点
        if L_points and R_points:
            i = 0
            while i < len(R_points):
                if abs(L_points[0][0] - R_points[i][0] - 360) < 35:
                    R_points.pop(i)
                else:
                    i += 1

        # 处理左边缘
        if L_points:
            min_y = L_points[0][1]
            i = 0
            while i < len(L_points):
                if L_points[i][1] <= min_y:
                    min_y = L_points[i][1]
                elif len(L_points) > 1:
                    L_points.pop(i)
                    continue
                i += 1
            if L_points:
                _, Left_edge, _ = self.lidar_find_2edge(L_points)
            else:
                Left_edge = [L_max_Angle, lidar_forward]
        else:
            Left_edge = [L_max_Angle, lidar_forward]

        # 处理右边缘
        if R_points:
            min_y = R_points[0][1]
            i = 0
            while i < len(R_points):
                if R_points[i][1] <= min_y:
                    min_y = R_points[i][1]
                elif len(R_points) > 1:
                    R_points.pop(i)
                    continue
                i += 1
            if R_points:
                _, Right_edge, _ = self.lidar_find_2edge(R_points)
            else:
                Right_edge = [R_max_Angle, lidar_forward]
        else:
            Right_edge = [R_max_Angle, lidar_forward]

        # 计算PID控制角度与归一化角速度
        mid_point = int((Left_edge[0] + (Right_edge[0] - 360)))
        self.servo.lidar_angle = mid_point
        control_angle = self.servo.servo_pid_control(mid_point) + base_angle
        normalized_angular_z = python_angle_to_angular_z(control_angle)
        self.get_logger().debug(
            f"中间角度：{mid_point}，控制角：{control_angle:.1f}°，归一化角速度：{normalized_angular_z:.2f}"
        )

        # 检查距离触发条件
        self.check_distance_trigger()

        # 速度模式切换
        if self.reached_traffic_light:
            pass  # 停车逻辑由定时器回调处理
        elif self.reached_sixth:
            self.cmd_vel.linear.x = self.default_linear_speed
            self.cmd_vel.angular.z = normalized_angular_z
            if abs(self.cmd_vel.angular.z - python_angle_to_angular_z(base_angle)) < 0.05:
                self.cmd_vel.linear.x += 0.03
            self.get_logger().info(f"Sixth Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        elif self.reached_fifth and not self.reached_sixth:
            self.cmd_vel.linear.x = self.special_linear_speed5
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed5)
            self.get_logger().info(f"Fifth Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        elif self.reached_fourth and not self.reached_fifth:
            self.cmd_vel.linear.x = self.special_linear_speed4
            self.cmd_vel.angular.z = normalized_angular_z
            self.get_logger().info(f"Fourth Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        elif self.reached_third and not self.reached_fourth:
            self.cmd_vel.linear.x = self.default_linear_speed
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed3)
            self.get_logger().info(f"Third Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        elif self.reached_second and not self.reached_third:
            self.cmd_vel.linear.x = self.default_linear_speed
            self.cmd_vel.angular.z = normalized_angular_z
            if abs(self.cmd_vel.angular.z - python_angle_to_angular_z(base_angle)) < 0.05:
                self.cmd_vel.linear.x += 0.03
            self.get_logger().info(f"Second Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        elif self.reached_first and not self.reached_second:
            self.cmd_vel.linear.x = self.special_linear_speed1
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed1)
            self.get_logger().info(f"First Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        else:
            self.cmd_vel.linear.x = self.default_linear_speed
            self.cmd_vel.angular.z = normalized_angular_z
            self.get_logger().info(f"Default Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")

    # 边缘计算
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
                    self.get_logger().debug(f"Edge Calculation: k={k:.2f}, b={b:.2f}")
                    edge_angle = int(lidar_forward / k - b / k)
                    return points[i-1], [edge_angle, lidar_forward], points[i]
        
        return points[0], points[0], points[0]

    # 边缘点查找
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
                        while (rgi <= max_rgi and abs(ranges[rgi_true] - ranges[(rgi_true + 1) % ranges_len]) < error_range):
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
                        while (rgi >= min_rgi and abs(ranges[rgi_true] - ranges[(rgi_true - 1) % ranges_len]) < error_range):
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
        # 手动中断时强制停车
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = python_angle_to_angular_z(base_angle)
        node.publisher_twist.publish(stop_cmd)
        node.get_logger().info("Node stopped by user - Robot forced stop")
    finally:
        if node.stop_timer is not None:
            node.stop_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()