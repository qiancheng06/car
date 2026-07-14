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
    "first": 2.5,        # 第一模式触发阈值 
    #"second": 6.0,       # 第二模式触发阈值
    "second": 8.5,       # 第二模式触发阈值
    #"third": 15.0,       # 第三模式触发阈值
    "third": 21.0,       # 第三模式触发阈值
    #"fourth": 17.0,      # 第四模式触发阈值
    "fourth": 22.0,      # 第四模式触发阈值
    "fifth": 24.7,       # 第五模式触发阈值  35速度
    #"fifth": 24.8,       # 第五模式触发阈值 30速度
    #"sixth": 20.0,       # 第六模式触发阈值
    #"sixth": 25.5,       # 第六模式触发阈值 30速度
    "sixth": 26.0,       # 第六模式触发阈值  35速度
    #"lap_total": 35.0,   # 单圈总距离（判断第二圈）
    "lap_total": 35.8,   # 单圈总距离（判断第二圈）
    "traffic_light": 27.0, # 红绿灯距离触发阈值
    "ab_stop": 32.8      # 第二圈AB停车触发距离（可根据实际调整）
}

# 角度→归一化角速度（[-1,1]）转换
def python_angle_to_angular_z(python_angle):
    max_angle_range = 45.0
    deviation = python_angle - base_angle
    normalized = deviation / max_angle_range
    return max(min(normalized, 1.0), -1.0)


class BucketPoint:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

class ServoPID:
    def __init__(self):
        self.lidar_angle = 0.0  
        self.P = 0.54         
        self.D = 0.10       
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
        self.default_linear_speed = 0.35
        self.special_linear_speed1 = 0.35
        self.special_angular_speed1 = 95.2
        self.special_angular_speed3 = 96.3
        self.special_linear_speed5 = 0.35
        #self.special_angular_speed5 = 97.5 #30角度
        self.special_angular_speed5 = 102.5 #35角度
        self.special_linear_speed4 = 0.35
        self.special_linear_speed7 = 0.0
        self.special_angular_speed7 = 90.0
        self.special_angular_speed8 = 120.0
        # AB模式参数（新增）
        self.ab_linear_speed = 0.2  # AB模式行驶速度
        self.ab_fixed_angle = 110.0  # AB模式固定角度（可根据需求调整）
        self.ab_target_distance = 1.0  # AB模式需行驶的距离（1米）

        # 桶判据算法参数（移植自 node4）
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
        self.lap_start_pose = None       # 起点坐标（用于圈数判断）
        self.lap_proximity_threshold = 0.4  # 判定回到起点的距离阈值（米）
        self.lap_proximity_hits = 0      # 累计满足阈值的连续计数
        self.lap_proximity_required = 3  # 连续命中次数，避免一次抖动误判
        self.min_lap_distance = DISTANCE_THRESHOLDS["lap_total"] * 0.5  # 至少跑够半圈才允许判定回到起点
        
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
        # 记录当前位置并进行基于位置的圈数判定
        position = msg.pose.pose.position
        current_x = position.x
        current_y = position.y

        if self.lap_start_pose is None:
            self.lap_start_pose = (current_x, current_y)
            self.get_logger().info(
                f"设置第一圈起点坐标：({current_x:.2f}, {current_y:.2f})"
            )

        self.maybe_complete_lap_by_position(current_x, current_y)
    
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
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
        
        # 达到停车时长后恢复
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.stop_start_time >= self.stop_duration:
            self.stop_timer.cancel()
            self.stop_timer = None
            self.reached_traffic_light = False
            self.traffic_light_passed = True
            self.servo.angle_integral_error = 0.0
            self.get_logger().info("Traffic light stop finished - Resuming movement")

    # 距离触发逻辑判断
    def check_distance_trigger(self):
        current_dist = self.servo.walk_distance
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

    def maybe_complete_lap_by_position(self, current_x, current_y):
        # 用“回到起点附近 + 至少跑够半圈”来判定完成第一圈
        if self.lap_start_pose is None:
            return
        if self.first_lap_completed:
            return
        if self.servo.walk_distance < self.min_lap_distance:
            return

        dx = current_x - self.lap_start_pose[0]
        dy = current_y - self.lap_start_pose[1]
        distance_to_start = math.hypot(dx, dy)
        # 需要连续多次命中阈值再判定，增强鲁棒性
        if distance_to_start <= self.lap_proximity_threshold:
            self.lap_proximity_hits += 1
        else:
            # 轻度衰减，防止偶发抖动清零
            self.lap_proximity_hits = max(self.lap_proximity_hits - 1, 0)

        if self.lap_proximity_hits >= self.lap_proximity_required:
            self.first_lap_completed = True
            self.lap_proximity_hits = 0
            # 重置累计里程以便第二圈沿用同一套分段阈值
            self.servo.walk_distance = 0.0
            self.reset_mode_states()
            # 继续用当前点作为下一圈的起点，防漂移
            self.lap_start_pose = (current_x, current_y)
            self.servo.ab_mode_distance = 0.0
            self.get_logger().info(
                f"基于位置判定完成第一圈：距起点 {distance_to_start:.2f} 米，累计里程重置"
            )

    def lidar_deal(self, data):
        # 最终停车状态保持
        if self.reached_stop:
            # 发布零速度确保真正停车（原先1500会让车继续动）
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
            return
        
        # AB模式处理（新增）：固定角度行驶1米后退出
        if self.in_ab_mode:
            # 检查是否已行驶够1米
            if self.servo.ab_mode_distance >= self.ab_target_distance:
                self.in_ab_mode = False
                self.get_logger().info(f"AB 模式完成（模式内里程：{self.servo.ab_mode_distance:.2f} 米）")
                
                self.reached_stop = True
                # 退出 AB 模式后立即发布零速度停止
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
        scan_resolution = len(ranges)
        angle_increment = data.angle_increment
        angle_min = data.angle_min

        # 激光雷达数据无效时强制停车
        if scan_resolution == 0 or all(math.isnan(r) or math.isinf(r) for r in ranges):
            self.get_logger().warn("激光雷达数据无效，强制停车")
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
            return

        # 红绿灯定时停车期间，持续发送停车指令
        if self.reached_traffic_light:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
            return

        # 先更新里程触发状态
        self.check_distance_trigger()

        red_points = []
        blue_points = []

        # 桶判据扫描
        for i in range(1, scan_resolution - self.bucket_threshold):
            current = ranges[i]

            # 过滤无效或超界点
            if math.isnan(current) or math.isinf(current):
                continue
            if current <= 0.0 or current > self.lidar_filter_max_dist:
                continue
            if current < self.lidar_filter_min_dist:
                continue

            # 跳变检查（用于找桶前沿）
            if ranges[i-1] - current < 2.0:
                continue
            if ranges[i-1] - ranges[i+1] < 2.0:
                continue

            # 连续性检查
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

            # ROI 过滤
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

            # 限幅
            final_angle = min(max(final_angle, 10.0), 170.0)
            angle_output = python_angle_to_angular_z(final_angle)

        # 桶算法输出作为基础角速度
        self.cmd_vel.linear.x = speed_output
        self.cmd_vel.angular.z = angle_output

        current_angular_z = self.cmd_vel.angular.z

        # 速度模式切换（保持原逻辑，覆盖线速度/部分角度）
        if self.reached_traffic_light:
            pass
        elif self.reached_sixth:
            self.cmd_vel.linear.x = self.default_linear_speed
            self.cmd_vel.angular.z = current_angular_z
            if abs(self.cmd_vel.angular.z - python_angle_to_angular_z(base_angle)) < 0.05:
                self.cmd_vel.linear.x += 0.03
            self.get_logger().info(f"Sixth Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        elif self.reached_fifth and not self.reached_sixth:
            self.cmd_vel.linear.x = self.special_linear_speed5
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed5)
            self.get_logger().info(f"Fifth Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        elif self.reached_fourth and not self.reached_fifth:
            self.cmd_vel.linear.x = self.special_linear_speed4
            self.cmd_vel.angular.z = current_angular_z
            self.get_logger().info(f"Fourth Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        elif self.reached_third and not self.reached_fourth:
            self.cmd_vel.linear.x = self.default_linear_speed
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed3)
            self.get_logger().info(f"Third Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        elif self.reached_second and not self.reached_third:
            self.cmd_vel.linear.x = self.default_linear_speed
            self.cmd_vel.angular.z = current_angular_z
            if abs(self.cmd_vel.angular.z - python_angle_to_angular_z(base_angle)) < 0.05:
                self.cmd_vel.linear.x += 0.03
            self.get_logger().info(f"Second Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        elif self.reached_first and not self.reached_second:
            self.cmd_vel.linear.x = self.special_linear_speed1
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed1)
            self.get_logger().info(f"First Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")
        else:
            self.cmd_vel.linear.x = speed_output
            self.cmd_vel.angular.z = current_angular_z
            self.get_logger().info(f"Default Mode: Linear={self.cmd_vel.linear.x:.2f}, Angular Z={self.cmd_vel.angular.z:.2f}")

    def adjust_bucket_counts(self, red_points, red_count, blue_points, blue_count):
        # 与 node4 同步：平衡左右桶数量，尽量各取最多 3 个
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