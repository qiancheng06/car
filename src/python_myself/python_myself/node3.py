#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 
import math
import numpy as np
import time
from nav_msgs.msg import Odometry 

# 激光雷达基础参数
start_angle = 0
end_angle = 252
L_max_Angle = 63
R_max_Angle = 189
number_fitin = 2.8
lidar_forward = 1.22
error_range = 0.15
base_angle = 90.0  # 基准角度（对应归一化后0）

# 距离触发阈值配置（单位：米，基于离原点半径）
DISTANCE_THRESHOLDS = {
    "first": 2.3,        # 半径 > 2.3 触发
    "second": 5.29,       # 半径 > 5.0 触发
    "third": 9.6,       # 半径 > 10.3 触发
    "fourth": 10.9,      # 半径 > 21.5 触发
    "fifth": 10.78,       # 半径 > 23.0 触发
    "sixth": 9.8,       # 半径 > 24.5 触发
}

# 策略配置 (红绿灯与AB模式)
STRATEGY_CONFIG = {
    # 红绿灯配置
    "traffic_light_radius": 8.7,       # 红绿灯触发半径
    "traffic_light_on_outbound": False, # True=去程触发(离场时), False=回程触发(归来时)
    
    # AB模式配置 (仅第二圈)
    "ab_trigger_radius": 5.0,          # AB模式触发半径 (建议小于红绿灯半径，确保在红绿灯后)
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
        self.P = 0.4       
        self.D = 0.12
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
    
    # 新增：位置式PID (适配 car_test.cpp 逻辑)
    def PIDPositional(self, error):
        # 复用现有逻辑，只是名字不同
        return self.servo_pid_control(error)

class LidarCoordinateControlNode(Node):
    def __init__(self):
        super().__init__('lidar_distance_control_node')
        self.servo = ServoPID()
        
        self.cmd_vel = Twist()
        self.publisher_twist = self.create_publisher(Twist, '/car_cmd_vel', 10)
        # 激光雷达数据订阅
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # 里程计订阅（修改为 /encoder_imu_odom）
        self.subscription_odom = self.create_subscription(Odometry, '/encoder_imu_odom', self.odometry_callback, 10)
        
        # 速度参数配置 (单位: m/s, 角度为归一化前的原始角度 0-180)
        self.default_linear_speed = 0.30      # 默认巡航速度 (直道/通用路段)
        self.special_linear_speed1 = 0.30     # 第1路段(起步/高速区)速度
        self.special_angular_speed1 = 95.2    # 第1路段强制转向角 (盲跑角度, >90左转, <90右转)
        self.special_angular_speed3 = 96.3    # 第3路段强制转向角
        self.special_linear_speed5 = 0.30     # 第5路段速度
        self.special_angular_speed5 = 97.5    # 第5路段强制转向角
        self.special_linear_speed4 = 0.30     # 第4路段速度
        self.special_linear_speed7 = 0.0      # (预留) 停车速度
        self.special_angular_speed7 = 90.0    # (预留) 停车回正角度
        self.special_angular_speed8 = 120.0   # 最终停车时的锁死角度 (防止溜车或作为标志)
        
        # AB模式参数 (仅第二圈触发)
        self.ab_linear_speed = 0.2            # AB模式下的固定速度
        self.ab_fixed_angle = 110.0           # AB模式下的固定转向角 (盲跑)
        self.ab_target_distance = 1.0         # AB模式持续行驶的距离 (米)
        
        # --- 移植自 car_test.cpp 的参数 ---
        self.bucket_threshold = 10            # 桶识别阈值: 多少个连续雷达点算一个桶
        self.valid_bucket_threshold = 6       # 有效桶阈值: 一个桶至少要包含多少个点才算有效
        self.lidar_filter_min_dist = 0.3      # 雷达过滤: 忽略距离小于 0.3m 的点 (车身遮挡/噪点)
        self.lidar_filter_max_dist = 2.0      # 雷达过滤: 忽略距离大于 2.5m 的点 (只关注近处赛道)
        self.roi_y_min = -0.2                 # 感兴趣区域(ROI) Y轴最小值 (车后方不看)
        self.roi_y_max = 3.0                  # 感兴趣区域(ROI) Y轴最大值 (只看前方3米内)
        self.max_left_distance = 1.5          # ROI X轴左边界 (只看左边1.5米内)
        self.min_right_distance = 1.5         # ROI X轴右边界 (只看右边1.5米内)
        self.steer_gain = 20.0                # 转向增益: 误差转角度的放大倍数 (P控制的一部分)
        self.steer_threshold_slow_down = 20.0 # 减速阈值: 当转向角超过20度时触发减速
        
        # 逻辑触发参数 (移植自 car_test.cpp)
        self.trigger_dist_go = 9.5
        # 触发换圈：必须回到更靠近原点的位置再重置，避免过早进入第二圈
        self.trigger_dist_change = 0.8
        
        self.flag_go = False
        self.flag_change = False
        # -------------------------------

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
        self.in_ab_mode = False
        self.ab_mode_triggered = False

        # 半径峰值检测：用于区分去程/回程
        self.peak_radius = 0.0
        self.peak_reached = False
        self.peak_epsilon = 0.05          # 半径回落阈值
        self.peak_descend_eps = 0.02      # 单次回落阈值（抗抖）
        self.peak_descend_required = 3    # 连续回落计数
        self.peak_descend_count = 0
        self.last_radius = None
        # 限制回程判定至少到达一定半径，避免小半径抖动误判回程
        self.min_peak_radius_for_return = max(
            DISTANCE_THRESHOLDS["third"], DISTANCE_THRESHOLDS["fourth"] * 0.9
        )

        # 回程段触发间隔，防止同帧触发 5/6
        self.fifth_to_sixth_gap = 0.05
        
        # 时间戳记录
        self.last_odom_timestamp = None
        # 红绿灯停车定时器
        self.stop_timer = None
        self.stop_start_time = None
        self.stop_duration = 3.0
        # 刹车计时（红绿灯/最终停车）
        self.brake_start_time_light = None
        self.brake_start_time_stop = None
        self.brake_time = 0.4     # 反向刹车时长（秒）
        self.brake_speed = -8.0    # 反向速度大小（m/s，负值为反向）

        self.get_logger().info("Lidar distance control node started (CarTest Logic Ported)")

    def log_info(self, tag: str, msg: str):
        """统一日志格式，便于筛选."""
        self.get_logger().info(f"[{tag}] {msg}")
    
    def odometry_callback(self, msg):
        # 修改：使用绝对坐标距离原点的距离
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dist_from_origin = math.hypot(x, y)
        
        # 更新 walk_distance 为离原点距离
        self.servo.walk_distance = dist_from_origin

        # AB模式仍需计算相对行驶距离，保留积分逻辑
        current_timestamp = self.get_clock().now().nanoseconds / 1e9  
        linear_speed = abs(msg.twist.twist.linear.x)
        
        if self.last_odom_timestamp is not None:
            dt = current_timestamp - self.last_odom_timestamp  
            if dt > 0:
                # self.servo.walk_distance += linear_speed * dt # 原逻辑已注释
                if self.in_ab_mode:
                    self.servo.ab_mode_distance += linear_speed * dt
        
        self.last_odom_timestamp = current_timestamp
        # 日志
        self.log_info(
            "ODOM",
            (
                f"r={self.servo.walk_distance:.2f}m, lap_done={self.first_lap_completed}, "
                f"AB={self.in_ab_mode}({self.servo.ab_mode_distance:.2f}m)"
            ),
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
        
        # 重置红绿灯状态，以便第二圈能再次触发
        self.reached_traffic_light = False
        self.traffic_light_passed = False
        
        # 重置AB模式状态
        self.in_ab_mode = False
        self.ab_mode_triggered = False
        self.servo.ab_mode_distance = 0.0

        # 重置峰值检测，重新识别去/回程
        self.peak_radius = 0.0
        self.peak_reached = False
        self.peak_descend_count = 0
        self.last_radius = None
        self.min_peak_radius_for_return = max(
            DISTANCE_THRESHOLDS["third"], DISTANCE_THRESHOLDS["fourth"] * 0.9
        )
        
        # 重置环形赛道标志位，以便进行第二圈检测
        self.flag_go = False
        self.flag_change = False
        
        # 注意：walk_distance 现在是实时半径，不需要也不应该重置为0
        # self.servo.walk_distance = 0.0 

    # 启动红绿灯停车（非阻塞）
    def start_traffic_light_stop(self):
        if self.stop_timer is None:
            self.stop_start_time = self.get_clock().now().nanoseconds / 1e9
            self.stop_timer = self.create_timer(0.1, self.traffic_light_stop_callback)
            self.get_logger().info("Traffic light stop started (3s)")

    # 红绿灯停车回调
    def traffic_light_stop_callback(self):
        # 仅用于计时检查，实际停车控制在 lidar_deal 中高优先级处理
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.stop_start_time >= self.stop_duration:
            self.stop_timer.cancel()
            self.stop_timer = None
            self.reached_traffic_light = False
            self.traffic_light_passed = True
            self.servo.angle_integral_error = 0.0
            self.brake_start_time_light = None
            self.get_logger().info("Traffic light stop finished - Resuming movement")

    def log_radius_state(self, name, threshold, current_distance):
        """Log when the robot crosses a configured radius threshold."""
        self.log_info(
            "SEG",
            f"到达{name}: 阈值 {threshold:.2f}m | 当前 {current_distance:.2f}m",
        )

    # 距离触发逻辑判断
    def check_distance_trigger(self):
        current_dist = self.servo.walk_distance

        # --- 峰值检测：用于划分去程/回程 ---
        if current_dist > self.peak_radius:
            self.peak_radius = current_dist

        if self.last_radius is not None:
            delta = current_dist - self.last_radius
            if delta < -self.peak_descend_eps:
                self.peak_descend_count += 1
            else:
                self.peak_descend_count = 0

            if (not self.peak_reached and
                self.peak_radius >= self.min_peak_radius_for_return and
                self.peak_descend_count >= self.peak_descend_required and
                self.peak_radius - current_dist > self.peak_epsilon):
                self.peak_reached = True
                self.log_info(
                    "PEAK",
                    f"半径回落触发回程：峰值 {self.peak_radius:.2f}m -> 当前 {current_dist:.2f}m",
                )
                if not self.reached_fourth:
                    self.reached_fourth = True
                    self.log_radius_state("第四段(峰值保底)", self.peak_radius, current_dist)

        self.last_radius = current_dist
        
        # --- 环形赛道逻辑 (基于半径) ---
        
        # 1. 离场检测 (去程)
        # 只有当半径大于 trigger_dist_go (9.5m) 时，才认为已经离开起跑区
        if current_dist > self.trigger_dist_go and not self.flag_go:
            self.flag_go = True
            self.log_info("STATE", f"离开起点，半径>{self.trigger_dist_go:.2f}m")

        # 2. 事件触发检测 (红绿灯 & AB模式)
        # 必须先离场 (flag_go=True) 或者是去程触发模式
        
        # --- 红绿灯检测 (第一圈和第二圈都执行) ---
        if not self.reached_traffic_light and not self.traffic_light_passed:
            should_trigger_light = False
            light_radius = STRATEGY_CONFIG["traffic_light_radius"]
            
            if STRATEGY_CONFIG["traffic_light_on_outbound"]:
                # 方案A：去程触发 (半径变大超过阈值)
                # 此时 flag_go 可能还未 True (取决于阈值)，主要靠距离判断
                # 为防止回程误触发，必须确保 flag_go 为 False (还没离场远) 或者 距离正在变大(难判断)
                # 简单逻辑：如果 flag_go 已经是 True (已离场很远)，且现在距离又变小了，那就是回程，不应触发去程逻辑
                # 但如果 light_radius < trigger_dist_go，去程时 flag_go 是 False。
                if current_dist >= light_radius and not self.flag_go:
                     should_trigger_light = True
            else:
                # 方案B：回程触发 (半径变小低于阈值)
                # 必须已离场 (flag_go=True) 且正在回程 (距离小于阈值)
                if self.flag_go and current_dist <= light_radius:
                    should_trigger_light = True
            
            if should_trigger_light:
                self.reached_traffic_light = True
                self.start_traffic_light_stop()
                lap_str = "2nd" if self.first_lap_completed else "1st"
                self.log_info("LIGHT", f"{lap_str} lap 触发红绿灯，半径 {current_dist:.2f}m")

        # --- AB模式检测 (仅第二圈) ---
        # 逻辑：第二圈 + 已离场(回程中) + 距离小于阈值
        if self.first_lap_completed and self.flag_go:
            ab_radius = STRATEGY_CONFIG["ab_trigger_radius"]
            if current_dist <= ab_radius:
                if not self.ab_mode_triggered and not self.in_ab_mode:
                    self.in_ab_mode = True
                    self.ab_mode_triggered = True
                    self.servo.ab_mode_distance = 0.0 # 从触发时刻开始清零计算
                    self.log_info("AB", f"第二圈触发AB模式，半径 {current_dist:.2f}m")

        # 3. 回原点/换圈检测 (需先离场 flag_go=True)
        if current_dist <= self.trigger_dist_change and self.flag_go and not self.flag_change:
            self.flag_change = True
            self.log_info("STATE", f"接近原点，半径<= {self.trigger_dist_change:.2f}m")
            
            if not self.first_lap_completed:
                self.first_lap_completed = True
                self.reset_mode_states() # 重置标志位，开始第二圈逻辑
                self.log_info("LAP", "第一圈完成，重置标志开始第二圈")
            else:
                self.log_info("LAP", "第二圈完成，可选择停车")
                # 可以选择在这里强制停车
                # self.reached_stop = True

        # --- 速度模式触发 (基于半径) ---
        # 假设这些阈值在去程有效 (或者全程有效，视赛道而定)
        # 为了防止回程时误触发低速模式，可以加 flag_go 判断，或者假设回程也需要同样的速度控制
        
        if not self.reached_first and current_dist >= DISTANCE_THRESHOLDS["first"]:
            # 去程阶段（未到最大半径）：1-4 采用 >= 判定
            self.reached_first = True
            self.log_radius_state("第一段", DISTANCE_THRESHOLDS["first"], current_dist)
        if (not self.peak_reached) and (not self.reached_second) and current_dist >= DISTANCE_THRESHOLDS["second"]:
            self.reached_second = True
            self.log_radius_state("第二段", DISTANCE_THRESHOLDS["second"], current_dist)
        if (not self.peak_reached) and (not self.reached_third) and current_dist >= DISTANCE_THRESHOLDS["third"]:
            self.reached_third = True
            self.log_radius_state("第三段", DISTANCE_THRESHOLDS["third"], current_dist)
        if (not self.peak_reached) and (not self.reached_fourth) and current_dist >= DISTANCE_THRESHOLDS["fourth"]:
            self.reached_fourth = True
            self.log_radius_state("第四段", DISTANCE_THRESHOLDS["fourth"], current_dist)

        # 回程阶段（已过最大半径）：5-6 采用 <= 判定
        if (
            self.peak_reached and
            self.peak_radius >= DISTANCE_THRESHOLDS["fifth"] and
            (not self.reached_fifth) and
            current_dist <= DISTANCE_THRESHOLDS["fifth"]
        ):
            self.reached_fifth = True
            self.log_radius_state("第五段", DISTANCE_THRESHOLDS["fifth"], current_dist)
        if (
            self.peak_reached and
            self.peak_radius >= DISTANCE_THRESHOLDS["sixth"] and
            self.reached_fifth and
            (not self.reached_sixth) and
            current_dist <= (DISTANCE_THRESHOLDS["sixth"] - self.fifth_to_sixth_gap)
        ):
            self.reached_sixth = True
            self.log_radius_state("第六段", DISTANCE_THRESHOLDS["sixth"], current_dist)

    def adjust_bucket_counts(self, red_points, red_count, blue_points, blue_count):
        if blue_count >= red_count:
            split_index = 100
            for i in range(1, blue_count):
                distance_squared = (blue_points[i-1].x - blue_points[i].x)**2 + \
                                   (blue_points[i-1].y - blue_points[i].y)**2
                if distance_squared > 1.4 * 1.4:
                    split_index = i
                    break
            
            if split_index != 100:
                # 移动点：Python list 操作
                # 将 blue_points[split_index:] 移动到 red_points 尾部
                # 注意 car_test.cpp 是反向填充 red_points
                # C++: red_points[i + blue_count - split_index] = red_points[i] (shift)
                #      red_points[i] = blue_points[split_index + i] (copy)
                
                # Python 实现：
                points_to_move = blue_points[split_index:]
                blue_points = blue_points[:split_index]
                # 插入到 red_points 头部
                red_points = points_to_move + red_points
                
                red_count = len(red_points)
                blue_count = len(blue_points)
        else:
            split_index = 100
            for i in range(red_count - 2, -1, -1):
                distance_squared = (red_points[i].x - red_points[i+1].x)**2 + \
                                   (red_points[i].y - red_points[i+1].y)**2
                if distance_squared > 1.4 * 1.4:
                    split_index = i
                    break
            
            if split_index != 100:
                # C++: blue_points[blue_count + i] = red_points[i] (copy first part to blue)
                #      red_points shift left
                
                # Python 实现：
                points_to_move = red_points[:split_index+1]
                red_points = red_points[split_index+1:]
                # 追加到 blue_points 尾部
                blue_points = blue_points + points_to_move
                
                red_count = len(red_points)
                blue_count = len(blue_points)

        # 限制数量
        red_points = red_points[:3]
        blue_points = blue_points[:3]
        red_count = len(red_points)
        blue_count = len(blue_points)

        if red_count > blue_count and blue_count > 0:
            if blue_count == 1:
                if red_count > 2:
                    # shift red right
                    red_points = [red_points[0]] + red_points[:2] # 逻辑有点怪，照搬C++: red[0]=red[1]? No.
                    # C++: red[0]=red[1], red[1]=red[2] (shift left?)
                    # C++: for(i=0;i<2) red[i]=red[i+1]. 这是一个左移操作，丢弃头部。
                    red_points = red_points[1:]
                    red_count = 2
                # blue[1] = blue[0] -> duplicate
                blue_points.append(blue_points[0])
                blue_count = 2
            elif blue_count == 2:
                blue_points.append(blue_points[1])
                blue_count = 3
        elif blue_count > red_count and red_count > 0:
            if red_count == 1:
                blue_count = 2 # C++ logic seems to just set count?
                # C++: blue_count=2; red[1]=red[0]; red_count=2;
                red_points.append(red_points[0])
                red_count = 2
            elif red_count == 2:
                # C++: red[2]=red[1]; red[1]=red[0]; red_count=3;
                # 这是一个右移操作，复制头部？
                # Wait, C++: red[2]=red[1], red[1]=red[0]. red[0] unchanged.
                # So [0, 1] -> [0, 0, 1].
                red_points = [red_points[0]] + red_points
                red_count = 3
        
        return red_points, blue_points

    def lidar_deal(self, data):
        # 1. 优先更新状态 (触发器检测)
        # 必须在处理逻辑之前调用，以免漏掉状态切换
        self.check_distance_trigger()

        # 2. 优先级最高的阻断状态 (停车/红绿灯)
        if self.reached_stop:
            now = self.get_clock().now().nanoseconds / 1e9
            if self.brake_start_time_stop is None:
                self.brake_start_time_stop = now
            if now - self.brake_start_time_stop < self.brake_time:
                self.cmd_vel.linear.x = self.brake_speed
            else:
                self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed8)
            return
        
        if self.reached_traffic_light:
            now = self.get_clock().now().nanoseconds / 1e9
            if self.brake_start_time_light is None:
                self.brake_start_time_light = now
            if now - self.brake_start_time_light < self.brake_time:
                self.cmd_vel.linear.x = self.brake_speed
            else:
                self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
            return
        
        # 3. AB模式处理 (仅第二圈)
        if self.in_ab_mode:
            if self.servo.ab_mode_distance >= self.ab_target_distance:
                self.in_ab_mode = False
                self.log_info("AB", f"AB 模式完成，行驶 {self.servo.ab_mode_distance:.2f}m")
                # AB模式结束后通常接停车，或者继续跑？这里假设停车
                self.reached_stop = True 
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)
                return
            else:
                self.cmd_vel.linear.x = self.ab_linear_speed
                self.cmd_vel.angular.z = python_angle_to_angular_z(self.ab_fixed_angle)
                return

        # --- 4. 常规巡线逻辑 (Bucket Algorithm) ---
        ranges = data.ranges
        scan_resolution = len(ranges)
        angle_increment = data.angle_increment
        angle_min = data.angle_min
        
        red_points = []
        blue_points = []
        
        # 遍历雷达点
        # C++: for (int i = 1; i < scan_resolution_ - bucket_threshold_; ++i)
        for i in range(1, scan_resolution - self.bucket_threshold):
            current = ranges[i]
            
            # 过滤无效点和距离
            if math.isnan(current) or math.isinf(current):
                continue
            if current <= 0.0 or current > self.lidar_filter_max_dist:
                continue
            if current < self.lidar_filter_min_dist:
                continue
            
            # 连续性检查 (跳变检测)
            if ranges[i-1] - current < 2.0: # C++: < 2.0f continue
                continue
            if ranges[i-1] - ranges[i+1] < 2.0:
                continue
            
            # 桶检测
            continue_ranges = 0
            for idx in range(1, self.bucket_threshold):
                if i + idx < scan_resolution:
                    if abs(current - ranges[i+idx]) < 0.2:
                        continue_ranges += 1
                        if continue_ranges >= self.valid_bucket_threshold:
                            break
            
            if continue_ranges < self.valid_bucket_threshold:
                continue
            
            # 坐标转换 (car_test.cpp 坐标系: x=r*sin, y=r*cos)
            theta = angle_min + i * angle_increment
            x = current * math.sin(theta)
            y = current * math.cos(theta)
            
            # ROI 过滤
            if (x <= self.max_left_distance and x >= -self.min_right_distance and 
                y < self.roi_y_max and y > self.roi_y_min):
                
                pt = BucketPoint(x, y)
                if x > 0.0:
                    red_points.append(pt)
                elif x <= 0.0:
                    blue_points.append(pt)
        
        # 调整桶
        red_points, blue_points = self.adjust_bucket_counts(red_points, len(red_points), blue_points, len(blue_points))
        
        red_count = len(red_points)
        blue_count = len(blue_points)
        error_count = min(red_count, blue_count)
        
        angle_output = 0.0
        speed_output = self.default_linear_speed # 默认速度
        
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
            
            # PID 计算
            angle_pid = self.servo.PIDPositional(error)
            
            # C++: twist_.angular.z = 90.0 + angle;
            final_angle = 90.0 + angle_pid
            
            # 减速逻辑
            if abs(angle_pid) >= self.steer_threshold_slow_down:
                speed_output -= 0.05 
                speed_output = max(speed_output, 0.1)

            # 限制角度
            if final_angle > 170.0: final_angle = 170.0
            if final_angle < 10.0: final_angle = 10.0 
            
            # 转换为 node2.py 的控制量
            normalized_angular_z = python_angle_to_angular_z(final_angle)
            
            self.cmd_vel.linear.x = speed_output
            self.cmd_vel.angular.z = normalized_angular_z
        else:
            # 未检测到桶，保持直行或上一状态
            self.cmd_vel.linear.x = 0.0 # 安全起见停车，或者低速直行
            self.log_info("LIDAR", "未检测到桶，停车")
            self.cmd_vel.angular.z = python_angle_to_angular_z(base_angle)

        # 5. 速度模式覆盖 (基于半径)
        # 如果处于特定距离模式，应用特定速度，但保留雷达计算出的转向角
        
        current_angular_z = self.cmd_vel.angular.z # 来自雷达计算
        
        if self.reached_sixth:
            self.cmd_vel.linear.x = self.default_linear_speed
            self.cmd_vel.angular.z = current_angular_z
            if abs(self.cmd_vel.angular.z - python_angle_to_angular_z(base_angle)) < 0.05:
                self.cmd_vel.linear.x += 0.03
        elif self.reached_fifth and not self.reached_sixth:
            self.cmd_vel.linear.x = self.special_linear_speed5
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed5) # 强制角度
        elif self.reached_fourth and not self.reached_fifth:
            self.cmd_vel.linear.x = self.special_linear_speed4
            self.cmd_vel.angular.z = current_angular_z
        elif self.reached_third and not self.reached_fourth:
            self.cmd_vel.linear.x = self.default_linear_speed
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed3) # 强制角度
        elif self.reached_second and not self.reached_third:
            self.cmd_vel.linear.x = self.default_linear_speed
            self.cmd_vel.angular.z = current_angular_z
            if abs(self.cmd_vel.angular.z - python_angle_to_angular_z(base_angle)) < 0.05:
                self.cmd_vel.linear.x += 0.03
        elif self.reached_first and not self.reached_second:
            self.cmd_vel.linear.x = self.special_linear_speed1
            self.cmd_vel.angular.z = python_angle_to_angular_z(self.special_angular_speed1) # 强制角度
        
        # 注意：这里不再调用 publish，统一由 lidar_callback 发布

    # 移除旧的 lidar_find_2edge 和 lidar_find_point


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
        
        # 关键修改：连续发送多次停车指令并延时，确保底层收到
        for _ in range(5):
            node.publisher_twist.publish(stop_cmd)
            time.sleep(0.1)
            
        node.get_logger().info("Node stopped by user - Robot forced stop")
    finally:
        if node.stop_timer is not None:
            node.stop_timer.cancel()
        # 等待一段时间再退出，以确保停车指令被执行
        time.sleep(1.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()