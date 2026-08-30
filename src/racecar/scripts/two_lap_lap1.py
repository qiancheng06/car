#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lap-1 driver: follow the corridor between red/blue cones while gmapping builds the map.

消息契约说明（本项目的 /teleop_cmd_vel 约定）：
  linear.x  = 油门 PWM（1500=停止，1500~1550=前进）
  angular.z = 舵角（度制，本文件以 82 = 直行为基准，<82 右转，>82 左转）
  这是 racecar_driver 手动模式的既定约定（teleop/lidar_tracking 均如此），
  并非标准 Twist 语义（m/s、rad/s）。
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class Lap1Driver(Node):
    def __init__(self):
        super().__init__("two_lap_lap1")
        self.cmd_pub = self.create_publisher(Twist, "/teleop_cmd_vel", 10)
        self.done_pub = self.create_publisher(Bool, "/two_lap/lap1_done", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom_combined", self.on_odom, 10)

        self.declare_parameter("speed", 1520.0)
        self.declare_parameter("turn_straight", 82.0)      # 校准后直行舵角（zero=2500 映射下）
        self.declare_parameter("kp", 10.0)                 # 间隙平衡转向增益（度/米）
        self.declare_parameter("lookahead", 1.2)           # 前视距离
        self.declare_parameter("turn_bias", 0.0)           # 恒定转向偏置（正=左偏，辅助逆时针）
        self.declare_parameter("steer_direction", -1.0)    # 坐标系方向：标准雷达系(x>0=物理左)为-1；若实测相反改+1
        self.declare_parameter("safe_distance", 0.3)       # 锥桶欧氏距离小于此值则紧急刹车
        self.declare_parameter("avoid_distance", 0.8)      # 锥桶进入此范围叠加避让转向
        self.declare_parameter("avoid_gain", 12.0)         # 避让最大叠加角（度）
        self.declare_parameter("min_track_len", 15.0)      # 最小累计路程后才判定回圈
        self.declare_parameter("return_threshold", 1.5)    # 距起点距离阈值（判定完成）
        self.declare_parameter("no_cone_speed", 1515.0)
        self.declare_parameter("no_cone_turn", 92.0)       # 无锥桶时温和直行（勿硬左转撞桶）
        self.declare_parameter("max_forward", 5.0)
        self.declare_parameter("max_lateral", 2.0)
        self.declare_parameter("side_offset", 0.75)        # 单侧锥桶目标横向距离
        self.declare_parameter("edge_drop", 0.3)           # 距离突变阈值（放宽：锥桶边缘渐变也识别）
        self.declare_parameter("noise_gap", 0.5)           # 相邻点差异超过此值视为噪声
        self.declare_parameter("plateau_tolerance", 0.3)
        self.declare_parameter("plateau_min", 2)           # 平台稳定点数（放宽：2点即判锥桶）
        self.declare_parameter("plateau_lookahead", 9)     # 平台检查窗口

        self.speed = float(self.get_parameter("speed").value)
        self.turn_straight = float(self.get_parameter("turn_straight").value)
        self.kp = float(self.get_parameter("kp").value)
        self.lookahead = float(self.get_parameter("lookahead").value)
        self.turn_bias = float(self.get_parameter("turn_bias").value)
        self.steer_direction = float(self.get_parameter("steer_direction").value)
        self.safe_distance = float(self.get_parameter("safe_distance").value)
        self.avoid_distance = float(self.get_parameter("avoid_distance").value)
        self.avoid_gain = float(self.get_parameter("avoid_gain").value)
        self.min_track_len = float(self.get_parameter("min_track_len").value)
        self.return_threshold = float(self.get_parameter("return_threshold").value)
        self.no_cone_speed = float(self.get_parameter("no_cone_speed").value)
        self.no_cone_turn = float(self.get_parameter("no_cone_turn").value)
        self.max_forward = float(self.get_parameter("max_forward").value)
        self.max_lateral = float(self.get_parameter("max_lateral").value)
        self.side_offset = float(self.get_parameter("side_offset").value)
        self.edge_drop = float(self.get_parameter("edge_drop").value)
        self.noise_gap = float(self.get_parameter("noise_gap").value)
        self.plateau_tolerance = float(self.get_parameter("plateau_tolerance").value)
        self.plateau_min = int(self.get_parameter("plateau_min").value)
        self.plateau_lookahead = int(self.get_parameter("plateau_lookahead").value)

        # 状态机：DRIVING -> DONE
        self.state = "DRIVING"
        self.odom_pos = None
        self.start_pos = None      # 首次 odom 位置作为回圈基准（不硬编码原点）
        self.last_odom = None
        self.traveled = 0.0
        self.last_error = 0.0
        self.done_ticks = 10       # DONE 状态重复发布完成信号的次数
        self.last_scan = None
        self.last_debug_log = 0.0
        self.create_timer(0.2, self.timer_cb)
        self.get_logger().info(
            f"Lap1 driver ready: speed={self.speed}, straight={self.turn_straight}, "
            f"kp={self.kp}, min_track={self.min_track_len}m, return<{self.return_threshold}m")

    # ------------------------------------------------------------------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.start_pos is None:
            self.start_pos = (p.x, p.y)
        if self.last_odom is not None:
            step = math.hypot(p.x - self.last_odom[0], p.y - self.last_odom[1])
            if 0.0 < step < 2.0:
                self.traveled += step
        self.last_odom = (p.x, p.y)
        self.odom_pos = (p.x, p.y)

    # ------------------------------------------------------------------
    def detect_cones(self, scan):
        """返回 (red_points, blue_points)，车体系坐标 [(x, y), ...]。
        x>0=右侧(红)，x<0=左侧(蓝)，y=前方。"""
        ranges = scan.ranges
        red, blue = [], []
        n = len(ranges)
        lookahead = self.plateau_lookahead
        # 主循环覆盖全部点（含尾部）；平台检查内部有越界保护
        for i in range(1, n - 1):
            r = ranges[i]
            if not math.isfinite(r) or r <= 0.0 or r > 3.0:
                continue
            prev, nxt = ranges[i - 1], ranges[i + 1]
            if not math.isfinite(prev) or not math.isfinite(nxt):
                continue
            # 距离突变判定：当前点 r 相比前点 prev 明显更近（锥桶近边缘）；
            # 第二条件跳过单点噪声（r 与 nxt 差异过大即视为尖刺）
            if prev - r < self.edge_drop:
                continue
            if abs(r - nxt) > self.noise_gap:
                continue
            # 平台稳定性：检查前后邻域与 r 接近的点数，确认是锥桶体而非孤立点
            # （后向邻域可提高锥桶体内点的检出密度，前向保留边缘起始定位）
            stable = 0
            for idx in range(1, lookahead + 1):
                j = i - idx
                if j < 0:
                    break
                nb = ranges[j]
                if math.isfinite(nb) and abs(r - nb) < self.plateau_tolerance:
                    stable += 1
            for idx in range(1, lookahead + 1):
                j = i + idx
                if j >= n:
                    break
                nb = ranges[j]
                if math.isfinite(nb) and abs(r - nb) < self.plateau_tolerance:
                    stable += 1
                if stable >= self.plateau_min:
                    break
            if stable < self.plateau_min:
                continue
            theta = scan.angle_min + i * scan.angle_increment
            x = r * math.sin(theta)
            y = r * math.cos(theta)
            if -self.max_lateral <= x <= self.max_lateral and 0.1 < y < self.max_forward:
                if x > 0:
                    red.append((x, y))
                else:
                    blue.append((x, y))
        return red, blue

    # ------------------------------------------------------------------
    @staticmethod
    def fit_line_x_at(points, target_y, max_points=8, weight_exp=0.3):
        """加权线性拟合 x = a*y + b，返回 target_y 处的 x。

        - 按 y 排序仅取最近 max_points 个点（近邻截断兜底）
        - 权重 w = 1/(y+weight_exp)：近处主导，远处衰减，
          锥桶缺口后的远锥桶几乎不影响走廊方向（软截断）
        """
        pts = sorted(points, key=lambda p: p[1])[:max_points]
        if len(pts) < 2:
            return None
        ws = [1.0 / (p[1] + weight_exp) for p in pts]
        sw = sum(ws)
        sy = sum(w * p[1] for w, p in zip(ws, pts))
        sx = sum(w * p[0] for w, p in zip(ws, pts))
        syy = sum(w * p[1] * p[1] for w, p in zip(ws, pts))
        sxy = sum(w * p[0] * p[1] for w, p in zip(ws, pts))
        denom = sw * syy - sy * sy
        if abs(denom) < 1e-9:
            return None
        a = (sw * sxy - sx * sy) / denom
        b = (sx - a * sy) / sw
        return a * target_y + b

    # ------------------------------------------------------------------
    @staticmethod
    def sector_min(ranges, n, a1, a2):
        """返回角度扇区 [a1,a2]（度，0=正前，逆时针为正）内的最小有效距离。"""
        best = 99.0
        for i in range(n):
            deg = 360.0 * i / n
            if a1 <= deg <= a2:
                r = ranges[i]
                if math.isfinite(r) and r > 0.15:
                    if r < best:
                        best = r
        return best

    # ------------------------------------------------------------------
    def on_scan(self, scan):
        """气泡避障：平衡左右最近障碍距离，哪边空往哪边走（无需锥桶识别）。"""
        if self.state != "DRIVING":
            return
        ranges = scan.ranges
        n = len(ranges)

        left = self.sector_min(ranges, n, 8.0, 82.0)       # 左前扇区
        right = self.sector_min(ranges, n, 278.0, 352.0)   # 右前扇区
        front = self.sector_min(ranges, n, 352.0, 368.0)   # 正前窄扇区

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.turn_straight

        # 1) 紧急刹车：正前过近
        if front < self.safe_distance:
            twist.linear.x = 1500.0
            self.cmd_pub.publish(twist)
            return

        # 2) 平衡间隙：左边空间大 -> 向左；右边大 -> 向右
        left_c = min(left, 3.0)
        right_c = min(right, 3.0)
        error = left_c - right_c
        angle = self.turn_straight + self.kp * error
        twist.angular.z = max(63.0, min(117.0, angle))

        # 3) 越窄越慢
        clearance = min(left_c, right_c)
        if clearance < 1.2:
            twist.linear.x = max(1505.0, self.speed - (1.2 - clearance) * 10.0)
        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    @staticmethod
    def compute_pursuit_target(red, blue, lookahead):
        """左右锥桶最近配对求中点，返回最接近前视距离的中心线点 (x, y) 或 None。"""
        mids = []
        for rx, ry in red:
            best = None
            best_d = 1e9
            for bx, by in blue:
                d = math.hypot(bx - rx, by - ry)
                if d < best_d:
                    best_d = d
                    best = (bx, by)
            if best is not None:
                mids.append(((rx + best[0]) / 2.0, (ry + best[1]) / 2.0))
        if len(mids) < 2:
            return None
        return min(mids, key=lambda p: abs(p[1] - lookahead))

    # ------------------------------------------------------------------
    def timer_cb(self):
        if self.state == "DRIVING":
            self.check_lap_done()
            # 调试：每 2 秒打印左右间隙
            if self.last_scan is not None and time.monotonic() - self.last_debug_log > 2.0:
                self.last_debug_log = time.monotonic()
                n = len(self.last_scan.ranges)
                left = self.sector_min(self.last_scan.ranges, n, 8.0, 82.0)
                right = self.sector_min(self.last_scan.ranges, n, 278.0, 352.0)
                self.get_logger().info(f"clearance: left={left:.2f} right={right:.2f}")
            return

        if self.state == "DONE":
            # 持续发布停车指令（不耗尽计数），避免看门狗行为不确定
            twist = Twist()
            twist.linear.x = 1500.0
            twist.angular.z = self.turn_straight
            self.cmd_pub.publish(twist)
            # 完成信号重复发布（非 latched 话题，防订阅方错过）
            if self.done_ticks > 0:
                msg = Bool()
                msg.data = True
                self.done_pub.publish(msg)
                self.done_ticks -= 1
            return

    # ------------------------------------------------------------------
    def check_lap_done(self):
        if self.odom_pos is None or self.start_pos is None:
            return
        pose_norm = math.hypot(
            self.odom_pos[0] - self.start_pos[0],
            self.odom_pos[1] - self.start_pos[1])
        if self.traveled > self.min_track_len and pose_norm < self.return_threshold:
            self.state = "DONE"
            self.done_ticks = 10
            self.get_logger().info(
                f"LAP-1 COMPLETE: traveled {self.traveled:.1f}m, "
                f"back at start ({pose_norm:.2f}m). Stopping.")


def main(args=None):
    rclpy.init(args=args)
    node = Lap1Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
