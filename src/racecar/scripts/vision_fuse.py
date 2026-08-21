#!/usr/bin/env python3
"""
vision_fuse.py  视觉 + 雷达 融合循迹节点

分工:
  雷达 /scan       10Hz  主控循迹(红蓝锥桶中间走,算法与 PID_test.cpp 一致)
  摄像头 YOLO      ~1Hz  颜色裁判:识别红/蓝锥桶在哪一侧 + 红绿灯状态
  融合:
    - 视觉给出"红蓝两排中心"的图像偏差(换算到米),与雷达偏差加权融合
    - 红灯 -> 停车;绿灯/无灯 -> 行驶
  发布: /teleop_cmd_vel  (协议与 PID_test.cpp 相同)

运行:
  ros2 run racecar vision_fuse.py [--ros-args -p kp:=0.88 ...]
"""
import os
import math
import time
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ultralytics import YOLO

from cone_color import classify_color, hsv_make

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'best.pt')

GREEN_HSV = [((45, 60, 60), (85, 255, 255))]


class VisionFuseNode(Node):
    def __init__(self):
        super().__init__('vision_fuse')
        # ---------- 参数(与 PID_test.cpp 对齐) ----------
        self.declare_parameter('kp', 0.88)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 1.2)
        self.declare_parameter('speed', 1520.0)
        self.declare_parameter('error_gain', 53.33)
        self.declare_parameter('min_dist', 0.5)
        self.declare_parameter('max_dist', 2.0)
        self.declare_parameter('back_dist', 0.2)
        self.declare_parameter('w_vis', 0.6)          # 视觉误差权重(0~1,0=纯雷达)
        self.declare_parameter('conf', 0.35)          # YOLO 置信度
        self.declare_parameter('imgsz', 320)          # YOLO 输入尺寸(小=快)
        self.declare_parameter('trim_gain', 0.05)     # 中值自动校准速度
        self.declare_parameter('trim_max', 8.0)       # 中值校准限幅
        self.declare_parameter('cam_id', 0)           # 摄像头编号

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.speed = self.get_parameter('speed').value
        self.error_gain = self.get_parameter('error_gain').value
        self.min_dist = self.get_parameter('min_dist').value
        self.max_dist = self.get_parameter('max_dist').value
        self.back_dist = self.get_parameter('back_dist').value
        self.w_vis = self.get_parameter('w_vis').value
        self.conf = self.get_parameter('conf').value
        self.imgsz = self.get_parameter('imgsz').value
        self.trim_gain = self.get_parameter('trim_gain').value
        self.trim_max = self.get_parameter('trim_max').value

        # ---------- YOLO 模型 ----------
        self.model = YOLO(MODEL_PATH)
        names = getattr(self.model, 'names', None)
        self.get_logger().info('模型加载完成,类别: %s' % (names,))

        # ---------- 摄像头 ----------
        self.cap = cv2.VideoCapture(self.get_parameter('cam_id').value)
        if not self.cap.isOpened():
            self.get_logger().error('摄像头打开失败,请检查 /dev/video*')
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # ---------- 视觉状态(视觉线程写,控制线程读) ----------
        self.vlock = threading.Lock()
        self.red_side = 0      # +1=右侧 -1=左侧 0=未识别
        self.blue_side = 0
        self.light = 'none'    # red / green / none
        self.vis_err = None    # 视觉偏差(米,与雷达同量纲)

        # ---------- PID 状态 ----------
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.last_error = 0.0
        self.trim = 0.0

        # ---------- ROS ----------
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        self.info_pub = self.create_publisher(String, '/vision_info', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        threading.Thread(target=self._vision_loop, daemon=True).start()
        self.get_logger().info(
            'vision_fuse 启动  kp=%.2f ki=%.2f kd=%.2f  w_vis=%.1f  速度=%.0f'
            % (kp, ki, kd, self.w_vis, self.speed))

    # ==================== 视觉线程(YOLO + HSV 分色) ====================
    def _vision_loop(self):
        while rclpy.ok():
            t0 = time.time()
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.2)
                continue
            h, w = frame.shape[:2]

            try:
                results = self.model.predict(frame, imgsz=self.imgsz,
                                             conf=self.conf, verbose=False)
                boxes = results[0].boxes
                red_cx, blue_cx, light = [], [], 'none'
                if boxes is not None and len(boxes) > 0:
                    for b in boxes:
                        x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                        cx = (x1 + x2) / 2.0 / w * 2.0 - 1.0   # 归一化 -1(左)~+1(右)
                        roi = frame[y1:y2, x1:x2]
                        color = classify_color(roi)
                        # 上半屏的红色/绿色小块视为红绿灯
                        if y2 < int(h * 0.35):
                            if color == 'red':
                                light = 'red'
                            else:
                                g = hsv_make(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), GREEN_HSV).sum()
                                if g > 200:
                                    light = 'green'
                        else:
                            if color == 'red':
                                red_cx.append(cx)
                            elif color == 'blue':
                                blue_cx.append(cx)
                # 更新状态
                with self.vlock:
                    self.red_side = 0
                    self.blue_side = 0
                    if red_cx:
                        self.red_side = 1 if np.mean(red_cx) > 0 else -1
                    if blue_cx:
                        self.blue_side = 1 if np.mean(blue_cx) > 0 else -1
                    self.light = light
                    # 计算视觉偏差(米):红蓝两排中点,左右各保持 0.6m 量纲
                    if red_cx and blue_cx:
                        if self.red_side != self.blue_side:      # 红蓝各在一侧才有效
                            e = (np.mean(red_cx) + np.mean(blue_cx)) / 2.0 * 0.6
                            self.vis_err = e if abs(e) < 1.2 else None
                        else:
                            self.vis_err = None
                    elif red_cx:
                        e = (np.mean(red_cx) * 0.6) - 0.3       # 只看红排:保持 0.3m 距离
                        self.vis_err = e if abs(e) < 1.2 else None
                    elif blue_cx:
                        e = (np.mean(blue_cx) * 0.6) + 0.3      # 只看蓝排:保持 0.3m 距离
                        self.vis_err = e if abs(e) < 1.2 else None
                    else:
                        self.vis_err = None
                self.get_logger().info(
                    '视觉 红=%d(侧%d) 蓝=%d(侧%d) 灯=%s 偏差=%.2f'
                    % (len(red_cx), self.red_side, len(blue_cx), self.blue_side,
                       light, self.vis_err if self.vis_err is not None else 0.0))
            except Exception as ex:
                self.get_logger().warn('视觉处理异常: %s' % ex)

            dt = time.time() - t0
            if dt < 0.9:
                time.sleep(0.9 - dt)

    # ==================== 雷达控制回调(10Hz) ====================
    def scan_cb(self, msg):
        sum_x = 0.0
        cnt = 0
        nearest_front = 100.0
        for i in range(len(msg.ranges)):
            r = msg.ranges[i]
            if not math.isfinite(r) or r <= 0:
                continue
            theta = msg.angle_min + i * msg.angle_increment
            x = r * math.sin(theta)
            y = r * math.cos(theta)
            if y > 0.05 and y > 0.8 * r and y < nearest_front:
                nearest_front = y
            if r <= self.min_dist or r > self.max_dist:
                continue
            if y < -self.back_dist:
                continue
            sum_x += x
            cnt += 1

        twist = Twist()
        # 防撞
        if nearest_front < 0.35:
            twist.linear.x = 1500.0
            twist.angular.z = 90.0 + self.trim
            self.pub.publish(twist)
            self.get_logger().warn('正前方障碍 %.2fm,紧急停车' % nearest_front)
            return
        if cnt == 0:
            twist.linear.x = 1500.0
            twist.angular.z = 90.0 + self.trim
            self.pub.publish(twist)
            return

        dev = sum_x / cnt

        # 融合:视觉可用时按权重混合
        with self.vlock:
            vis_err = self.vis_err
            light = self.light
        wv = self.w_vis if vis_err is not None else 0.0
        fused = dev * (1.0 - wv) + (vis_err if vis_err is not None else 0.0) * wv

        # 中值自动校准(与 PID_test.cpp 相同符号)
        if abs(fused) < 0.15:
            self.trim -= fused * self.trim_gain
            self.trim = max(-self.trim_max, min(self.trim_max, self.trim))

        # PID(位置式)
        err_deg = fused * self.error_gain
        self.integral += err_deg * 0.1
        self.integral = max(-8.0, min(8.0, self.integral))
        out = self.kp * err_deg + self.ki * self.integral + self.kd * (err_deg - self.last_error)
        self.last_error = err_deg

        z = 90.0 + self.trim - out
        z = max(60.0, min(120.0, z))
        twist.angular.z = z

        # 红绿灯:红灯停车
        if light == 'red':
            twist.linear.x = 1500.0
        else:
            spd = self.speed
            if abs(out) > 20:
                spd -= 10
            if abs(out) > 35:
                spd -= 15
            spd = max(1503.0, spd)
            twist.linear.x = spd

        self.pub.publish(twist)

        info = String()
        info.data = '红侧:%d 蓝侧:%d 灯:%s 视觉偏差:%.2f 雷达偏差:%.2f 融合:%.2f 舵机角:%.1f 速度:%.0f' % (
            self.red_side, self.blue_side, light,
            vis_err if vis_err is not None else 0.0, dev, fused, z, twist.linear.x)
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = VisionFuseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
