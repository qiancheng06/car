#!/usr/bin/env python3
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
        print("DBG1 before super", flush=True)
        super().__init__('vision_fuse')
        print("DBG2 after super", flush=True)
        self.declare_parameter('kp', 0.88)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 1.2)
        self.declare_parameter('speed', 1520.0)
        self.declare_parameter('error_gain', 53.33)
        self.declare_parameter('min_dist', 0.5)
        self.declare_parameter('max_dist', 2.0)
        self.declare_parameter('back_dist', 0.2)
        self.declare_parameter('w_vis', 0.6)
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('trim_gain', 0.05)
        self.declare_parameter('trim_max', 8.0)
        self.declare_parameter('cam_id', 0)
        print("DBG3 params declared", flush=True)

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
        print("DBG4 params got", flush=True)

        self.model = YOLO(MODEL_PATH)
        names = getattr(self.model, 'names', None)
        self.get_logger().info('模型加载完成,类别: %s' % (names,))
        print("DBG5 model loaded", flush=True)

        self.cap = cv2.VideoCapture(self.get_parameter('cam_id').value)
        if not self.cap.isOpened():
            self.get_logger().error('摄像头打开失败')
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("DBG6 camera opened=%s" % self.cap.isOpened(), flush=True)

        self.vlock = threading.Lock()
        self.red_side = 0
        self.blue_side = 0
        self.light = 'none'
        self.vis_err = None

        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.last_error = 0.0
        self.trim = 0.0
        print("DBG7 state init", flush=True)

        print("DBG8 creating publisher", flush=True)
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        print("DBG9 publisher ok", flush=True)
        self.info_pub = self.create_publisher(String, '/vision_info', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        print("DBG10 sub ok", flush=True)

        threading.Thread(target=self._vision_loop, daemon=True).start()
        self.get_logger().info('vision_fuse 启动')
        print("DBG11 init done", flush=True)

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
                        cx = (x1 + x2) / 2.0 / w * 2.0 - 1.0
                        roi = frame[y1:y2, x1:x2]
                        color = classify_color(roi)
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
                with self.vlock:
                    self.red_side = 0
                    self.blue_side = 0
                    if red_cx:
                        self.red_side = 1 if np.mean(red_cx) > 0 else -1
                    if blue_cx:
                        self.blue_side = 1 if np.mean(blue_cx) > 0 else -1
                    self.light = light
                    if red_cx and blue_cx:
                        if self.red_side != self.blue_side:
                            e = (np.mean(red_cx) + np.mean(blue_cx)) / 2.0 * 0.6
                            self.vis_err = e if abs(e) < 1.2 else None
                        else:
                            self.vis_err = None
                    elif red_cx:
                        e = (np.mean(red_cx) * 0.6) - 0.3
                        self.vis_err = e if abs(e) < 1.2 else None
                    elif blue_cx:
                        e = (np.mean(blue_cx) * 0.6) + 0.3
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

    def scan_cb(self, msg):
        pass


def main(args=None):
    print("MAIN1 enter main", flush=True)
    print("MAIN2 importing rclpy", flush=True)
    import rclpy as _r
    print("MAIN3 rclpy imported", flush=True)
    _r.init(args=args)
    print("MAIN4 rclpy.init done", flush=True)
    node = VisionFuseNode()
    print("MAIN5 node created", flush=True)
    try:
        _r.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        _r.shutdown()


if __name__ == '__main__':
    main()
