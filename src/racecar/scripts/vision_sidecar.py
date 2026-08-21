#!/usr/bin/env python3
"""
vision_sidecar.py  视觉辅助节点(只负责看,不负责控制)

职责:
  摄像头 + YOLO(best.pt) + HSV 分色
  -> 每 ~1 秒发布一次 /vision_status(String)
     格式: "light:red|green|none red_side:+1|-1|0 blue_side:+1|-1|0 red_n:.. blue_n:.."
  雷达循迹由 PID_test(C++)主控,本节点挂掉不影响循迹,只丢红绿灯停车。

健壮性设计:
  - 重活(模型加载/开摄像头)在 rclpy 初始化之前完成
  - ROS 初始化失败自动重试 5 次
  - 识别异常全部 try/except 兜底

运行:
  python3 ~/racecar/src/racecar/scripts/vision_sidecar.py
"""
import os
import time

import cv2
import numpy as np

from cone_color import classify_color, hsv_make

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'best.pt')
GREEN_HSV = [((45, 60, 60), (85, 255, 255))]
LIGHT_ZONE_H = 0.35    # 画面上方 35% 视为红绿灯区域
PUB_INTERVAL = 1.0     # 发布间隔(秒)
CONF = 0.35
IMGSZ = 320
CAM_ID = 0


def load_model():
    from ultralytics import YOLO
    model = YOLO(MODEL_PATH)
    print('模型加载完成,类别: %s' % (getattr(model, 'names', None)), flush=True)
    return model


def open_camera():
    cap = cv2.VideoCapture(CAM_ID)
    if not cap.isOpened():
        print('警告: 摄像头打开失败', flush=True)
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap


def process_frame(model, frame):
    h, w = frame.shape[:2]
    red_cx, blue_cx = [], []
    light = 'none'
    try:
        results = model.predict(frame, imgsz=IMGSZ, conf=CONF, verbose=False)
        boxes = results[0].boxes
        if boxes is not None and len(boxes) > 0:
            for b in boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                cx = (x1 + x2) / 2.0 / w * 2.0 - 1.0
                roi = frame[y1:y2, x1:x2]
                color = classify_color(roi)
                cls_name = str(model.names[int(b.cls[0])]) if hasattr(model, 'names') else ''
                if y2 < int(h * LIGHT_ZONE_H):
                    # 只有 YOLO 识别为"灯"的物体才参与红绿灯判断,锥桶(cone)永不触发
                    if 'light' in cls_name:
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
    except Exception as ex:
        print('识别异常: %s' % ex, flush=True)

    red_side = 1 if red_cx and float(np.mean(red_cx)) > 0 else (-1 if red_cx else 0)
    blue_side = 1 if blue_cx and float(np.mean(blue_cx)) > 0 else (-1 if blue_cx else 0)
    return 'light:%s red_side:%d blue_side:%d red_n:%d blue_n:%d' % (
        light, red_side, blue_side, len(red_cx), len(blue_cx))


def main(args=None):
    # ---------- 先干重活(不涉及 ROS,避免破坏 rclpy 上下文) ----------
    model = load_model()
    cap = open_camera()

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    # ---------- 再初始化 ROS,失败自动重试 ----------
    node = None
    for attempt in range(5):
        try:
            rclpy.init(args=args)
            node = Node('vision_sidecar')
            pub = node.create_publisher(String, '/vision_status', 1)
            node.get_logger().info('vision_sidecar 启动  conf=%.2f imgsz=%d' % (CONF, IMGSZ))
            break
        except Exception as ex:
            print('ROS 初始化第 %d 次失败: %s' % (attempt + 1, ex), flush=True)
            try:
                rclpy.shutdown()
            except Exception:
                pass
            time.sleep(2)
    if node is None:
        print('ROS 初始化失败 5 次,退出(不影响雷达循迹)', flush=True)
        return

    try:
        fail_count = 0
        while rclpy.ok():
            t0 = time.time()
            ret, frame = cap.read()
            if not ret:
                fail_count += 1
                time.sleep(min(0.2 * fail_count, 5.0))   # 失败重试延时递增,上限 5s
                continue
            fail_count = 0
            status = process_frame(model, frame)
            msg = String()
            msg.data = status
            pub.publish(msg)
            node.get_logger().info('视觉 %s' % status)
            dt = time.time() - t0
            if dt < PUB_INTERVAL:
                time.sleep(PUB_INTERVAL - dt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
