#!/usr/bin/env python3
"""HSV 实时调参工具: 订阅 /image_raw, 6 滑块实时预览掩膜, 按 q 退出并打印参数"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
import cv_bridge

WIN = "HSV_Calib"
SWITCH = "Preset"

PRESETS = {
    "yellow": (26, 43, 46, 34, 255, 255),
    "black":  (0, 0, 0, 180, 255, 46),
    "red":    (0, 100, 80, 10, 255, 255),
    "green":  (35, 43, 46, 77, 255, 255),
    "blue":   (100, 43, 46, 124, 255, 255),
}

def nothing(x):
    pass

class HsvCalib(Node):
    def __init__(self):
        super().__init__("hsv_calib")
        self.bridge = cv_bridge.CvBridge()
        qos = QoSProfile(depth=10)
        self.create_subscription(Image, "/image_raw", self.cb, qos)
        self.frame = None
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.createTrackbar("H_min", WIN, 0, 179, nothing)
        cv2.createTrackbar("H_max", WIN, 179, 179, nothing)
        cv2.createTrackbar("S_min", WIN, 0, 255, nothing)
        cv2.createTrackbar("S_max", WIN, 255, 255, nothing)
        cv2.createTrackbar("V_min", WIN, 0, 255, nothing)
        cv2.createTrackbar("V_max", WIN, 255, 255, nothing)
        cv2.createTrackbar(SWITCH, WIN, 0, len(PRESETS) - 1, nothing)

    def cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.frame is None:
                continue
            img = self.frame
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            p = cv2.getTrackbarPos(SWITCH, WIN)
            if p < len(PRESETS):
                name = list(PRESETS.keys())[p]
                l, s, v, uh, us, uv = PRESETS[name]
                cv2.setTrackbarPos("H_min", WIN, l)
                cv2.setTrackbarPos("H_max", WIN, uh)
                cv2.setTrackbarPos("S_min", WIN, s)
                cv2.setTrackbarPos("S_max", WIN, us)
                cv2.setTrackbarPos("V_min", WIN, v)
                cv2.setTrackbarPos("V_max", WIN, uv)
            lower = np.array([cv2.getTrackbarPos("H_min", WIN),
                              cv2.getTrackbarPos("S_min", WIN),
                              cv2.getTrackbarPos("V_min", WIN)], dtype=np.uint8)
            upper = np.array([cv2.getTrackbarPos("H_max", WIN),
                              cv2.getTrackbarPos("S_max", WIN),
                              cv2.getTrackbarPos("V_max", WIN)], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            preview = np.hstack([img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
            cv2.imshow(WIN, preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("FINAL_HSV = (%d, %d, %d, %d, %d, %d)" % tuple(lower) + tuple(upper))
                break

def main():
    rclpy.init()
    node = HsvCalib()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
