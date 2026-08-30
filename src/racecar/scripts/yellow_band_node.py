#!/usr/bin/env python3
"""Model-independent yellow track-band detector for the racecar chain."""

from __future__ import annotations

import json

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from racecar.vision_logic import detect_yellow_band


class YellowBandNode(Node):
    def __init__(self) -> None:
        super().__init__("yellow_band_detector")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("yellow_band_topic", "/racecar/yellow_band")
        self.declare_parameter("debug_topic", "/racecar/yellow_band/debug_image")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("process_rate_hz", 15.0)
        self.declare_parameter("roi_top_ratio", 0.45)
        self.declare_parameter("roi_bottom_ratio", 1.0)
        self.declare_parameter("min_area_ratio", 0.012)
        self.declare_parameter("min_width_ratio", 0.08)
        self.declare_parameter("hue_min", 18)
        self.declare_parameter("hue_max", 38)
        self.declare_parameter("saturation_min", 80)
        self.declare_parameter("value_min", 80)

        self.bridge = CvBridge()
        self.last_process_time = 0.0
        self.result_pub = self.create_publisher(
            String, str(self.get_parameter("yellow_band_topic").value), 10
        )
        self.debug_pub = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), 2
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            10,
        )
        self.get_logger().info("Yellow band detector ready")

    def image_callback(self, msg: Image) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        rate = max(1.0, float(self.get_parameter("process_rate_hz").value))
        if now - self.last_process_time < 1.0 / rate:
            return
        self.last_process_time = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Yellow band image conversion failed: {exc}")
            return

        params = {
            "roi_top_ratio": float(self.get_parameter("roi_top_ratio").value),
            "roi_bottom_ratio": float(self.get_parameter("roi_bottom_ratio").value),
            "min_area_ratio": float(self.get_parameter("min_area_ratio").value),
            "min_width_ratio": float(self.get_parameter("min_width_ratio").value),
            "hue_min": int(self.get_parameter("hue_min").value),
            "hue_max": int(self.get_parameter("hue_max").value),
            "saturation_min": int(self.get_parameter("saturation_min").value),
            "value_min": int(self.get_parameter("value_min").value),
        }
        result = detect_yellow_band(frame, **params)
        output = String()
        output.data = json.dumps(result, separators=(",", ":"))
        self.result_pub.publish(output)

        if bool(self.get_parameter("publish_debug_image").value):
            debug = frame.copy()
            height, width = debug.shape[:2]
            top = int(height * params["roi_top_ratio"])
            cv2.line(debug, (0, top), (width, top), (0, 255, 255), 2)

            # Draw every usable yellow contour so the operator can tune HSV
            # thresholds from the real camera image.  The detector itself
            # still selects the strongest contour for its numeric result.
            roi = frame[top:height, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(
                hsv,
                (params["hue_min"], params["saturation_min"], params["value_min"]),
                (params["hue_max"], 255, 255),
            )
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) < params["min_area_ratio"] * max(1, roi.shape[0] * roi.shape[1]):
                    continue
                contour = contour.copy()
                contour[:, 0, 1] += top
                cv2.drawContours(debug, [contour], -1, (0, 165, 255), 3)

            if bool(result["detected"]):
                center = int(float(result["center_x_ratio"]) * width)
                cv2.line(debug, (center, top), (center, height), (0, 255, 0), 3)
                x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea)) if contours else (0, 0, 0, 0)
                if w and h:
                    cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
                label = f"YELLOW error={float(result['error']):+.3f} area={float(result['area_ratio']):.3f}"
            else:
                label = "YELLOW not detected"
            cv2.putText(debug, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.putText(debug, "orange=yellow mask  green=center", (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YellowBandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
