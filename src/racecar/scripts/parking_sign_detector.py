#!/usr/bin/env python3
"""Template-based A/B parking sign detector.

The competition rule provides printable A/B sign source files.  After printing
and mounting them, save one clear camera crop of each sign and pass the crop
paths through parameters.  The detector publishes "A" or "B" to
/racecar/parking_sign when a stable match is observed.
"""

from __future__ import annotations

import os
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ParkingSignDetector(Node):
    def __init__(self) -> None:
        super().__init__("parking_sign_detector")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("parking_sign_topic", "/racecar/parking_sign")
        self.declare_parameter("template_a_path", "")
        self.declare_parameter("template_b_path", "")
        self.declare_parameter("score_threshold", 0.45)
        self.declare_parameter("stable_frames", 3)
        self.declare_parameter("roi_top_ratio", 0.0)
        self.declare_parameter("roi_bottom_ratio", 1.0)
        self.declare_parameter("debug_view", False)

        self.bridge = CvBridge()
        self.template_a = self.load_template("template_a_path")
        self.template_b = self.load_template("template_b_path")
        self.last_vote: Optional[str] = None
        self.vote_count = 0

        self.publisher = self.create_publisher(
            String, str(self.get_parameter("parking_sign_topic").value), 5
        )
        self.subscriber = self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self.image_callback, 5
        )

        if self.template_a is None or self.template_b is None:
            self.get_logger().warn(
                "A/B template paths are not both configured. The detector is running "
                "but will not publish until template_a_path and template_b_path are set."
            )
        else:
            self.get_logger().info("Parking sign detector ready.")

    def load_template(self, parameter_name: str) -> Optional[np.ndarray]:
        path = str(self.get_parameter(parameter_name).value)
        if not path:
            return None
        expanded = os.path.expanduser(path)
        if not os.path.exists(expanded):
            self.get_logger().warn(f"Template path does not exist: {expanded}")
            return None
        image = cv2.imread(expanded, cv2.IMREAD_GRAYSCALE)
        if image is None:
            self.get_logger().warn(f"Failed to read template: {expanded}")
            return None
        return cv2.equalizeHist(image)

    def image_callback(self, msg: Image) -> None:
        if self.template_a is None or self.template_b is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Image conversion failed: {exc}")
            return

        h = frame.shape[0]
        top = int(h * float(self.get_parameter("roi_top_ratio").value))
        bottom = int(h * float(self.get_parameter("roi_bottom_ratio").value))
        roi = frame[max(0, top) : min(h, bottom), :]
        gray = cv2.equalizeHist(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY))

        score_a = self.match_template(gray, self.template_a)
        score_b = self.match_template(gray, self.template_b)
        best_score = max(score_a, score_b)
        if best_score < float(self.get_parameter("score_threshold").value):
            self.last_vote = None
            self.vote_count = 0
            return

        vote = "A" if score_a >= score_b else "B"
        if vote == self.last_vote:
            self.vote_count += 1
        else:
            self.last_vote = vote
            self.vote_count = 1

        if self.vote_count >= int(self.get_parameter("stable_frames").value):
            msg_out = String()
            msg_out.data = vote
            self.publisher.publish(msg_out)
            self.get_logger().info(
                f"Parking sign detected: {vote} (A={score_a:.2f}, B={score_b:.2f})"
            )

        if bool(self.get_parameter("debug_view").value):
            cv2.putText(
                roi,
                f"A={score_a:.2f} B={score_b:.2f} vote={vote}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.imshow("parking_sign_detector", roi)
            cv2.waitKey(1)

    @staticmethod
    def match_template(gray: np.ndarray, template: np.ndarray) -> float:
        if template.shape[0] >= gray.shape[0] or template.shape[1] >= gray.shape[1]:
            return 0.0
        scales = (0.65, 0.8, 1.0, 1.2, 1.4)
        best = 0.0
        for scale in scales:
            resized = cv2.resize(template, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            if resized.shape[0] >= gray.shape[0] or resized.shape[1] >= gray.shape[1]:
                continue
            result = cv2.matchTemplate(gray, resized, cv2.TM_CCOEFF_NORMED)
            best = max(best, float(result.max()))
        return best


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingSignDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
