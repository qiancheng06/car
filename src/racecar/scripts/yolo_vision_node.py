#!/usr/bin/env python3
"""ROS2 YOLO detector for parking signs, red lights and race cones.

This node publishes perception results only.  It never publishes a velocity,
arm or estop command, so it is safe to run while the chassis is disarmed.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from racecar.vision_logic import BinaryDebouncer, ClassVote, classify_cone_color


CLASS_ALIASES = {
    "parking_a": "parking_A",
    "parking_b": "parking_B",
    "red_light": "red_light",
    "redlight": "red_light",
    "cone": "cone",
}


class YoloVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_vision")
        default_model = os.path.expanduser(
            "~/racecar/vision_data/models/runs/v8_exp/weights/best.pt"
        )
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("inference_rate_hz", 5.0)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("torch_threads", 4)
        self.declare_parameter("base_confidence", 0.25)
        self.declare_parameter("iou_threshold", 0.55)
        self.declare_parameter("max_detections", 80)
        self.declare_parameter("parking_confidence", 0.65)
        self.declare_parameter("red_light_confidence", 0.65)
        self.declare_parameter("cone_confidence", 0.45)
        self.declare_parameter("parking_min_area_ratio", 0.0008)
        self.declare_parameter("red_light_min_area_ratio", 0.0003)
        self.declare_parameter("red_light_roi_top_ratio", 0.0)
        self.declare_parameter("red_light_roi_bottom_ratio", 0.80)
        self.declare_parameter("red_window", 5)
        self.declare_parameter("red_min_hits", 3)
        self.declare_parameter("red_clear_frames", 3)
        self.declare_parameter("parking_window", 7)
        self.declare_parameter("parking_min_votes", 4)
        self.declare_parameter("cone_min_color_ratio", 0.02)
        self.declare_parameter("cone_color_dominance", 1.25)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("log_interval_s", 5.0)

        model_path = os.path.expanduser(str(self.get_parameter("model_path").value))
        if not Path(model_path).is_file():
            raise FileNotFoundError(f"YOLO model not found: {model_path}")

        try:
            import torch
            from ultralytics import YOLO
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "Ultralytics/Torch is unavailable. Install the deployment dependencies first."
            ) from exc

        threads = max(1, int(self.get_parameter("torch_threads").value))
        torch.set_num_threads(threads)
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.names = self._normalized_names(self.model.names)
        required = {"parking_A", "parking_B", "red_light", "cone"}
        missing = required - set(self.names.values())
        if missing:
            raise RuntimeError(f"model is missing expected classes: {sorted(missing)}")

        self.bridge = CvBridge()
        self.red_filter = BinaryDebouncer(
            int(self.get_parameter("red_window").value),
            int(self.get_parameter("red_min_hits").value),
            int(self.get_parameter("red_clear_frames").value),
        )
        self.parking_filter = ClassVote(
            int(self.get_parameter("parking_window").value),
            int(self.get_parameter("parking_min_votes").value),
        )
        self.last_inference_started = 0.0
        self.last_log = 0.0
        self.processed_frames = 0
        self.last_parking: Optional[str] = None

        self.detections_pub = self.create_publisher(String, "/racecar/vision/detections", 5)
        self.status_pub = self.create_publisher(String, "/racecar/vision/status", 5)
        self.red_pub = self.create_publisher(Bool, "/racecar/red_light_detected", 5)
        self.parking_pub = self.create_publisher(String, "/racecar/parking_sign", 5)
        self.cones_pub = self.create_publisher(String, "/racecar/cones", 5)
        self.debug_pub = self.create_publisher(Image, "/racecar/vision/debug_image", 2)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            image_qos,
        )
        self.get_logger().info(
            "YOLO vision ready (perception only; no vehicle command publishers). "
            f"classes={self.names} device={self.get_parameter('device').value}"
        )

    @staticmethod
    def _normalized_names(names: Any) -> Dict[int, str]:
        items = names.items() if isinstance(names, dict) else enumerate(names)
        normalized: Dict[int, str] = {}
        for class_id, raw_name in items:
            key = str(raw_name).strip().lower().replace("-", "_").replace(" ", "_")
            normalized[int(class_id)] = CLASS_ALIASES.get(key, str(raw_name))
        return normalized

    def image_callback(self, msg: Image) -> None:
        started = time.perf_counter()
        rate = max(0.1, float(self.get_parameter("inference_rate_hz").value))
        if started - self.last_inference_started < 1.0 / rate:
            return
        self.last_inference_started = started

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            results = self.model.predict(
                source=frame,
                imgsz=int(self.get_parameter("imgsz").value),
                conf=float(self.get_parameter("base_confidence").value),
                iou=float(self.get_parameter("iou_threshold").value),
                max_det=int(self.get_parameter("max_detections").value),
                device=str(self.get_parameter("device").value),
                verbose=False,
            )[0]
            self.handle_result(frame, results, msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"vision inference failed: {exc}", throttle_duration_sec=5.0)
            return

        latency_ms = (time.perf_counter() - started) * 1000.0
        self.processed_frames += 1
        now = time.monotonic()
        if now - self.last_log >= float(self.get_parameter("log_interval_s").value):
            fps = 1000.0 / max(latency_ms, 0.001)
            self.get_logger().info(
                f"vision alive: frame={self.processed_frames} latency={latency_ms:.1f}ms "
                f"inference_fps={fps:.2f} red={self.red_filter.active} "
                f"parking={self.parking_filter.value or '-'}"
            )
            status = String()
            status.data = json.dumps(
                {
                    "ok": True,
                    "frames": self.processed_frames,
                    "latency_ms": round(latency_ms, 1),
                    "inference_fps": round(fps, 2),
                    "device": str(self.get_parameter("device").value),
                },
                ensure_ascii=False,
            )
            self.status_pub.publish(status)
            self.last_log = now

    def handle_result(self, frame: np.ndarray, result: Any, source_msg: Image) -> None:
        height, width = frame.shape[:2]
        frame_area = float(max(1, height * width))
        detections: List[Dict[str, Any]] = []
        cone_results: List[Dict[str, Any]] = []
        best_parking: Optional[Tuple[str, float]] = None
        red_seen = False

        boxes = result.boxes if result.boxes is not None else []
        for box in boxes:
            class_id = int(box.cls[0])
            class_name = self.names.get(class_id, str(class_id))
            confidence = float(box.conf[0])
            x1, y1, x2, y2 = [int(round(v)) for v in box.xyxy[0].tolist()]
            x1, x2 = max(0, x1), min(width, x2)
            y1, y2 = max(0, y1), min(height, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            area_ratio = ((x2 - x1) * (y2 - y1)) / frame_area
            accepted = False
            color: Optional[str] = None

            if class_name in ("parking_A", "parking_B"):
                accepted = confidence >= float(self.get_parameter("parking_confidence").value)
                accepted &= area_ratio >= float(self.get_parameter("parking_min_area_ratio").value)
                if accepted:
                    sign = "A" if class_name == "parking_A" else "B"
                    if best_parking is None or confidence > best_parking[1]:
                        best_parking = (sign, confidence)
            elif class_name == "red_light":
                center_y_ratio = ((y1 + y2) * 0.5) / max(1, height)
                accepted = confidence >= float(self.get_parameter("red_light_confidence").value)
                accepted &= area_ratio >= float(self.get_parameter("red_light_min_area_ratio").value)
                accepted &= float(self.get_parameter("red_light_roi_top_ratio").value) <= center_y_ratio
                accepted &= center_y_ratio <= float(
                    self.get_parameter("red_light_roi_bottom_ratio").value
                )
                red_seen |= accepted
            elif class_name == "cone":
                accepted = confidence >= float(self.get_parameter("cone_confidence").value)
                if accepted:
                    color = classify_cone_color(
                        frame[y1:y2, x1:x2],
                        float(self.get_parameter("cone_min_color_ratio").value),
                        float(self.get_parameter("cone_color_dominance").value),
                    )
                    cone_results.append(
                        {
                            "color": color,
                            "confidence": round(confidence, 4),
                            "center_x": round(((x1 + x2) * 0.5) / width, 4),
                            "center_y": round(((y1 + y2) * 0.5) / height, 4),
                            "area_ratio": round(area_ratio, 6),
                        }
                    )
            if not accepted:
                continue
            item: Dict[str, Any] = {
                "class": class_name,
                "confidence": round(confidence, 4),
                "box": [x1, y1, x2, y2],
                "area_ratio": round(area_ratio, 6),
            }
            if color is not None:
                item["color"] = color
            detections.append(item)

        red_active = self.red_filter.update(red_seen)
        parking = self.parking_filter.update(best_parking[0] if best_parking else None)

        red_msg = Bool()
        red_msg.data = red_active
        self.red_pub.publish(red_msg)
        if parking is not None:
            parking_msg = String()
            parking_msg.data = parking
            self.parking_pub.publish(parking_msg)
        cones_msg = String()
        cones_msg.data = json.dumps(cone_results, separators=(",", ":"))
        self.cones_pub.publish(cones_msg)

        output = String()
        output.data = json.dumps(
            {
                "stamp": {"sec": source_msg.header.stamp.sec, "nanosec": source_msg.header.stamp.nanosec},
                "image_size": [width, height],
                "red_light": red_active,
                "parking_sign": parking,
                "detections": detections,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.detections_pub.publish(output)

        if bool(self.get_parameter("publish_debug_image").value):
            self.publish_debug(frame, detections, red_active, parking, source_msg)

    def publish_debug(
        self,
        frame: np.ndarray,
        detections: List[Dict[str, Any]],
        red_active: bool,
        parking: Optional[str],
        source_msg: Image,
    ) -> None:
        debug = frame.copy()
        palette = {
            "parking_A": (0, 255, 0),
            "parking_B": (255, 0, 255),
            "red_light": (0, 0, 255),
            "cone": (255, 180, 0),
        }
        for det in detections:
            x1, y1, x2, y2 = det["box"]
            color = palette.get(det["class"], (0, 255, 255))
            label = f"{det['class']} {det['confidence']:.2f}"
            if "color" in det:
                label += f" {det['color']}"
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            cv2.putText(debug, label, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        cv2.putText(
            debug,
            f"RED={'YES' if red_active else 'no'}  PARK={parking or '-'}",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255) if red_active else (0, 255, 0),
            2,
        )
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = source_msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[YoloVisionNode] = None
    try:
        node = YoloVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
