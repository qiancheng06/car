#!/usr/bin/env python3
"""Ascend ACL ROS2 detector for the Atlas 200I A2 racecar.

The process needs access to /dev/davinci*, so start this node through sudo.
It publishes perception only and never publishes vehicle commands.
"""

from __future__ import annotations

import json
import gc
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from racecar.vision_logic import BinaryDebouncer, ClassVote, classify_cone_color


CLASS_NAMES = ("parking_A", "parking_B", "red_light", "cone")


def letterbox(image: np.ndarray, size: int = 640) -> Tuple[np.ndarray, float, float, float]:
    """YOLO letterbox preprocessing; returns NCHW float32 RGB input."""
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    resized_w = max(1, int(round(width * scale)))
    resized_h = max(1, int(round(height * scale)))
    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    pad_x = (size - resized_w) / 2.0
    pad_y = (size - resized_h) / 2.0
    left, top = int(round(pad_x - 0.1)), int(round(pad_y - 0.1))
    right, bottom = size - resized_w - left, size - resized_h - top
    canvas = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
    tensor = canvas[:, :, ::-1].transpose(2, 0, 1)
    tensor = np.ascontiguousarray(tensor, dtype=np.float32) / 255.0
    return tensor[None, ...], scale, float(left), float(top)


def nms(boxes: np.ndarray, scores: np.ndarray, threshold: float) -> List[int]:
    if boxes.size == 0:
        return []
    order = scores.argsort()[::-1]
    keep: List[int] = []
    while order.size:
        index = int(order[0])
        keep.append(index)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[index, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[index, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[index, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[index, 3], boxes[rest, 3])
        intersection = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        area_a = np.maximum(0.0, boxes[index, 2] - boxes[index, 0]) * np.maximum(
            0.0, boxes[index, 3] - boxes[index, 1]
        )
        area_b = np.maximum(0.0, boxes[rest, 2] - boxes[rest, 0]) * np.maximum(
            0.0, boxes[rest, 3] - boxes[rest, 1]
        )
        iou = intersection / np.maximum(area_a + area_b - intersection, 1e-9)
        order = rest[iou <= threshold]
    return keep


def decode_yolov8(
    output: np.ndarray,
    scale: float,
    pad_x: float,
    pad_y: float,
    image_width: int,
    image_height: int,
    confidence: float,
    iou_threshold: float,
    max_detections: int,
) -> List[Tuple[int, float, Tuple[int, int, int, int]]]:
    prediction = np.asarray(output, dtype=np.float32)
    if prediction.ndim == 3:
        prediction = prediction[0]
    if prediction.shape[0] == 4 + len(CLASS_NAMES):
        prediction = prediction.T
    if prediction.ndim != 2 or prediction.shape[1] != 4 + len(CLASS_NAMES):
        raise ValueError(f"unexpected YOLO output shape: {output.shape}")

    class_scores = prediction[:, 4:]
    class_ids = class_scores.argmax(axis=1)
    scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
    selected = scores >= confidence
    if not np.any(selected):
        return []
    raw_boxes = prediction[selected, :4]
    scores = scores[selected]
    class_ids = class_ids[selected]

    boxes = np.empty_like(raw_boxes, dtype=np.float32)
    boxes[:, 0] = (raw_boxes[:, 0] - raw_boxes[:, 2] / 2.0 - pad_x) / scale
    boxes[:, 1] = (raw_boxes[:, 1] - raw_boxes[:, 3] / 2.0 - pad_y) / scale
    boxes[:, 2] = (raw_boxes[:, 0] + raw_boxes[:, 2] / 2.0 - pad_x) / scale
    boxes[:, 3] = (raw_boxes[:, 1] + raw_boxes[:, 3] / 2.0 - pad_y) / scale
    boxes[:, (0, 2)] = boxes[:, (0, 2)].clip(0, image_width - 1)
    boxes[:, (1, 3)] = boxes[:, (1, 3)].clip(0, image_height - 1)

    detections: List[Tuple[int, float, Tuple[int, int, int, int]]] = []
    for class_id in np.unique(class_ids):
        indices = np.flatnonzero(class_ids == class_id)
        kept = nms(boxes[indices], scores[indices], iou_threshold)
        for local_index in kept:
            index = int(indices[local_index])
            box = tuple(int(round(value)) for value in boxes[index])
            detections.append((int(class_id), float(scores[index]), box))
    detections.sort(key=lambda item: item[1], reverse=True)
    return detections[:max_detections]


class AscendModel:
    def __init__(self, path: str, device_id: int = 0) -> None:
        acllite_dir = "/usr/local/Ascend/thirdpart/aarch64/acllite"
        if acllite_dir not in sys.path:
            sys.path.insert(0, acllite_dir)
        from acllite_model import AclLiteModel
        from acllite_resource import AclLiteResource

        self.resource = AclLiteResource(device_id=device_id)
        self.resource.init()
        self.model = AclLiteModel(path)

    def infer(self, tensor: np.ndarray) -> np.ndarray:
        outputs = self.model.execute([tensor])
        if not outputs:
            raise RuntimeError("ACL model returned no output")
        return outputs[0]

    def close(self) -> None:
        if self.model is not None:
            self.model.destroy()
            self.model = None
        if self.resource is not None:
            resource = self.resource
            self.resource = None
            del resource
            gc.collect()


class NpuVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("npu_vision")
        default_model = os.path.expanduser(
            "~/racecar/vision_data/models/runs/v8_exp/weights/best_310b.om"
        )
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("input_size", 640)
        self.declare_parameter("inference_rate_hz", 15.0)
        self.declare_parameter("device_id", 0)
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
            raise FileNotFoundError(f"OM model not found: {model_path}")
        self.get_logger().info(f"Loading Ascend OM model: {model_path}")
        self.backend = AscendModel(model_path, int(self.get_parameter("device_id").value))
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
        self.last_log_frames = 0
        self.processed_frames = 0

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
        self.get_logger().info("Ascend NPU vision ready (perception only; no vehicle commands).")

    def image_callback(self, msg: Image) -> None:
        started = time.perf_counter()
        rate = max(0.1, float(self.get_parameter("inference_rate_hz").value))
        if started - self.last_inference_started < 1.0 / rate:
            return
        self.last_inference_started = started
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            height, width = frame.shape[:2]
            tensor, scale, pad_x, pad_y = letterbox(
                frame, int(self.get_parameter("input_size").value)
            )
            output = self.backend.infer(tensor)
            raw = decode_yolov8(
                output,
                scale,
                pad_x,
                pad_y,
                width,
                height,
                float(self.get_parameter("base_confidence").value),
                float(self.get_parameter("iou_threshold").value),
                int(self.get_parameter("max_detections").value),
            )
            detections, red_active, parking, cones = self.filter_detections(frame, raw)
            self.publish_results(msg, frame, detections, red_active, parking, cones)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"NPU vision inference failed: {exc}", throttle_duration_sec=5.0)
            return

        latency_ms = (time.perf_counter() - started) * 1000.0
        self.processed_frames += 1
        now = time.monotonic()
        if now - self.last_log >= float(self.get_parameter("log_interval_s").value):
            fps = 1000.0 / max(latency_ms, 0.001)
            elapsed = now - self.last_log if self.last_log > 0.0 else 0.0
            pipeline_fps = (
                (self.processed_frames - self.last_log_frames) / elapsed if elapsed > 0.0 else 0.0
            )
            status = {
                "ok": True,
                "frames": self.processed_frames,
                "latency_ms": round(latency_ms, 1),
                "capacity_fps": round(fps, 2),
                "pipeline_fps": round(pipeline_fps, 2),
                "device": "ascend_npu",
            }
            self.get_logger().info(
                f"NPU vision alive: frame={self.processed_frames} latency={latency_ms:.1f}ms "
                f"capacity_fps={fps:.2f} pipeline_fps={pipeline_fps:.2f} "
                f"red={self.red_filter.active} parking={self.parking_filter.value or '-'}"
            )
            status_msg = String()
            status_msg.data = json.dumps(status, separators=(",", ":"))
            self.status_pub.publish(status_msg)
            self.last_log = now
            self.last_log_frames = self.processed_frames

    def filter_detections(self, frame, raw):
        height, width = frame.shape[:2]
        frame_area = float(max(1, height * width))
        accepted: List[Dict[str, Any]] = []
        cones: List[Dict[str, Any]] = []
        red_seen = False
        best_parking: Optional[Tuple[str, float]] = None
        for class_id, confidence, box in raw:
            class_name = CLASS_NAMES[class_id]
            x1, y1, x2, y2 = box
            area_ratio = max(0, x2 - x1) * max(0, y2 - y1) / frame_area
            color: Optional[str] = None
            keep = False
            if class_name in ("parking_A", "parking_B"):
                keep = confidence >= float(self.get_parameter("parking_confidence").value)
                keep &= area_ratio >= float(self.get_parameter("parking_min_area_ratio").value)
                if keep:
                    sign = "A" if class_name == "parking_A" else "B"
                    if best_parking is None or confidence > best_parking[1]:
                        best_parking = (sign, confidence)
            elif class_name == "red_light":
                center_y = (y1 + y2) * 0.5 / height
                keep = confidence >= float(self.get_parameter("red_light_confidence").value)
                keep &= area_ratio >= float(self.get_parameter("red_light_min_area_ratio").value)
                keep &= float(self.get_parameter("red_light_roi_top_ratio").value) <= center_y
                keep &= center_y <= float(self.get_parameter("red_light_roi_bottom_ratio").value)
                red_seen |= keep
            elif class_name == "cone":
                keep = confidence >= float(self.get_parameter("cone_confidence").value)
                if keep:
                    color = classify_cone_color(
                        frame[y1:y2, x1:x2],
                        float(self.get_parameter("cone_min_color_ratio").value),
                        float(self.get_parameter("cone_color_dominance").value),
                    )
                    cones.append(
                        {
                            "color": color,
                            "confidence": round(confidence, 4),
                            "center_x": round((x1 + x2) * 0.5 / width, 4),
                            "center_y": round((y1 + y2) * 0.5 / height, 4),
                            "area_ratio": round(area_ratio, 6),
                        }
                    )
            if keep:
                item = {
                    "class": class_name,
                    "confidence": round(confidence, 4),
                    "box": [x1, y1, x2, y2],
                    "area_ratio": round(area_ratio, 6),
                }
                if color is not None:
                    item["color"] = color
                accepted.append(item)
        red_active = self.red_filter.update(red_seen)
        parking = self.parking_filter.update(best_parking[0] if best_parking else None)
        return accepted, red_active, parking, cones

    def publish_results(self, source_msg, frame, detections, red_active, parking, cones):
        red_msg = Bool()
        red_msg.data = red_active
        self.red_pub.publish(red_msg)
        if parking is not None:
            parking_msg = String()
            parking_msg.data = parking
            self.parking_pub.publish(parking_msg)
        cones_msg = String()
        cones_msg.data = json.dumps(cones, separators=(",", ":"))
        self.cones_pub.publish(cones_msg)
        output = String()
        output.data = json.dumps(
            {
                "stamp": {"sec": source_msg.header.stamp.sec, "nanosec": source_msg.header.stamp.nanosec},
                "image_size": [frame.shape[1], frame.shape[0]],
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

    def publish_debug(self, frame, detections, red_active, parking, source_msg):
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
            f"NPU RED={'YES' if red_active else 'no'} PARK={parking or '-'}",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255) if red_active else (0, 255, 0),
            2,
        )
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = source_msg.header
        self.debug_pub.publish(debug_msg)

    def close(self) -> None:
        self.backend.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[NpuVisionNode] = None
    try:
        node = NpuVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
