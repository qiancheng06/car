#!/usr/bin/env python3
"""Track-center estimator from colour-classified cone detections.

The track runs counterclockwise: the inner row is red cones and the outer row
is blue cones, so with the camera facing forward the red row is on the left
and the blue row on the right.  Subscribing to /racecar/cones (JSON list from
the YOLO/NPU vision nodes) this node estimates where the track center lies in
the image and publishes the normalized offset from the image center.

``error`` follows the same convention as the yellow band signal: positive
means the track center is to the RIGHT of the image center, i.e. the car is
left of the track center and needs to steer right.  The signal is meant as a
cross-check / trim input for the lidar follower, never a direct command.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TrackCenterNode(Node):
    def __init__(self) -> None:
        super().__init__("track_center")
        self.declare_parameter("cones_topic", "/racecar/cones")
        self.declare_parameter("track_center_topic", "/racecar/track_center")
        self.declare_parameter("inner_color", "red")
        self.declare_parameter("outer_color", "blue")
        self.declare_parameter("min_cones_per_row", 1)
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("single_row_bias", 0.12)
        self.declare_parameter("max_valid_offset", 0.45)

        self.result_pub = self.create_publisher(
            String, str(self.get_parameter("track_center_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("cones_topic").value),
            self.cones_callback,
            10,
        )
        self.smoothed_error: float | None = None
        self.get_logger().info("Track center estimator ready")

    def cones_callback(self, msg: String) -> None:
        try:
            cones = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(cones, list):
            return

        inner = str(self.get_parameter("inner_color").value)
        outer = str(self.get_parameter("outer_color").value)
        min_per_row = int(self.get_parameter("min_cones_per_row").value)
        max_offset = float(self.get_parameter("max_valid_offset").value)

        inner_x = [
            float(c["center_x"]) for c in cones
            if isinstance(c, dict) and c.get("color") == inner and "center_x" in c
        ]
        outer_x = [
            float(c["center_x"]) for c in cones
            if isinstance(c, dict) and c.get("color") == outer and "center_x" in c
        ]

        result = {
            "detected": False,
            "rows": 0,
            "error": 0.0,
            "inner_x": round(sum(inner_x) / len(inner_x), 4) if inner_x else None,
            "outer_x": round(sum(outer_x) / len(outer_x), 4) if outer_x else None,
            "n_inner": len(inner_x),
            "n_outer": len(outer_x),
        }

        have_inner = len(inner_x) >= min_per_row
        have_outer = len(outer_x) >= min_per_row
        if have_inner and have_outer:
            # Track center is the midpoint between the two rows in the image.
            raw = (sum(inner_x) / len(inner_x) + sum(outer_x) / len(outer_x)) / 2.0 - 0.5
            rows = 2
        elif have_outer:
            # Only the outer row visible: the car hugs the outer edge.
            raw = -float(self.get_parameter("single_row_bias").value)
            rows = 1
        elif have_inner:
            # Only the inner row visible: the car hugs the inner edge.
            raw = float(self.get_parameter("single_row_bias").value)
            rows = 1
        else:
            raw = None
            rows = 0

        if raw is not None:
            raw = max(-max_offset, min(max_offset, raw))
            alpha = min(1.0, max(0.0, float(self.get_parameter("smoothing_alpha").value)))
            if self.smoothed_error is None:
                self.smoothed_error = raw
            else:
                self.smoothed_error = alpha * raw + (1.0 - alpha) * self.smoothed_error
            result["detected"] = True
            result["rows"] = rows
            result["error"] = round(self.smoothed_error, 4)

        output = String()
        output.data = json.dumps(result, separators=(",", ":"))
        self.result_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackCenterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
