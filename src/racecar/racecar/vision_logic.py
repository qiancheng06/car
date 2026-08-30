"""Small, ROS-independent helpers used by the racecar vision node."""

from __future__ import annotations

from collections import Counter, deque
from typing import Deque, Dict, Optional

import cv2
import numpy as np


class BinaryDebouncer:
    """Confirm a detection across several frames and clear it conservatively."""

    def __init__(self, window: int = 5, min_hits: int = 3, clear_frames: int = 3) -> None:
        if window < 1 or not 1 <= min_hits <= window or clear_frames < 1:
            raise ValueError("invalid debouncer parameters")
        self._samples: Deque[bool] = deque(maxlen=window)
        self._min_hits = min_hits
        self._clear_frames = clear_frames
        self._misses = 0
        self.active = False

    def update(self, detected: bool) -> bool:
        detected = bool(detected)
        self._samples.append(detected)
        if detected:
            self._misses = 0
        else:
            self._misses += 1

        if not self.active and sum(self._samples) >= self._min_hits:
            self.active = True
        elif self.active and self._misses >= self._clear_frames:
            self.active = False
            self._samples.clear()
        return self.active


class ClassVote:
    """Sliding vote for mutually exclusive A/B detections."""

    def __init__(self, window: int = 7, min_votes: int = 4) -> None:
        if window < 1 or not 1 <= min_votes <= window:
            raise ValueError("invalid vote parameters")
        self._votes: Deque[Optional[str]] = deque(maxlen=window)
        self._min_votes = min_votes
        self.value: Optional[str] = None

    def update(self, value: Optional[str]) -> Optional[str]:
        value = value.upper() if value else None
        if value not in (None, "A", "B"):
            raise ValueError(f"unsupported vote: {value}")
        self._votes.append(value)
        counts = Counter(v for v in self._votes if v is not None)
        if counts:
            winner, votes = counts.most_common(1)[0]
            other = "B" if winner == "A" else "A"
            if votes >= self._min_votes and votes > counts.get(other, 0):
                self.value = winner
                return self.value
        if len(self._votes) == self._votes.maxlen and all(v is None for v in self._votes):
            self.value = None
        return self.value


def classify_cone_color(
    roi_bgr: np.ndarray,
    min_color_ratio: float = 0.02,
    dominance_ratio: float = 1.25,
) -> str:
    """Return red/blue/unknown without forcing an ambiguous crop into a color."""

    if roi_bgr is None or roi_bgr.size == 0:
        return "unknown"
    height, width = roi_bgr.shape[:2]
    if height < 3 or width < 3:
        return "unknown"

    # Discard a small border because a YOLO box normally contains background.
    y_margin = max(1, int(height * 0.08))
    x_margin = max(1, int(width * 0.08))
    inner = roi_bgr[y_margin : height - y_margin, x_margin : width - x_margin]
    if inner.size == 0:
        inner = roi_bgr

    hsv = cv2.cvtColor(inner, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, np.array((0, 70, 55)), np.array((13, 255, 255)))
    red |= cv2.inRange(hsv, np.array((168, 70, 55)), np.array((180, 255, 255)))
    blue = cv2.inRange(hsv, np.array((92, 65, 45)), np.array((135, 255, 255)))
    pixels = float(max(1, inner.shape[0] * inner.shape[1]))
    red_ratio = cv2.countNonZero(red) / pixels
    blue_ratio = cv2.countNonZero(blue) / pixels
    strongest = max(red_ratio, blue_ratio)
    weakest = min(red_ratio, blue_ratio)

    if strongest < min_color_ratio:
        return "unknown"
    if weakest > 0.0 and strongest / weakest < dominance_ratio:
        return "unknown"
    return "red" if red_ratio > blue_ratio else "blue"


def detect_yellow_band(
    frame_bgr: np.ndarray,
    roi_top_ratio: float = 0.45,
    roi_bottom_ratio: float = 1.0,
    min_area_ratio: float = 0.012,
    min_width_ratio: float = 0.08,
    hue_min: int = 18,
    hue_max: int = 38,
    saturation_min: int = 80,
    value_min: int = 80,
) -> Dict[str, float | bool]:
    """Detect a yellow track band in the lower camera ROI.

    ``error`` is the normalized horizontal offset from image center, positive
    when the detected band is to the right. The detector is model-independent
    so CPU YOLO and Atlas NPU deployments use the same tracking signal.
    """
    result: Dict[str, float | bool] = {
        "detected": False,
        "error": 0.0,
        "center_x_ratio": 0.5,
        "area_ratio": 0.0,
        "width_ratio": 0.0,
    }
    if frame_bgr is None or frame_bgr.size == 0 or frame_bgr.ndim != 3:
        return result

    height, width = frame_bgr.shape[:2]
    if height < 8 or width < 8:
        return result
    top = max(0, min(height - 1, int(height * roi_top_ratio)))
    bottom = max(top + 1, min(height, int(height * roi_bottom_ratio)))
    roi = frame_bgr[top:bottom, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array((hue_min, saturation_min, value_min), dtype=np.uint8),
        np.array((hue_max, 255, 255), dtype=np.uint8),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return result

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    area_ratio = area / float(max(1, roi.shape[0] * roi.shape[1]))
    _, _, box_width, _ = cv2.boundingRect(contour)
    width_ratio = box_width / float(width)
    result["area_ratio"] = area_ratio
    result["width_ratio"] = width_ratio
    if area_ratio < min_area_ratio or width_ratio < min_width_ratio:
        return result

    moments = cv2.moments(contour)
    if moments["m00"] <= 0.0:
        return result
    center_x_ratio = float(moments["m10"] / moments["m00"]) / float(width)
    result.update(
        detected=True,
        center_x_ratio=center_x_ratio,
        error=center_x_ratio - 0.5,
    )
    return result
