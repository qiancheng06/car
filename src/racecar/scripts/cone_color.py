import cv2
import sys

import numpy as np

RED_HSV = [((0, 60, 60), (12, 255, 255)), ((170, 60, 60), (180, 255, 255))]
BLUE_HSV = [((100, 60, 60), (130, 255, 255))]
RATIO = 1.3


def hsv_make(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return mask


def classify_color(roi_bgr):
    """判断检测框内锥桶颜色，返回 'red' / 'blue' / 'unknown'"""
    if roi_bgr is None or roi_bgr.size == 0:
        return "unknown"
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    red_px = int(hsv_make(hsv, RED_HSV).sum())
    blue_px = int(hsv_make(hsv, BLUE_HSV).sum())
    total = red_px + blue_px
    if total == 0:
        return "unknown"
    if red_px > blue_px * RATIO:
        return "red"
    if blue_px > red_px * RATIO:
        return "blue"
    return "unknown"


def classify_image(path, box=None):
    """整图判断，或对 box=(x1,y1,x2,y2) 区域判断"""
    img = cv2.imread(path)
    if img is None:
        return "unknown"
    if box is not None:
        x1, y1, x2, y2 = map(int, box)
        img = img[y1:y2, x1:x2]
    return classify_color(img)


def main():
    if len(sys.argv) < 2:
        print("用法: python cone_color.py 图片路径 [x1 y1 x2 y2]")
        return
    path = sys.argv[1]
    box = None
    if len(sys.argv) == 6:
        box = tuple(map(float, sys.argv[2:6]))
    print(classify_image(path, box))


if __name__ == "__main__":
    main()
