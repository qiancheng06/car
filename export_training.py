#!/usr/bin/env python3
"""把训练 bag 导出为图片序列 + labels.csv，方便在任意机器上训练。

直接用 sqlite3 读 db3，不依赖 metadata.yaml 和 rosbag2 插件。
用法: python3 export_training.py <bag目录或db3文件> [输出目录]
输出: frames/000000.jpg ... 和 labels.csv(frame,stamp_ns,cmd_stamp_ns,throttle_pwm,steering_deg)
控制量取图像时间戳前 120ms 内最近一条 /teleop_cmd_vel。
"""
import csv
import os
import sqlite3
import sys

import cv2
import numpy as np
from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

MAX_CMD_LAG_NS = 120_000_000


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    bag = sys.argv[1].rstrip("/")
    if bag.endswith(".db3"):
        db = bag
        out = sys.argv[2] if len(sys.argv) > 2 else bag[:-4] + "_export"
    else:
        db = os.path.join(bag, "bag_0.db3")
        out = sys.argv[2] if len(sys.argv) > 2 else bag + "_export"
    frames_dir = os.path.join(out, "frames")
    os.makedirs(frames_dir, exist_ok=True)

    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    topics = {tid: name for tid, name in con.execute("SELECT id, name FROM topics")}
    img_tid = next((tid for tid, name in topics.items() if name == "/image_raw/compressed"), None)
    cmd_tid = next((tid for tid, name in topics.items() if name == "/teleop_cmd_vel"), None)
    if img_tid is None:
        print("db3 里没有 /image_raw/compressed 话题")
        sys.exit(1)

    rows = []
    cmds = []  # (ts_ns, throttle_pwm, steering_deg)
    idx = 0
    for tid, data, ts in con.execute(
        "SELECT topic_id, data, timestamp FROM messages ORDER BY timestamp"
    ):
        if cmd_tid is not None and tid == cmd_tid:
            m = deserialize_message(bytes(data), Twist)
            cmds.append((ts, m.linear.x, m.angular.z))
        elif tid == img_tid:
            m = deserialize_message(bytes(data), CompressedImage)
            img = cv2.imdecode(np.frombuffer(m.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            name = f"{idx:06d}.jpg"
            cv2.imwrite(os.path.join(frames_dir, name), img)
            cmd_ts = pwm = deg = ""
            for cts, cp, cd in reversed(cmds):
                if cts <= ts and ts - cts <= MAX_CMD_LAG_NS:
                    cmd_ts, pwm, deg = cts, cp, cd
                    break
            rows.append((name, ts, cmd_ts, pwm, deg))
            idx += 1
    con.close()

    with open(os.path.join(out, "labels.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "stamp_ns", "cmd_stamp_ns", "throttle_pwm", "steering_deg"])
        w.writerows(rows)
    print(f"导出完成: {idx} 帧 -> {out}")


if __name__ == "__main__":
    main()
