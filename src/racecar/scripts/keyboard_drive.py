#!/usr/bin/env python3
"""Low-speed, dead-man keyboard control for the physical racecar."""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


NEUTRAL_PWM = 1500.0
STEERING_CENTER_DEG = 90.0
STEERING_LEFT_DEG = 115.0
STEERING_RIGHT_DEG = 65.0
MIN_POWER = 50
MAX_POWER = 150
POWER_STEP = 10
DEADMAN_SECONDS = 0.8


def read_key(timeout_sec=0.05):
    readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not readable:
        return ""
    first = os.read(sys.stdin.fileno(), 1).decode(errors="ignore")
    if first != "\x1b":
        return first
    sequence = first
    end = time.monotonic() + 0.03
    while time.monotonic() < end and len(sequence) < 3:
        readable, _, _ = select.select([sys.stdin], [], [], 0.005)
        if readable:
            sequence += os.read(sys.stdin.fileno(), 1).decode(errors="ignore")
    return sequence


class KeyboardDrive(Node):
    def __init__(self):
        super().__init__("keyboard_drive")
        self.arm_pub = self.create_publisher(Bool, "/nav/arm", 10)
        self.cmd_pub = self.create_publisher(Twist, "/teleop_cmd_vel", 10)
        self.speed_direction = 0
        self.steering_deg = STEERING_CENTER_DEG
        self.power = 60
        self.motion_deadline = 0.0

    def wait_for_driver(self):
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.arm_pub.get_subscription_count() >= 1
                and self.cmd_pub.get_subscription_count() >= 1
            ):
                return
        raise RuntimeError("没有发现底盘驱动订阅，未解锁")

    def publish_arm(self, armed):
        self.arm_pub.publish(Bool(data=armed))

    def publish_command(self):
        now = time.monotonic()
        if self.speed_direction and now >= self.motion_deadline:
            self.speed_direction = 0
            self.steering_deg = STEERING_CENTER_DEG
        command = Twist()
        command.linear.x = NEUTRAL_PWM + self.speed_direction * self.power
        command.angular.z = self.steering_deg
        self.cmd_pub.publish(command)

    def stop(self, center=True):
        self.speed_direction = 0
        self.motion_deadline = 0.0
        if center:
            self.steering_deg = STEERING_CENTER_DEG
        self.publish_command()

    def print_status(self, label):
        throttle = int(NEUTRAL_PWM + self.speed_direction * self.power)
        print(
            f"{label:<12} 油门={throttle} 方向角={self.steering_deg:.0f}° "
            f"功率差={self.power}",
            flush=True,
        )

    def change_direction(self, desired):
        if self.speed_direction and desired != self.speed_direction:
            self.stop()
            self.print_status("先停车")
            return
        self.speed_direction = desired
        self.motion_deadline = time.monotonic() + DEADMAN_SECONDS
        self.print_status("前进" if desired > 0 else "倒车")

    def handle_key(self, key):
        lower = key.lower()
        if lower in ("w", "\x1b[a"):
            self.steering_deg = STEERING_CENTER_DEG
            self.change_direction(1)
        elif lower in ("s", "\x1b[b"):
            self.steering_deg = STEERING_CENTER_DEG
            self.change_direction(-1)
        elif lower in ("a", "\x1b[d"):
            self.steering_deg = STEERING_LEFT_DEG
            if self.speed_direction:
                self.motion_deadline = time.monotonic() + DEADMAN_SECONDS
            self.print_status("向左")
        elif lower in ("d", "\x1b[c"):
            self.steering_deg = STEERING_RIGHT_DEG
            if self.speed_direction:
                self.motion_deadline = time.monotonic() + DEADMAN_SECONDS
            self.print_status("向右")
        elif lower in ("e", "c"):
            self.steering_deg = STEERING_CENTER_DEG
            if self.speed_direction:
                self.motion_deadline = time.monotonic() + DEADMAN_SECONDS
            self.print_status("方向回中")
        elif key in (" ", "x", "k"):
            self.stop()
            self.print_status("停车")
        elif key in ("+", "="):
            if self.speed_direction:
                print("请先按空格停车，再调功率", flush=True)
            else:
                self.power = min(MAX_POWER, self.power + POWER_STEP)
                self.print_status("功率增加")
        elif key in ("-", "_"):
            if self.speed_direction:
                print("请先按空格停车，再调功率", flush=True)
            else:
                self.power = max(MIN_POWER, self.power - POWER_STEP)
                self.print_status("功率降低")
        elif lower == "q" or key == "\x03":
            return False
        return True

    def run(self):
        self.wait_for_driver()
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            for _ in range(10):
                self.publish_arm(True)
                self.stop()
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.05)
            print(
                "\n键盘控制已启动（低速安全模式）\n"
                "  W/↑ 前进     S/↓ 倒车\n"
                "  A/← 左转     D/→ 右转     C/E 方向回中\n"
                "  空格/X/K 停车    +/- 调功率（必须先停车）\n"
                "  Q 或 Ctrl+C 退出\n"
                "松开按键约 0.8 秒自动停车。起步不动时：停车后按 +，每次增加10。\n",
                flush=True,
            )
            running = True
            while running:
                key = read_key()
                if key:
                    running = self.handle_key(key)
                self.publish_command()
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.02)
        finally:
            self.stop()
            for _ in range(12):
                self.publish_arm(False)
                self.publish_command()
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.05)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = KeyboardDrive()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
