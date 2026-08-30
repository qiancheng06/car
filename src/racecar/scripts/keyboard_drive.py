#!/usr/bin/env python3
"""Low-speed, dead-man keyboard control for the physical racecar."""

import json
import math
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


NEUTRAL_PWM = 1500.0
DEFAULT_STEERING_CENTER_DEG = 90.0
DEFAULT_TURN_DELTA_DEG = 30.0
DEFAULT_FORWARD_POWER = 55
DEFAULT_REVERSE_POWER = 70
MIN_STEERING_CENTER_DEG = 80.0
MAX_STEERING_CENTER_DEG = 100.0
MIN_STEERING_DEG = 60.0
MAX_STEERING_DEG = 120.0
MIN_TURN_DELTA_DEG = 5.0
MAX_TURN_DELTA_DEG = 30.0
MIN_POWER = 20
MAX_POWER = 150
POWER_LEVELS = (20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 80, 90, 100, 120, 150)
LOW_SPEED_PRESETS = (20, 25, 30, 35, 40, 45, 50, 55, 60)
DEADMAN_SECONDS = 0.8
REVERSE_ARM_SECONDS = 3.0
CALIBRATION_PATH = Path.home() / ".config" / "racecar" / "keyboard_drive.json"


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def read_key(timeout_sec=0.05):
    readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not readable:
        return ""
    first_bytes = os.read(sys.stdin.fileno(), 1)
    if not first_bytes:
        return None
    first = first_bytes.decode(errors="ignore")
    if first != "\x1b":
        return first
    sequence = first
    end = time.monotonic() + 0.03
    while time.monotonic() < end and len(sequence) < 3:
        readable, _, _ = select.select([sys.stdin], [], [], 0.005)
        if readable:
            next_bytes = os.read(sys.stdin.fileno(), 1)
            if not next_bytes:
                return None
            sequence += next_bytes.decode(errors="ignore")
    return sequence


class KeyboardDrive(Node):
    def __init__(self):
        super().__init__("keyboard_drive")
        self.cmd_pub = self.create_publisher(Twist, "/teleop_cmd_vel", 10)
        calibration = self.load_calibration()
        self.steering_center_deg = calibration["steering_center_deg"]
        self.turn_delta_deg = calibration["turn_delta_deg"]
        self.steering_direction = calibration["steering_direction"]
        self.forward_power = calibration["forward_power"]
        self.reverse_power = calibration["reverse_power"]
        self.speed_direction = 0
        self.power_target_direction = 1
        self.steering_deg = self.steering_center_deg
        self.motion_deadline = 0.0
        self.reverse_arm_deadline = 0.0

    def load_calibration(self):
        defaults = {
            "steering_center_deg": DEFAULT_STEERING_CENTER_DEG,
            "turn_delta_deg": DEFAULT_TURN_DELTA_DEG,
            "steering_direction": 1,
            "forward_power": DEFAULT_FORWARD_POWER,
            "reverse_power": DEFAULT_REVERSE_POWER,
        }
        try:
            loaded = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
            if not isinstance(loaded, dict):
                raise ValueError("calibration root must be an object")
        except FileNotFoundError:
            loaded = {}
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                f"Ignoring invalid keyboard calibration {CALIBRATION_PATH}: {error}"
            )
            loaded = {}

        def bounded_number(name, minimum, maximum):
            try:
                value = float(loaded.get(name, defaults[name]))
                if not math.isfinite(value):
                    raise ValueError(f"{name} must be finite")
            except (TypeError, ValueError):
                value = float(defaults[name])
            return clamp(value, minimum, maximum)

        center = bounded_number(
            "steering_center_deg",
            MIN_STEERING_CENTER_DEG,
            MAX_STEERING_CENTER_DEG,
        )
        max_turn_delta = min(
            MAX_TURN_DELTA_DEG,
            center - MIN_STEERING_DEG,
            MAX_STEERING_DEG - center,
        )
        turn_delta = bounded_number(
            "turn_delta_deg", MIN_TURN_DELTA_DEG, max_turn_delta
        )
        turn_delta_override = os.environ.get("RACECAR_TURN_DELTA_DEG")
        if turn_delta_override is not None:
            try:
                override_value = float(turn_delta_override)
                if not math.isfinite(override_value):
                    raise ValueError("turn delta must be finite")
                turn_delta = clamp(
                    override_value, MIN_TURN_DELTA_DEG, max_turn_delta
                )
            except ValueError as error:
                self.get_logger().warning(
                    "Ignoring invalid RACECAR_TURN_DELTA_DEG=%r: %s"
                    % (turn_delta_override, error)
                )
        try:
            direction = -1 if float(loaded.get("steering_direction", 1)) < 0 else 1
        except (TypeError, ValueError):
            direction = 1

        return {
            "steering_center_deg": center,
            "turn_delta_deg": turn_delta,
            "steering_direction": direction,
            "forward_power": int(
                round(bounded_number("forward_power", MIN_POWER, MAX_POWER))
            ),
            "reverse_power": int(
                round(bounded_number("reverse_power", MIN_POWER, MAX_POWER))
            ),
        }

    def save_calibration(self):
        data = {
            "steering_center_deg": self.steering_center_deg,
            "turn_delta_deg": self.turn_delta_deg,
            "steering_direction": self.steering_direction,
            "forward_power": self.forward_power,
            "reverse_power": self.reverse_power,
        }
        try:
            CALIBRATION_PATH.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = CALIBRATION_PATH.with_suffix(".tmp")
            temporary_path.write_text(
                json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            temporary_path.replace(CALIBRATION_PATH)
        except OSError as error:
            self.get_logger().warning(f"Unable to save keyboard calibration: {error}")

    def reset_calibration(self):
        if self.speed_direction:
            print("请先按空格停车，再恢复默认标定", flush=True)
            return
        self.steering_center_deg = DEFAULT_STEERING_CENTER_DEG
        self.turn_delta_deg = DEFAULT_TURN_DELTA_DEG
        self.steering_direction = 1
        self.forward_power = DEFAULT_FORWARD_POWER
        self.reverse_power = DEFAULT_REVERSE_POWER
        self.power_target_direction = 1
        self.reverse_arm_deadline = 0.0
        self.steering_deg = self.steering_center_deg
        self.save_calibration()
        self.print_status("已恢复默认")

    def steering_for_turn(self, turn_direction):
        return clamp(
            self.steering_center_deg
            + self.steering_direction * turn_direction * self.turn_delta_deg,
            MIN_STEERING_DEG,
            MAX_STEERING_DEG,
        )

    def motion_power(self):
        return self.reverse_power if self.speed_direction < 0 else self.forward_power

    def adjust_power(self, delta):
        if self.speed_direction:
            print("请先按空格停车，再调功率", flush=True)
            return
        if self.power_target_direction < 0:
            self.reverse_power = self.next_power_level(self.reverse_power, delta)
            label = "倒车功率"
        else:
            self.forward_power = self.next_power_level(self.forward_power, delta)
            label = "前进功率"
        self.save_calibration()
        self.print_status(label)

    @staticmethod
    def next_power_level(current, direction):
        if direction > 0:
            return next((level for level in POWER_LEVELS if level > current), MAX_POWER)
        return next(
            (level for level in reversed(POWER_LEVELS) if level < current), MIN_POWER
        )

    def select_low_speed(self, preset_index):
        if self.speed_direction:
            print("请先按空格停车，再选择低速档", flush=True)
            return
        selected = LOW_SPEED_PRESETS[preset_index]
        if self.power_target_direction < 0:
            self.reverse_power = selected
            label = f"倒车低速{preset_index + 1}档"
        else:
            self.forward_power = selected
            label = f"前进低速{preset_index + 1}档"
        self.save_calibration()
        self.print_status(label)

    def adjust_center(self, delta):
        if self.speed_direction:
            print("请先按空格停车，再调直行中值", flush=True)
            return
        self.steering_center_deg = clamp(
            self.steering_center_deg + delta,
            MIN_STEERING_CENTER_DEG,
            MAX_STEERING_CENTER_DEG,
        )
        max_turn_delta = min(
            MAX_TURN_DELTA_DEG,
            self.steering_center_deg - MIN_STEERING_DEG,
            MAX_STEERING_DEG - self.steering_center_deg,
        )
        self.turn_delta_deg = min(self.turn_delta_deg, max_turn_delta)
        self.steering_deg = self.steering_center_deg
        self.save_calibration()
        self.print_status("直行中值")

    def adjust_turn_delta(self, delta):
        if self.speed_direction:
            print("请先按空格停车，再调转向幅度", flush=True)
            return
        max_turn_delta = min(
            MAX_TURN_DELTA_DEG,
            self.steering_center_deg - MIN_STEERING_DEG,
            MAX_STEERING_DEG - self.steering_center_deg,
        )
        self.turn_delta_deg = clamp(
            self.turn_delta_deg + delta, MIN_TURN_DELTA_DEG, max_turn_delta
        )
        self.save_calibration()
        self.print_status("转向幅度")

    def invert_steering(self):
        if self.speed_direction:
            print("请先按空格停车，再交换左右方向", flush=True)
            return
        self.steering_direction *= -1
        self.steering_deg = self.steering_center_deg
        self.save_calibration()
        self.print_status("左右已交换")

    def arm_reverse(self):
        if self.speed_direction:
            print("请先按空格停车，再按 V 准备倒车", flush=True)
            return
        self.power_target_direction = -1
        self.reverse_arm_deadline = time.monotonic() + REVERSE_ARM_SECONDS
        self.print_status("倒车已准备")
        print("3秒内按/按住 S/↓ 开始倒车", flush=True)

    def wait_for_driver(self):
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.cmd_pub.get_subscription_count() >= 1:
                return
        raise RuntimeError("没有发现底盘驱动订阅")

    def publish_command(self):
        now = time.monotonic()
        if self.speed_direction and now >= self.motion_deadline:
            self.speed_direction = 0
            self.steering_deg = self.steering_center_deg
        command = Twist()
        command.linear.x = NEUTRAL_PWM + self.speed_direction * self.motion_power()
        command.angular.z = self.steering_deg
        self.cmd_pub.publish(command)

    def stop(self, center=True, clear_reverse=True):
        self.speed_direction = 0
        self.motion_deadline = 0.0
        if clear_reverse:
            self.reverse_arm_deadline = 0.0
        if center:
            self.steering_deg = self.steering_center_deg
        self.publish_command()

    def print_status(self, label):
        throttle = int(NEUTRAL_PWM + self.speed_direction * self.motion_power())
        print(
            f"{label:<12} 油门={throttle} 方向角={self.steering_deg:.0f}° "
            f"中值={self.steering_center_deg:.0f}° 转幅={self.turn_delta_deg:.0f}° "
            f"前/后功率={self.forward_power}/{self.reverse_power}",
            flush=True,
        )

    def change_direction(self, desired):
        now = time.monotonic()
        self.power_target_direction = desired

        if desired < 0 and self.speed_direction != -1:
            if now > self.reverse_arm_deadline:
                was_moving = bool(self.speed_direction)
                self.stop(center=was_moving)
                self.power_target_direction = -1
                self.print_status("倒车未准备")
                print("先按 V，3秒内再按/按住 S/↓", flush=True)
                return
        elif desired > 0:
            self.reverse_arm_deadline = 0.0

        if self.speed_direction and desired != self.speed_direction:
            self.stop()
            self.print_status("已停车，再按")
            return

        self.speed_direction = desired
        self.reverse_arm_deadline = 0.0
        self.motion_deadline = now + DEADMAN_SECONDS
        self.print_status("前进" if desired > 0 else "倒车")

    def handle_key(self, key):
        lower = key.lower()
        if lower in ("w", "\x1b[a"):
            self.change_direction(1)
        elif lower in ("s", "\x1b[b"):
            self.change_direction(-1)
        elif lower in ("a", "\x1b[d"):
            self.steering_deg = self.steering_for_turn(1)
            if self.speed_direction:
                self.motion_deadline = time.monotonic() + DEADMAN_SECONDS
            self.print_status("向左" if self.speed_direction else "左转已选")
        elif lower in ("d", "\x1b[c"):
            self.steering_deg = self.steering_for_turn(-1)
            if self.speed_direction:
                self.motion_deadline = time.monotonic() + DEADMAN_SECONDS
            self.print_status("向右" if self.speed_direction else "右转已选")
        elif lower in ("e", "c"):
            self.steering_deg = self.steering_center_deg
            self.print_status("方向回中")
        elif key in (" ", "x", "k"):
            self.stop()
            self.print_status("停车")
        elif key in ("+", "="):
            self.adjust_power(1)
        elif key in ("-", "_"):
            self.adjust_power(-1)
        elif key in "123456789":
            self.select_low_speed(int(key) - 1)
        elif key == "[":
            self.adjust_center(-1.0 * self.steering_direction)
        elif key == "]":
            self.adjust_center(1.0 * self.steering_direction)
        elif key == ",":
            self.adjust_turn_delta(-1.0)
        elif key == ".":
            self.adjust_turn_delta(1.0)
        elif lower == "i":
            self.invert_steering()
        elif lower == "v":
            self.arm_reverse()
        elif lower == "r":
            self.reset_calibration()
        elif lower == "q" or key == "\x03":
            return False
        return True

    def run(self):
        self.wait_for_driver()
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            for _ in range(10):
                self.stop()
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.05)
            print(
                "\n键盘控制已启动（低速安全模式）\n"
                "  W/↑ 前进；每次倒车起步：停车后按 V，再按/按住 S/↓\n"
                "  A/← 选择左转  D/→ 选择右转  C/E 回中；W启动后可按住A/D转弯\n"
                "  空格/X/K 停车；1~9 选择低速档（PWM差20~60），+/- 逐档微调\n"
                "  功率调整作用于刚选择的方向：W=前进，第一次 S=倒车\n"
                "  [ / ] 调直行中值：向左跑偏按 [，向右跑偏按 ]\n"
                "  , / . 调转向幅度；I 交换 A/D；R 恢复默认标定\n"
                "  Q 或 Ctrl+C 退出\n"
                "阿克曼小车不能原地转。松开按键约0.8秒自动停车，标定会自动保存。\n",
                flush=True,
            )
            self.print_status("当前标定")
            running = True
            while running:
                key = read_key()
                if key is None:
                    print("\nTerminal connection closed; stopping.", flush=True)
                    break
                if key:
                    running = self.handle_key(key)
                self.publish_command()
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.02)
        finally:
            self.stop()
            for _ in range(12):
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
