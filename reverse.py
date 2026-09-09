#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""1400 倒车（电调自动解锁）。用法: python3 reverse.py [倒车秒数, 默认2]"""
import sys
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

HOLD_PWM = 1400.0
UNLOCK_SEQ = [1470, 1450, 1430, 1400, 1370]  # 由浅到深，实测解锁点 1370，电调热状态下 1470 即可
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0

rclpy.init()
node = Node('reverse_cmd')
pub = node.create_publisher(Twist, '/teleop_cmd_vel', 10)
state = {'v': 0.0}
node.create_subscription(Odometry, '/encoder', lambda m: state.update(v=m.twist.twist.linear.x), 10)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
time.sleep(1.0)


def twist_msg(throttle):
    m = Twist()
    m.linear.x = float(throttle)
    m.angular.z = 90.0
    return m


def send(duration, throttle, hz=20):
    msg = twist_msg(throttle)
    end = time.time() + duration
    while time.time() < end:
        pub.publish(msg)
        time.sleep(1.0 / hz)


def measured(duration, throttle):
    msg = twist_msg(throttle)
    peak = 0.0
    t0 = time.time()
    while time.time() - t0 < duration:
        pub.publish(msg)
        peak = max(peak, abs(state['v']))
        time.sleep(0.05)
    return peak


send(1.0, 1500)
unlocked = None
for v in UNLOCK_SEQ:
    print(f'unlock try {v} ...', flush=True)
    p1 = measured(0.6, v)
    send(0.4, 1500)
    p2 = measured(0.6, v)
    send(0.4, 1500)
    p3 = measured(1.0, HOLD_PWM)
    if max(p1, p2, p3) > 0.03:
        unlocked = v
        print(f'  -> UNLOCKED (wheel {max(p1, p2, p3):.2f} m/s)', flush=True)
        break
if unlocked is None:
    print('FAILED: 电调未解锁，请检查电调模式', flush=True)
    send(0.5, 1500)
    sys.exit(1)

print(f'REVERSE {HOLD_PWM:.0f} for {DUR}s ...', flush=True)
measured(0.5, unlocked)   # 重新确认倒挡咬合
send(0.3, 1500)
t0 = time.time()
samples = []
while time.time() - t0 < DUR:
    pub.publish(twist_msg(HOLD_PWM))
    samples.append(abs(state['v']))
    time.sleep(0.05)
print(f'DONE: avg {sum(samples) / len(samples):.2f} m/s', flush=True)
send(1.0, 1500)
print('COMPLETE', flush=True)
