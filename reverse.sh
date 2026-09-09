#!/bin/bash
# 1400 倒车 N 秒（自动电调解锁）。用法: bash ~/racecar/reverse.sh [秒数]
source /opt/ros/humble/setup.bash
source ~/racecar/install/setup.bash
python3 ~/racecar/reverse.py "${1:-2}"
