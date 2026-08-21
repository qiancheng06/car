#!/bin/bash
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: /opt/ros/humble/setup.bash not found. Install/source ROS2 Humble first." >&2
  exit 1
fi

echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
if [ "${ROS_DISTRO:-}" != "humble" ]; then
  echo "ERROR: This competition stack is prepared for ROS2 Humble." >&2
  exit 1
fi

python3 -m py_compile \
  src/racecar/scripts/race_manager.py \
  src/racecar/scripts/parking_sign_detector.py \
  src/racecar/scripts/race_preflight.py \
  src/racecar/scripts/yolo_vision_node.py \
  src/racecar/scripts/npu_vision_node.py \
  src/racecar/racecar/vision_logic.py \
  src/racecar/launch/competition.launch.py \
  src/racecar/launch/vision.launch.py \
  src/racecar/launch/Run_nav.launch.py

colcon build --symlink-install --packages-select racecar lidar_tracking slam_gmapping racecar_driver encoder hipnuc_imu lslidar_driver lslidar_msgs

source install/setup.bash

ros2 pkg executables racecar | grep -E 'race_manager.py|parking_sign_detector.py|race_preflight.py|yolo_vision_node.py|npu_vision_node.py'
ros2 pkg prefix racecar >/dev/null

echo "Competition package build checks passed."
echo "For live topic checks, start competition.sh and run:"
echo "  ros2 run racecar race_preflight.py"
