#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/davinci-mini/racecar/install/setup.bash
set -Eeuo pipefail
cd "$(dirname "$0")"
exec python3 pure_lidar_b_node.py \
  --ros-args \
  --params-file pure_lidar_b_config.yaml \
  -p enable_motion:=false \
  "$@"
