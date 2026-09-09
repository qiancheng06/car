#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
if [ -f /home/davinci-mini/racecar/install/setup.bash ]; then
  source /home/davinci-mini/racecar/install/setup.bash
fi
cd "$(dirname "$0")"
exec python3 pure_lidar_node.py --ros-args --params-file pure_lidar_config.yaml "$@"
