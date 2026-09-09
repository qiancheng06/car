#!/usr/bin/env bash
BASE=/home/davinci-mini/racecar
HERE=/home/davinci-mini/racecar/src/cpp_pkg/src/xdl_test/python_myself
MAP="$HOME/racecar_maps/ai_map.yaml"
[[ -s "$MAP" ]] || { echo "ERROR: map does not exist: $MAP" >&2; exit 1; }
source /opt/ros/humble/setup.bash
source "$BASE/install/setup.bash"
set -Eeuo pipefail
echo "[second lap] starting map localization/navigation stack..."
echo "[second lap] Pure Lidar uses /pure_lidar/cmd_suggestion to avoid command conflicts."
bash "$BASE/nav-one.sh" > /tmp/pure_lidar_B_localization.log 2>&1 &
NAV_PID=$!
trap 'kill -TERM "$NAV_PID" 2>/dev/null || true' EXIT INT TERM
for _ in $(seq 1 45); do timeout 2 ros2 topic echo /amcl_pose --once >/dev/null 2>&1 && break; sleep 1; done
if ! timeout 3 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
  echo "ERROR: localization did not become ready. See /tmp/pure_lidar_B_localization.log" >&2; exit 1
fi
cd "$HERE"
exec python3 pure_lidar_b_node.py --ros-args \
  --params-file pure_lidar_b_config.yaml \
  -p enable_motion:=false \
  -p cmd_topic:=/pure_lidar/cmd_suggestion
