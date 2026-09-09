#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/davinci-mini/racecar/install/setup.bash
set -Eeuo pipefail
wait_topic() {
  local topic="$1"; local seconds="$2"
  for _ in $(seq 1 "$seconds"); do
    timeout 2 ros2 topic echo "$topic" --once >/dev/null 2>&1 && return 0
    sleep 1
  done
  return 1
}
if ! pgrep -f "ros2 launch racecar Run_car.launch.py" >/dev/null 2>&1; then
  echo "[track_B] starting the hardware stack..."
  nohup ros2 launch racecar Run_car.launch.py \
    enable_legacy_pwm_input:=true \
    min_steering_pwm:=880 \
    max_steering_pwm:=2200 \
    > /tmp/pure_lidar_B_run_car.log 2>&1 &
fi
echo "[track_B] checking /scan, /IMU_data and /odom_combined..."
for topic in /scan /IMU_data /odom_combined; do
  if ! wait_topic "$topic" 20; then
    echo "[track_B] ERROR: no live $topic message. Motion was not started." >&2
    echo "[track_B] inspect: tail -150 /tmp/pure_lidar_B_run_car.log" >&2
    exit 1
  fi
done
cd "$(dirname "$0")"
echo "[track_B] all required sensors are live; starting Track-B controller."
exec python3 pure_lidar_b_node.py \
  --ros-args \
  --params-file pure_lidar_b_config.yaml \
  -p enable_motion:=true \
  "$@"
