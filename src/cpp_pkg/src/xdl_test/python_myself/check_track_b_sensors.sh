#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/davinci-mini/racecar/install/setup.bash
set -Eeuo pipefail
check_topic() {
  local topic="$1"
  if timeout 5 ros2 topic echo "$topic" --once >/dev/null 2>&1; then echo "OK    $topic"; else echo "FAIL  $topic"; fi
}
echo "=== required sensor messages ==="
check_topic /scan
check_topic /IMU_data
check_topic /odom_combined
echo; echo "=== active rates (5-second sample) ==="
timeout 6 ros2 topic hz /scan --window 5 2>&1 || true
timeout 6 ros2 topic hz /odom_combined --window 5 2>&1 || true
echo; echo "=== relevant processes ==="
ps -ef | grep -E 'lslidar|encoder_node|hipnuc|racecar_driver|ekf_node' | grep -v grep || true
echo; echo "=== recent hardware errors ==="
for log in /tmp/pure_lidar_B_run_car.log /tmp/run_car.log; do
  [[ -f "$log" ]] || continue
  echo "--- $log ---"
  grep -iE 'error|exception|died|disconnected|serial open fail' "$log" | tail -40 || true
done
