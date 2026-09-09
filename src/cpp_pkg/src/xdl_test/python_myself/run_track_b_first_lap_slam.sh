#!/usr/bin/env bash
BASE=/home/davinci-mini/racecar
HERE=/home/davinci-mini/racecar/src/cpp_pkg/src/xdl_test/python_myself
source /opt/ros/humble/setup.bash
source "$BASE/install/setup.bash"
set -Eeuo pipefail
cleanup() {
  if [[ -n "${GMAPPING_PID:-}" ]] && kill -0 "$GMAPPING_PID" 2>/dev/null; then kill -TERM "$GMAPPING_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM
echo "[first lap] starting/reusing hardware and Gmapping..."
START_RVIZ=false bash "$BASE/gmapping.sh" > /tmp/pure_lidar_B_gmapping.log 2>&1 &
GMAPPING_PID=$!
for _ in $(seq 1 45); do
  timeout 2 ros2 topic echo /map --once --qos-durability transient_local --qos-reliability reliable >/dev/null 2>&1 && break
  sleep 1
done
if ! timeout 3 ros2 topic echo /map --once --qos-durability transient_local --qos-reliability reliable >/dev/null 2>&1; then
  echo "[first lap] ERROR: map is not active. See /tmp/pure_lidar_B_gmapping.log" >&2; exit 1
fi
echo "[first lap] Gmapping is active. Starting Pure Lidar B control."
echo "[first lap] Save map in another terminal: bash $BASE/save.sh"
cd "$HERE"
exec ./run_track_b.sh
