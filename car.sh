#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash does not exist." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash

if [[ ! -f install/setup.bash ]]; then
  echo "ERROR: workspace is not built. Run: bash setup_teb.sh" >&2
  exit 1
fi
source install/setup.bash
set -u

HARDWARE_LOCK="${XDG_RUNTIME_DIR:-/tmp}/racecar_hardware_start.lock"
exec 8>"$HARDWARE_LOCK"
if ! flock -w 15 8; then
  echo "ERROR: timed out waiting for the hardware startup lock." >&2
  exit 1
fi

if ros2 node list 2>/dev/null | grep -qx '/racecar_driver'; then
  echo "ERROR: racecar hardware is already running; do not open the serial port twice." >&2
  exit 1
fi

PIDS=()
cleanup() {
  timeout 1 ros2 topic pub --once /nav/arm std_msgs/msg/Bool \
    '{data: false}' >/dev/null 2>&1 || true
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  for pid in "${PIDS[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "Starting lidar, IMU, encoder, EKF and the DISARMED chassis driver..."
echo "The PWM-style /teleop_cmd_vel input is enabled for low-speed mapping/bench control."
ros2 launch racecar Run_car.launch.py \
  enable_legacy_pwm_input:=true \
  enable_legacy_normalized_input:=false \
  "$@" 8>&- &
PIDS+=("$!")

for _ in {1..20}; do
  ros2 node list 2>/dev/null | grep -qx '/racecar_driver' && break
  sleep 0.5
done
if ! ros2 node list 2>/dev/null | grep -qx '/racecar_driver'; then
  echo "ERROR: racecar_driver did not start." >&2
  exit 1
fi
flock -u 8

echo "Driver is DISARMED. Arm only after lifting the drive wheels:"
echo "  ros2 topic pub --once /nav/arm std_msgs/msg/Bool '{data: true}'"
wait "${PIDS[0]}"
