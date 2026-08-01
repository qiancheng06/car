#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

if pgrep racecar_driver_node >/dev/null 2>&1; then
  echo "ERROR: racecar_driver 已在运行，先停止 car/gmapping/nav。" >&2
  exit 2
fi

driver_pid=""
cleanup() {
  if [[ -n "$driver_pid" ]]; then
    kill -TERM -- "-$driver_pid" 2>/dev/null || kill -TERM "$driver_pid" 2>/dev/null || true
    wait "$driver_pid" 2>/dev/null || true
  fi
  stty sane 2>/dev/null || true
}
trap cleanup EXIT INT TERM

setsid ros2 run racecar_driver racecar_driver_node --ros-args \
  -p default_armed:=false \
  -p enable_legacy_inputs:=false \
  -p enable_legacy_pwm_input:=true \
  -p enable_legacy_normalized_input:=false \
  -p min_throttle_pwm:=1350.0 \
  -p max_throttle_pwm:=1650.0 \
  >/tmp/keyboard_drive_driver.log 2>&1 &
driver_pid=$!

python3 "$SCRIPT_DIR/src/racecar/scripts/keyboard_drive.py"
