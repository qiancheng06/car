#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

HARDWARE_LOCK="${XDG_RUNTIME_DIR:-/tmp}/racecar_hardware_start.lock"
exec 8>"$HARDWARE_LOCK"
if ! flock -n 8; then
  echo "ERROR: another racecar hardware stack is starting or running." >&2
  exit 2
fi

if [[ ! -e /dev/car ]]; then
  echo "ERROR: /dev/car does not exist. Check the chassis USB cable and udev rules." >&2
  exit 2
fi

if fuser /dev/car >/dev/null 2>&1; then
  echo "ERROR: /dev/car is already in use. Stop car.sh, gmapping.sh, nav-one.sh, or an old keyboard session first." >&2
  fuser -v /dev/car >&2 || true
  exit 2
fi

driver_pid=""
cleanup_started=false
cleanup() {
  if [[ "$cleanup_started" == true ]]; then
    return
  fi
  cleanup_started=true
  trap - EXIT INT TERM HUP

  if [[ -n "$driver_pid" ]]; then
    kill -TERM -- "-$driver_pid" 2>/dev/null || kill -TERM "$driver_pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$driver_pid" 2>/dev/null || break
      sleep 0.05
    done
    if kill -0 "$driver_pid" 2>/dev/null; then
      kill -KILL -- "-$driver_pid" 2>/dev/null || kill -KILL "$driver_pid" 2>/dev/null || true
    fi
  fi
  stty sane 2>/dev/null || true
}
trap cleanup EXIT INT TERM HUP

setsid ros2 run racecar_driver racecar_driver_node --ros-args \
  -p enable_legacy_inputs:=false \
  -p enable_legacy_pwm_input:=true \
  -p enable_legacy_normalized_input:=false \
  -p min_throttle_pwm:=1350.0 \
  -p max_throttle_pwm:=1650.0 \
  -p min_steering_pwm:=1150.0 \
  -p max_steering_pwm:=1850.0 \
  8>&- \
  >/tmp/keyboard_drive_driver.log 2>&1 &
driver_pid=$!

if python3 "$SCRIPT_DIR/src/racecar/scripts/keyboard_drive.py"; then
  exit 0
else
  controller_status=$?
  echo "" >&2
  echo "Bottom-driver log:" >&2
  tail -n 40 /tmp/keyboard_drive_driver.log >&2 || true
  exit "$controller_status"
fi
