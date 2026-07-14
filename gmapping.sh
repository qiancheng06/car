#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/racecar_stack.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "ERROR: mapping or navigation is already running." >&2
  fuser -v "$LOCK_FILE" 2>/dev/null || true
  echo "If ros2 node list is empty, stop the displayed orphan PID before retrying." >&2
  exit 1
fi

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

PIDS=()
CORE_PIDS=()

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

if ros2 node list 2>/dev/null | grep -qx '/slam_gmapping'; then
  echo "ERROR: slam_gmapping is already running." >&2
  exit 1
fi

HARDWARE_LOCK="${XDG_RUNTIME_DIR:-/tmp}/racecar_hardware_start.lock"
exec 8>"$HARDWARE_LOCK"
if ! flock -w 15 8; then
  echo "ERROR: timed out waiting for the hardware startup lock." >&2
  exit 1
fi

if ! ros2 node list 2>/dev/null | grep -qx '/racecar_driver'; then
  echo "Starting hardware for mapping (driver remains DISARMED)..."
  ros2 launch racecar Run_car.launch.py \
    enable_legacy_pwm_input:=true \
    enable_legacy_normalized_input:=false \
    min_throttle_pwm:=1475.0 8>&- 9>&- &
  PIDS+=("$!")
  CORE_PIDS+=("$!")
  for _ in {1..20}; do
    ros2 node list 2>/dev/null | grep -qx '/racecar_driver' && break
    sleep 0.5
  done
  if ! ros2 node list 2>/dev/null | grep -qx '/racecar_driver'; then
    echo "ERROR: racecar_driver did not start." >&2
    exit 1
  fi
else
  echo "Reusing the already running hardware stack; no second serial driver will be started."
  if ! legacy_all_mode=$(ros2 param get /racecar_driver enable_legacy_inputs 2>/dev/null) || \
    ! legacy_pwm_mode=$(ros2 param get /racecar_driver enable_legacy_pwm_input 2>/dev/null) || \
    ! legacy_normalized_mode=$(ros2 param get /racecar_driver enable_legacy_normalized_input 2>/dev/null) || \
    ! minimum_throttle=$(ros2 param get /racecar_driver min_throttle_pwm 2>/dev/null)
  then
    echo "ERROR: cannot read the running driver's safety mode." >&2
    exit 1
  fi
  if grep -qi true <<<"$legacy_all_mode" || \
    ! grep -qi true <<<"$legacy_pwm_mode" || \
    grep -qi true <<<"$legacy_normalized_mode" || \
    ! grep -Eq '(^|[^0-9])1475(\.0+)?([^0-9]|$)' <<<"$minimum_throttle"
  then
    echo "ERROR: the running driver is not in PWM-only manual mode." >&2
    echo "Stop it, then run gmapping.sh again so mapping starts the correct driver mode." >&2
    exit 1
  fi
fi
flock -u 8

echo "Starting gmapping..."
ros2 launch slam_gmapping slam_gmapping.launch.py use_sim_time:=false 8>&- 9>&- &
PIDS+=("$!")
CORE_PIDS+=("$!")

RVIZ_CONFIG="$SCRIPT_DIR/src/racecar/rviz/slam.rviz"
if [[ "${START_RVIZ:-true}" == "true" ]]; then
  if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "Starting RViz for mapping..."
    rviz2 -d "$RVIZ_CONFIG" 8>&- 9>&- &
    PIDS+=("$!")
  else
    echo "RViz was not started because this terminal has no graphical display."
    echo "Open a desktop terminal and run: rviz2 -d $RVIZ_CONFIG"
  fi
fi

echo "Mapping is running. In another terminal use: bash save.sh"
echo "The chassis is DISARMED. For a lifted-wheel/manual test, arm with:"
echo "  ros2 topic pub --once /nav/arm std_msgs/msg/Bool '{data: true}'"
echo "Then start the low-speed keyboard controller in another terminal:"
echo "  ros2 run racecar racecar_teleop.py"
echo "Software stop:"
echo "  ros2 topic pub --once /nav/estop std_msgs/msg/Bool '{data: true}'"

wait -n "${CORE_PIDS[@]}"
