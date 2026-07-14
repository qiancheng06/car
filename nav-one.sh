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

HAS_MAP_ARGUMENT=false
for argument in "$@"; do
  if [[ "$argument" == auto_arm:=* ]]; then
    echo "ERROR: nav-one.sh always starts disarmed; remove '$argument'." >&2
    exit 1
  fi
  if [[ "$argument" == map:=* ]]; then
    HAS_MAP_ARGUMENT=true
  fi
done

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

if ! ros2 pkg prefix teb_local_planner >/dev/null 2>&1; then
  echo "ERROR: TEB is missing. Run: bash setup_teb.sh" >&2
  exit 1
fi
if ! ros2 pkg prefix costmap_converter >/dev/null 2>&1; then
  echo "ERROR: costmap_converter is missing. Run: bash setup_teb.sh" >&2
  exit 1
fi

DEFAULT_MAP="$SCRIPT_DIR/src/racecar/map/ai_map.yaml"
if [[ "$HAS_MAP_ARGUMENT" == false && ! -s "$DEFAULT_MAP" ]]; then
  echo "ERROR: default map does not exist: $DEFAULT_MAP" >&2
  echo "Run gmapping.sh + save.sh, or pass map:=/absolute/path/map.yaml" >&2
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

if ros2 node list 2>/dev/null | grep -Eq '^/(bt_navigator|nav_cmd_adapter)$'; then
  echo "ERROR: Nav2 is already running." >&2
  exit 1
fi

HARDWARE_LOCK="${XDG_RUNTIME_DIR:-/tmp}/racecar_hardware_start.lock"
exec 8>"$HARDWARE_LOCK"
if ! flock -w 15 8; then
  echo "ERROR: timed out waiting for the hardware startup lock." >&2
  exit 1
fi

if ! ros2 node list 2>/dev/null | grep -qx '/racecar_driver'; then
  echo "Starting hardware with all legacy command inputs disabled..."
  ros2 launch racecar Run_car.launch.py \
    enable_legacy_pwm_input:=false \
    enable_legacy_normalized_input:=false \
    min_throttle_pwm:=1500.0 8>&- 9>&- &
  PIDS+=("$!")
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
  if grep -qi true <<<"$legacy_all_mode" || grep -qi true <<<"$legacy_pwm_mode" || \
    grep -qi true <<<"$legacy_normalized_mode" || \
    ! grep -Eq '(^|[^0-9])1500(\.0+)?([^0-9]|$)' <<<"$minimum_throttle"
  then
    echo "ERROR: the running driver has a legacy control input enabled." >&2
    echo "Stop car.sh/gmapping.sh first, then run nav-one.sh again." >&2
    exit 1
  fi
fi
flock -u 8

echo "Starting Hybrid-A*, TEB and the DISARMED command adapter..."
if [[ "$HAS_MAP_ARGUMENT" == true ]]; then
  ros2 launch racecar Run_nav.launch.py "$@" 8>&- 9>&- &
else
  ros2 launch racecar Run_nav.launch.py "$@" \
    "map:=$DEFAULT_MAP" 8>&- 9>&- &
fi
PIDS+=("$!")

echo "After checking /scan, /odom_combined, TF and the steering direction, arm with:"
echo "  ros2 topic pub --once /nav/arm std_msgs/msg/Bool '{data: true}'"
echo "Software stop (keep a person at the physical power switch):"
echo "  ros2 topic pub --once /nav/estop std_msgs/msg/Bool '{data: true}'"

wait -n "${PIDS[@]}"
