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
cleanup_started=false

cleanup() {
  if [[ "$cleanup_started" == true ]]; then
    return
  fi
  cleanup_started=true
  trap - EXIT INT TERM HUP
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  for pid in "${PIDS[@]:-}"; do
    for _ in {1..20}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.05
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM HUP

wait_for_message() {
  local topic="$1"
  local seconds="$2"
  local durability="${3:-volatile}"
  local reliability="${4:-best_effort}"
  timeout "$seconds" ros2 topic echo "$topic" --once \
    --qos-reliability "$reliability" --qos-durability "$durability" \
    >/dev/null 2>&1
}

node_exists() {
  local node_name="$1"
  local nodes
  nodes="$(timeout 5 ros2 node list 2>/dev/null)" || return 1
  grep -qx "$node_name" <<<"$nodes"
}

driver_param() {
  timeout 5 ros2 param get /racecar_driver "$1" 2>/dev/null
}

wait_for_transform() {
  local parent="$1"
  local child="$2"
  local output
  output="$(timeout 10 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 || true)"
  grep -q 'Translation:' <<<"$output"
}

if node_exists /slam_gmapping; then
  echo "ERROR: slam_gmapping is already running." >&2
  exit 1
fi

HARDWARE_LOCK="${XDG_RUNTIME_DIR:-/tmp}/racecar_hardware_start.lock"
exec 8>"$HARDWARE_LOCK"
if ! flock -w 15 8; then
  echo "ERROR: timed out waiting for the hardware startup lock." >&2
  exit 1
fi

if ! node_exists /racecar_driver; then
  echo "Starting hardware for mapping..."
  setsid ros2 launch racecar Run_car.launch.py \
    enable_legacy_pwm_input:=true \
    enable_legacy_normalized_input:=false \
    min_throttle_pwm:=1350.0 \
    max_throttle_pwm:=1650.0 \
    min_steering_pwm:=1000.0 \
    max_steering_pwm:=2000.0 8>&- 9>&- &
  PIDS+=("$!")
  CORE_PIDS+=("$!")
  for _ in {1..40}; do
    node_exists /racecar_driver && break
    sleep 0.5
  done
  if ! node_exists /racecar_driver; then
    echo "ERROR: racecar_driver did not start." >&2
    exit 1
  fi
else
  echo "Reusing the already running hardware stack; no second serial driver will be started."
  if ! legacy_all_mode=$(driver_param enable_legacy_inputs) || \
    ! legacy_pwm_mode=$(driver_param enable_legacy_pwm_input) || \
    ! legacy_normalized_mode=$(driver_param enable_legacy_normalized_input) || \
    ! minimum_throttle=$(driver_param min_throttle_pwm) || \
    ! maximum_throttle=$(driver_param max_throttle_pwm) || \
    ! minimum_steering=$(driver_param min_steering_pwm) || \
    ! maximum_steering=$(driver_param max_steering_pwm)
  then
    echo "ERROR: cannot read the running driver's safety mode." >&2
    exit 1
  fi
  if grep -qi true <<<"$legacy_all_mode" || \
    ! grep -qi true <<<"$legacy_pwm_mode" || \
    grep -qi true <<<"$legacy_normalized_mode" || \
    ! grep -Eq '\11000[^0-9])1350(\.0+)?([^0-9]|$)' <<<"$minimum_throttle" || \
    ! grep -Eq '\11000[^0-9])1650(\.0+)?([^0-9]|$)' <<<"$maximum_throttle" || \
    ! grep -Eq '(^|[^0-9])1000(\.0+)?([^0-9]|$)' <<<"$minimum_steering" || \
    ! grep -Eq '(^|[^0-9])2000(\.0+)?([^0-9]|$)' <<<"$maximum_steering"
  then
    echo "ERROR: the running driver is not in PWM-only manual mode." >&2
    echo "Stop it, then run gmapping.sh again so mapping starts the correct driver mode." >&2
    exit 1
  fi
fi
flock -u 8

echo "Checking live lidar, odometry and TF..."
if ! wait_for_message /scan 20; then
  echo "ERROR: no /scan message arrived within 20 seconds." >&2
  exit 1
fi
if ! wait_for_message /odom_combined 20; then
  echo "ERROR: no /odom_combined message arrived within 20 seconds." >&2
  exit 1
fi
if ! wait_for_transform odom_combined base_footprint; then
  echo "ERROR: missing TF odom_combined -> base_footprint." >&2
  exit 1
fi

echo "Starting gmapping..."
setsid ros2 launch slam_gmapping slam_gmapping_params.launch.py use_sim_time:=false 8>&- 9>&- &
PIDS+=("$!")
CORE_PIDS+=("$!")

for _ in {1..40}; do
  node_exists /slam_gmapping && break
  sleep 0.5
done
if ! node_exists /slam_gmapping; then
  echo "ERROR: slam_gmapping did not start." >&2
  exit 1
fi
if ! wait_for_message /map 30 transient_local reliable; then
  echo "ERROR: gmapping did not publish a map within 30 seconds." >&2
  exit 1
fi

RVIZ_CONFIG="$SCRIPT_DIR/src/racecar/rviz/slam.rviz"
if [[ "${START_RVIZ:-true}" == "true" ]]; then
  if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "Starting RViz for mapping..."
    setsid rviz2 -d "$RVIZ_CONFIG" 8>&- 9>&- &
    PIDS+=("$!")
  else
    echo "RViz was not started because this terminal has no graphical display."
    echo "Open a desktop terminal and run: rviz2 -d $RVIZ_CONFIG"
  fi
fi

echo "Mapping is ready. In another terminal use: bash save.sh"
echo "Default output: $HOME/racecar_maps/ai_map.yaml"
echo "The chassis is ready for a lifted-wheel/manual test."
echo "Then start the low-speed keyboard controller in another terminal:"
echo "  ros2 run racecar racecar_teleop.py"
echo "Or use the all-in-one workflow next time: bash mapping-one.sh"
echo "Software stop:"
echo "  ros2 topic pub --once /nav/estop std_msgs/msg/Bool '{data: true}'"

wait -n "${CORE_PIDS[@]}"
