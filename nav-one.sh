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

RUNTIME_MAP="$HOME/racecar_maps/ai_map.yaml"
SOURCE_MAP="$SCRIPT_DIR/src/racecar/map/ai_map.yaml"
if [[ -s "$RUNTIME_MAP" ]]; then
  DEFAULT_MAP="$RUNTIME_MAP"
else
  DEFAULT_MAP="$SOURCE_MAP"
  echo "WARNING: no runtime map exists; using the repository test map: $SOURCE_MAP" >&2
fi
if [[ "$HAS_MAP_ARGUMENT" == false && ! -s "$DEFAULT_MAP" ]]; then
  echo "ERROR: default map does not exist: $DEFAULT_MAP" >&2
  echo "Run gmapping.sh + save.sh, or pass map:=/absolute/path/map.yaml" >&2
  exit 1
fi

PIDS=()
cleanup_started=false

NAV_PROCESS_PATTERNS=(
  "Run_nav.launch.py" "nav_cmd_adapter" "bt_navigator" "map_server" "amcl"
  "planner_server" "controller_server" "behavior_server" "smoother_server"
  "velocity_smoother" "waypoint_follower" "lifecycle_manager" "nav2_container"
)

cleanup_stale_nav() {
  local signal="${1:-soft}"
  local pid
  for pattern in "${NAV_PROCESS_PATTERNS[@]}"; do
    if [[ "$signal" == "hard" ]]; then
      pkill -KILL -f "$pattern" 2>/dev/null || true
    else
      pkill -TERM -f "$pattern" 2>/dev/null || true
    fi
  done
  if [[ "$signal" == "hard" ]]; then
    return 0
  fi
  for _ in {1..20}; do
    if ! pgrep -f "bt_navigator|nav_cmd_adapter" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  cleanup_stale_nav hard
}

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
    for _ in {1..40}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.05
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  cleanup_stale_nav hard
}
trap cleanup EXIT INT TERM HUP

wait_for_message() {
  local topic="$1"
  local seconds="$2"
  timeout "$seconds" ros2 topic echo "$topic" --once \
    >/dev/null 2>&1
}

wait_for_transform() {
  local parent="$1"
  local child="$2"
  local output
  output="$(timeout 10 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 || true)"
  grep -q 'Translation:' <<<"$output"
}

wait_for_active() {
  local node="$1"
  local state
  for _ in {1..120}; do
    state="$(timeout 5 ros2 lifecycle get "$node" 2>/dev/null || true)"
    grep -qi 'active' <<<"$state" && return 0
    sleep 0.5
  done
  # The ros2 CLI is unreliable on this board (intermittent "context is invalid").
  # Fall back to verifying the process is alive instead of failing the whole stack.
  if pgrep -f "nav2_.*/$node" >/dev/null 2>&1 || pgrep -f "${node#/}" >/dev/null 2>&1; then
    echo "WARNING: lifecycle query for $node was inconclusive, but the process is running." >&2
    return 0
  fi
  echo "ERROR: lifecycle node did not become active: $node" >&2
  return 1
}

if pgrep -f "bt_navigator|nav_cmd_adapter" >/dev/null 2>&1; then
  # The stack lock is held by THIS process, so no other mapping/nav session can be
  # running. Any visible Nav2 node must be a leftover from a session whose terminal
  # was closed without cleanup. Stop it instead of failing.
  echo "WARNING: stale Nav2 processes found from a previous session; cleaning them up..."
  cleanup_stale_nav soft
fi

HARDWARE_LOCK="${XDG_RUNTIME_DIR:-/tmp}/racecar_hardware_start.lock"
exec 8>"$HARDWARE_LOCK"
if ! flock -w 15 8; then
  echo "ERROR: timed out waiting for the hardware startup lock." >&2
  exit 1
fi

if ! pgrep -f "racecar_driver/.*racecar_driver_node" >/dev/null 2>&1; then
  echo "Starting hardware with all legacy command inputs disabled..."
  setsid ros2 launch racecar Run_car.launch.py \
    enable_legacy_pwm_input:=false \
    enable_legacy_normalized_input:=false \
    min_throttle_pwm:=1500.0 \
    max_throttle_pwm:=1550.0 \
    min_steering_pwm:=1150.0 \
    max_steering_pwm:=1850.0 8>&- 9>&- &
  PIDS+=("$!")
  driver_seen=false
  for _ in {1..120}; do
    if pgrep -f "racecar_driver/.*racecar_driver_node" >/dev/null 2>&1 || \
      ros2 node list 2>/dev/null | grep -qx '/racecar_driver'; then
      driver_seen=true
      break
    fi
    sleep 0.5
  done
  if [[ "$driver_seen" != true ]]; then
    echo "ERROR: racecar_driver did not start." >&2
    exit 1
  fi
else
  echo "Reusing the already running hardware stack; no second serial driver will be started."
  if ! legacy_all_mode=$(ros2 param get /racecar_driver enable_legacy_inputs 2>/dev/null) || \
    ! legacy_pwm_mode=$(ros2 param get /racecar_driver enable_legacy_pwm_input 2>/dev/null) || \
    ! legacy_normalized_mode=$(ros2 param get /racecar_driver enable_legacy_normalized_input 2>/dev/null) || \
    ! minimum_throttle=$(ros2 param get /racecar_driver min_throttle_pwm 2>/dev/null) || \
    ! minimum_steering=$(ros2 param get /racecar_driver min_steering_pwm 2>/dev/null) || \
    ! maximum_steering=$(ros2 param get /racecar_driver max_steering_pwm 2>/dev/null)
  then
    echo "ERROR: cannot read the running driver's safety mode." >&2
    exit 1
  fi
  if grep -qi true <<<"$legacy_all_mode" || grep -qi true <<<"$legacy_pwm_mode" || \
    grep -qi true <<<"$legacy_normalized_mode" || \
    ! grep -Eq '(^|[^0-9])1500(\.0+)?([^0-9]|$)' <<<"$minimum_throttle" || \
    ! grep -Eq '(^|[^0-9])1150(\.0+)?([^0-9]|$)' <<<"$minimum_steering" || \
    ! grep -Eq '(^|[^0-9])1850(\.0+)?([^0-9]|$)' <<<"$maximum_steering"
  then
    echo "ERROR: the running driver has a legacy control input enabled." >&2
    echo "Stop car.sh/gmapping.sh first, then run nav-one.sh again." >&2
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

echo "Starting A*, TEB and the command adapter..."
if [[ "$HAS_MAP_ARGUMENT" == true ]]; then
  setsid ros2 launch racecar Run_nav.launch.py "$@" 8>&- 9>&- &
else
  setsid ros2 launch racecar Run_nav.launch.py "$@" \
    "map:=$DEFAULT_MAP" 8>&- 9>&- &
fi
PIDS+=("$!")

for lifecycle_node in /map_server /amcl /planner_server /controller_server /bt_navigator; do
  wait_for_active "$lifecycle_node"
done
if ! pgrep -f "nav_cmd_adapter" >/dev/null 2>&1; then
  echo "ERROR: nav_cmd_adapter did not start." >&2
  exit 1
fi

echo "Navigation is active."
echo "Map: $DEFAULT_MAP"
echo "Software stop (keep a person at the physical power switch):"
echo "  ros2 topic pub --once /nav/estop std_msgs/msg/Bool '{data: true}'"

wait -n "${PIDS[@]}"
