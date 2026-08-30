#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ ! -t 0 ]]; then
  echo "ERROR: mapping-one.sh needs an interactive terminal for keyboard control." >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash || ! -f install/setup.bash ]]; then
  echo "ERROR: ROS2 Humble or the built racecar workspace is missing." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

MAP_NAME="${1:-ai_map}"
LOG_DIR="$HOME/racecar_logs"
mkdir -p "$LOG_DIR"
MAPPING_LOG="$LOG_DIR/mapping-$(date +%Y%m%d-%H%M%S).log"
mapping_pid=""
cleanup_started=false

cleanup() {
  if [[ "$cleanup_started" == true ]]; then
    return
  fi
  cleanup_started=true
  trap - EXIT INT TERM HUP

  if [[ -n "$mapping_pid" ]] && kill -0 "$mapping_pid" 2>/dev/null; then
    kill -TERM -- "-$mapping_pid" 2>/dev/null || kill -TERM "$mapping_pid" 2>/dev/null || true
    for _ in {1..80}; do
      kill -0 "$mapping_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$mapping_pid" 2>/dev/null; then
      kill -KILL -- "-$mapping_pid" 2>/dev/null || kill -KILL "$mapping_pid" 2>/dev/null || true
    fi
  fi
  stty sane 2>/dev/null || true
}
trap cleanup EXIT INT TERM HUP

node_exists() {
  local node_name="$1"
  local nodes
  nodes="$(timeout 5 ros2 node list 2>/dev/null)" || return 1
  grep -qx "$node_name" <<<"$nodes"
}

topic_exists() {
  local topic_name="$1"
  local topics
  topics="$(timeout 5 ros2 topic list 2>/dev/null)" || return 1
  grep -qx "$topic_name" <<<"$topics"
}

echo "Starting mapping."
echo "Please wait about 20-40 seconds. Detailed log: $MAPPING_LOG"
START_RVIZ="${START_RVIZ:-false}" setsid bash "$SCRIPT_DIR/gmapping.sh" \
  >"$MAPPING_LOG" 2>&1 &
mapping_pid=$!

map_topic_ready=false
for startup_tick in {1..180}; do
  if ! kill -0 "$mapping_pid" 2>/dev/null; then
    echo "ERROR: gmapping.sh exited before the map became ready." >&2
    echo "Last mapping log lines:" >&2
    tail -60 "$MAPPING_LOG" >&2 || true
    exit 1
  fi
  if node_exists /slam_gmapping && topic_exists /map; then
    map_topic_ready=true
    break
  fi
  if (( startup_tick % 10 == 0 )); then
    echo "  still starting..."
  fi
  sleep 0.5
done
if [[ "$map_topic_ready" != true ]] || \
  ! timeout 15 ros2 topic echo /map --once \
    --qos-reliability reliable --qos-durability transient_local \
    >/dev/null 2>&1
then
  echo "ERROR: no complete /map message arrived." >&2
  echo "Last mapping log lines:" >&2
  tail -60 "$MAPPING_LOG" >&2 || true
  exit 1
fi

echo ""
echo "Map is live. Keyboard control will now ARM the chassis; steering amplitude is 30 degrees."
echo "Press Q when the lap is complete. The map will then be saved automatically."
RACECAR_TURN_DELTA_DEG="${RACECAR_TURN_DELTA_DEG:-30}" \
  python3 "$SCRIPT_DIR/src/racecar/scripts/keyboard_drive.py"

echo "Keyboard control ended; saving the map while SLAM is still running..."
map_saved=false
for attempt in 1 2 3; do
  if bash "$SCRIPT_DIR/save.sh" "$MAP_NAME"; then
    map_saved=true
    break
  fi
  echo "Map save attempt $attempt/3 failed; SLAM is still running." >&2
  sleep 1
done

while [[ "$map_saved" != true ]]; do
  echo "ERROR: automatic save failed, but the live map has not been stopped." >&2
  read -r -p "Press R to retry saving, or Q to abandon this map: " answer
  case "${answer,,}" in
    r)
      if bash "$SCRIPT_DIR/save.sh" "$MAP_NAME"; then
        map_saved=true
      fi
      ;;
    q)
      echo "Map save abandoned by the operator." >&2
      exit 1
      ;;
  esac
done
echo "Mapping workflow complete."
echo "Saved map: $HOME/racecar_maps/${MAP_NAME%.yaml}.yaml"
echo "Detailed run log: $MAPPING_LOG"
