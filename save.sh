#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash does not exist." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi

MAP_NAME="${1:-ai_map}"
MAP_NAME="${MAP_NAME%.yaml}"
MAP_NAME="${MAP_NAME%.pgm}"

if [[ "$MAP_NAME" == */* ]]; then
  MAP_OUTPUT="$MAP_NAME"
else
  MAP_OUTPUT="$SCRIPT_DIR/src/racecar/map/$MAP_NAME"
fi

if ! ros2 node list 2>/dev/null | grep -qx '/slam_gmapping'; then
  echo "ERROR: /slam_gmapping is not running. Refusing to overwrite a map." >&2
  exit 1
fi
if ! ros2 topic list 2>/dev/null | grep -qx '/map'; then
  echo "ERROR: gmapping has not published /map yet." >&2
  exit 1
fi

MAP_DIR="$(dirname "$MAP_OUTPUT")"
MAP_BASE="$(basename "$MAP_OUTPUT")"
mkdir -p "$MAP_DIR"
TEMP_DIR="$(mktemp -d "$MAP_DIR/.${MAP_BASE}.save.XXXXXX")"
TEMP_OUTPUT="$TEMP_DIR/$MAP_BASE"

cleanup_temp() {
  rm -f -- "$TEMP_OUTPUT.yaml" "$TEMP_OUTPUT.pgm"
  rmdir -- "$TEMP_DIR" 2>/dev/null || true
}
trap cleanup_temp EXIT

echo "Saving gmapping output to: $MAP_OUTPUT.yaml and $MAP_OUTPUT.pgm"
ros2 run nav2_map_server map_saver_cli -f "$TEMP_OUTPUT"

if [[ ! -s "$TEMP_OUTPUT.yaml" || ! -s "$TEMP_OUTPUT.pgm" ]]; then
  echo "ERROR: temporary map files were not created correctly." >&2
  exit 1
fi

# Move the image first and the YAML last so consumers never see a new YAML
# pointing at an image that has not arrived yet.
mv -f -- "$TEMP_OUTPUT.pgm" "$MAP_OUTPUT.pgm"
mv -f -- "$TEMP_OUTPUT.yaml" "$MAP_OUTPUT.yaml"
rmdir -- "$TEMP_DIR"
trap - EXIT
echo "Map saved successfully."
