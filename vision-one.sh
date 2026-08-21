#!/usr/bin/env bash
set -Eeo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

LOCK="${XDG_RUNTIME_DIR:-/tmp}/racecar_vision.lock"
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "ERROR: racecar vision is already running." >&2
  exit 1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
if [ ! -f install/setup.bash ]; then
  echo "ERROR: workspace is not built. Run: colcon build --packages-select racecar" >&2
  exit 1
fi
source install/setup.bash

MODEL="$ROOT/vision_data/models/runs/v8_exp/weights/best.pt"
if [ ! -f "$MODEL" ]; then
  echo "ERROR: deployed model not found: $MODEL" >&2
  exit 1
fi

USE_CAMERA=true
if ros2 topic info /image_raw 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
  USE_CAMERA=false
  echo "Using the existing /image_raw publisher."
elif [ ! -e /dev/video0 ]; then
  echo "ERROR: /dev/video0 is missing and /image_raw has no publisher." >&2
  exit 1
fi

echo "Starting racecar YOLO vision (perception only; chassis is not armed)."
echo "Camera source: $([ "$USE_CAMERA" = true ] && echo /dev/video0 || echo /image_raw)"
echo "Detection topics:"
echo "  /racecar/red_light_detected"
echo "  /racecar/parking_sign"
echo "  /racecar/cones"
echo "  /racecar/vision/detections"
echo "Press Ctrl+C to stop."

exec ros2 launch racecar vision.launch.py use_camera:="$USE_CAMERA" "$@"
