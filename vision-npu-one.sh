#!/usr/bin/env bash
set -Eeo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash

MODEL="$ROOT/vision_data/models/runs/v8_exp/weights/best_310b.om"
PARAMS="$ROOT/install/racecar/share/racecar/config/vision_npu.yaml"
if [ ! -f "$MODEL" ]; then
  echo "ERROR: NPU model not found: $MODEL" >&2
  exit 1
fi

LOCK="${XDG_RUNTIME_DIR:-/tmp}/racecar_vision.lock"
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "ERROR: another racecar vision process is running." >&2
  exit 1
fi

CAMERA_PID=""
cleanup() {
  if [ -n "$CAMERA_PID" ]; then
    kill -INT "$CAMERA_PID" 2>/dev/null || true
    wait "$CAMERA_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ! ros2 topic info /image_raw 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
  ros2 launch racecar camer.launch.py video_device:=/dev/video0 &
  CAMERA_PID=$!
  sleep 3
fi

echo "Starting Ascend NPU vision as davinci-mini (NPU device groups enabled)."
echo "Model: $MODEL"
echo "Viewer topic: /racecar/vision/debug_image"
echo "Press Ctrl+C to stop."

source /usr/local/Ascend/ascend-toolkit/set_env.sh
ros2 run racecar npu_vision_node.py \
  --ros-args --params-file "$PARAMS" -p model_path:="$MODEL"
