#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

TEB_URL="https://github.com/rst-tu-dortmund/teb_local_planner.git"
TEB_COMMIT="630a22e88dc9fd45be726a762edbb5b776bef231"
TEB_DIR="$SCRIPT_DIR/src/teb_local_planner_ros2"
COSTMAP_URL="https://github.com/rst-tu-dortmund/costmap_converter.git"
COSTMAP_COMMIT="b1a7d891f71cc346f452b786e36f4637b47a02c6"
COSTMAP_DIR="$SCRIPT_DIR/src/costmap_converter_ros2"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: this workspace is configured for ROS2 Humble." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash

for required_command in git rosdep colcon; do
  if ! command -v "$required_command" >/dev/null 2>&1; then
    echo "ERROR: required command '$required_command' is not installed." >&2
    exit 1
  fi
done

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "ERROR: rosdep is not initialized. Run:" >&2
  echo "  sudo rosdep init" >&2
  echo "  rosdep update" >&2
  exit 1
fi

if [[ ! -d "$TEB_DIR/.git" ]]; then
  echo "Cloning the official ROS2 Humble TEB source..."
  git clone --filter=blob:none --no-checkout "$TEB_URL" "$TEB_DIR"
fi
git -C "$TEB_DIR" remote set-url origin "$TEB_URL"

echo "Checking out the pinned TEB Humble revision..."
git -C "$TEB_DIR" fetch --depth 1 origin "$TEB_COMMIT"
git -C "$TEB_DIR" checkout --detach "$TEB_COMMIT"

# TEB's Humble branch requires the unreleased ROS2 costmap_converter packages.
if [[ ! -d "$COSTMAP_DIR/.git" ]]; then
  echo "Cloning the ROS2 costmap converter source..."
  mkdir -p "$COSTMAP_DIR"
  git -C "$COSTMAP_DIR" init
  git -C "$COSTMAP_DIR" remote add origin "$COSTMAP_URL"
fi
git -C "$COSTMAP_DIR" remote set-url origin "$COSTMAP_URL"

echo "Checking out the pinned Humble-compatible costmap converter revision..."
git -C "$COSTMAP_DIR" fetch --depth 1 origin refs/pull/27/head
git -C "$COSTMAP_DIR" checkout --detach "$COSTMAP_COMMIT"

echo "Installing declared ROS dependencies..."
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  --recursive \
  --yes

echo "Building TEB and the modified navigation/driver packages..."
colcon build \
  --symlink-install \
  --parallel-workers 2 \
  --packages-up-to racecar

echo "Build complete. Load this workspace with:"
echo "  source $SCRIPT_DIR/install/setup.bash"
echo "Then run the safe navigation stack with:"
echo "  bash $SCRIPT_DIR/nav.sh"
