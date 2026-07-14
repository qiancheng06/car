#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi

python3 -m py_compile \
  src/racecar/scripts/nav_cmd_adapter.py \
  src/racecar/scripts/racecar_teleop.py \
  src/racecar/launch/Run_nav.launch.py \
  src/racecar/launch/Run_car.launch.py \
  src/lslidar_driver/launch/lslidar_launch.py

python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

config = Path("src/racecar/config/nav_astar_teb.yaml")
data = yaml.safe_load(config.read_text(encoding="utf-8"))
planner = data["planner_server"]["ros__parameters"]["GridBased"]["plugin"]
controller = data["controller_server"]["ros__parameters"]["FollowPath"]["plugin"]
assert planner == "nav2_smac_planner/SmacPlannerHybrid", planner
assert controller == "teb_local_planner::TebLocalPlannerROS", controller
adapter = data["nav_cmd_adapter"]["ros__parameters"]
assert adapter["output_topic"] == "/racecar_driver/cmd_pwm"
behaviors = data["behavior_server"]["ros__parameters"]["behavior_plugins"]
assert behaviors == ["wait"], behaviors
for tree in Path("src/racecar/behavior_trees").glob("*.xml"):
    root = ET.parse(tree).getroot()
    tags = {element.tag for element in root.iter()}
    assert "Spin" not in tags and "BackUp" not in tags, (tree, tags)
print("YAML OK: Hybrid-A* + TEB")
PY

if ros2 pkg prefix teb_local_planner >/dev/null 2>&1; then
  echo "TEB package: FOUND"
else
  echo "TEB package: MISSING (run bash setup_teb.sh)" >&2
  exit 2
fi

if ros2 pkg prefix costmap_converter >/dev/null 2>&1; then
  echo "costmap_converter package: FOUND"
else
  echo "costmap_converter package: MISSING (run bash setup_teb.sh)" >&2
  exit 2
fi

ros2 launch racecar Run_nav.launch.py --show-args >/dev/null
echo "Launch parsing: OK"

for entrypoint in car.sh gmapping.sh save.sh nav-one.sh; do
  [[ -x "$entrypoint" ]] || echo "NOTE: run with 'bash $entrypoint' (execute bit is not required)"
  bash -n "$entrypoint"
done
echo "Static navigation checks passed."
