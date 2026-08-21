#!/usr/bin/env bash
# ============================================================
# 两圈自动导航：第一圈 gmapping 建图（锥桶走廊驾驶），
# 第二圈 A* 全局 + DWA 局部导航回原点。
# 用法: bash two_lap.sh
# ============================================================
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/davinci-mini/fastdds_udp.xml
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

MAP_FILE="$HOME/racecar_maps/ai_map.yaml"
DWA_PARAMS="$SCRIPT_DIR/src/racecar/config/nav_dwa.yaml"
LOG_DIR="$HOME/racecar_logs"
mkdir -p "$LOG_DIR"

echo "================ 两圈自动导航开始 ================"

# ---------- 清理旧栈 ----------
echo "[0/6] 清理旧进程..."
pkill -f nav-one.sh 2>/dev/null || true
pkill -f gmapping.sh 2>/dev/null || true
pkill -f car.sh 2>/dev/null || true
pkill -f lidar_tracking 2>/dev/null || true
sleep 3
pkill -9 -f "[l]slidar" 2>/dev/null || true
pkill -9 -f "[e]kf_node" 2>/dev/null || true
pkill -9 -f "[e]ncoder_node" 2>/dev/null || true
pkill -9 -f "[r]acecar_driver" 2>/dev/null || true
pkill -9 -f "[h]ipnuc" 2>/dev/null || true
pkill -9 -f "[s]tatic_transform" 2>/dev/null || true
pkill -9 -f "[r]obot_state" 2>/dev/null || true
pkill -9 -f "[n]av2_container" 2>/dev/null || true
pkill -9 -f "[r]os2_daemon" 2>/dev/null || true
sleep 2
echo "    清理完成"

# ---------- 第一圈：建图 ----------
echo "[1/6] 启动建图栈 (gmapping.sh)..."
nohup bash gmapping.sh </dev/null > "$LOG_DIR/lap1_mapping.log" 2>&1 &
GMAP_PID=$!
for i in $(seq 1 60); do
  if grep -q "Mapping is ready" "$LOG_DIR/lap1_mapping.log" 2>/dev/null; then
    break
  fi
  if ! kill -0 "$GMAP_PID" 2>/dev/null; then
    echo "错误: gmapping.sh 启动失败"; tail -10 "$LOG_DIR/lap1_mapping.log"; exit 1
  fi
  sleep 2
done
echo "    建图栈就绪"

echo "[2/6] 第一圈行驶（锥桶走廊跟随 + 建图）..."
echo "    遥控杆/急停就位，车将自动沿锥桶行驶，回到起点后自动停止"
set +e
timeout 1800 python3 "$SCRIPT_DIR/src/racecar/scripts/two_lap_lap1.py" 2>&1 | tee "$LOG_DIR/lap1_drive.log"
LAP1_RC=${PIPESTATUS[0]}
set -e
if [ "$LAP1_RC" -ne 0 ]; then
  echo "错误: 第一圈未完成(退出码 $LAP1_RC)，流程中止"
  echo "    重新运行 bash two_lap.sh"
  exit 1
fi
echo "    第一圈行驶结束"

# ---------- 保存地图 ----------
echo "[3/6] 保存地图..."
if timeout 60 bash save.sh > "$LOG_DIR/lap1_save.log" 2>&1; then
  echo "    地图已保存: $MAP_FILE"
else
  echo "警告: save.sh 失败，尝试直接保存"
  timeout 60 ros2 run nav2_map_server map_saver_cli -f "$MAP_FILE" --ros-args \
    -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0 \
    >> "$LOG_DIR/lap1_save.log" 2>&1 || echo "错误: 地图保存失败"
fi

# ---------- 停止建图 ----------
echo "[4/6] 停止建图栈..."
kill -TERM "$GMAP_PID" 2>/dev/null || true
sleep 8
pkill -9 -f "[l]slidar" 2>/dev/null || true
pkill -9 -f "[e]kf_node" 2>/dev/null || true
pkill -9 -f "[e]ncoder_node" 2>/dev/null || true
pkill -9 -f "[r]acecar_driver" 2>/dev/null || true
pkill -9 -f "[h]ipnuc" 2>/dev/null || true
pkill -9 -f "[s]tatic_transform" 2>/dev/null || true
pkill -9 -f "[r]obot_state" 2>/dev/null || true
pkill -9 -f "[r]os2_daemon" 2>/dev/null || true
sleep 3
echo "    建图栈已停止"

# ---------- 第二圈：导航 ----------
echo "[5/6] 启动导航栈 (A* 全局 + DWA 局部)..."
nohup bash nav-one.sh use_composition:=True params:="$DWA_PARAMS" \
  </dev/null > "$LOG_DIR/lap2_nav.log" 2>&1 &
NAV_PID=$!
for i in $(seq 1 90); do
  if grep -q "Navigation is active" "$LOG_DIR/lap2_nav.log" 2>/dev/null; then
    break
  fi
  if ! kill -0 "$NAV_PID" 2>/dev/null; then
    echo "错误: nav-one.sh 启动失败"; tail -15 "$LOG_DIR/lap2_nav.log"; exit 1
  fi
  sleep 2
done
echo "    导航栈就绪，等待定位..."
for i in $(seq 1 60); do
  TF_OUT="$(timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 || true)"
  if echo "$TF_OUT" | grep -q "Translation"; then
    break
  fi
  sleep 2
done
echo "    定位建立"

echo "[6/6] 发送第二圈目标：导航回原点 (0,0,0)..."
python3 - "$MAP_FILE" <<'PYEOF' || echo "提示: 目标发送失败，请手动发送"
import rclpy, sys
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

rclpy.init()
n = Node("lap2_goal")
client = ActionClient(n, NavigateToPose, "/navigate_to_pose")
if not client.wait_for_server(timeout_sec=30.0):
    print("错误: navigate_to_pose 服务不可用")
    sys.exit(1)
goal = NavigateToPose.Goal()
goal.pose = PoseStamped()
goal.pose.header.frame_id = "map"
goal.pose.header.stamp.sec = 0
goal.pose.pose.position.x = 0.0
goal.pose.pose.position.y = 0.0
goal.pose.pose.orientation.w = 1.0
print("发送目标: 原点 (0, 0, 0)")
future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(n, future, timeout_sec=15.0)
if future.done() and future.result().accepted:
    print("目标已接受，等待到达...")
    rf = future.result().get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=300.0)
    print("导航结果:", rf.result().status if rf.done() else "超时")
else:
    print("目标被拒绝")
PYEOF

echo ""
echo "================ 两圈流程结束 ================"
echo "日志: $LOG_DIR/"
echo "急停: ros2 topic pub --once /nav/estop std_msgs/msg/Bool '{data: true}'"
