#!/usr/bin/env bash
# 键盘训练数据录制：JPEG 压缩相机图 + /teleop_cmd_vel 控制量
# 用法: bash record_training.sh          # Ctrl+C 结束录制
#       bash record_training.sh 300      # 录 300 秒后自动结束
# 注意: 先 source 再开 set -u，ROS setup.bash 在 nounset 下会炸
source /opt/ros/humble/setup.bash
source "$HOME/racecar/install/setup.bash"
set -uo pipefail
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_udp.xml"

STAMP=$(date +%Y%m%d-%H%M%S)
OUT="$HOME/racecar_logs/training-$STAMP"
mkdir -p "$OUT"

# 清掉可能残留的压缩转发进程；相机没起就拉起（camer.launch.py 自带 respawn）
pkill -f "image_transport republish" 2>/dev/null
if ! ros2 node list 2>/dev/null | grep -qx '/usb_cam'; then
  echo "[1/3] 启动相机..."
  nohup ros2 launch racecar camer.launch.py </dev/null > "$OUT/camera.log" 2>&1 &
  sleep 6
fi

echo "[2/3] 启动图像 JPEG 压缩转发..."
nohup ros2 run image_transport republish raw in:=/image_raw compressed out:=/image_raw \
  </dev/null > "$OUT/republish.log" 2>&1 &
REPUBLISH_PID=$!
trap 'kill $REPUBLISH_PID 2>/dev/null' EXIT
sleep 3

echo "[3/3] 录制开始: $OUT/bag"
if [[ $# -ge 1 ]]; then
  timeout "$1" ros2 bag record -o "$OUT/bag" /image_raw/compressed /teleop_cmd_vel
  RC=$?
  [[ $RC -eq 124 ]] && RC=0
else
  ros2 bag record -o "$OUT/bag" /image_raw/compressed /teleop_cmd_vel
  RC=$?
fi

echo
echo "录制结束: $OUT"
ros2 bag info "$OUT/bag" | sed -n '1,12p'
exit 0
