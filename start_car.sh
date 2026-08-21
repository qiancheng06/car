#!/bin/bash
# start_car.sh  一键启动小车 ROS 系统(幂等:重复运行会先清理旧的再启动)
# 用法: bash ~/racecar/start_car.sh
set -e

echo "== 清理旧 ROS 进程 =="
pkill -f 'ros2 launch' 2>/dev/null || true
pkill -f lslidar_driver_node 2>/dev/null || true
pkill -f ekf_node 2>/dev/null || true
pkill -f encoder_node 2>/dev/null || true
pkill -f racecar_driver_node 2>/dev/null || true
pkill -f hipnuc 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
sleep 3
pkill -9 -f lslidar_driver_node 2>/dev/null || true
pkill -9 -f racecar_driver_node 2>/dev/null || true
sleep 1

echo "== 启动 Run_car.launch.py =="
source /opt/ros/humble/setup.bash
source ~/racecar/install/setup.bash

nohup ros2 launch racecar Run_car.launch.py \
  enable_legacy_pwm_input:=true \
  min_steering_pwm:=880 max_steering_pwm:=2200 \
  > /tmp/run_car.log 2>&1 &
echo "launch 已在后台启动,日志: /tmp/run_car.log"
echo "等待 12 秒确认 /scan ..."
sleep 12
echo "== 状态检查 =="
echo "lslidar 进程数(应为 1): $(ps aux | grep lslidar_driver_node | grep -v grep | wc -l)"
echo "racecar_driver 进程数(应为 1): $(ps aux | grep racecar_driver_node | grep -v grep | wc -l)"
source /opt/ros/humble/setup.bash 2>/dev/null || true
timeout 5 ros2 topic hz /scan --window 3 2>/dev/null | head -1 || echo "( /scan 未发布,请查看 /tmp/run_car.log )"
echo "== 完成 =="
