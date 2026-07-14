#!/usr/bin/env python3
"""
激光雷达循迹ROS2包启动文件
功能：启动激光雷达循迹节点，配置相关参数
作者：zyh
版本：1.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    生成ROS2启动描述
    
    Returns:
        LaunchDescription: 包含节点配置的启动描述对象
    """
    return LaunchDescription([
        Node(
            package='lidar_tracking',                    # 包名：激光雷达循迹包
            executable='lidar_tracking',                 # 可执行文件名：与CMakeLists.txt中保持一致
            name='lidar_tracking',                       # 节点名称：用于标识节点
            output='screen',                             # 输出方式：在终端显示日志信息
            
            # 节点参数配置
            parameters=[{
                # ===== 基础控制参数 =====
                'topic': 'teleop_cmd_vel',               # 发布话题：速度控制指令话题
                # 'topic': 'car/cmd_vel',                # 备用话题：小车速度控制话题（已注释）
                
                # ===== PID控制参数 =====
                'speed': 1535,                           # 基础行驶速度：小车正常循迹时的线速度
                'kp': 0.93,                              # 比例系数：PID控制器的比例增益
                'ki': 0.0,                               # 积分系数：PID控制器的积分增益（当前未使用）
                'kd': 1.4,                               # 微分系数：PID控制器的微分增益
                
                # ===== 锥桶检测参数 =====
                'max_right_dis': 1.5,                    # 最大右侧距离：检测锥桶的最大横向距离
                
                # ===== 路径规划权重参数 =====
                'rate1': 0.8,                            # 第一对锥桶权重：最近锥桶对的路径规划权重
                'rate2': 2.2,                            # 第二对锥桶权重：中等距离锥桶对的路径规划权重（最重要）
                'rate3': 0.75,                           # 第三对锥桶权重：较远锥桶对的路径规划权重
                
                # ===== 无锥桶巡航状态参数 =====
                'no_cone_detection_range': 1.0,          # 无锥桶检测范围：1米范围内无锥桶时进入巡航状态
                'no_cone_speed': 1525,                   # 无锥桶状态速度：巡航搜索时的线速度（比正常速度稍慢）
                'no_cone_turn_angle': 120.0,             # 无锥桶状态转向角：巡航搜索时的固定转向角度（左转）
            }]
        )
    ])