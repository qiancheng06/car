#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("lslidar_driver")
    driver_params = os.path.join(package_share, "params", "lsx10.yaml")
    rviz_config = os.path.join(package_share, "rviz", "nav2_default_view.rviz")
    use_rviz = LaunchConfiguration("use_rviz")

    driver_node = Node(
        package="lslidar_driver",
        executable="lslidar_driver_node",
        name="lslidar_driver_node",
        output="screen",
        emulate_tty=True,
        namespace="",
        parameters=[driver_params],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="lidar_rviz",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz (disabled by default on the headless Atlas board)",
            ),
            driver_node,
            rviz_node,
        ]
    )
