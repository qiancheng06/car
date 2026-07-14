import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    racecar_share = get_package_share_directory('racecar')
    run_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(racecar_share, 'launch', 'Run_car.launch.py'))
    )

    cv_node = Node(
        package='two_lap_control',
        executable='cv_follow_node',
        name='cv_follow_node',
        output='screen',
        parameters=[{'auto_start': False}]
    )

    car_test_node = Node(
        package='two_lap_control',
        executable='car_test_node',
        name='car_test_node',
        output='screen'
    )

    return LaunchDescription([
        run_car,
        cv_node,
        car_test_node,
    ])
