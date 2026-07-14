import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('racecar')
    run_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'Run_car.launch.py')
        )
    )

   # bus_control_node = Node(
    #    package='racecar',
     #   executable='bus_control.py',
      #  name='bus_control',
       # output='screen'
    #)

    car_test_node = Node(
        package='racecar',
        executable='car_test',
        name='car_test',
        output='screen'
    )

    car_controller_node = Node(
        package='racecar',
        executable='car_controller_new',
        name='car_controller',
        output='screen'
    )

    return LaunchDescription([
        run_car,
        #bus_control_node,
        car_test_node,
        car_controller_node,
    ])
