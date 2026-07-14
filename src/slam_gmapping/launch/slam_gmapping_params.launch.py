from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'config',
        'gmapping_params.yaml'
    )

    return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        )
    ])
