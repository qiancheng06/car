from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    car_test_params = {
        'cv_cmd_topic': '/cv_cmd_vel',
        'start_with_cv_phase': True,
    }

    cv_follow_params = {
        'output_topic': '/cv_cmd_vel',
    }

    car_test_node = Node(
        package='racecar_twoloop',
        executable='car_test',
        name='car_test',
        output='screen',
        parameters=[car_test_params],
    )

    cv_follow_node = Node(
        package='racecar_twoloop',
        executable='cv_follow_node',
        name='cv_follow_node',
        output='screen',
        parameters=[cv_follow_params],
    )

    return LaunchDescription([
        car_test_node,
        cv_follow_node,
    ])
