import launch
import launch_ros


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="python_myself",
            executable="node2d",
            output="screen",
        ),
    ])
