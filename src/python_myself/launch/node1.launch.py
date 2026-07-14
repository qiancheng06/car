import launch
import launch_ros

def generate_launch_description():
    action_myself_node1 = launch_ros.actions.Node(
        package="python_myself",
        executable="node1",
        output="screen",
    )
    # action_myself_node2 = launch_ros.actions.Node(
    #     package="myself_pkg",
    #     executable="myself_node2",
    #     output="screen",
    # )
    # action_myself_node4 = launch_ros.actions.Node(
    #     package="myself_pkg",
    #     executable="myself_node4",
    #     output="screen",
    # )

    launch_description = launch.LaunchDescription([
        action_myself_node1,
        # action_myself_node2,
        #action_myself_node3,
        # action_myself_node4,
    ])
    return launch_description
