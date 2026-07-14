import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    racecar_share = get_package_share_directory("racecar")
    nav2_share = get_package_share_directory("nav2_bringup")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_rviz = LaunchConfiguration("use_rviz")
    start_waypoint_cycle = LaunchConfiguration("start_waypoint_cycle")

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "default_nav_to_pose_bt_xml": os.path.join(
                racecar_share, "behavior_trees", "ackermann_navigate_to_pose.xml"
            ),
            "default_nav_through_poses_bt_xml": os.path.join(
                racecar_share,
                "behavior_trees",
                "ackermann_navigate_through_poses.xml",
            ),
        },
        convert_types=True,
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "params_file": configured_params,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_composition": use_composition,
        }.items(),
    )

    command_adapter = Node(
        package="racecar",
        executable="nav_cmd_adapter.py",
        name="nav_cmd_adapter",
        output="screen",
        parameters=[configured_params],
    )

    waypoint_cycle = Node(
        condition=IfCondition(start_waypoint_cycle),
        package="nav2_waypoint_cycle",
        executable="nav2_waypoint_cycle",
        name="waypoint_cycle",
        output="screen",
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="nav_rviz",
        arguments=["-d", os.path.join(racecar_share, "rviz", "navigation.rviz")],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(racecar_share, "map", "ai_map.yaml"),
                description="Absolute path to the saved map YAML file",
            ),
            DeclareLaunchArgument(
                "params",
                default_value=os.path.join(
                    racecar_share, "config", "nav_astar_teb.yaml"
                ),
                description="Nav2 Hybrid-A* + TEB parameter file",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_composition", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("start_waypoint_cycle", default_value="false"),
            command_adapter,
            nav2_bringup,
            waypoint_cycle,
            rviz,
        ]
    )
