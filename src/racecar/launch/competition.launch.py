import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    racecar_dir = get_package_share_directory("racecar")
    lslidar_tracking_dir = get_package_share_directory("lidar_tracking")
    slam_dir = get_package_share_directory("slam_gmapping")

    use_camera = LaunchConfiguration("use_camera")
    use_gmapping = LaunchConfiguration("use_gmapping")
    use_lidar_tracking = LaunchConfiguration("use_lidar_tracking")
    use_parking_detector = LaunchConfiguration("use_parking_detector")
    use_yolo_vision = LaunchConfiguration("use_yolo_vision")
    use_npu_vision = LaunchConfiguration("use_npu_vision")
    vision_params = LaunchConfiguration("vision_params")
    vision_model = LaunchConfiguration("vision_model")
    vision_npu_params = LaunchConfiguration("vision_npu_params")
    vision_npu_model = LaunchConfiguration("vision_npu_model")
    race_manager_params = LaunchConfiguration("race_manager_params")

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_gmapping", default_value="true"),
        DeclareLaunchArgument("use_lidar_tracking", default_value="true"),
        DeclareLaunchArgument("use_parking_detector", default_value="false"),
        DeclareLaunchArgument("use_yolo_vision", default_value="false"),
        DeclareLaunchArgument("use_npu_vision", default_value="true"),
        DeclareLaunchArgument(
            "race_manager_params",
            default_value=os.path.join(racecar_dir, "config", "race_manager.yaml"),
        ),
        DeclareLaunchArgument(
            "vision_params",
            default_value=os.path.join(racecar_dir, "config", "vision.yaml"),
        ),
        DeclareLaunchArgument(
            "vision_model",
            default_value=os.path.expanduser(
                "~/racecar/vision_data/models/runs/v8_exp/weights/best.pt"
            ),
        ),
        DeclareLaunchArgument(
            "vision_npu_params",
            default_value=os.path.join(racecar_dir, "config", "vision_npu.yaml"),
        ),
        DeclareLaunchArgument(
            "vision_npu_model",
            default_value=os.path.expanduser(
                "~/racecar/vision_data/models/runs/v8_exp/weights/best_310b.om"
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(racecar_dir, "launch", "Run_car.launch.py")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(racecar_dir, "launch", "camer.launch.py")),
            condition=IfCondition(use_camera),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_dir, "launch", "slam_gmapping.launch.py")),
            condition=IfCondition(use_gmapping),
        ),

        Node(
            package="lidar_tracking",
            executable="lidar_tracking",
            name="lidar_tracking",
            output="screen",
            parameters=[os.path.join(lslidar_tracking_dir, "config", "params.yaml")],
            remappings=[
                ("/teleop_cmd_vel", "/racecar/follow_cmd_vel"),
                ("teleop_cmd_vel", "/racecar/follow_cmd_vel"),
            ],
            condition=IfCondition(use_lidar_tracking),
        ),

        Node(
            package="racecar",
            executable="race_manager.py",
            name="race_manager",
            output="screen",
            parameters=[race_manager_params],
        ),

        Node(
            package="racecar",
            executable="parking_sign_detector.py",
            name="parking_sign_detector",
            output="screen",
            parameters=[race_manager_params],
            condition=IfCondition(use_parking_detector),
        ),
        Node(
            package="racecar",
            executable="yolo_vision_node.py",
            name="yolo_vision",
            output="screen",
            parameters=[vision_params, {"model_path": vision_model}],
            condition=IfCondition(use_yolo_vision),
        ),
        Node(
            package="racecar",
            executable="npu_vision_node.py",
            name="npu_vision",
            output="screen",
            parameters=[vision_npu_params, {"model_path": vision_npu_model}],
            condition=IfCondition(use_npu_vision),
        ),
    ])
