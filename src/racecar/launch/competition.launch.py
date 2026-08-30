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
    slam_dir = get_package_share_directory("slam_gmapping")

    use_camera = LaunchConfiguration("use_camera")
    use_gmapping = LaunchConfiguration("use_gmapping")
    use_lidar_tracking = LaunchConfiguration("use_lidar_tracking")
    use_parking_detector = LaunchConfiguration("use_parking_detector")
    use_yolo_vision = LaunchConfiguration("use_yolo_vision")
    use_npu_vision = LaunchConfiguration("use_npu_vision")
    use_yellow_band = LaunchConfiguration("use_yellow_band")
    vision_params = LaunchConfiguration("vision_params")
    vision_model = LaunchConfiguration("vision_model")
    vision_npu_params = LaunchConfiguration("vision_npu_params")
    vision_npu_model = LaunchConfiguration("vision_npu_model")
    yellow_band_params = LaunchConfiguration("yellow_band_params")
    race_manager_params = LaunchConfiguration("race_manager_params")

    # Hardware chain arguments forwarded to Run_car.launch.py. The steering
    # envelope 880-2200 is required so the driver accepts the full 35-145 deg
    # steering space used by race_manager (90 deg maps to the 1500 PWM center).
    legacy_pwm_input = LaunchConfiguration("enable_legacy_pwm_input")
    min_steering_pwm = LaunchConfiguration("min_steering_pwm")
    max_steering_pwm = LaunchConfiguration("max_steering_pwm")

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_gmapping", default_value="true"),
        DeclareLaunchArgument("use_lidar_tracking", default_value="true"),
        DeclareLaunchArgument("use_parking_detector", default_value="false"),
        DeclareLaunchArgument("use_yolo_vision", default_value="false"),
        DeclareLaunchArgument("use_npu_vision", default_value="true"),
        DeclareLaunchArgument("use_yellow_band", default_value="true"),
        DeclareLaunchArgument("enable_legacy_pwm_input", default_value="true"),
        DeclareLaunchArgument("min_steering_pwm", default_value="880.0"),
        DeclareLaunchArgument("max_steering_pwm", default_value="2200.0"),
        DeclareLaunchArgument(
            "yellow_band_params",
            default_value=os.path.join(racecar_dir, "config", "yellow_band.yaml"),
        ),
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
            launch_arguments={
                "enable_legacy_pwm_input": legacy_pwm_input,
                "min_steering_pwm": min_steering_pwm,
                "max_steering_pwm": max_steering_pwm,
            }.items(),
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
            # ★切换跟随节点: cpp_pkg/laser_go (当前主力循迹代码) 替代 lidar_tracking。
            #   laser_go 的输出话题是参数,直接指向 /racecar/follow_cmd_vel,
            #   保持 race_manager 对 /teleop_cmd_vel 的独占仲裁(红灯/黄带/停车)。
            package="cpp_pkg",
            executable="laser_go",
            name="laser_go",
            output="screen",
            parameters=[{"topic": "/racecar/follow_cmd_vel"}],
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
        Node(
            package="racecar",
            executable="yellow_band_node.py",
            name="yellow_band_detector",
            output="screen",
            parameters=[yellow_band_params],
            condition=IfCondition(use_yellow_band),
        ),
    ])
