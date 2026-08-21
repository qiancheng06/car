import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    racecar_dir = get_package_share_directory("racecar")
    default_model = os.path.expanduser(
        "~/racecar/vision_data/models/runs/v8_exp/weights/best.pt"
    )
    default_params = os.path.join(racecar_dir, "config", "vision.yaml")

    use_camera = LaunchConfiguration("use_camera")
    video_device = LaunchConfiguration("video_device")
    model_path = LaunchConfiguration("model_path")
    vision_params = LaunchConfiguration("vision_params")
    debug_image = LaunchConfiguration("debug_image")

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("model_path", default_value=default_model),
        DeclareLaunchArgument("vision_params", default_value=default_params),
        DeclareLaunchArgument("debug_image", default_value="false"),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            output="screen",
            condition=IfCondition(use_camera),
            parameters=[{
                "video_device": video_device,
                "framerate": 30.0,
                "io_method": "mmap",
                "frame_id": "camera_link",
                "pixel_format": "mjpeg2rgb",
                "av_device_format": "YUV422P",
                "image_width": 640,
                "image_height": 480,
                "camera_name": "racecar_camera",
            }],
        ),
        Node(
            package="racecar",
            executable="yolo_vision_node.py",
            name="yolo_vision",
            output="screen",
            parameters=[vision_params, {
                "model_path": model_path,
                "publish_debug_image": debug_image,
            }],
        ),
    ])
