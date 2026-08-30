from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明摄像头设备参数
    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='Video device path for the USB camera'
    )
    video_device = LaunchConfiguration('video_device')

    # 启动 usb_cam 节点
    # respawn: 8月29日试跑中 usb_cam 崩过一次("Invalid v4l2 format", mjpeg 解码),
    # 之后 /image_raw 断流导致视觉全程致盲——必须自动拉起。
    # YUYV: 该模组 YUYV 640x480@30 带宽 ~18MB/s,USB2.0 扛得住,绕开 mjpeg 解码崩溃路径。
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': video_device,
            'framerate': 30.0,
            'io_method': 'mmap',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv2rgb',
            'av_device_format': 'YUV422P',
            'frame_id': 'camera_link'
        }],
        respawn=True,
        respawn_delay=2.0,
        output='screen'
    )

    # 创建 LaunchDescription 并添加动作
    ld = LaunchDescription()
    ld.add_action(video_device_arg)
    ld.add_action(usb_cam_node)

    return ld

#查看摄像头路径：ls /dev/video*
# 安装usb_cam：
#     sudo apt update
#     sudo apt install ros-humble-usb-cam
# 启动摄像头：ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py
# ros2 topic list
#显示画面：ros2 run rqt_image_view rqt_image_view
