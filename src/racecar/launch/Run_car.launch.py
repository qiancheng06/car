import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('racecar')
    launch_dir = os.path.join(bringup_dir, 'launch')
    robot_description = Path(
        bringup_dir, 'urdf', 'racecar.urdf').read_text(encoding='utf-8')
        
    ekf_config = Path(get_package_share_directory('racecar'), 'config', 'ekf.yaml')

    ekf_carto_config = Path(get_package_share_directory('racecar'), 'config', 'ekf_carto.yaml')
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')
    legacy_pwm_input = LaunchConfiguration('enable_legacy_pwm_input')
    legacy_normalized_input = LaunchConfiguration('enable_legacy_normalized_input')
    min_throttle_pwm = LaunchConfiguration('min_throttle_pwm')
    max_throttle_pwm = LaunchConfiguration('max_throttle_pwm')
    min_steering_pwm = LaunchConfiguration('min_steering_pwm')
    max_steering_pwm = LaunchConfiguration('max_steering_pwm')
    start_encoder_imu = LaunchConfiguration('start_encoder_imu')
    legacy_pwm_input_dec = DeclareLaunchArgument(
        'enable_legacy_pwm_input', default_value='false')
    legacy_normalized_input_dec = DeclareLaunchArgument(
        'enable_legacy_normalized_input', default_value='false')
    min_throttle_pwm_dec = DeclareLaunchArgument(
        'min_throttle_pwm', default_value='1500.0',
        description='Minimum allowed throttle PWM; use 1475 only for supervised manual reverse')
    max_throttle_pwm_dec = DeclareLaunchArgument(
        'max_throttle_pwm', default_value='1550.0',
        description='Maximum allowed throttle PWM')
    min_steering_pwm_dec = DeclareLaunchArgument(
        'min_steering_pwm', default_value='1200.0',
        description='Minimum allowed steering PWM')
    max_steering_pwm_dec = DeclareLaunchArgument(
        'max_steering_pwm', default_value='1800.0',
        description='Maximum allowed steering PWM')
    start_encoder_imu_dec = DeclareLaunchArgument(
        'start_encoder_imu', default_value='false',
        description='Start the legacy /encoder_imu_odom fusion node')

    # imu
    lslidar_driver_share_dir = get_package_share_directory('lslidar_driver')
    imu_launch_share_dir = get_package_share_directory('hipnuc_imu')
    # 声明参数
    imu_package_arg = DeclareLaunchArgument(
        'imu_package', default_value='spec',
        description='package type [spec, 0x91]'
    )

    imu_package = LaunchConfiguration('imu_package')
            
     
    robot_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf.launch.py')),
        launch_arguments={'carto_slam':carto_slam}.items(),            
    )
                                                            
        
    # imu_filter_node =  launch_ros.actions.Node(
    #     package='hipnuc_imu',
    #     executable='imu_filter_node',
    #     name='imu_filter_node',
    # )

        
                           
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )


    return LaunchDescription([
        robot_ekf,
        carto_slam_dec, legacy_pwm_input_dec, legacy_normalized_input_dec,
        min_throttle_pwm_dec, max_throttle_pwm_dec,
        min_steering_pwm_dec, max_steering_pwm_dec,
        start_encoder_imu_dec,
        robot_state_publisher_node,imu_package_arg,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lslidar_driver_share_dir, 'launch', 'lslidar_launch.py')
            )
        ),

        # Include IMU launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(imu_launch_share_dir, 'launch', 'imu_'), imu_package, '_msg.launch.py']
            )
        ),

        # imu_filter_node,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2base_link',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2laser_link',
            arguments=['--x', '0.07', '--y', '0.0', '--z', '0.0', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_footprint', '--child-frame-id', 'laser_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2imu',
            arguments=['--x', '0.1653', '--y', '0.0', '--z', '0.0', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_footprint', '--child-frame-id', 'IMU_link']
        ),

        Node(
            package='encoder',
            executable='encoder_node',
            name='encoder_vel',
            output='screen',
        ),
        Node(
            condition=IfCondition(start_encoder_imu),
            package='encoder',
            executable='encoder_imu_node',
            name='encoder_imu',
            output='screen',
            ros_arguments=['--log-level', 'warn'],
        ),
        Node(
            package='racecar_driver',
            executable='racecar_driver_node',
            name='racecar_driver',
            output='screen',
            parameters=[{
                'enable_legacy_inputs': False,
                'enable_legacy_pwm_input': ParameterValue(
                    legacy_pwm_input, value_type=bool),
                'enable_legacy_normalized_input': ParameterValue(
                    legacy_normalized_input, value_type=bool),
                'command_topic': '/racecar_driver/cmd_pwm',
                'estop_topic': '/nav/estop',
                'min_throttle_pwm': ParameterValue(
                    min_throttle_pwm, value_type=float),
                'max_throttle_pwm': ParameterValue(
                    max_throttle_pwm, value_type=float),
                'min_steering_pwm': ParameterValue(
                    min_steering_pwm, value_type=float),
                'max_steering_pwm': ParameterValue(
                    max_steering_pwm, value_type=float),
                'teleop_zero_degree_pwm': 2500.0,
                'neutral_steering_pwm': 1500.0,
            }],
        ),
    ])

