from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    declare_launch_camera = DeclareLaunchArgument('launch_camera', default_value='true')
    rplidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rplidar_ros'),
                    'launch',
                    'rplidar_s2_launch.py'
                ])
            ]),
        )
    motor_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orbit_driver'),
                    'launch',
                    'orbit_driver.launch.py'
                ])
            ]),
        )
    wheeltec_n100_imu = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('wheeltec_n100_imu'),
                    'launch',
                    'wheeltec_n100_imu.launch.py'
                ])
            ]),
        )
    
    arduino_controls = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('arduino_controls'),
                    'launch',
                    'arduino_control.launch.py'
                ])
            ]),
        )
    
    camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration("launch_camera"))
        )
    state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orbit'),
                    'launch',
                    'sensor_bringup',
                    'robot_state_publisher.launch.py'
                ])
            ]),
        )
    robot_localization = Node(
            package='robot_localization',
            executable='ekf_node',
            parameters=[os.path.join(get_package_share_directory("orbit"), 'config', 'ekf.yaml'),],
            remappings=[('/odometry/filtered', '/odom')]
        )
    return LaunchDescription([
        declare_launch_camera,
        rplidar,
        motor_driver,
        wheeltec_n100_imu,
        camera,
        state_publisher
    ])
