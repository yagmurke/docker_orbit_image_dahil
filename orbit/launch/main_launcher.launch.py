import os
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pid = LaunchConfiguration('pid')
    declare_pid = DeclareLaunchArgument('pid', default_value="'999'")
    sensor_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orbit'),
                    'launch',
                    'sensor_bringup',
                    'sensor_bringup.launch.py'
                ])
            ]),
            launch_arguments={
                'launch_camera': 'true'
            }.items(),
        )
    
    sensor_fusion = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orbit'),
                    'launch',
                    'sensor_bringup',
                    'sensor_fusion.launch.py'
                ])
            ]),
            launch_arguments={
                'launch_pc_lc': 'true'
            }.items(),
        )
    
    # custom_nodes = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             PathJoinSubstitution([
    #                 FindPackageShare('orbit'),
    #                 'launch',
    #                 'custom_nodes.launch.py'
    #             ])
    #         ]),
    #     )
    
    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orbit'),
                    'launch',
                    'nav2_launchers',
                    'nav2.launch.py'
                ])
            ]),
        )
    return LaunchDescription([   
        declare_pid, 
        sensor_bringup,
        sensor_fusion,
        # custom_nodes,
        # nav2
    ])
