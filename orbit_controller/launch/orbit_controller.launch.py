from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    orbit_controller = Node(
        package='orbit_controller',
        executable='orbit_movement'
    )
    bs64totext_node = Node(
        package='orbit_controller',
        executable='bs64totext_node'
    )



    # LaunchDescription'a ROS2 nodları ve Python dosyasını ekliyoruz
    return LaunchDescription([
        orbit_controller,
        bs64totext_node
    ])
