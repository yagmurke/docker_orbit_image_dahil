from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    arduino_control = Node(
            package='arduino_controls',
            executable='arduino_controls_node'
        )
    return LaunchDescription([
        arduino_control
    ])