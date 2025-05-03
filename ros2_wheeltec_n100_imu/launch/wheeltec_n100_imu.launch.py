from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheeltec_n100_imu',
            executable='imu_node',
            name='imu_node'
        ),
    ])