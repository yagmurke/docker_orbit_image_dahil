from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbit_driver',
            executable='orbit_driver_nodee',
            name='orbit_driver_nodee'
        ),
        Node(
            package='orbit_driver',
            executable='batterry_smooth_node',
            name='batterry_smooth_node'
        ),
        Node(
            package='orbit_driver',
            executable='imu_status_node',
            name='imu_status_node'
        ),
    ])

