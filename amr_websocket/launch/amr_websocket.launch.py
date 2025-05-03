from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_websocket',
            executable='action_server',
        ),
        Node(
            package='amr_websocket',
            executable='tf_listener',
            name='tf_listener'
        ),
        Node(
            package='amr_websocket',
            executable='sensor_control',
            name='sensor_control'
        ),
        Node(
            package='amr_websocket',
            executable='file_sender',
            name='file_sender'
        ),
        Node(
            package='amr_websocket',
            executable='delivery_control',
        ),
        Node(
            package='amr_websocket',
            executable='auto_dock',
        ),
        Node(
            package='amr_websocket',
            executable='map_publisher',
        ),
        Node(
            package='amr_websocket',
            executable='keepout_point_publisher',
        ),
        # Node(
        #     package='amr_websocket',
        #     executable='path_publisher',
        # ),
        
    ])