from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    chatbot_node = ExecuteProcess(
        cmd=['python3', os.path.join(os.path.expanduser('~'), 'orbit_ws', 'src', 'chatbot', 'chatbot', 'main.py')],
        shell=True,
        name='chatbot_node',
        output='screen'
    )


    return LaunchDescription([
        chatbot_node
    ])
