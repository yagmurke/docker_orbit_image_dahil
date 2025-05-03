from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
def generate_launch_description():
    # ROS 2 nodlar
    play_face = Node(
        package='play_face',
        executable='play_face_node'
    )
    mp3_amp_publisher = Node(
        package='play_face',
        executable='mp3_amp_publisher'
    )
    text_amp_publisher = Node(
        package='play_face',
        executable='text_amp_publisher'
    )
    camera_publish = Node(
        package='play_face',
        executable='camera_publish'
    )
    move_subs = Node(
        package='play_face',
        executable='move_subs'
    )
    tasks_subs = Node(
        package='play_face',
        executable='tasks_subs'
    )
    # orbit_movement = Node(
    #     package='orbit_controller',
    #     executable='orbit_movement'
    # )
    orbit_controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orbit_controller'),
                    'launch',
                    'orbit_controller.launch.py'
                ])
            ]),
        )
    arduino_controls_node = Node(
        package='arduino_controls',
        executable='arduino_controls_node'
    )
    setup_node = Node(
        package='play_face',
        executable='setup_pub_node'
    )
    # wifi_connect.py dosyasını çalıştırmak için ExecuteProcess
    websocket_control = ExecuteProcess(
        cmd=['python3', os.path.join(os.path.expanduser('~'), 'orbit_ws', 'src', 'play_face', 'play_face', 'websocket_control.py')],
        shell=True,
        name='websocket_control',
        output='screen'
    )
    wifi_connect_process = ExecuteProcess(
        cmd=['python3', os.path.join(os.path.expanduser('~'), 'orbit_ws', 'src', 'play_face', 'play_face', 'wifi_connect.py')],
        shell=True,
        name='wifi_connect_process',
        output='screen'
    )
    wifi_isconnected = ExecuteProcess(
        cmd=['python3', os.path.join(os.path.expanduser('~'), 'orbit_ws', 'src', 'play_face', 'play_face', 'wifi_isconnected.py')],
        shell=True,
        name='wifi_connect_process',
        output='screen'
    )
    # ardiuno_pub = ExecuteProcess(
    #     cmd=['python3', os.path.join(os.path.expanduser('~'), 'orbit_ws', 'src', 'play_face', 'play_face', 'ardiuno_pub.py')],
    #     shell=True,
    #     name='wifi_connect_process',
    #     output='screen'
    # )



    # LaunchDescription'a ROS2 nodları ve Python dosyasını ekliyoruz
    return LaunchDescription([
        play_face,
        mp3_amp_publisher,
        text_amp_publisher,
        camera_publish,
        move_subs,
        tasks_subs,
        orbit_controller_launch,
        arduino_controls_node,
        setup_node,
        wifi_connect_process,
        websocket_control,
        wifi_isconnected
    ])
