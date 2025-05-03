import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # rosbridge_launch = os.path.join(
    #     get_package_share_directory('rosbridge_server'),
    #     'launch',
    #     'rosbridge_websocket_launch.xml'
    # )
    sensor_bringup_launch = os.path.join(
        get_package_share_directory('orbit'),
        'launch',
        'sensor_bringup',
        'sensor_bringup.launch.py'
    )
    sensor_fusion_launch = os.path.join(
        get_package_share_directory('orbit'),
        'launch',
        'sensor_bringup',
        'sensor_fusion.launch.py'
    )
    orbit_face = os.path.join(
        get_package_share_directory('play_face'),
        'launch',
        'orbit_face.launch.py'
    )

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     FrontendLaunchDescriptionSource(rosbridge_launch)
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_bringup_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_fusion_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbit_face)
        ),
    ])