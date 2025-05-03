import os
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_pc_lc = LaunchConfiguration('launch_pc_lc', default='true')
    declare_launch_pc_lc = DeclareLaunchArgument('launch_pc_lc', default_value='true')
    image_to_laserscan = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('depthimage_to_laserscan'),
                    'launch',
                    'depthimage_to_laserscan-launch.py'
                ])
            ]),
        )
    nct_planner = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nct_planner'),
                    'launch',
                    'nct_planner.launch.py'
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
        declare_launch_pc_lc,
        # laser_merger,
        # image_to_laserscan,
        # nct_planner,
        robot_localization
    ])
