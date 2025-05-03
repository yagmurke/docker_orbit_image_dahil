# Author: Addison Sears-Collins
# Date: December 1, 2021
# Description: Launch a two-wheeled robot URDF file using Rviz.
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  # Set the path to this package.
  pkg_share = FindPackageShare(package='orbit').find('orbit')
  # Set the path to the URDF file
  default_urdf_model_path = os.path.join(pkg_share, 'urdf/orbit_urdf.urdf')

  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  urdf_model = LaunchConfiguration('urdf_model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_sim_time = LaunchConfiguration('use_sim_time')

  # Declare the launch arguments  
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')
  
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true')
   
  # Specify the actions
  
  joint_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher'
  )
  
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': False, 
    'robot_description': Command(['xacro ', urdf_model])}],
    arguments=[default_urdf_model_path])
  
  # Create the launch description and populate
  ld = LaunchDescription()

  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_joint_state_publisher_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  

  ld.add_action(joint_publisher)
  ld.add_action(start_robot_state_publisher_cmd)

  return ld
