import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_type = LaunchConfiguration('sim_type')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('copernicus_sim'))
    # Get the path to the URDF file
    xacro_file = os.path.join(pkg_path,'urdf','copernicus.xacro')
    robot_description_config = xacro.process_file(xacro_file, mapping={'sim_type': sim_type})
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'sim_type',
            default_value='gazebo',
            description='Use "gazebo" for Gazebo simulation, "gz_sim" for gz_sim simulation'),

        node_robot_state_publisher,
        odom_tf
    ])