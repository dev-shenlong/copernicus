import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,Command
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_type = LaunchConfiguration('sim_type')
    pkg_name = 'copernicus_sim_clean'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    imu_enabled = LaunchConfiguration('imu_enabled')
    velodyne_enabled = LaunchConfiguration('velodyne_enabled')

    xacro_file = os.path.join(pkg_path, 'urdf', 'copernicus.xacro')
    
    robot_description_config = ParameterValue(Command(['xacro ', xacro_file,  ' sim_type:=', sim_type, ' imu_enabled:=', imu_enabled, ' velodyne_enabled:=', velodyne_enabled ]), value_type=str)
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[params]
    )

    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    launch_args  = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'sim_type',
            default_value='gz_sim',
            description='Choose simulator to use'
        ),
        DeclareLaunchArgument(
            'imu_enabled',
            default_value='true',
            description="Enable/Disable IMU"
        ),
        DeclareLaunchArgument(
            'velodyne_enabled',
            default_value='true',
            description="Enable/Disable Velodyne"
        )


    ]

    return LaunchDescription([
        *launch_args,
        robot_state_publisher,
        odom_tf,
        
    ])