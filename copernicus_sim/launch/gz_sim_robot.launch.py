import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory("copernicus_sim")
    world_file_name = "university.sdf"
    world = os.path.join(pkg_share, "worlds", world_file_name)

    sim_type = LaunchConfiguration("sim_type")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', world], 
        'on_exit_shutdown': 'true'}.items()
        # launch_arguments={'world': world_path}.items(),
    )
    xacro_file = os.path.join(pkg_share, 'urdf', 'copernicus.xacro')
    robot_description = xacro.process_file(xacro_file, mapping={'sim_type': 'gz_sim'})
    # robot_description = xacro.process_file(xacro_file, mapping={'sim_type': sim_type})
    params = {'robot_description': robot_description.toxml(), 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",

            "-Y",
            "0.0",
        ],
        output="screen",
    )
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    #         '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    #         '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
    #                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
    #                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
    #     output='screen'
    # )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [pkg_share, 'config', 'ros_gz_bridge.yaml']
                )
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_type',
            default_value='gz_sim',
            description='gz_sim or gazebo for simulations'),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge
    ])