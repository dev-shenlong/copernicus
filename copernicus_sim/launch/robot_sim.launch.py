import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch.actions import SetEnvironmentVariable,AppendEnvironmentVariable

def generate_launch_description():


    description_package = 'copernicus_sim'
    world_file_name = "botanical_garden.sdf"#
    world_path = os.path.join(
        get_package_share_directory(description_package),
        'worlds',
        world_file_name
    )
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package),'launch','rsp.launch.py')])
                , launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    rviz = Node(
            package="rviz2", 
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', os.path.join(get_package_share_directory(description_package), 'rviz', 'default.rviz')]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path}.items()
    )

    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'copernicus','-timeout', '600.0'], #Test if the entity name is changed,
                        output="screen"

    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot_1", description="Declare the name of the robot"),
        
        AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory(description_package), "models")),
        rsp,
        rviz,
        gazebo,
        spawn_entity,
       # diff_drive_event_handler,
       # joint_broad_event_handler,
         ]
    )