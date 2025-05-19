import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition

from launch.actions import SetEnvironmentVariable,AppendEnvironmentVariable

# This launch file is used to start the simulation of the robot in Gazebo with a specific world and spawn the robot model.
# It includes the necessary nodes and configurations for the simulation environment.

def generate_launch_description():

    # Set the package name and world file name
    description_package = 'copernicus_sim'
    default_world_file_name = "botanical_garden.sdf"
    world_file_name = LaunchConfiguration('world_file')
    use_rviz = LaunchConfiguration('use_rviz')
    # Get the path to the world file
    
    world_path = PathJoinSubstitution([
        get_package_share_directory(description_package),
        'worlds',
        world_file_name
    ])

    # Launch the robot description using the Xacro file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package),'launch','rsp.launch.py')])
                , launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Launch Rviz with the specified configuration file
    rviz = Node(
            package="rviz2", 
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', os.path.join(get_package_share_directory(description_package), 'rviz', 'view_robot.rviz')]
            ,
            condition=IfCondition(use_rviz)

    )

    #Launch Gazebo 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path}.items()
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'copernicus','-timeout', '600.0'], #Test if the entity name is changed,
                        output="screen"

    )

    return LaunchDescription([
        # Set the environment variable for Gazebo model path
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.path.join(get_package_share_directory(description_package), "models")),
        
        # Declare the launch arguments
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument("robot_name", default_value="robot_1", description="Declare the name of the robot"),
        # World file name must be present in the worlds directory
        DeclareLaunchArgument("world_file", default_value=default_world_file_name, description="Declare the name of the world file"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Use rviz if true"),
        rsp,
        rviz,
        gazebo,
        spawn_entity,
         ]
    )