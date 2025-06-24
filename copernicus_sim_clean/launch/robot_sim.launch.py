import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution,PythonExpression
from launch.conditions import IfCondition

from launch.actions import SetEnvironmentVariable,AppendEnvironmentVariable,LogInfo

# This launch file is used to start the simulation of the robot in Gazebo with a specific world and spawn the robot model.
# It includes the necessary nodes and configurations for the simulation environment.
def launch_sim_nodes(context, *args, **kwargs):
    sim_type = LaunchConfiguration("sim_type").perform(context)
    world = LaunchConfiguration('world').perform(context)
    world_package = 'mesh_nav_maps'
    description_package = 'copernicus_sim_clean'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package),'launch','rsp.launch.py')])
                , launch_arguments={'use_sim_time': 'true', 'sim_type': sim_type}.items()
    )

    actions = list((rsp,))
    if(sim_type == "gazebo"):
        world_file = world + '.sdf'
        world_path = os.path.join(get_package_share_directory(world_package), 'worlds',world, world_file)
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={'world': world_path}.items(),
            condition =IfCondition(PythonExpression(["'", sim_type, "'", " == ", "'gazebo'"]))
        )
        spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'copernicus','-timeout', '600.0'], #Test if the entity name is changed,
                        output="screen",
                        condition =IfCondition(PythonExpression(["'", sim_type, "'", " == ", "'gazebo'"]))
        

        )
        actions.append(gazebo)
        actions.append(spawn_entity)

    elif(sim_type == "gz_sim"):
        world_file = world + '.world'
        world_path = os.path.join(get_package_share_directory(world_package), 'worlds',world, world_file)
        gz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r ', world_path], 'on_exit_shutdown': 'true'}.items(),
                    condition =IfCondition(PythonExpression(["'", sim_type, "'", " == ", "'gz_sim'"]))
             )

        gz_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'copernicus',
                                   '-z', '0.1'],
                        output='screen',
                        condition =IfCondition(PythonExpression(["'", sim_type, "'", " == ", "'gz_sim'"]))
                )
        
        bridge_params = os.path.join(get_package_share_directory(description_package), 'config', 'ros_gz_bridge.yaml')
        ros_gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}'
            ]
        )
        actions.extend([gz, gz_spawn_entity, ros_gz_bridge])
    return actions




def generate_launch_description():

    # Set the package name and world file name
    description_package = 'copernicus_sim_clean'
    world_package = 'mesh_nav_maps'    
    default_world_file_name = "botanical_garden"
    use_rviz = LaunchConfiguration('use_rviz')

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

    return LaunchDescription([
        # Set the environment variable for Gazebo model path
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.path.join(get_package_share_directory(world_package), "models")),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=os.path.join(get_package_share_directory(world_package), "models")),
        AppendEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=os.path.join(get_package_share_directory(world_package), "models")),
        # Declare the launch arguments
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument("robot_name", default_value="robot_1", description="Declare the name of the robot"),
        # World file name must be present in the worlds directory
        DeclareLaunchArgument("world", default_value=default_world_file_name, description="Declare the name of the world file"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Use rviz if true"),
        DeclareLaunchArgument("sim_type", default_value="gazebo", description= "Simulator to use"),
        OpaqueFunction(function=launch_sim_nodes),
        rviz,
         ]
    )
