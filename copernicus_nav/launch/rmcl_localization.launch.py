import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "copernicus_nav"
    pkg_share = get_package_share_directory(pkg_name)

    
    sim_pkg = "copernicus_sim"
    sim_pkg_share = get_package_share_directory(sim_pkg)

    #Extension of the mesh map files
    mesh_map_nav_ext = ".ply"

    # Get thhe Available maps
    available_map_names = [
        f[:-len(mesh_map_nav_ext)]
        for f in os.listdir(os.path.join(sim_pkg_share, "maps"))
        if f.endswith(mesh_map_nav_ext)
    ]

    map_name_default = "botanical_garden"
    # Launch arguments
    map_name = LaunchConfiguration("map_name")
    #world_name = LaunchConfiguration("world_name")
    localization_type = LaunchConfiguration("localization_type")
    use_rviz = LaunchConfiguration("use_rviz")

    # Launch the simulation of the robot in Gazebo
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([sim_pkg_share, "launch", "robot_sim.launch.py"])]
        ),
        launch_arguments={
            "world_file": PythonExpression(['"', map_name, ".sdf", '"']),
            "use_rviz": "false",
            "use_sim_time": "true",
        }.items(),

    )

    #Ekf node the configuration is still under development
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {'use_sim_time': True},
            PathJoinSubstitution([pkg_share, "config", "ekf.yaml"])],
    )
    
    #Ground truth map localization
    map_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            name='static_map_to_odom'
        )
    
    #RMCL localization
    #Still under development
    map_loc_rmcl_micp = Node(
        package="rmcl_ros",
        executable="micp_localization_node",
        name="micp_localization",
        output="screen",
        remappings=[
            ("pose_wc", "/initialpose"),
        ],
        parameters=[
            {
                "use_sim_time": True,
                "map_file": PathJoinSubstitution(
                    [
                        sim_pkg_share,
                        "maps",
                        PythonExpression(["'", map_name, ".dae'"])
                    ]
                ),
            },
            PathJoinSubstitution([pkg_share, "config", "rmcl_micpl.yaml"]),
        ],
    )

    pc2_topic = LaunchConfiguration("pc2_topic", default="/velodyne/velodyne_points")
    pc2_to_o1dn = Node(
        package="rmcl_ros",
        name="pc2_to_o1dn",
        executable="conv_pc2_to_o1dn_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                

            },
            PathJoinSubstitution([pkg_share, "config", "rmcl_micpl.yaml"]),
        ],
        remappings=[
            ("input", pc2_topic),
            ("output", "/o1dn/pointcloud"),

        ]
    )

    #Launch the Rviz visualization tool
    rviz = Node(
            package="rviz2", 
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', os.path.join(get_package_share_directory(sim_pkg), 'rviz', 'default.rviz')]
            ,
            condition=IfCondition(use_rviz)

    )


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_name",
                description="Name of the map to be used for navigation"
                + '(see copernicus_sim\' "maps" directory).',
                default_value=map_name_default,
                choices=available_map_names,
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Use rviz if true",
            ),
            rviz,
            simulation_launch,
            #ekf,
            map_tf,
            pc2_to_o1dn,
            map_loc_rmcl_micp
            #move_base_flex,
            
        ]
    )