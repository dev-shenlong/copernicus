import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("copernicus_sim")
    mesh_map_nav_ext = ".ply"
    available_map_names = [
        f[:-len(mesh_map_nav_ext)]
        for f in os.listdir(os.path.join(pkg_share, "maps"))
        if f.endswith(mesh_map_nav_ext)
    ]

    map_name_default = "botanical_garden"#""university"#
    # Launch arguments
    map_name = LaunchConfiguration("map_name")
    #world_name = LaunchConfiguration("world_name")
    localization_type = LaunchConfiguration("localization_type")

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_share, "launch", "robot_sim.launch.py"])]
        ),

    )
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {'use_sim_time': True},
            PathJoinSubstitution([pkg_share, "config", "ekf.yaml"])],
    )
    # Ground truth map localization
    map_loc_gt = Node(
        package="mesh_navigation_tutorials_sim",
        executable="ground_truth_localization_node",
        name="ground_truth_localization_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "gz_parent_frame": map_name,
                "gz_child_frame": "copernicus",
                "ros_parent_frame": "map",
                "ros_child_frame": "base_link",
                "ros_odom_frame": "odom",
            }
        ],
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
        package="rmcl",
        executable="micp_localization",
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
                        pkg_share,
                        "maps",
                        map_name,
                        ".dae"
                        #"PythonExpression(['"', map_name, '" + ".dae"'])",
                    ]
                ),
            },
            PathJoinSubstitution([pkg_share, "config", "rmcl_micpl.yaml"]),
        ],
    )

    move_base_flex = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_share, "launch", "mbf_mesh_navigation_server_launch.py"]
            )
        ),
        launch_arguments={
            "mesh_map_path": PathJoinSubstitution(
                [
                    pkg_share,
                    "maps",
                    PythonExpression(['"', map_name, mesh_map_nav_ext, '"']),

                ]
            ),
            "mesh_map_working_path": PathJoinSubstitution(
                [
                    pkg_share,
                    "maps",
                    PythonExpression(['"', map_name, ".h5", '"']),
                ]
            )
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",

    )

    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #     "map_name",
            #     description="Name of the map to be used for navigation"
            #     + '(see copernicus_sim\' "maps" directory).',
            #     default_value=LaunchConfiguration("world_name"),
            #     choices=available_map_names,
            # ),
            DeclareLaunchArgument(
                "localization_type",
                description="How the robot shall localize itself",
                default_value="ground_truth",
                choices=["ground_truth", "ekf"],
            ),
            DeclareLaunchArgument(
                "map_name",
                description="Name of the map to be used for navigation"
                + '(see copernicus_sim\' "maps" directory).',
                default_value=map_name_default,
            ),
            simulation_launch,
            #ekf,
            map_tf,
            move_base_flex,
            #rviz,
            #map_loc_gt
            #map_loc_rmcl_micp
        ]
    )