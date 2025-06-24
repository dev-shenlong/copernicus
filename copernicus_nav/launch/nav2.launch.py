import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration

def generate_launch_description():
    pkg_sim = "copernicus_sim"
    pkg_sim_share = get_package_share_directory(pkg_sim)


    pkg_nav = "copernicus_nav"
    pkg_nav_share = get_package_share_directory(pkg_nav)

    map_name = LaunchConfiguration("map_name")


    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_sim_share, "launch", "robot_sim.launch.py"])]
        ),
        launch_arguments={
            "world_file": PythonExpression(['"', map_name, ".sdf", '"']),
            "use_rviz": "false",
            "use_sim_time": "true",
        }.items(),

    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_nav_share, "launch", "nav2_bringup.launch.py"])]
        ),
        launch_arguments={
            'use_sim_time': "true",
            "map": PathJoinSubstitution([pkg_sim_share, "maps", PythonExpression(['"', map_name, ".yaml", '"'])]),
            "params_file": PathJoinSubstitution([pkg_nav_share, "config", "nav2_params.yaml"]),
        }.items()
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            "map_name",
            default_value="botanical_garden",
            description="Name of the map to use for navigation"
        ),
        DeclareLaunchArgument(
            "localization_type",
            default_value="ekf",
            description="Type of localization to use (ekf, amcl, etc.)"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz for visualization"
        ),
        simulation_launch,
        nav2_launch,
        
        
    ])

