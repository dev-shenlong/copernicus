# Copyright 2024 Nature Robots GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Nature Robots GmbH nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "mesh_map_path",
            description="Path to the mesh file that defines the map."
            "Allowed formats are our internal HDF5 format and all"
            "standard mesh formats loadable by Assimp.",
        ),
        DeclareLaunchArgument(
            "mesh_map_working_path",
            description="Path to the mesh file used by the mesh navigation "
            "to store costs during operation. Only HDF5 formats are permitted.",
        ),
    ]
    mesh_map_path = LaunchConfiguration("mesh_map_path")
    mesh_map_working_path = LaunchConfiguration("mesh_map_working_path")

    mbf_mesh_nav_config = os.path.join(
        get_package_share_directory("copernicus_nav"), "config", "mbf_mesh_nav.yaml"
    )
    # Launch the mesh navigation server
    mesh_nav_server = Node(
        name="move_base_flex",
        package="mbf_mesh_nav",
        executable="mbf_mesh_nav",
        remappings=[
            ("/move_base_flex/cmd_vel", "/cmd_vel_stamped"),
        ],
        parameters=[
            mbf_mesh_nav_config,
            {
                "mesh_map.mesh_file": mesh_map_path,
                "mesh_map.mesh_working_file": mesh_map_working_path
            }
        ]
    )
    #Launch the mesh_stamped2unstamped node to convert
    #the stamped cmd_vel messages to unstamped ones
    mesh_stamped2unstamped = Node(
        name="Stamped2Unstamped",
        package="mesh_nav_cmd_cvt",
        executable="stmp2uns",

    )

    return LaunchDescription(
        launch_args
        + [
            mesh_nav_server,
            mesh_stamped2unstamped
        ]
    )
