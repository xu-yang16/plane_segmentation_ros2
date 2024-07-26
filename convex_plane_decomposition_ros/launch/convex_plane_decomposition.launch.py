import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="parameter_file",
                default_value=get_package_share_directory(
                    "convex_plane_decomposition_ros"
                )
                + "/config/parameters.yaml",
            ),
            launch.actions.DeclareLaunchArgument(
                name="node_parameter_file",
                default_value=get_package_share_directory(
                    "convex_plane_decomposition_ros"
                )
                + "/config/node.yaml",
            ),
            launch_ros.actions.Node(
                package="convex_plane_decomposition_ros",
                executable="convex_plane_decomposition_ros_node",
                name="convex_plane_decomposition_ros_node",
                prefix="",
                output="screen",
                parameters=[
                    launch.substitutions.LaunchConfiguration("parameter_file"),
                    launch.substitutions.LaunchConfiguration("node_parameter_file"),
                ],
                remappings=[
                    ("/filtered_map", "/convex_plane_decomposition_ros/filtered_map"),
                    ("/boundaries", "/convex_plane_decomposition_ros/boundaries"),
                    ("/planes", "/convex_plane_decomposition_ros/planes"),
                    (
                        "/planar_terrain",
                        "/convex_plane_decomposition_ros/planar_terrain",
                    ),
                ],
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
