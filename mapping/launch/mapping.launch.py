"""Launch file for 3D environment reconstruction mapping node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_mapping",
            executable="mapping_node",
            name="sih_mapping_node",
            output="screen",
            parameters=[{
                "voxel_size": 0.1,
                "max_points": 500000,
                "map_frame": "disaster_world",
            }],
        )
    ])
