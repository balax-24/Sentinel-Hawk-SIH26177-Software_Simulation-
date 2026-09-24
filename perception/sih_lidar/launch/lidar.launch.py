"""Launch file for LiDAR processing node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_lidar",
            executable="lidar_processor_node",
            name="sih_lidar_node",
            output="screen",
            parameters=[{
                "min_range": 0.5,
                "max_range": 40.0,
                "ground_threshold_z": 0.25,
                "voxel_size": 0.1,
            }],
        )
    ])
