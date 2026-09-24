"""Launch file for survivor detection node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_perception",
            executable="survivor_detector_node",
            name="sih_survivor_detector_node",
            output="screen",
            parameters=[{
                "fov_deg": 62.2,
                "confidence_threshold": 0.6,
            }],
        )
    ])
