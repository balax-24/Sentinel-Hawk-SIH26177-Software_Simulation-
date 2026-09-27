"""Launch modular hazard detection node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_perception",
            executable="hazard_detector_node",
            name="sih_hazard_detector_node",
            output="screen",
        )
    ])
