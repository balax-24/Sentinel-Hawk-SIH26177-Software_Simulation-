"""Launch file for payload delivery node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_mission",
            executable="payload_delivery_node",
            name="sih_payload_delivery_node",
            output="screen",
        )
    ])
