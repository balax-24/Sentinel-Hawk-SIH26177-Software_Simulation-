"""Launch file for mission manager node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_mission",
            executable="mission_manager_node",
            name="sih_mission_manager_node",
            output="screen",
        )
    ])
