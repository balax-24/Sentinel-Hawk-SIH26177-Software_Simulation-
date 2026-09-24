"""Launch file for operator dashboard node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_dashboard",
            executable="dashboard_node",
            name="sih_dashboard_node",
            output="screen",
        )
    ])
