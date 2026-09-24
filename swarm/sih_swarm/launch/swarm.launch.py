"""Launch file for multi-UAV swarm manager node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_swarm",
            executable="swarm_manager_node",
            name="sih_swarm_node",
            output="screen",
        )
    ])
