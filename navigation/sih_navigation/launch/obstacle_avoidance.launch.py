"""Launch file for local obstacle avoidance node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_navigation",
            executable="obstacle_avoidance_node",
            name="sih_obstacle_avoidance_node",
            output="screen",
            parameters=[{
                "safety_radius_m": 2.0,
                "repulsion_gain": 1.5,
                "max_speed_mps": 3.0,
            }],
        )
    ])
