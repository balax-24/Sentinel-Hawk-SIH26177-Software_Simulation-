"""Launch file for path planner node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_navigation",
            executable="planner_node",
            name="sih_planner_node",
            output="screen",
            parameters=[{
                "altitude_m": 3.5,
                "lane_spacing_m": 3.0,
            }],
        )
    ])
