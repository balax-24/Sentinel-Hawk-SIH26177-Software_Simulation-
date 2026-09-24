"""Launch file for optical monocular depth perception node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sih_perception",
            executable="optical_depth_node",
            name="sih_optical_depth_node",
            output="screen",
            parameters=[{
                "model_type": "midas_small",
                "model_path": "models/model-small.onnx",
                "fov_deg": 62.2,
            }],
        )
    ])
