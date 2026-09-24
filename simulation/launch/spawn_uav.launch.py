"""Spawn the simulated Search-and-Rescue UAV in Ignition Gazebo Fortress."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sih_simulation = get_package_share_directory("sih_simulation")
    sdf_model_path = os.path.join(pkg_sih_simulation, "models", "uav", "model.sdf")

    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Spawn X coordinate")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y coordinate")
    z_arg = DeclareLaunchArgument("z", default_value="1.0", description="Spawn Z coordinate")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn Yaw angle")
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="sih_uav", description="Entity name")

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "disaster_world",
            "-file", sdf_model_path,
            "-name", LaunchConfiguration("robot_name"),
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        robot_name_arg,
        spawn_entity,
    ])
