"""Unified simulation launch file: starts Gazebo world, spawns the UAV, and starts the ROS-Gazebo bridge."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_sih_simulation = get_package_share_directory("sih_simulation")

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Whether to run Gazebo with graphical UI ('false' for headless/Docker, 'true' for GUI)",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Spawn X coordinate")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y coordinate")
    z_arg = DeclareLaunchArgument("z", default_value="1.0", description="Spawn Z coordinate")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn Yaw angle")
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="sih_uav", description="UAV robot entity name")

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sih_simulation, "launch", "disaster_world.launch.py")
        ),
        launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
    )

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sih_simulation, "launch", "spawn_uav.launch.py")
        ),
        launch_arguments={
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
            "robot_name": LaunchConfiguration("robot_name"),
        }.items(),
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sih_simulation, "launch", "bridge.launch.py")
        ),
    )

    return LaunchDescription([
        gui_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        robot_name_arg,
        world_launch,
        spawn_launch,
        bridge_launch,
    ])
