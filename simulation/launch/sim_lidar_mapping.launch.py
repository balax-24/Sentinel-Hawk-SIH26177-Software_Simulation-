"""Combined launch file for Gazebo simulation, live LiDAR processing, and 3D mapping."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_sim = get_package_share_directory("sih_simulation")
    pkg_lidar = get_package_share_directory("sih_lidar")
    pkg_mapping = get_package_share_directory("sih_mapping")

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Whether to run Gazebo with graphical UI ('false' for headless/Docker, 'true' for GUI)",
    )

    # 1. Gazebo simulation stack (disaster_world + UAV + ros_gz_bridge)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, "launch", "simulation.launch.py")
        ),
        launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
    )

    # 2. Live LiDAR point cloud processor
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, "launch", "lidar.launch.py")
        )
    )

    # 3. Live 3D environment reconstruction & map accumulator
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mapping, "launch", "mapping.launch.py")
        )
    )

    return LaunchDescription([
        gui_arg,
        sim_launch,
        lidar_launch,
        mapping_launch,
    ])
