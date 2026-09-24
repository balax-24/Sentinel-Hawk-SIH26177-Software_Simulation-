"""Master launch file executing the complete end-to-end SIH26177 Search & Rescue mission."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_sim = get_package_share_directory("sih_simulation")
    pkg_lidar = get_package_share_directory("sih_lidar")
    pkg_perception = get_package_share_directory("sih_perception")
    pkg_mapping = get_package_share_directory("sih_mapping")
    pkg_navigation = get_package_share_directory("sih_navigation")
    pkg_mission = get_package_share_directory("sih_mission")
    pkg_dashboard = get_package_share_directory("sih_dashboard")

    gui_arg = DeclareLaunchArgument("gui", default_value="false", description="Gazebo GUI enable")

    # 1. Simulation (Disaster world + UAV)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, "launch", "simulation.launch.py")),
        launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
    )

    # 2. LiDAR Processing
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_lidar, "launch", "lidar.launch.py"))
    )

    # 3. Perception (Optical Depth + Survivor Detection)
    optical_depth_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_perception, "launch", "optical_depth.launch.py"))
    )
    survivor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_perception, "launch", "survivor_detection.launch.py"))
    )

    # 4. 3D Mapping
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_mapping, "launch", "mapping.launch.py"))
    )

    # 5. Navigation & Local Obstacle Avoidance
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation, "launch", "planner.launch.py"))
    )
    avoidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation, "launch", "obstacle_avoidance.launch.py"))
    )

    # 6. Mission Management & Payload Delivery
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_mission, "launch", "mission_manager.launch.py"))
    )
    payload_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_mission, "launch", "payload_delivery.launch.py"))
    )

    # 7. Dashboard
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dashboard, "launch", "dashboard.launch.py"))
    )

    return LaunchDescription([
        gui_arg,
        sim_launch,
        lidar_launch,
        optical_depth_launch,
        survivor_launch,
        mapping_launch,
        planner_launch,
        avoidance_launch,
        mission_launch,
        payload_launch,
        dashboard_launch,
    ])
