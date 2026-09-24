"""Launch Ignition Gazebo Fortress with the self-contained SIH disaster world."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_sih_simulation = get_package_share_directory("sih_simulation")

    world_path = os.path.join(pkg_sih_simulation, "worlds", "disaster_world.world")

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Set to 'false' to run headless (Docker, server), 'true' for GUI",
    )

    gz_args_expr = PythonExpression([
        "'-r ' + ('-s ' if '", LaunchConfiguration("gui"), "' == 'false' else '') + '", world_path, "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args_expr}.items(),
    )

    return LaunchDescription([gui_arg, gz_sim])
