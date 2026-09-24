#!/bin/bash
set -e

# Source ROS 2 base environment
source "/opt/ros/humble/setup.bash"

# Source local built workspace if available
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
fi

exec "$@"
