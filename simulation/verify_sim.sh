#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "=================================================="
echo "SIH26177 GAZEBO FORTRESS SIMULATION VERIFICATION"
echo "=================================================="

# Start simulation in background
echo "[1/7] Launching simulation stack (disaster_world + spawn_uav + ros_gz_bridge)..."
ros2 launch sih_simulation simulation.launch.py gui:=false > /tmp/sim_launch.log 2>&1 &
SIM_PID=$!

# Cleanup on exit
cleanup() {
    echo "Terminating simulation (PID $SIM_PID)..."
    kill -SIGINT $SIM_PID 2>/dev/null || true
    wait $SIM_PID 2>/dev/null || true
}
trap cleanup EXIT

echo "Waiting for simulation, world, entity creation, and bridge to initialize..."
sleep 6

echo ""
echo "=================================================="
echo "CHECK 1: ROS 2 NODE LIST"
echo "=================================================="
ros2 node list

echo ""
echo "=================================================="
echo "CHECK 2: ROS 2 TOPIC LIST"
echo "=================================================="
ros2 topic list

echo ""
echo "=================================================="
echo "CHECK 3: LIDAR SENSOR DATA (/lidar/points)"
echo "=================================================="
ros2 topic echo /lidar/points --once

echo ""
echo "=================================================="
echo "CHECK 4: CAMERA SENSOR DATA (/camera/image)"
echo "=================================================="
# Camera image data is large binary, display header and metadata
ros2 topic echo /camera/image --once --field header
ros2 topic echo /camera/image --once --field height
ros2 topic echo /camera/image --once --field width
ros2 topic echo /camera/image --once --field encoding

echo ""
echo "=================================================="
echo "CHECK 5: IMU SENSOR DATA (/drone/imu/data)"
echo "=================================================="
ros2 topic echo /drone/imu/data --once

echo ""
echo "=================================================="
echo "CHECK 6: GPS SENSOR DATA (/drone/gps/fix)"
echo "=================================================="
ros2 topic echo /drone/gps/fix --once

echo ""
echo "=================================================="
echo "CHECK 7: UAV POSE & MOVEMENT VERIFICATION"
echo "=================================================="
echo "--- Initial Pose Before Movement Command ---"
ros2 topic echo /drone/pose --once

echo ""
echo "--- Publishing Velocity Command: linear.x = 2.0 m/s ---"
for i in {1..10}; do
    ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1
    sleep 0.1
done

echo "Waiting for UAV displacement in Gazebo..."
sleep 2

echo "--- Updated Pose After Movement Command ---"
ros2 topic echo /drone/pose --once

echo ""
echo "=================================================="
echo "ALL VERIFICATION CHECKS COMPLETED SUCCESSFULLY!"
echo "=================================================="
