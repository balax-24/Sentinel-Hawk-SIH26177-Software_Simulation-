#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "===================================================================="
echo " SIH26177 LIVE GAZEBO + LIDAR PROCESSING + 3D MAPPING VERIFICATION"
echo "===================================================================="

echo "[1/6] Launching sim_lidar_mapping stack in background..."
ros2 launch sih_simulation sim_lidar_mapping.launch.py gui:=false > /tmp/sim_lidar_mapping.log 2>&1 &
SIM_PID=$!

cleanup() {
    echo "Terminating test processes..."
    kill -SIGTERM $SIM_PID 2>/dev/null || true
    pkill -f "ros_gz_bridge" 2>/dev/null || true
    pkill -f "lidar_processor_node" 2>/dev/null || true
    pkill -f "mapping_node" 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
}
trap cleanup EXIT

echo "Waiting for Gazebo, UAV spawn, ros_gz_bridge, lidar_node, and mapping_node to initialize..."
sleep 8

echo ""
echo "===================================================================="
echo "CHECK 1: ACTIVE ROS 2 NODES"
echo "===================================================================="
ros2 node list

echo ""
echo "===================================================================="
echo "CHECK 2: ACTIVE ROS 2 TOPICS"
echo "===================================================================="
ros2 topic list

echo ""
echo "===================================================================="
echo "CHECK 3: RAW GAZEBO LIDAR TOPIC INFO (/lidar/points)"
echo "===================================================================="
ros2 topic info /lidar/points

echo ""
echo "===================================================================="
echo "CHECK 4: LIVE PROCESSED LIDAR OUTPUT (/lidar/processed_points)"
echo "===================================================================="
ros2 topic echo /lidar/processed_points --once

echo ""
echo "===================================================================="
echo "CHECK 5: INITIAL 3D GLOBAL MAP STATE (/map/info & /map/pointcloud)"
echo "===================================================================="
echo "Map Info (initial):"
ros2 topic echo /map/info --once
echo "Map PointCloud header (initial):"
ros2 topic echo /map/pointcloud --once --field header
ros2 topic echo /map/pointcloud --once --field width

echo ""
echo "===================================================================="
echo "CHECK 6: DYNAMIC MAP ACCUMULATION UNDER UAV MOVEMENT"
echo "===================================================================="
echo "Initial UAV Pose:"
ros2 topic echo /drone/pose --once

echo "Publishing velocity commands to move UAV..."
for i in {1..12}; do
    ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1
    sleep 0.1
done

sleep 3

echo "Updated UAV Pose:"
ros2 topic echo /drone/pose --once

echo "Updated Map Info after movement:"
ros2 topic echo /map/info --once

echo "Updated Map PointCloud header after movement:"
ros2 topic echo /map/pointcloud --once --field header
ros2 topic echo /map/pointcloud --once --field width

echo ""
echo "===================================================================="
echo "ALL LIVE VERIFICATION CHECKS COMPLETED SUCCESSFULLY!"
echo "===================================================================="
