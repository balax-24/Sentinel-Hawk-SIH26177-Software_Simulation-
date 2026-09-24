#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "===================================================================="
echo " SIH26177 LIVE GAZEBO + LIDAR PROCESSING + 3D MAPPING VERIFICATION"
echo "===================================================================="

cleanup() {
    echo "Terminating test processes..."
    kill -SIGTERM $MAP_PID 2>/dev/null || true
    kill -SIGTERM $LIDAR_PID 2>/dev/null || true
    kill -SIGTERM $SIM_PID 2>/dev/null || true
    pkill -f "ros_gz_bridge" 2>/dev/null || true
    pkill -f "lidar_processor_node" 2>/dev/null || true
    pkill -f "mapping_node" 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
}
trap cleanup EXIT

echo "[1/4] Launching Gazebo Fortress simulation stack..."
ros2 launch sih_simulation simulation.launch.py gui:=false > /tmp/sim_launch.log 2>&1 &
SIM_PID=$!
sleep 6

echo "[2/4] Launching live LiDAR processor..."
ros2 launch sih_lidar lidar.launch.py > /tmp/lidar_launch.log 2>&1 &
LIDAR_PID=$!
sleep 2

echo "[3/4] Launching live 3D mapping node..."
ros2 launch sih_mapping mapping.launch.py > /tmp/mapping_launch.log 2>&1 &
MAP_PID=$!
sleep 3

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
echo "CHECK 3: RAW GAZEBO LIDAR TOPIC (/lidar/points)"
echo "===================================================================="
ros2 topic info /lidar/points
ros2 topic echo /lidar/points --once --field header
ros2 topic echo /lidar/points --once --field width

echo ""
echo "===================================================================="
echo "CHECK 4: LIVE PROCESSED LIDAR OUTPUT (/lidar/processed_points)"
echo "===================================================================="
ros2 topic echo /lidar/processed_points --once --field header
ros2 topic echo /lidar/processed_points --once --field width

echo ""
echo "===================================================================="
echo "CHECK 5: LIVE 3D MAP OUTPUT (/map/info & /map/pointcloud)"
echo "===================================================================="
echo "Initial Map Info:"
ros2 topic echo /map/info --once
echo "Initial Map PointCloud:"
ros2 topic echo /map/pointcloud --once --field header
ros2 topic echo /map/pointcloud --once --field width

echo ""
echo "===================================================================="
echo "CHECK 6: DYNAMIC MAP UPDATE WITH UAV MOVEMENT (/drone/cmd_vel)"
echo "===================================================================="
echo "Initial UAV Pose:"
ros2 topic echo /drone/pose --once

echo "Publishing velocity command: linear.x = 2.0 m/s..."
for i in {1..10}; do
    ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1
    sleep 0.1
done

sleep 3

echo "Updated UAV Pose after movement:"
ros2 topic echo /drone/pose --once

echo "Updated Map Info after movement:"
ros2 topic echo /map/info --once

echo ""
echo "===================================================================="
echo "ALL LIVE VERIFICATION CHECKS COMPLETED SUCCESSFULLY!"
echo "===================================================================="
