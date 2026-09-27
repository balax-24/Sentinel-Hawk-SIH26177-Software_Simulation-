#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "===================================================================="
echo " SIH26177 MILESTONE 2: LIVE AUTONOMOUS NAVIGATION & AVOIDANCE VERIFICATION"
echo "===================================================================="

cleanup() {
    echo "Terminating test processes..."
    kill -SIGTERM $AVOID_PID 2>/dev/null || true
    kill -SIGTERM $PLAN_PID 2>/dev/null || true
    kill -SIGTERM $LIDAR_PID 2>/dev/null || true
    kill -SIGTERM $SIM_PID 2>/dev/null || true
    pkill -f "ros_gz_bridge" 2>/dev/null || true
    pkill -f "lidar_processor_node" 2>/dev/null || true
    pkill -f "planner_node" 2>/dev/null || true
    pkill -f "obstacle_avoidance_node" 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
}
trap cleanup EXIT

echo "[1/4] Launching Gazebo Fortress simulation stack..."
ros2 launch sih_simulation simulation.launch.py gui:=false > /tmp/sim_nav.log 2>&1 &
SIM_PID=$!
sleep 8

echo "[2/4] Launching LiDAR obstacle extraction node..."
ros2 launch sih_lidar lidar.launch.py > /tmp/lidar_nav.log 2>&1 &
LIDAR_PID=$!
sleep 3

echo "[3/4] Launching path planner node..."
ros2 launch sih_navigation planner.launch.py > /tmp/planner_nav.log 2>&1 &
PLAN_PID=$!
sleep 3

echo "[4/4] Launching local obstacle avoidance controller..."
ros2 launch sih_navigation obstacle_avoidance.launch.py > /tmp/avoid_nav.log 2>&1 &
AVOID_PID=$!
sleep 4

echo ""
echo "===================================================================="
echo "CHECK 1: ACTIVE NAVIGATION NODES"
echo "===================================================================="
ros2 node list

echo ""
echo "===================================================================="
echo "CHECK 2: ACTIVE NAVIGATION TOPICS"
echo "===================================================================="
ros2 topic list | grep -E "navigation|drone|obstacles|lidar"

echo ""
echo "===================================================================="
echo "CHECK 3: PLANNED PATH OUTPUT (/navigation/path)"
echo "===================================================================="
ros2 topic echo /navigation/path --once --field header

echo ""
echo "===================================================================="
echo "CHECK 4: ACTIVE WAYPOINT GOAL (/navigation/current_goal)"
echo "===================================================================="
ros2 topic echo /navigation/current_goal --once

echo ""
echo "===================================================================="
echo "CHECK 5: AUTONOMOUS AVOIDANCE VELOCITY (/drone/cmd_vel)"
echo "===================================================================="
ros2 topic echo /drone/cmd_vel --once

echo ""
echo "===================================================================="
echo "CHECK 6: DYNAMIC UAV MOVEMENT UNDER AUTONOMOUS CONTROL"
echo "===================================================================="
echo "Pose at Start of Autonomous Navigation:"
ros2 topic echo /drone/pose --once

echo "Waiting for autonomous flight trajectory execution (6 seconds)..."
sleep 6

echo "Pose after Autonomous Navigation:"
ros2 topic echo /drone/pose --once

echo ""
echo "===================================================================="
echo "NAVIGATION STATUS TELEMETRY (/navigation/status):"
echo "===================================================================="
ros2 topic echo /navigation/status --once

echo ""
echo "===================================================================="
echo "MILESTONE 2 AUTONOMOUS NAVIGATION VERIFICATION COMPLETE!"
echo "===================================================================="
