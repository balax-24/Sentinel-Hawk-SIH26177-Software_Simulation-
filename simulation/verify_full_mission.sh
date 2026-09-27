#!/bin/bash

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "===================================================================="
echo " SIH26177 FULL END-TO-END AUTONOMOUS MISSION INTEGRATION TEST"
echo "===================================================================="

cleanup() {
    echo "Terminating mission processes..."
    kill -SIGINT $FULL_PID 2>/dev/null || true
    sleep 2
    pkill -f "ros_gz_bridge" 2>/dev/null || true
    pkill -f "lidar_processor_node" 2>/dev/null || true
    pkill -f "optical_depth_node" 2>/dev/null || true
    pkill -f "survivor_detector_node" 2>/dev/null || true
    pkill -f "hazard_detector_node" 2>/dev/null || true
    pkill -f "mapping_node" 2>/dev/null || true
    pkill -f "planner_node" 2>/dev/null || true
    pkill -f "obstacle_avoidance_node" 2>/dev/null || true
    pkill -f "mission_manager_node" 2>/dev/null || true
    pkill -f "payload_delivery_node" 2>/dev/null || true
    pkill -f "swarm_manager_node" 2>/dev/null || true
    pkill -f "dashboard_node" 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
}
trap cleanup EXIT

echo "Launching full mission system (12 nodes + Gazebo Fortress)..."
ros2 launch sih_system full_mission.launch.py gui:=false > /tmp/full_mission.log 2>&1 &
FULL_PID=$!

echo "Allowing all subsystems to initialize and start mission loops (15 seconds)..."
sleep 15

echo ""
echo "=== LOG OUTPUT CHECK (tail -n 40 /tmp/full_mission.log) ==="
tail -n 40 /tmp/full_mission.log || true

echo ""
echo "===================================================================="
echo "1. VERIFY ALL ACTIVE ROS 2 NODES"
echo "===================================================================="
timeout 5s ros2 node list || echo "Timeout listing nodes"

echo ""
echo "===================================================================="
echo "2. VERIFY ACTIVE ROS 2 TOPICS"
echo "===================================================================="
timeout 5s ros2 topic list || echo "Timeout listing topics"

echo ""
echo "===================================================================="
echo "3. VERIFY ACTIVE ROS 2 SERVICES"
echo "===================================================================="
timeout 5s ros2 service list || echo "Timeout listing services"

echo ""
echo "===================================================================="
echo "4. VERIFY MISSION STATE MACHINE (/mission/state)"
echo "===================================================================="
timeout 5s ros2 topic echo /mission/state --once || echo "Unable to read /mission/state"

echo ""
echo "===================================================================="
echo "5. VERIFY SWARM STATE (/swarm/state)"
echo "===================================================================="
timeout 5s ros2 topic echo /swarm/state --once || echo "Unable to read /swarm/state"

echo ""
echo "===================================================================="
echo "6. VERIFY PAYLOAD DELIVERY SERVICE"
echo "===================================================================="
if timeout 3s ros2 service list | grep -q "/mission/trigger_payload"; then
    timeout 5s ros2 service call /mission/trigger_payload sih_interfaces/srv/TriggerPayload "{bay_id: 'bay_1', confirm_target_clear: true}" || true
else
    echo "Service /mission/trigger_payload not found in active services."
fi

echo ""
echo "===================================================================="
echo "7. VERIFY PAYLOAD STATUS (/payload/status)"
echo "===================================================================="
timeout 5s ros2 topic echo /payload/status --once || echo "Unable to read /payload/status"

echo ""
echo "===================================================================="
echo "8. VERIFY SITREP REPORT OUTPUT"
echo "===================================================================="
if [ -f "/ros2_ws/src/sih26177/data/output/sitrep_report.json" ]; then
    cat /ros2_ws/src/sih26177/data/output/sitrep_report.json
else
    echo "No sitrep_report.json found yet in data/output"
fi

echo ""
echo "===================================================================="
echo "FULL MISSION INTEGRATION VERIFICATION COMPLETE!"
echo "===================================================================="
