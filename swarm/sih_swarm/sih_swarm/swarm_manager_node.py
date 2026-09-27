"""Multi-UAV Swarm Management and Task Allocation Node for SIH26177.

Partitions disaster search zones among multiple UAVs, monitors swarm health,
tracks cumulative search area coverage, and maintains shared situational awareness.
Publishes:
  /swarm/state (interfaces/msg/SwarmState)
  /swarm/shared_map (std_msgs/String)
"""

from __future__ import annotations
import json
import logging
import math
import sys
import time
from typing import List, Dict, Any, Tuple, Optional

logger = logging.getLogger("SwarmManager")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Header, String
    from geometry_msgs.msg import Point, PoseStamped
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    Header = Any
    String = Any
    Point = Any
    PoseStamped = Any
    qos_profile_sensor_data = None

try:
    from sih_interfaces.msg import SwarmState, DroneState, SurvivorArray, HazardArray
    HAS_SWARM_MSG = True
except ImportError:
    HAS_SWARM_MSG = False
    SwarmState = Any
    DroneState = Any
    SurvivorArray = Any
    HazardArray = Any


class SwarmCoordinator:
    """Allocates search sectors to multiple drones in the swarm and manages health."""

    def __init__(self, swarm_id: str = "sih_swarm_alpha", drones: Optional[List[str]] = None) -> None:
        self.swarm_id = swarm_id
        self.drones = drones if drones is not None else ["uav_1", "uav_2", "uav_3"]
        self.sector_assignments: Dict[str, Dict[str, Any]] = {}
        self.drone_status: Dict[str, Dict[str, Any]] = {}
        self.shared_survivors: Dict[int, Dict[str, Any]] = {}
        self.shared_hazards: Dict[int, Dict[str, Any]] = {}
        self.leader_id = self.drones[0] if self.drones else "uav_1"
        self.mission_phase = "INIT"
        self.coverage_pct = 0.0

        for d in self.drones:
            self.drone_status[d] = {
                "active": True,
                "position": (0.0, 0.0, 0.0),
                "battery": 100.0,
                "flight_mode": "IDLE",
                "assigned_task": "STANDBY",
                "last_heartbeat": time.time(),
            }

    def partition_disaster_area(
        self, x_min: float, x_max: float, y_min: float, y_max: float
    ) -> Dict[str, Dict[str, Any]]:
        """Divide bounding box into non-overlapping sub-sectors along X."""
        active_drone_ids = [d for d in self.drones if self.drone_status[d]["active"]]
        if not active_drone_ids:
            active_drone_ids = self.drones

        num_drones = len(active_drone_ids)
        dx_per_drone = (x_max - x_min) / float(num_drones)

        self.sector_assignments.clear()
        for i, drone_id in enumerate(active_drone_ids):
            sub_xmin = x_min + i * dx_per_drone
            sub_xmax = sub_xmin + dx_per_drone
            self.sector_assignments[drone_id] = {
                "sector_id": f"sector_{i+1}",
                "bounds": [float(sub_xmin), float(sub_xmax), float(y_min), float(y_max)],
                "task": "GRID_SEARCH",
            }
            self.drone_status[drone_id]["assigned_task"] = f"SEARCH_SECTOR_{i+1}"

        self.mission_phase = "GRID_SEARCH"
        return self.sector_assignments

    def handle_uav_failure(self, failed_uav_id: str, area_bounds: Tuple[float, float, float, float]) -> None:
        """Mark a failed UAV and reassign its sector to active drones."""
        if failed_uav_id in self.drone_status:
            self.drone_status[failed_uav_id]["active"] = False
            self.drone_status[failed_uav_id]["assigned_task"] = "FAILED"
            logger.warn(f"Swarm failure detected: {failed_uav_id} is OFFLINE! Triggering dynamic sector reassignment.")
            # Re-partition among surviving drones
            self.partition_disaster_area(*area_bounds)


class SwarmManagerNode(Node if HAS_ROS2 else object):
    """ROS 2 Node coordinating multi-UAV swarm and shared situational awareness."""

    def __init__(self) -> None:
        if HAS_ROS2:
            if not rclpy.ok():
                rclpy.init()
            super().__init__("sih_swarm_node")
            self.declare_parameter("swarm_id", "sih_swarm_alpha")
            self.declare_parameter("drones", ["uav_1", "uav_2", "uav_3"])
            self.declare_parameter("x_min", -10.0)
            self.declare_parameter("x_max", 30.0)
            self.declare_parameter("y_min", -10.0)
            self.declare_parameter("y_max", 30.0)

            swarm_id = str(self.get_parameter("swarm_id").value)
            drone_list = list(self.get_parameter("drones").value)
            self.x_min = float(self.get_parameter("x_min").value)
            self.x_max = float(self.get_parameter("x_max").value)
            self.y_min = float(self.get_parameter("y_min").value)
            self.y_max = float(self.get_parameter("y_max").value)

            self.coordinator = SwarmCoordinator(swarm_id=swarm_id, drones=drone_list)
            self.coordinator.partition_disaster_area(self.x_min, self.x_max, self.y_min, self.y_max)

            # Publishers
            if HAS_SWARM_MSG:
                self.pub_swarm_state = self.create_publisher(SwarmState, "/swarm/state", 10)
            else:
                self.pub_swarm_state = None

            self.pub_shared_map = self.create_publisher(String, "/swarm/shared_map", 10)

            # Subscribers: Support namespaced or unified drone telemetry
            # 1. Fallback single-drone / primary drone topics
            self.create_subscription(
                DroneState, "/drone/state", self._on_single_drone_state, 10
            ) if HAS_SWARM_MSG else None

            # 2. Per-UAV subscriptions for UAV-1, UAV-2, UAV-3
            for d in drone_list:
                self._setup_drone_subscriptions(d)

            # Swarm management heartbeat timer (1 Hz)
            self.timer = self.create_timer(1.0, self._step_swarm_management)
            self.get_logger().info(f"sih_swarm_node active. Managing {len(drone_list)} UAVs: {drone_list}")
        else:
            self.coordinator = SwarmCoordinator()
            self.x_min, self.x_max, self.y_min, self.y_max = -10.0, 30.0, -10.0, 30.0
            self.coordinator.partition_disaster_area(self.x_min, self.x_max, self.y_min, self.y_max)
            logger.info("SwarmCoordinator initialized in standalone mode.")

    def _setup_drone_subscriptions(self, drone_id: str) -> None:
        """Dynamically create ROS 2 topic subscribers for an individual drone."""
        if not HAS_ROS2:
            return

        def state_cb(msg: DroneState):
            if drone_id in self.coordinator.drone_status:
                self.coordinator.drone_status[drone_id].update({
                    "active": True,
                    "position": (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                    "battery": msg.battery_percentage,
                    "flight_mode": msg.flight_mode,
                    "last_heartbeat": time.time(),
                })

        def surv_cb(msg: SurvivorArray):
            for s in msg.survivors:
                self.coordinator.shared_survivors[s.id] = {
                    "id": s.id,
                    "classification": s.classification,
                    "confidence": s.confidence,
                    "lat": s.gps_coordinate.latitude,
                    "lon": s.gps_coordinate.longitude,
                    "x": s.local_position.x,
                    "y": s.local_position.y,
                    "z": s.local_position.z,
                    "reported_by": drone_id,
                }
                self.coordinator.mission_phase = "SURVIVOR_FOUND"

        def haz_cb(msg: HazardArray):
            for h in msg.hazards:
                self.coordinator.shared_hazards[h.id] = {
                    "id": h.id,
                    "type": h.hazard_type,
                    "severity": h.severity,
                    "confidence": h.confidence,
                    "lat": h.gps_coordinate.latitude,
                    "lon": h.gps_coordinate.longitude,
                    "reported_by": drone_id,
                }

        if HAS_SWARM_MSG:
            self.create_subscription(DroneState, f"/{drone_id}/drone/state", state_cb, 10)
            self.create_subscription(SurvivorArray, f"/{drone_id}/survivors", surv_cb, 10)
            self.create_subscription(HazardArray, f"/{drone_id}/hazards", haz_cb, 10)

    def _on_single_drone_state(self, msg: DroneState) -> None:
        """Handle root /drone/state topic mapping to uav_1."""
        d_id = msg.drone_id if msg.drone_id in self.coordinator.drone_status else "uav_1"
        self.coordinator.drone_status[d_id].update({
            "active": True,
            "position": (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            "battery": msg.battery_percentage,
            "flight_mode": msg.flight_mode,
            "last_heartbeat": time.time(),
        })

    def _step_swarm_management(self) -> None:
        if not HAS_ROS2:
            return

        now = time.time()
        # 1. Heartbeat monitoring & fault tolerance
        for d, status in list(self.coordinator.drone_status.items()):
            if status["active"] and (now - status["last_heartbeat"]) > 10.0:
                # Trigger failover
                self.coordinator.handle_uav_failure(
                    failed_uav_id=d,
                    area_bounds=(self.x_min, self.x_max, self.y_min, self.y_max)
                )

        # 2. Coverage calculation
        active_count = sum(1 for d in self.coordinator.drone_status.values() if d["active"])
        self.coordinator.coverage_pct = min(100.0, self.coordinator.coverage_pct + active_count * 0.5)

        # 3. Publish SwarmState
        if self.pub_swarm_state is not None:
            sw_msg = SwarmState()
            sw_msg.header = Header()
            sw_msg.header.stamp = self.get_clock().now().to_msg()
            sw_msg.header.frame_id = "disaster_world"
            sw_msg.swarm_id = self.coordinator.swarm_id
            sw_msg.mission_phase = self.coordinator.mission_phase
            sw_msg.leader_drone_id = self.coordinator.leader_id
            sw_msg.total_search_area_sqm = int((self.x_max - self.x_min) * (self.y_max - self.y_min))
            sw_msg.coverage_percentage = float(self.coordinator.coverage_pct)

            for d, st in self.coordinator.drone_status.items():
                if st["active"]:
                    sw_msg.active_drones.append(d)
                    p = st["position"]
                    sw_msg.drone_positions.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
                    sw_msg.assigned_tasks.append(st["assigned_task"])

            self.pub_swarm_state.publish(sw_msg)

        # 4. Publish Shared Situational Map (JSON)
        shared_map_data = {
            "timestamp": now,
            "swarm_id": self.coordinator.swarm_id,
            "mission_phase": self.coordinator.mission_phase,
            "coverage_pct": round(self.coordinator.coverage_pct, 1),
            "drones": self.coordinator.drone_status,
            "sectors": self.coordinator.sector_assignments,
            "survivors": self.coordinator.shared_survivors,
            "hazards": self.coordinator.shared_hazards,
        }
        map_str_msg = String()
        map_str_msg.data = json.dumps(shared_map_data)
        self.pub_shared_map.publish(map_str_msg)


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 MULTI-UAV SWARM ALLOCATION DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    swarm = SwarmCoordinator()
    sectors = swarm.partition_disaster_area(x_min=-10.0, x_max=20.0, y_min=-10.0, y_max=20.0)

    print(f" Swarm ID: {swarm.swarm_id} ({len(swarm.drones)} UAVs configured)")
    print(" Allocated Search Sectors:")
    for drone_id, info in sectors.items():
        b = info["bounds"]
        print(f"   * [{drone_id}]: Assigned to {info['sector_id']} -> X:[{b[0]:.1f}, {b[1]:.1f}]m, Y:[{b[2]:.1f}, {b[3]:.1f}]m")

    print("\n Simulating UAV-3 failure and dynamic reallocation...")
    swarm.handle_uav_failure("uav_3", area_bounds=(-10.0, 20.0, -10.0, 20.0))
    print(" Reassigned Sectors across surviving drones:")
    for drone_id, info in swarm.sector_assignments.items():
        b = info["bounds"]
        print(f"   * [{drone_id}]: {info['sector_id']} -> X:[{b[0]:.1f}, {b[1]:.1f}]m")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if HAS_ROS2:
        rclpy.init(args=args)
        node = SwarmManagerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        run_standalone_demo()


if __name__ == "__main__":
    main()
