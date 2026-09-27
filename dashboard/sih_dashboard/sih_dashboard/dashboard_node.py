"""Telemetry, Situational Awareness, and Mission Status Dashboard for SIH26177.

Consumes live ROS 2 telemetry topics:
  /swarm/state (interfaces/msg/SwarmState)
  /swarm/shared_map (std_msgs/String)
  /drone/state (interfaces/msg/DroneState)
  /drone/pose (geometry_msgs/PoseStamped)
  /drone/gps/fix (sensor_msgs/NavSatFix)
  /mission/state (std_msgs/String)
  /survivors (interfaces/msg/SurvivorArray)
  /hazards (interfaces/msg/HazardArray)
  /payload/status (interfaces/msg/PayloadStatus)
  /map/info (std_msgs/String)
"""

from __future__ import annotations
import json
import logging
import os
import sys
import time
from pathlib import Path
from typing import Dict, Any, List, Optional

logger = logging.getLogger("Dashboard")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    PoseStamped = Any
    NavSatFix = Any
    String = Any
    qos_profile_sensor_data = None

try:
    from sih_interfaces.msg import (
        SwarmState,
        DroneState,
        SurvivorArray,
        HazardArray,
        PayloadStatus,
    )
    HAS_SIH_MSGS = True
except ImportError:
    HAS_SIH_MSGS = False
    SwarmState = Any
    DroneState = Any
    SurvivorArray = Any
    HazardArray = Any
    PayloadStatus = Any


class ConsoleDashboardViewer:
    """Renders ANSI / formatted live tactical dashboard in terminal."""

    def render_view(self, data: Dict[str, Any]) -> str:
        uav_positions = data.get("uav_positions", {"uav_1": (0.0, 0.0, 0.0)})
        survivors = data.get("survivors", [])
        hazards = data.get("hazards", [])
        payloads = data.get("payloads", {})
        map_info = data.get("map_info", {})
        swarm = data.get("swarm", {})

        header = (
            "================================================================================\n"
            "         SIH26177 SEARCH-AND-RESCUE DISASTER SWARM OPERATOR DASHBOARD\n"
            "================================================================================"
        )

        # Swarm section
        swarm_line = (
            f" [SWARM]: {swarm.get('id', 'sih_swarm_alpha')} | Phase: {swarm.get('phase', 'GRID_SEARCH'):<14} | "
            f"Coverage: {swarm.get('coverage_pct', 0.0):>4.1f}% | Active UAVs: {len(uav_positions)}"
        )

        # Drone positions
        drone_lines = [" [UAV TELEMETRY & SECTORS]:"]
        for d_id, pos in uav_positions.items():
            batt = data.get("batteries", {}).get(d_id, 95.0)
            task = data.get("tasks", {}).get(d_id, "SURVEY")
            drone_lines.append(
                f"   * {d_id:<7}: Pos=({pos[0]:>5.1f}, {pos[1]:>5.1f}, {pos[2]:>4.1f}m) | "
                f"Batt={batt:>5.1f}% | Task: {task}"
            )

        # Map info
        map_line = (
            f" [3D MAP RECONSTRUCTION]: Scans: {map_info.get('scan_count', 0)} | "
            f"Global Points: {map_info.get('total_map_points', 0):,} | Voxel: 0.1m"
        )

        # Survivors
        surv_lines = [f" [SURVIVORS DETECTED]: {len(survivors)} Identified & Geotagged"]
        if survivors:
            for s in survivors[:4]:
                s_id = s.get("id", 1)
                cls = s.get("classification", "PERSON")
                conf = s.get("confidence", 0.9)
                lat = s.get("lat", 0.0)
                lon = s.get("lon", 0.0)
                surv_lines.append(
                    f"   * #{s_id}: [{cls}] Conf={conf:.2f} -> Lat: {lat:.6f}, Lon: {lon:.6f}"
                )
        else:
            surv_lines.append("   (Scanning disaster zone... No casualties identified yet)")

        # Hazards
        haz_lines = [f" [HAZARDS DETECTED]: {len(hazards)} Active Environmental Danger Zones"]
        if hazards:
            for h in hazards[:4]:
                h_type = h.get("type", "UNKNOWN")
                sev = h.get("severity", "HIGH")
                conf = h.get("confidence", 0.8)
                src = h.get("source", "uav_1")
                haz_lines.append(
                    f"   * [{h_type}] Severity: {sev} (Conf: {conf:.2f}) - Reported by {src}"
                )
        else:
            haz_lines.append("   (No active fire, chemical, or structural collapse hazards flagged)")

        # Payload status
        payload_lines = [" [PAYLOAD DISPENSER]:"]
        if payloads:
            for b_id, p_info in payloads.items():
                payload_lines.append(
                    f"   * {b_id}: {p_info.get('type', 'MEDKIT')} - Status: {p_info.get('status', 'LOADED')}"
                )
        else:
            payload_lines.append("   * Bay 1 [First Aid Kit]: LOADED | Bay 2 [Water]: LOADED")

        footer = "================================================================================"

        lines = [
            header,
            swarm_line,
            "--------------------------------------------------------------------------------",
            "\n".join(drone_lines),
            "--------------------------------------------------------------------------------",
            map_line,
            "--------------------------------------------------------------------------------",
            "\n".join(surv_lines),
            "--------------------------------------------------------------------------------",
            "\n".join(haz_lines),
            "--------------------------------------------------------------------------------",
            "\n".join(payload_lines),
            footer,
        ]
        return "\n".join(lines)


class DashboardNode(Node if HAS_ROS2 else object):
    """ROS 2 Node displaying situational awareness and generating SitRep reports."""

    def __init__(self) -> None:
        if HAS_ROS2:
            if not rclpy.ok():
                rclpy.init()
            super().__init__("sih_dashboard_node")
            self.viewer = ConsoleDashboardViewer()

            self.data: Dict[str, Any] = {
                "uav_positions": {"uav_1": (0.0, 0.0, 0.0)},
                "batteries": {"uav_1": 98.0},
                "tasks": {"uav_1": "GRID_SURVEY"},
                "survivors": [],
                "hazards": [],
                "payloads": {},
                "map_info": {},
                "swarm": {"id": "sih_swarm_alpha", "phase": "GRID_SEARCH", "coverage_pct": 0.0},
            }

            # Subscriptions
            self.sub_pose = self.create_subscription(
                PoseStamped, "/drone/pose", self._pose_callback, qos_profile_sensor_data
            )
            self.sub_map_info = self.create_subscription(
                String, "/map/info", self._map_info_callback, 10
            )

            if HAS_SIH_MSGS:
                self.sub_swarm = self.create_subscription(
                    SwarmState, "/swarm/state", self._swarm_callback, 10
                )
                self.sub_drone_state = self.create_subscription(
                    DroneState, "/drone/state", self._drone_state_callback, 10
                )
                self.sub_survivors = self.create_subscription(
                    SurvivorArray, "/survivors", self._survivors_callback, 10
                )
                self.sub_hazards = self.create_subscription(
                    HazardArray, "/hazards", self._hazards_callback, 10
                )
                self.sub_payload = self.create_subscription(
                    PayloadStatus, "/payload/status", self._payload_callback, 10
                )

            # Render HUD timer (1 Hz)
            self.timer = self.create_timer(1.0, self._render_cycle)
            self.get_logger().info("sih_dashboard_node active. Ingesting live ROS 2 telemetry...")
        else:
            self.viewer = ConsoleDashboardViewer()
            logger.info("ConsoleDashboardViewer initialized in standalone mode.")

    def _pose_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.data["uav_positions"]["uav_1"] = (p.x, p.y, p.z)

    def _map_info_callback(self, msg: String) -> None:
        try:
            self.data["map_info"] = json.loads(msg.data)
        except Exception:
            pass

    def _swarm_callback(self, msg: SwarmState) -> None:
        self.data["swarm"] = {
            "id": msg.swarm_id,
            "phase": msg.mission_phase,
            "coverage_pct": msg.coverage_percentage,
        }
        for d_id, pos in zip(msg.active_drones, msg.drone_positions):
            self.data["uav_positions"][d_id] = (pos.x, pos.y, pos.z)
        for d_id, task in zip(msg.active_drones, msg.assigned_tasks):
            self.data["tasks"][d_id] = task

    def _drone_state_callback(self, msg: DroneState) -> None:
        d_id = msg.drone_id if msg.drone_id else "uav_1"
        self.data["batteries"][d_id] = msg.battery_percentage
        p = msg.pose.pose.position
        self.data["uav_positions"][d_id] = (p.x, p.y, p.z)

    def _survivors_callback(self, msg: SurvivorArray) -> None:
        survs = []
        for s in msg.survivors:
            survs.append({
                "id": s.id,
                "classification": s.classification,
                "confidence": s.confidence,
                "lat": s.gps_coordinate.latitude,
                "lon": s.gps_coordinate.longitude,
            })
        self.data["survivors"] = survs

    def _hazards_callback(self, msg: HazardArray) -> None:
        haz = []
        for h in msg.hazards:
            haz.append({
                "id": h.id,
                "type": h.hazard_type,
                "severity": h.severity,
                "confidence": h.confidence,
                "source": h.source_uav,
            })
        self.data["hazards"] = haz

    def _payload_callback(self, msg: PayloadStatus) -> None:
        self.data["payloads"][msg.bay_id] = {
            "type": msg.payload_type,
            "status": msg.status,
            "weight": msg.payload_weight_kg,
        }

    def _render_cycle(self) -> None:
        if not HAS_ROS2:
            return

        rendered = self.viewer.render_view(self.data)
        # Output clean HUD
        print("\033[H\033[J", end="")  # Clear screen ANSI
        print(rendered)

        # Periodically dump Situational Report (SitRep)
        try:
            out_dir = Path("data/output")
            out_dir.mkdir(parents=True, exist_ok=True)
            sitrep_file = out_dir / "sitrep_report.json"
            sitrep_data = {
                "timestamp": time.time(),
                "swarm": self.data.get("swarm", {}),
                "uav_positions": self.data.get("uav_positions", {}),
                "survivors": self.data.get("survivors", []),
                "hazards": self.data.get("hazards", []),
                "map_summary": self.data.get("map_info", {}),
            }
            with open(sitrep_file, "w", encoding="utf-8") as f:
                json.dump(sitrep_data, f, indent=2)
        except Exception:
            pass


def run_standalone_demo():
    viewer = ConsoleDashboardViewer()
    view_text = viewer.render_view({
        "swarm": {"id": "sih_swarm_alpha", "phase": "GRID_SEARCH", "coverage_pct": 34.5},
        "uav_positions": {
            "uav_1": (4.5, 5.2, 3.5),
            "uav_2": (12.0, 6.0, 3.5),
            "uav_3": (20.5, 4.8, 3.5),
        },
        "batteries": {"uav_1": 91.2, "uav_2": 88.4, "uav_3": 94.0},
        "tasks": {
            "uav_1": "SECTOR_1_SEARCH",
            "uav_2": "PAYLOAD_DROP",
            "uav_3": "SECTOR_3_SEARCH",
        },
        "survivors": [
            {"id": 1, "classification": "PERSON_TRAPPED", "confidence": 0.94, "lat": 28.613942, "lon": 77.209040},
            {"id": 2, "classification": "PERSON_CONSCIOUS", "confidence": 0.88, "lat": 28.613936, "lon": 77.209048},
        ],
        "hazards": [
            {"type": "FIRE", "severity": "CRITICAL", "confidence": 0.95, "source": "uav_1"},
            {"type": "DAMAGED_STRUCTURE", "severity": "HIGH", "confidence": 0.89, "source": "uav_2"},
        ],
        "payloads": {
            "bay_1": {"type": "FIRST_AID_KIT", "status": "RELEASED"},
            "bay_2": {"type": "WATER_RATION", "status": "LOADED"},
        },
        "map_info": {"scan_count": 210, "total_map_points": 5840},
    })
    print(view_text)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if HAS_ROS2:
        rclpy.init(args=args)
        node = DashboardNode()
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
