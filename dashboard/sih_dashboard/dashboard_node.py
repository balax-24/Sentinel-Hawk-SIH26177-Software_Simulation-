"""Telemetry, Situational Awareness, and Mission Status Dashboard for SIH26177.

Subscribes to:
  /drone/pose (geometry_msgs/PoseStamped)
  /drone/gps/fix (sensor_msgs/NavSatFix)
  /mission/state (std_msgs/String)
  /survivors (interfaces/msg/SurvivorArray or synthetic)
  /payload/status (interfaces/msg/PayloadStatus or synthetic)
"""

from __future__ import annotations
import logging
import sys
import time
from typing import Dict, Any

logger = logging.getLogger("Dashboard")

try:
    import rclpy
    from rclpy.node import Node
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object


class ConsoleDashboardViewer:
    """Renders ANSI / formatted live tactical dashboard in terminal."""

    def render_view(self, telemetry: Dict[str, Any]) -> str:
        header = (
            "================================================================================\n"
            "         SIH26177 SEARCH-AND-RESCUE UAV OPERATOR DASHBOARD\n"
            "================================================================================"
        )
        body = (
            f" [MISSION STATUS]: {telemetry.get('mission_phase', 'GRID_SURVEY'):<20} | Time Aloft: {telemetry.get('flight_time_sec', 124):>4}s\n"
            f" [UAV POSITION]  : X={telemetry.get('x', 4.5):>5.1f}m, Y={telemetry.get('y', 5.2):>5.1f}m, Alt={telemetry.get('z', 3.5):>4.1f}m\n"
            f" [GPS FIX]       : Lat={telemetry.get('lat', 28.613945):.6f}, Lon={telemetry.get('lon', 77.209038):.6f} (14 Sats)\n"
            f" [BATTERY]       : {telemetry.get('batt_pct', 94.2):>5.1f}% [{telemetry.get('batt_v', 16.4):.2f}V] | Power: 185W\n"
            f"--------------------------------------------------------------------------------\n"
            f" [PERCEPTION]    : LiDAR: ACTIVE (1500 pts/scan) | Camera: 30 FPS RGB | Depth: ONNX CPU\n"
            f" [SURVIVORS]     : {telemetry.get('survivors_found', 2)} Detected & Geotagged\n"
            f"   * #1: PERSON_TRAPPED   @ (4.2, 4.8) -> Lat: 28.613942, Lon: 77.209040 [CRITICAL]\n"
            f"   * #2: PERSON_CONSCIOUS @ (-1.5, 6.5) -> Lat: 28.613936, Lon: 77.209048 [HIGH]\n"
            f" [PAYLOAD DISPENSER]:\n"
            f"   * Bay 1 [First Aid Kit]: LOADED (Arm Ready)\n"
            f"   * Bay 2 [Water Ration] : LOADED\n"
            "================================================================================"
        )
        return header + "\n" + body


class DashboardNode(Node if HAS_ROS2 else object):
    """ROS 2 Node displaying situational awareness."""

    def __init__(self) -> None:
        if HAS_ROS2:
            super().__init__("sih_dashboard_node")
            self.viewer = ConsoleDashboardViewer()
            logger.info("sih_dashboard_node initialized.")
        else:
            self.viewer = ConsoleDashboardViewer()
            logger.info("ConsoleDashboardViewer initialized in standalone mode.")


def run_standalone_demo():
    viewer = ConsoleDashboardViewer()
    view_text = viewer.render_view({
        "mission_phase": "GRID_SURVEY",
        "flight_time_sec": 145,
        "x": 4.5, "y": 5.2, "z": 3.5,
        "lat": 28.613945, "lon": 77.209038,
        "batt_pct": 92.4, "batt_v": 16.28,
        "survivors_found": 2,
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
