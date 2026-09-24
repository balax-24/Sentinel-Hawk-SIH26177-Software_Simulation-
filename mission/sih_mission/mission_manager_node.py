"""Mission state machine and mission coordinator for SIH Search & Rescue.

Orchestrates the entire high-level mission lifecycle:
  IDLE -> PREFLIGHT -> TAKEOFF -> GRID_SURVEY -> SURVIVOR_FOUND -> DELIVER_SUPPLIES -> RTL
Publishes:
  /mission/state (std_msgs/String)
"""

from __future__ import annotations
import enum
import logging
import sys
import time
from typing import Dict, Any

logger = logging.getLogger("MissionManager")

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    String = Any


class MissionState(enum.Enum):
    IDLE = "IDLE"
    PREFLIGHT_CHECK = "PREFLIGHT_CHECK"
    TAKEOFF = "TAKEOFF"
    GRID_SURVEY = "GRID_SURVEY"
    SURVIVOR_FOUND = "SURVIVOR_FOUND"
    PAYLOAD_DELIVERY = "PAYLOAD_DELIVERY"
    RETURN_TO_LAUNCH = "RETURN_TO_LAUNCH"
    MISSION_COMPLETE = "MISSION_COMPLETE"


class SARMissionCoordinator:
    """Manages disaster search and rescue phases and emergency transitions."""

    def __init__(self, uav_id: str = "uav_alpha_1") -> None:
        self.uav_id = uav_id
        self.current_state = MissionState.IDLE
        self.survivors_located = 0
        self.payloads_delivered = 0

    def transition_to(self, new_state: MissionState) -> None:
        old_state = self.current_state
        self.current_state = new_state
        logger.info(
            "Mission Phase Transition: [%s] ===> [%s]",
            old_state.value,
            new_state.value,
        )

    def on_survivor_reported(self, survivor_id: int) -> None:
        """Handle alert from perception node."""
        self.survivors_located += 1
        logger.info("Survivor #%d confirmed! Entering precision drop protocol.", survivor_id)
        self.transition_to(MissionState.SURVIVOR_FOUND)


class MissionManagerNode(Node if HAS_ROS2 else object):
    """ROS 2 Node publishing mission status."""

    def __init__(self) -> None:
        if HAS_ROS2:
            super().__init__("sih_mission_manager_node")
            self.coordinator = SARMissionCoordinator()
            self.pub_state = self.create_publisher(String, "/mission/state", 10)
            logger.info("sih_mission_manager_node initialized.")
        else:
            self.coordinator = SARMissionCoordinator()
            logger.info("SARMissionCoordinator initialized in standalone mode.")


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 MISSION MANAGER DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    coordinator = SARMissionCoordinator()

    timeline = [
        (MissionState.PREFLIGHT_CHECK, "Checking 3D LiDAR, Cameras, GPS fix (12 satellites), Battery=100%"),
        (MissionState.TAKEOFF, "Arming motors, climbing to survey altitude Z=3.5m"),
        (MissionState.GRID_SURVEY, "Traversing lawnmower search lanes over disaster rubble"),
        (MissionState.SURVIVOR_FOUND, "Optical perception detected casualty at Lat=28.613945, Lon=77.209041"),
        (MissionState.PAYLOAD_DELIVERY, "Hovering over pocket, triggering Bay 1 first aid drop"),
        (MissionState.RETURN_TO_LAUNCH, "Supplies delivered. Returning to base station GPS coordinates"),
        (MissionState.MISSION_COMPLETE, "UAV safely landed. Mission logged."),
    ]

    for state, log_msg in timeline:
        coordinator.transition_to(state)
        print(f" -> Current State: {coordinator.current_state.value:<18} | {log_msg}")
        time.sleep(0.3)

    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if HAS_ROS2:
        rclpy.init(args=args)
        node = MissionManagerNode()
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
