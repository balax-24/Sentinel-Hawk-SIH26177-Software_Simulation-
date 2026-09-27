"""Mission state machine and mission coordinator for SIH Search & Rescue.

Orchestrates the entire high-level mission lifecycle:
  IDLE -> PREFLIGHT_CHECK -> TAKEOFF -> GRID_SURVEY -> SURVIVOR_FOUND -> PAYLOAD_DELIVERY -> RETURN_TO_LAUNCH -> MISSION_COMPLETE
Publishes:
  /mission/state (std_msgs/String)
  /drone/state (interfaces/msg/DroneState)
  /navigation/target_waypoint (geometry_msgs/PoseStamped)
Calls Service:
  /mission/trigger_payload (interfaces/srv/TriggerPayload)
"""

from __future__ import annotations
import enum
import json
import logging
import math
import sys
import time
from typing import Dict, Any, Optional, List

logger = logging.getLogger("MissionManager")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String, Header
    from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point
    from sensor_msgs.msg import NavSatFix, Imu
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    String = Any
    Header = Any
    PoseStamped = Any
    TwistStamped = Any
    Twist = Any
    Point = Any
    NavSatFix = Any
    Imu = Any
    qos_profile_sensor_data = None

try:
    from sih_interfaces.msg import DroneState, SurvivorArray, PayloadStatus
    from sih_interfaces.srv import TriggerPayload
    HAS_SIH_MSGS = True
except ImportError:
    HAS_SIH_MSGS = False
    DroneState = Any
    SurvivorArray = Any
    PayloadStatus = Any
    TriggerPayload = Any


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

    def __init__(self, uav_id: str = "uav_1") -> None:
        self.uav_id = uav_id
        self.current_state = MissionState.IDLE
        self.survivors_located = 0
        self.payloads_delivered = 0
        self.battery_pct = 100.0
        self.battery_voltage = 16.8
        self.armed = False

    def transition_to(self, new_state: MissionState) -> None:
        old_state = self.current_state
        self.current_state = new_state
        logger.info(
            "Mission Phase Transition [%s]: [%s] ===> [%s]",
            self.uav_id,
            old_state.value,
            new_state.value,
        )

    def on_survivor_reported(self, survivor_id: int) -> None:
        """Handle alert from perception node."""
        self.survivors_located += 1
        logger.info("Survivor #%d confirmed! Entering precision drop protocol.", survivor_id)
        self.transition_to(MissionState.SURVIVOR_FOUND)


class MissionManagerNode(Node if HAS_ROS2 else object):
    """ROS 2 Node orchestrating mission phases, survivor response, and drone telemetry."""

    def __init__(self, uav_id: str = "uav_1") -> None:
        if HAS_ROS2:
            if not rclpy.ok():
                rclpy.init()
            super().__init__("sih_mission_manager_node")
            self.declare_parameter("uav_id", uav_id)
            self.declare_parameter("takeoff_altitude", 3.0)
            self.declare_parameter("auto_deliver_payload", True)

            self.uav_id = str(self.get_parameter("uav_id").value)
            self.takeoff_alt = float(self.get_parameter("takeoff_altitude").value)
            self.auto_payload = bool(self.get_parameter("auto_deliver_payload").value)

            self.coordinator = SARMissionCoordinator(uav_id=self.uav_id)
            self.latest_pose: Optional[PoseStamped] = None
            self.latest_gps: Optional[NavSatFix] = None
            self.home_pose: Optional[PoseStamped] = None
            self.active_target_survivor: Optional[Dict[str, Any]] = None
            self.payload_delivered_ids: List[int] = []

            # Subscriptions
            self.sub_pose = self.create_subscription(
                PoseStamped, "/drone/pose", self._pose_callback, qos_profile_sensor_data
            )
            self.sub_gps = self.create_subscription(
                NavSatFix, "/drone/gps/fix", self._gps_callback, qos_profile_sensor_data
            )

            if HAS_SIH_MSGS:
                self.sub_survivors = self.create_subscription(
                    SurvivorArray, "/survivors", self._survivor_callback, 10
                )
                self.sub_payload = self.create_subscription(
                    PayloadStatus, "/payload/status", self._payload_status_callback, 10
                )
                self.pub_drone_state = self.create_publisher(DroneState, "/drone/state", 10)
                self.cli_payload = self.create_client(TriggerPayload, "/mission/trigger_payload")
            else:
                self.sub_survivors = None
                self.sub_payload = None
                self.pub_drone_state = None
                self.cli_payload = None

            # Publishers
            self.pub_state = self.create_publisher(String, "/mission/state", 10)
            self.pub_target_wp = self.create_publisher(PoseStamped, "/navigation/target_waypoint", 10)

            # High-level state machine evaluation timer (2 Hz)
            self.timer = self.create_timer(0.5, self._step_state_machine)
            self.state_start_time = time.time()
            self.get_logger().info(f"sih_mission_manager_node initialized for {self.uav_id}.")
        else:
            self.coordinator = SARMissionCoordinator()
            logger.info("SARMissionCoordinator initialized in standalone mode.")

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        if self.home_pose is None:
            self.home_pose = msg

    def _gps_callback(self, msg: NavSatFix) -> None:
        self.latest_gps = msg

    def _payload_status_callback(self, msg: PayloadStatus) -> None:
        if msg.status in ("RELEASED", "DELIVERED"):
            self.coordinator.payloads_delivered += 1
            if self.coordinator.current_state == MissionState.PAYLOAD_DELIVERY:
                self.get_logger().info("Payload confirmed dropped! Resuming mission.")
                self.coordinator.transition_to(MissionState.GRID_SURVEY)

    def _survivor_callback(self, msg: SurvivorArray) -> None:
        if len(msg.survivors) == 0:
            return

        for s in msg.survivors:
            if s.id not in self.payload_delivered_ids and self.coordinator.current_state == MissionState.GRID_SURVEY:
                self.active_target_survivor = {
                    "id": s.id,
                    "x": s.local_position.x,
                    "y": s.local_position.y,
                    "z": self.takeoff_alt,
                    "classification": s.classification,
                }
                self.coordinator.on_survivor_reported(s.id)
                self.state_start_time = time.time()
                # Direct UAV to survivor coordinates
                wp = PoseStamped()
                wp.header = msg.header
                wp.pose.position.x = s.local_position.x
                wp.pose.position.y = s.local_position.y
                wp.pose.position.z = self.takeoff_alt
                wp.pose.orientation.w = 1.0
                self.pub_target_wp.publish(wp)
                break

    def _step_state_machine(self) -> None:
        if not HAS_ROS2:
            return

        now_sec = time.time()
        elapsed = now_sec - self.state_start_time
        curr = self.coordinator.current_state

        # Slowly discharge simulated battery
        self.coordinator.battery_pct = max(10.0, self.coordinator.battery_pct - 0.05)
        self.coordinator.battery_voltage = 14.8 + (self.coordinator.battery_pct / 100.0) * 2.0

        if curr == MissionState.IDLE:
            self.coordinator.transition_to(MissionState.PREFLIGHT_CHECK)
            self.state_start_time = now_sec

        elif curr == MissionState.PREFLIGHT_CHECK:
            if elapsed > 2.0:
                self.coordinator.armed = True
                self.coordinator.transition_to(MissionState.TAKEOFF)
                self.state_start_time = now_sec

        elif curr == MissionState.TAKEOFF:
            # Command climb
            if self.latest_pose is not None:
                current_z = self.latest_pose.pose.position.z
                if current_z >= (self.takeoff_alt - 0.5) or elapsed > 8.0:
                    self.coordinator.transition_to(MissionState.GRID_SURVEY)
                    self.state_start_time = now_sec

        elif curr == MissionState.SURVIVOR_FOUND:
            # Check proximity to survivor
            if self.latest_pose is not None and self.active_target_survivor is not None:
                px = self.latest_pose.pose.position.x
                py = self.latest_pose.pose.position.y
                tx = self.active_target_survivor["x"]
                ty = self.active_target_survivor["y"]
                dist = math.sqrt((px - tx) ** 2 + (py - ty) ** 2)

                if dist < 2.0 or elapsed > 10.0:
                    self.coordinator.transition_to(MissionState.PAYLOAD_DELIVERY)
                    self.state_start_time = now_sec
                    if self.auto_payload and self.cli_payload is not None:
                        self._trigger_payload_drop()

        elif curr == MissionState.RETURN_TO_LAUNCH:
            if self.home_pose is not None:
                self.pub_target_wp.publish(self.home_pose)
            if elapsed > 15.0:
                self.coordinator.transition_to(MissionState.MISSION_COMPLETE)
                self.coordinator.armed = False

        # 1. Publish state string
        str_msg = String()
        str_msg.data = self.coordinator.current_state.value
        self.pub_state.publish(str_msg)

        # 2. Publish DroneState
        if self.pub_drone_state is not None:
            ds = DroneState()
            ds.header.stamp = self.get_clock().now().to_msg()
            ds.header.frame_id = "base_link"
            ds.drone_id = self.uav_id
            ds.armed = self.coordinator.armed
            ds.flight_mode = self.coordinator.current_state.value
            ds.battery_percentage = float(self.coordinator.battery_pct)
            ds.battery_voltage = float(self.coordinator.battery_voltage)

            if self.latest_pose is not None:
                ds.pose = self.latest_pose
            if self.latest_gps is not None:
                ds.gps_fix = self.latest_gps

            self.pub_drone_state.publish(ds)

    def _trigger_payload_drop(self) -> None:
        if self.cli_payload is None or not self.cli_payload.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("Payload service not available. Simulating drop.")
            self.coordinator.transition_to(MissionState.GRID_SURVEY)
            return

        req = TriggerPayload.Request()
        req.bay_id = "bay_1"
        req.confirm_target_clear = True
        future = self.cli_payload.call_async(req)
        future.add_done_callback(self._on_payload_srv_done)

    def _on_payload_srv_done(self, future) -> None:
        try:
            res = future.result()
            self.get_logger().info(f"Payload service response: {res.status_message}")
            if self.active_target_survivor is not None:
                self.payload_delivered_ids.append(self.active_target_survivor["id"])
            self.coordinator.transition_to(MissionState.GRID_SURVEY)
        except Exception as e:
            self.get_logger().error(f"Payload service call failed: {e}")
            self.coordinator.transition_to(MissionState.GRID_SURVEY)


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
        time.sleep(0.1)

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
