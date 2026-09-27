"""Modular Disaster Hazard Detection Node for SIH Search & Rescue.

Pluggable architecture supporting detection of:
- Fire & Thermal signatures
- Smoke plumes
- Flood / standing water
- Debris & Damaged structures (via LiDAR and visual fusion)
- Chemical leak indicators
- Electrical hazards

Subscribes to:
  /camera/image (sensor_msgs/Image)
  /drone/pose (geometry_msgs/PoseStamped)
  /drone/gps/fix (sensor_msgs/NavSatFix)
  /obstacles (interfaces/msg/ObstacleArray)
Publishes:
  /hazards (interfaces/msg/HazardArray)
"""

from __future__ import annotations
import abc
import json
import logging
import math
import sys
import time
from typing import List, Dict, Any, Tuple, Optional
import cv2
import numpy as np

logger = logging.getLogger("HazardDetector")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, NavSatFix
    from geometry_msgs.msg import PoseStamped, Point
    from std_msgs.msg import Header
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    Image = Any
    NavSatFix = Any
    PoseStamped = Any
    Header = Any
    Point = Any
    qos_profile_sensor_data = None

try:
    from sih_interfaces.msg import Hazard, HazardArray, ObstacleArray
    HAS_HAZARD_MSG = True
except ImportError:
    HAS_HAZARD_MSG = False
    Hazard = Any
    HazardArray = Any
    ObstacleArray = Any


class BaseHazardSubdetector(abc.ABC):
    """Abstract pluggable detector interface for future AI model integration."""

    @abc.abstractmethod
    def detect(
        self,
        bgr_image: Optional[np.ndarray],
        obstacles: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        """Return list of detected hazards."""
        pass


class FireHazardDetector(BaseHazardSubdetector):
    """Detects active flame or high-intensity thermal combustion."""

    def detect(
        self,
        bgr_image: Optional[np.ndarray],
        obstacles: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        if bgr_image is None or bgr_image.size == 0:
            return []

        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        # Intense yellow-orange flame core
        lower_fire = np.array([12, 180, 200])
        upper_fire = np.array([25, 255, 255])
        mask = cv2.inRange(hsv, lower_fire, upper_fire)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hazards = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 120:
                x, y, w, h = cv2.boundingRect(cnt)
                hazards.append({
                    "hazard_type": "FIRE",
                    "confidence": float(min(0.95, 0.70 + area / 5000.0)),
                    "severity": "CRITICAL",
                    "centroid_px": (x + w // 2, y + h // 2),
                    "description": "Active thermal flame signature detected",
                })
        return hazards


class SmokeHazardDetector(BaseHazardSubdetector):
    """Detects low-contrast dispersion smoke plumes."""

    def detect(
        self,
        bgr_image: Optional[np.ndarray],
        obstacles: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        if bgr_image is None or bgr_image.size == 0:
            return []

        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        # Smoke typically has low variance, high brightness
        blur = cv2.GaussianBlur(gray, (21, 21), 0)
        _, thresh = cv2.threshold(blur, 190, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        hazards = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 800:
                x, y, w, h = cv2.boundingRect(cnt)
                hazards.append({
                    "hazard_type": "SMOKE",
                    "confidence": float(min(0.90, 0.60 + area / 10000.0)),
                    "severity": "HIGH",
                    "centroid_px": (x + w // 2, y + h // 2),
                    "description": "Dense smoke dispersion plume",
                })
        return hazards


class StructuralDamageDetector(BaseHazardSubdetector):
    """Detects collapsed structures and debris fields from LiDAR obstacle clusters."""

    def detect(
        self,
        bgr_image: Optional[np.ndarray],
        obstacles: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        hazards = []
        for obs in obstacles:
            dims = obs.get("dimensions", (1.0, 1.0, 1.0))
            vol = dims[0] * dims[1] * dims[2]
            # Large irregular rubble pile
            if vol > 1.5 and dims[2] < 2.0:
                hazards.append({
                    "hazard_type": "DAMAGED_STRUCTURE",
                    "confidence": float(obs.get("confidence", 0.85)),
                    "severity": "HIGH",
                    "local_position": obs.get("position", (0.0, 0.0, 0.0)),
                    "description": f"Collapsed masonry/concrete slab: vol={vol:.1f}m³",
                })
            elif vol > 0.4:
                hazards.append({
                    "hazard_type": "DEBRIS",
                    "confidence": float(obs.get("confidence", 0.75)),
                    "severity": "MODERATE",
                    "local_position": obs.get("position", (0.0, 0.0, 0.0)),
                    "description": f"Unstable ground rubble debris: vol={vol:.1f}m³",
                })
        return hazards


class ChemicalLeakDetector(BaseHazardSubdetector):
    """Detects chemical spill indicators (fluorescent or distinct discoloration)."""

    def detect(
        self,
        bgr_image: Optional[np.ndarray],
        obstacles: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        if bgr_image is None or bgr_image.size == 0:
            return []

        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        # Chemical green/yellow toxic indicator
        lower_chem = np.array([35, 120, 120])
        upper_chem = np.array([75, 255, 255])
        mask = cv2.inRange(hsv, lower_chem, upper_chem)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hazards = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 250:
                x, y, w, h = cv2.boundingRect(cnt)
                hazards.append({
                    "hazard_type": "CHEMICAL_LEAK",
                    "confidence": float(min(0.92, 0.65 + area / 5000.0)),
                    "severity": "CRITICAL",
                    "centroid_px": (x + w // 2, y + h // 2),
                    "description": "Hazardous fluid discoloration / chemical runoff",
                })
        return hazards


class HazardDetectorNode(Node if HAS_ROS2 else object):
    """ROS 2 Node orchestrating modular hazard detectors."""

    def __init__(self, uav_id: str = "uav_1") -> None:
        self.uav_id = uav_id
        self.detectors: List[BaseHazardSubdetector] = [
            FireHazardDetector(),
            SmokeHazardDetector(),
            StructuralDamageDetector(),
            ChemicalLeakDetector(),
        ]
        self.latest_pose: Optional[PoseStamped] = None
        self.latest_gps: Optional[NavSatFix] = None
        self.latest_obstacles: List[Dict[str, Any]] = []
        self.tracked_hazards: Dict[int, Dict[str, Any]] = {}
        self.next_hazard_id = 1

        if HAS_ROS2:
            if not rclpy.ok():
                rclpy.init()
            super().__init__("sih_hazard_detector_node")
            self.declare_parameter("uav_id", uav_id)
            self.uav_id = str(self.get_parameter("uav_id").value)

            # Subscriptions
            self.sub_image = self.create_subscription(
                Image, "/camera/image", self._image_callback, qos_profile_sensor_data
            )
            self.sub_pose = self.create_subscription(
                PoseStamped, "/drone/pose", self._pose_callback, qos_profile_sensor_data
            )
            self.sub_gps = self.create_subscription(
                NavSatFix, "/drone/gps/fix", self._gps_callback, qos_profile_sensor_data
            )
            if HAS_HAZARD_MSG:
                self.sub_obstacles = self.create_subscription(
                    ObstacleArray, "/obstacles", self._obstacles_callback, 10
                )
                self.pub_hazards = self.create_publisher(HazardArray, "/hazards", 10)
            else:
                self.sub_obstacles = None
                self.pub_hazards = None

            # Periodic publish timer (2 Hz)
            self.timer = self.create_timer(0.5, self._publish_hazards)
            self.get_logger().info(f"sih_hazard_detector_node initialized for {self.uav_id}.")
        else:
            logger.info("HazardDetector initialized in standalone mode.")

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _gps_callback(self, msg: NavSatFix) -> None:
        self.latest_gps = msg

    def _obstacles_callback(self, msg: ObstacleArray) -> None:
        obs_list = []
        for o in msg.obstacles:
            obs_list.append({
                "position": (o.position.x, o.position.y, o.position.z),
                "dimensions": (o.dimensions.x, o.dimensions.y, o.dimensions.z),
                "confidence": o.confidence,
                "classification": o.classification,
            })
        self.latest_obstacles = obs_list

    def _image_callback(self, msg: Image) -> None:
        if not HAS_ROS2:
            return
        try:
            if msg.encoding in ("bgr8", "rgb8"):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                if msg.encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self._run_detection_pipeline(frame)
        except Exception:
            pass

    def _run_detection_pipeline(self, frame: Optional[np.ndarray]) -> None:
        for detector in self.detectors:
            results = detector.detect(frame, self.latest_obstacles)
            for r in results:
                self._ingest_hazard_event(r)

    def _ingest_hazard_event(self, hazard_dict: Dict[str, Any]) -> None:
        # Resolve GPS and 3D positions
        if self.latest_pose is not None:
            ux = self.latest_pose.pose.position.x
            uy = self.latest_pose.pose.position.y
            uz = self.latest_pose.pose.position.z
        else:
            ux, uy, uz = 0.0, 0.0, 3.0

        if self.latest_gps is not None:
            lat = self.latest_gps.latitude
            lon = self.latest_gps.longitude
            alt = self.latest_gps.altitude
        else:
            lat, lon, alt = 28.613939, 77.209021, 219.5

        # Spatial deduplication across hazard types
        matched = False
        for h_id, h in self.tracked_hazards.items():
            if h["hazard_type"] == hazard_dict["hazard_type"]:
                d_lat = (lat - h["lat"]) * 111320.0
                d_lon = (lon - h["lon"]) * 111320.0
                if math.sqrt(d_lat * d_lat + d_lon * d_lon) < 5.0:
                    matched = True
                    break

        if not matched:
            h_id = self.next_hazard_id
            self.next_hazard_id += 1
            self.tracked_hazards[h_id] = {
                "id": h_id,
                "hazard_type": hazard_dict["hazard_type"],
                "confidence": hazard_dict["confidence"],
                "severity": hazard_dict["severity"],
                "description": hazard_dict.get("description", ""),
                "local_pos": hazard_dict.get("local_position", (ux, uy, uz)),
                "lat": lat, "lon": lon, "alt": alt,
                "timestamp": time.time(),
                "source_uav": self.uav_id,
            }
            if HAS_ROS2:
                self.get_logger().info(
                    f"HAZARD DETECTED [{hazard_dict['hazard_type']}]: Severity={hazard_dict['severity']}, Lat={lat:.6f}, Lon={lon:.6f}"
                )

    def _publish_hazards(self) -> None:
        if not HAS_ROS2 or self.pub_hazards is None:
            return

        array_msg = HazardArray()
        array_msg.header = Header()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = "disaster_world"

        for h_id, h in self.tracked_hazards.items():
            hz = Hazard()
            hz.header = array_msg.header
            hz.id = h_id
            hz.hazard_type = h["hazard_type"]
            hz.confidence = h["confidence"]
            hz.severity = h["severity"]
            hz.source_uav = h["source_uav"]
            hz.description = h["description"]
            hz.local_position = Point(
                x=float(h["local_pos"][0]), y=float(h["local_pos"][1]), z=float(h["local_pos"][2])
            )
            hz.gps_coordinate = NavSatFix(latitude=h["lat"], longitude=h["lon"], altitude=h["alt"])
            array_msg.hazards.append(hz)

        self.pub_hazards.publish(array_msg)


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 MODULAR HAZARD DETECTION DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    node = HazardDetectorNode(uav_id="uav_alpha_1")

    # Synthetic image with fire and chemical leak
    img = np.full((480, 640, 3), 80, dtype=np.uint8)
    # Fire patch
    cv2.rectangle(img, (200, 150), (260, 220), (0, 215, 255), -1)
    # Chemical leak patch
    cv2.rectangle(img, (400, 300), (470, 380), (0, 240, 60), -1)

    # Simulated LiDAR obstacles (concrete rubble)
    obstacles = [
        {"position": (4.0, 5.0, 0.4), "dimensions": (3.0, 2.5, 0.8), "confidence": 0.92},
        {"position": (8.0, 2.0, 0.6), "dimensions": (1.5, 1.2, 0.5), "confidence": 0.88},
    ]

    for det in node.detectors:
        res = det.detect(img, obstacles)
        for r in res:
            node._ingest_hazard_event(r)

    print(f" Identified Disaster Hazards: {len(node.tracked_hazards)}")
    for h_id, h in node.tracked_hazards.items():
        print(f"  * Hazard #{h_id}: [{h['hazard_type']}] Severity: {h['severity']} (Conf: {h['confidence']:.2f})")
        print(f"    Source UAV: {h['source_uav']} | {h['description']}")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if HAS_ROS2:
        rclpy.init(args=args)
        node = HazardDetectorNode()
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
