"""Survivor detection and geo-location identification node for SIH Search & Rescue.

Subscribes to:
  /camera/image (sensor_msgs/Image)
  /camera/depth (sensor_msgs/Image)
  /drone/pose (geometry_msgs/PoseStamped)
  /drone/gps/fix (sensor_msgs/NavSatFix)
Publishes:
  /survivors (interfaces/msg/SurvivorArray)
"""

from __future__ import annotations
import json
import logging
import math
import sys
import time
from typing import List, Dict, Any, Tuple, Optional
import cv2
import numpy as np

logger = logging.getLogger("SurvivorDetector")

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
    from sih_interfaces.msg import Survivor, SurvivorArray
    HAS_SURVIVOR_MSG = True
except ImportError:
    HAS_SURVIVOR_MSG = False
    Survivor = Any
    SurvivorArray = Any


class SurvivorDetector:
    """Detects human targets and calculates real-world GPS coordinates."""

    def __init__(self, fov_deg: float = 62.2, detector_mode: str = "PROTOTYPE_HSV") -> None:
        self.fov_deg = fov_deg
        self.detector_mode = detector_mode  # "PROTOTYPE_HSV", "SIMULATED_TARGET", "TRAINED_MODEL"

    def detect_in_frame(
        self, bgr_image: np.ndarray
    ) -> List[Dict[str, Any]]:
        """Detect human / high-visibility emergency targets in RGB frame.

        Uses HSV color thresholding (for search-and-rescue high-visibility orange/red vest)
        as a deterministic, lightweight CPU detector ready for Raspberry Pi 4.
        """
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # Orange/Red disaster vest range in HSV
        lower_orange = np.array([5, 120, 100])
        upper_orange = np.array([24, 255, 255])
        mask1 = cv2.inRange(hsv, lower_orange, upper_orange)

        # Thermal hotspot / bright rescue beacon simulation (pure red wrap)
        lower_red = np.array([170, 120, 100])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        combined_mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []

        for idx, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > 100:  # Minimum pixel area threshold
                x, y, bw, bh = cv2.boundingRect(cnt)
                cx = x + bw // 2
                cy = y + bh // 2
                conf = float(min(1.0, 0.65 + area / 4000.0))
                priority = 1 if conf > 0.85 else 2

                detections.append({
                    "id": idx + 1,
                    "bbox": [int(x), int(y), int(x + bw), int(y + bh)],
                    "centroid_px": (int(cx), int(cy)),
                    "confidence": conf,
                    "classification": "PERSON_TRAPPED" if area > 300 else "PERSON_CONSCIOUS",
                    "priority_level": priority,
                    "detector_source": self.detector_mode,
                })

        return detections

    def calculate_gps_location(
        self,
        centroid_px: Tuple[int, int],
        estimated_depth_m: float,
        img_width: int,
        img_height: int,
        drone_pose: Tuple[float, float, float, float],  # x, y, z, yaw
        drone_gps: Tuple[float, float, float],          # lat, lon, alt
    ) -> Tuple[float, float, float]:
        """Project pixel and depth to world frame, then convert to GPS coordinates."""
        u, v = centroid_px
        z_c = max(0.5, estimated_depth_m)

        fx = (img_width / 2.0) / math.tan(math.radians(self.fov_deg / 2.0))
        fy = fx
        cx = img_width / 2.0
        cy = img_height / 2.0

        # Camera frame: X right, Y down, Z forward
        x_c = (u - cx) * z_c / fx
        y_c = (v - cy) * z_c / fy

        # Transform to UAV world ENU frame (yaw rotation)
        dx_drone, dy_drone, dz_drone, yaw = drone_pose
        world_x = dx_drone + x_c * math.cos(yaw) - z_c * math.sin(yaw)
        world_y = dy_drone + x_c * math.sin(yaw) + z_c * math.cos(yaw)

        # Convert local offset to WGS84 GPS
        lat_ref, lon_ref, _ = drone_gps
        d_lat = (world_y - dy_drone) / 111320.0
        d_lon = (world_x - dx_drone) / (111320.0 * math.cos(math.radians(lat_ref)))

        survivor_lat = lat_ref + d_lat
        survivor_lon = lon_ref + d_lon
        survivor_alt = max(0.0, drone_gps[2] - z_c)

        return survivor_lat, survivor_lon, survivor_alt


class SurvivorDetectorNode(Node if HAS_ROS2 else object):
    """ROS 2 Node publishing survivor detections and geo-locations."""

    def __init__(self) -> None:
        if HAS_ROS2:
            super().__init__("sih_survivor_detector_node")
            self.declare_parameter("fov_deg", 62.2)
            self.declare_parameter("detector_mode", "PROTOTYPE_HSV")
            self.declare_parameter("default_altitude", 3.0)

            fov = float(self.get_parameter("fov_deg").value)
            mode = str(self.get_parameter("detector_mode").value)
            self.default_altitude = float(self.get_parameter("default_altitude").value)

            self.detector = SurvivorDetector(fov_deg=fov, detector_mode=mode)
            self.latest_pose: Optional[PoseStamped] = None
            self.latest_gps: Optional[NavSatFix] = None
            self.confirmed_survivors: Dict[int, Dict[str, Any]] = {}
            self.next_survivor_id = 1

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

            # Publisher
            if HAS_SURVIVOR_MSG:
                self.pub_survivors = self.create_publisher(SurvivorArray, "/survivors", 10)
            else:
                self.pub_survivors = None

            self.get_logger().info(
                f"sih_survivor_detector_node initialized in [{mode}] mode. Subscribing to /camera/image..."
            )
        else:
            self.detector = SurvivorDetector()
            logger.info("SurvivorDetector initialized in standalone mode.")

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _gps_callback(self, msg: NavSatFix) -> None:
        self.latest_gps = msg

    def _image_callback(self, msg: Image) -> None:
        if not HAS_ROS2 or self.pub_survivors is None:
            return

        # Decode Image ROS 2 message to BGR OpenCV image
        if len(msg.data) == 0:
            return

        try:
            if msg.encoding in ("bgr8", "rgb8"):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                if msg.encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                return
        except Exception:
            return

        # Run detector
        raw_detections = self.detector.detect_in_frame(frame)
        if len(raw_detections) == 0:
            return

        # Extract current drone pose & GPS
        if self.latest_pose is not None:
            px = self.latest_pose.pose.position.x
            py = self.latest_pose.pose.position.y
            pz = self.latest_pose.pose.position.z
            # Approximate yaw from quaternion
            qz = self.latest_pose.pose.orientation.z
            qw = self.latest_pose.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            drone_pose = (px, py, pz, yaw)
            alt_m = max(1.0, pz)
        else:
            drone_pose = (0.0, 0.0, self.default_altitude, 0.0)
            alt_m = self.default_altitude

        if self.latest_gps is not None:
            drone_gps = (self.latest_gps.latitude, self.latest_gps.longitude, self.latest_gps.altitude)
        else:
            # Default disaster coordinate reference
            drone_gps = (28.613939, 77.209021, 219.5)

        h, w = frame.shape[:2]
        survivor_array_msg = SurvivorArray()
        survivor_array_msg.header = msg.header

        for det in raw_detections:
            lat, lon, alt = self.detector.calculate_gps_location(
                centroid_px=det["centroid_px"],
                estimated_depth_m=alt_m,
                img_width=w,
                img_height=h,
                drone_pose=drone_pose,
                drone_gps=drone_gps,
            )

            # Spatial deduplication across frames: match within ~2 meters
            matched_id = None
            for s_id, s_info in self.confirmed_survivors.items():
                d_lat = (lat - s_info["lat"]) * 111320.0
                d_lon = (lon - s_info["lon"]) * 111320.0 * math.cos(math.radians(lat))
                dist = math.sqrt(d_lat * d_lat + d_lon * d_lon)
                if dist < 2.5:
                    matched_id = s_id
                    break

            if matched_id is None:
                matched_id = self.next_survivor_id
                self.next_survivor_id += 1
                self.confirmed_survivors[matched_id] = {
                    "lat": lat, "lon": lon, "alt": alt,
                    "confidence": det["confidence"],
                    "classification": det["classification"],
                    "priority": det["priority_level"],
                }
                self.get_logger().info(
                    f"NEW SURVIVOR IDENTIFIED #{matched_id} [{det['classification']}]: Lat={lat:.6f}, Lon={lon:.6f}"
                )
            else:
                # Update confidence
                self.confirmed_survivors[matched_id]["confidence"] = max(
                    self.confirmed_survivors[matched_id]["confidence"], det["confidence"]
                )

            s_msg = Survivor()
            s_msg.header = msg.header
            s_msg.id = matched_id
            s_msg.classification = det["classification"]
            s_msg.confidence = det["confidence"]
            s_msg.priority_level = det["priority_level"]
            s_msg.bounding_box = det["bbox"]

            s_msg.local_position = Point(x=float(drone_pose[0]), y=float(drone_pose[1]), z=float(drone_pose[2]))
            s_msg.gps_coordinate = NavSatFix(latitude=lat, longitude=lon, altitude=alt)

            survivor_array_msg.survivors.append(s_msg)

        self.pub_survivors.publish(survivor_array_msg)


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 SURVIVOR DETECTION DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    detector = SurvivorDetector(detector_mode="PROTOTYPE_HSV")

    frame = np.full((720, 1280, 3), 110, dtype=np.uint8)
    frame[300:500, 400:800] = [80, 80, 85]
    cv2.rectangle(frame, (580, 380), (620, 440), (25, 130, 240), -1)

    detections = detector.detect_in_frame(frame)
    print(f" Detected Potential Survivors: {len(detections)}")

    drone_pose = (5.0, 5.0, 3.5, 0.0)
    drone_gps = (28.613939, 77.209021, 219.5)

    for d in detections:
        surv_lat, surv_lon, surv_alt = detector.calculate_gps_location(
            centroid_px=d["centroid_px"],
            estimated_depth_m=3.5,
            img_width=1280,
            img_height=720,
            drone_pose=drone_pose,
            drone_gps=drone_gps,
        )
        print(f"  * Survivor #{d['id']}: [{d['classification']}] Conf={d['confidence']:.2f}")
        print(f"    Bounding Box: {d['bbox']}")
        print(f"    Calculated GPS: Lat={surv_lat:.7f}, Lon={surv_lon:.7f}, Alt={surv_alt:.1f}m")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if HAS_ROS2:
        rclpy.init(args=args)
        node = SurvivorDetectorNode()
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
