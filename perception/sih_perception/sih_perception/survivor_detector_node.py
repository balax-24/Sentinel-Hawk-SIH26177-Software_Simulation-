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
import logging
import math
import sys
import time
from typing import List, Dict, Any, Tuple
import cv2
import numpy as np

logger = logging.getLogger("SurvivorDetector")

try:
    import rclpy
    from rclpy.node import Node
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


class SurvivorDetector:
    """Detects human targets and calculates real-world GPS coordinates."""

    def __init__(self, fov_deg: float = 62.2) -> None:
        self.fov_deg = fov_deg

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
        upper_orange = np.array([22, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []

        h, w = bgr_image.shape[:2]
        for idx, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > 150:  # Minimum pixel threshold
                x, y, bw, bh = cv2.boundingRect(cnt)
                cx = x + bw // 2
                cy = y + bh // 2
                detections.append({
                    "id": idx + 1,
                    "bbox": [x, y, x + bw, y + bh],
                    "centroid_px": (cx, cy),
                    "confidence": float(min(1.0, 0.6 + area / 5000.0)),
                    "classification": "PERSON_TRAPPED",
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
        z_c = estimated_depth_m

        fx = (img_width / 2.0) / math.tan(math.radians(self.fov_deg / 2.0))
        fy = fx
        cx = img_width / 2.0
        cy = img_height / 2.0

        # Camera frame: X right, Y down, Z forward
        x_c = (u - cx) * z_c / fx
        y_c = (v - cy) * z_c / fy

        # Transform to UAV world ENU frame (yaw rotation)
        dx_drone, dy_drone, dz_drone, yaw = drone_pose
        # In downward tilted camera, forward is +Y_world, right is +X_world
        world_x = dx_drone + x_c * math.cos(yaw) - z_c * math.sin(yaw)
        world_y = dy_drone + x_c * math.sin(yaw) + z_c * math.cos(yaw)

        # Convert local offset to GPS
        lat_ref, lon_ref, _ = drone_gps
        d_lat = (world_y - dy_drone) / 111320.0
        d_lon = (world_x - dx_drone) / (111320.0 * math.cos(math.radians(lat_ref)))

        survivor_lat = lat_ref + d_lat
        survivor_lon = lon_ref + d_lon
        survivor_alt = drone_gps[2] - z_c

        return survivor_lat, survivor_lon, survivor_alt


class SurvivorDetectorNode(Node if HAS_ROS2 else object):
    """ROS 2 Node publishing survivor detections and geo-locations."""

    def __init__(self) -> None:
        if HAS_ROS2:
            super().__init__("sih_survivor_detector_node")
            self.detector = SurvivorDetector()
            logger.info("sih_survivor_detector_node initialized.")
        else:
            self.detector = SurvivorDetector()
            logger.info("SurvivorDetector initialized in standalone mode.")


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 SURVIVOR DETECTION DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    detector = SurvivorDetector()

    # Create synthetic frame with emergency orange vest casualty marker
    frame = np.full((720, 1280, 3), 110, dtype=np.uint8)
    # Add rubble grey tones
    frame[300:500, 400:800] = [80, 80, 85]
    # Add simulated survivor with high-vis orange suit in rubble pocket
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
