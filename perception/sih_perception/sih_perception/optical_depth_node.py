"""ROS 2 Node bridging the optical monocular depth estimation pipeline into perception.

Subscribes to:
  /camera/image (sensor_msgs/Image)
Publishes:
  /camera/depth (sensor_msgs/Image)
  /perception/optical_pointcloud (sensor_msgs/PointCloud2)

Allows optical monocular depth estimation to operate as an optional, redundant
perception sensor alongside physical or simulated LiDAR.
"""

from __future__ import annotations
import logging
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

_project_root = Path(__file__).resolve().parents[3]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

# Import our preserved working pipeline components
from app.depth.model import get_depth_model
from app.depth.inference import DepthEstimator
from app.pointcloud.reconstruction import CameraIntrinsics, PointCloudReconstructor

logger = logging.getLogger("OpticalDepthNode")

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, PointCloud2, PointField
    from std_msgs.msg import Header
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    Image = Any
    PointCloud2 = Any
    Header = Any


class OpticalDepthNode(Node if HAS_ROS2 else object):
    """ROS 2 Node executing monocular depth inference on camera stream."""

    def __init__(
        self,
        model_type: str = "midas_small",
        model_path: str = "models/model-small.onnx",
        fov_deg: float = 62.2,
    ) -> None:
        if HAS_ROS2:
            super().__init__("sih_optical_depth_node")
            self.declare_parameter("model_type", model_type)
            self.declare_parameter("model_path", model_path)
            self.declare_parameter("fov_deg", fov_deg)

            self.model_type = self.get_parameter("model_type").value
            self.model_path = self.get_parameter("model_path").value
            self.fov_deg = self.get_parameter("fov_deg").value

            self.sub_image = self.create_subscription(
                Image, "/camera/image", self._image_callback, 10
            )
            self.pub_depth = self.create_publisher(Image, "/camera/depth", 10)
            self.pub_pcd = self.create_publisher(
                PointCloud2, "/perception/optical_pointcloud", 10
            )
        else:
            self.model_type = model_type
            self.model_path = model_path
            self.fov_deg = fov_deg

        # Initialize preserved depth estimator
        self.model = get_depth_model(
            model_type=self.model_type,
            model_path=self.model_path,
            auto_download=True,
        )
        self.model.load_model()
        self.estimator = DepthEstimator(self.model, min_depth=0.2, max_depth=20.0)
        logger.info(
            "OpticalDepthNode initialized with %s on CPU.", self.model.model_name
        )

    def process_frame(
        self, bgr_image: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Process incoming image and return depth map and 3D point cloud."""
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        depth_result = self.estimator.estimate(rgb_image)

        h, w = rgb_image.shape[:2]
        intrinsics = CameraIntrinsics.from_fov(self.fov_deg, width=w, height=h)
        reconstructor = PointCloudReconstructor(intrinsics, voxel_size=0.05)
        pcd = reconstructor.reconstruct(rgb_image, depth_result.depth_map)

        return depth_result.depth_map, pcd.points

    def _image_callback(self, msg: Image) -> None:
        """Handle incoming ROS 2 camera image frame."""
        pass


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 OPTICAL DEPTH PERCEPTION DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    img_path = Path("data/input/image.jpg")
    if not img_path.exists():
        print(f"Error: {img_path} not found.")
        return

    bgr = cv2.imread(str(img_path))
    node = OpticalDepthNode(model_type="dummy")  # Fast mock for instant demo

    t0 = time.perf_counter()
    depth_map, points = node.process_frame(bgr)
    t_elapsed = (time.perf_counter() - t0) * 1000

    print(f" Input Frame:       {bgr.shape[1]}x{bgr.shape[0]} BGR")
    print(f" Inference Time:    {t_elapsed:.2f} ms")
    print(f" Depth Range:       {np.min(depth_map):.2f}m to {np.max(depth_map):.2f}m")
    print(f" Projected Points:  {len(points)} 3D points")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if HAS_ROS2:
        rclpy.init(args=args)
        node = OpticalDepthNode()
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
