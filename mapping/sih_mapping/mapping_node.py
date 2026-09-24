"""3D Environment Reconstruction and Global Mapping Node for SIH26177.

Subscribes to:
  /lidar/processed_points (sensor_msgs/PointCloud2)
  /drone/pose (geometry_msgs/PoseStamped)
Publishes:
  /map/pointcloud (sensor_msgs/PointCloud2)
  /map/info (std_msgs/String)
"""

from __future__ import annotations
import json
import logging
import math
import sys
import time
from pathlib import Path
from typing import List, Tuple, Any, Optional
import numpy as np

logger = logging.getLogger("SIH_Mapping")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Header, String
    import sensor_msgs_py.point_cloud2 as pc2
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    PointCloud2 = Any
    PoseStamped = Any
    Header = Any
    String = Any
    pc2 = None
    qos_profile_sensor_data = None


class GlobalMapAccumulator:
    """Transforms and accumulates successive LiDAR scans into a global 3D map."""

    def __init__(
        self,
        voxel_size: float = 0.1,
        max_points: int = 500000,
        sensor_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self.voxel_size = voxel_size
        self.max_points = max_points
        self.sensor_offset = sensor_offset
        self.global_points = np.empty((0, 3), dtype=np.float32)

    def add_scan(
        self, local_pts: np.ndarray, drone_pose: Tuple[float, ...]
    ) -> int:
        """Transform local sensor points by drone pose into global frame and merge.

        Args:
            local_pts: (N, 3) point coordinates in UAV sensor frame
            drone_pose: (x, y, z, yaw) or (x, y, z, qx, qy, qz, qw)
        """
        if len(local_pts) == 0:
            return len(self.global_points)

        local_pts = local_pts.astype(np.float32)

        if len(drone_pose) == 4:
            # Planar yaw rotation + 3D translation (backward compatible with demo)
            dx, dy, dz, yaw = drone_pose
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            rot_pts = np.copy(local_pts)
            rot_pts[:, 0] = local_pts[:, 0] * cos_yaw - local_pts[:, 1] * sin_yaw + dx
            rot_pts[:, 1] = local_pts[:, 0] * sin_yaw + local_pts[:, 1] * cos_yaw + dy
            rot_pts[:, 2] = local_pts[:, 2] + dz
        elif len(drone_pose) >= 7:
            # Full 6-DOF 3D transformation with quaternion
            dx, dy, dz, qx, qy, qz, qw = drone_pose[:7]
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if norm > 0.0:
                qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

            R = np.array([
                [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
                [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx)],
                [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy)],
            ], dtype=np.float32)

            # Apply sensor mounting offset relative to base_link
            so = np.array(self.sensor_offset, dtype=np.float32)
            pts_base = local_pts + so
            rot_pts = (R @ pts_base.T).T + np.array([dx, dy, dz], dtype=np.float32)
        else:
            rot_pts = local_pts

        if len(self.global_points) == 0:
            self.global_points = rot_pts
        else:
            self.global_points = np.vstack([self.global_points, rot_pts])

        # Voxel downsample to prevent unbounded memory growth
        if len(self.global_points) > self.max_points or (len(self.global_points) % 2000 < len(rot_pts)):
            self._downsample()

        return len(self.global_points)

    def _downsample(self) -> None:
        if len(self.global_points) == 0:
            return
        voxel_idx = np.floor(self.global_points / self.voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_idx, axis=0, return_index=True)
        self.global_points = self.global_points[unique_idx]

    def save_ply(self, file_path: Path) -> Path:
        """Export global map to standard PLY format."""
        file_path = Path(file_path)
        file_path.parent.mkdir(parents=True, exist_ok=True)
        pts = self.global_points.astype(np.float32)
        n = len(pts)

        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {n}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "end_header\n"
        ).encode("ascii")

        with open(file_path, "wb") as f:
            f.write(header)
            f.write(pts.tobytes())

        logger.info("Saved 3D global map to %s (%d points)", file_path, n)
        return file_path


class MappingNode(Node if HAS_ROS2 else object):
    """ROS 2 Node accumulating 3D map from processed LiDAR data and live UAV pose."""

    def __init__(self) -> None:
        if HAS_ROS2:
            super().__init__("sih_mapping_node")

            self.declare_parameter("voxel_size", 0.1)
            self.declare_parameter("max_points", 500000)
            self.declare_parameter("map_frame", "disaster_world")
            self.declare_parameter("sensor_offset_x", 0.15)
            self.declare_parameter("sensor_offset_y", 0.0)
            self.declare_parameter("sensor_offset_z", 0.10)

            voxel_size = float(self.get_parameter("voxel_size").value)
            max_points = int(self.get_parameter("max_points").value)
            self.map_frame = str(self.get_parameter("map_frame").value)

            ox = float(self.get_parameter("sensor_offset_x").value)
            oy = float(self.get_parameter("sensor_offset_y").value)
            oz = float(self.get_parameter("sensor_offset_z").value)

            self.mapper = GlobalMapAccumulator(
                voxel_size=voxel_size,
                max_points=max_points,
                sensor_offset=(ox, oy, oz),
            )

            # Subscriptions
            self.sub_points = self.create_subscription(
                PointCloud2, "/lidar/processed_points", self._points_callback, qos_profile_sensor_data
            )
            self.sub_pose = self.create_subscription(
                PoseStamped, "/drone/pose", self._pose_callback, qos_profile_sensor_data
            )

            # Publishers
            self.pub_map = self.create_publisher(
                PointCloud2, "/map/pointcloud", 10
            )
            self.pub_info = self.create_publisher(
                String, "/map/info", 10
            )

            self.latest_pose: Optional[PoseStamped] = None
            self.scan_count = 0
            self.get_logger().info("sih_mapping_node initialized. Listening on /lidar/processed_points and /drone/pose...")
        else:
            self.mapper = GlobalMapAccumulator()
            logger.info("GlobalMapAccumulator initialized in standalone mode.")

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _points_callback(self, msg: PointCloud2) -> None:
        if not HAS_ROS2 or pc2 is None:
            return

        # 1. Decode incoming PointCloud2
        try:
            pts = pc2.read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)
        except Exception as e:
            self.get_logger().error(f"Failed to decode PointCloud2 in mapping: {e}")
            return

        if pts is None or len(pts) == 0:
            return

        finite_mask = np.isfinite(pts).all(axis=1)
        pts = pts[finite_mask]
        if len(pts) == 0:
            return

        # 2. Extract live UAV pose
        if self.latest_pose is not None:
            p = self.latest_pose.pose.position
            o = self.latest_pose.pose.orientation
            drone_pose = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        else:
            drone_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        # 3. Accumulate scan into global 3D map
        total_pts = self.mapper.add_scan(pts, drone_pose)

        # 4. Publish accumulated map point cloud
        if len(self.mapper.global_points) > 0:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = self.map_frame
            map_msg = pc2.create_cloud_xyz32(header, self.mapper.global_points)
            self.pub_map.publish(map_msg)

        # 5. Publish map metadata info
        info_data = {
            "scan_count": self.scan_count + 1,
            "total_map_points": total_pts,
            "last_scan_points": len(pts),
            "pose": {
                "x": round(drone_pose[0], 3),
                "y": round(drone_pose[1], 3),
                "z": round(drone_pose[2], 3),
            },
        }
        info_msg = String()
        info_msg.data = json.dumps(info_data)
        self.pub_info.publish(info_msg)

        self.scan_count += 1
        if self.scan_count % 20 == 1:
            self.get_logger().info(
                f"[3D Map] Scan #{self.scan_count}: added {len(pts)} pts -> Total global map points: {total_pts}"
            )


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 3D MAPPING ACCUMULATOR DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    mapper = GlobalMapAccumulator(voxel_size=0.1)

    # Simulate 5 flight poses as drone surveys disaster area
    waypoints = [
        (0.0, 0.0, 3.0, 0.0),
        (2.0, 1.0, 3.0, 0.1),
        (4.0, 2.0, 3.0, 0.2),
        (6.0, 3.0, 3.0, 0.3),
        (8.0, 4.0, 3.0, 0.4),
    ]

    for i, wp in enumerate(waypoints, 1):
        # Generate simulated scan of an obstacle at (5.0, 5.0, 0.5)
        local_scan = np.random.uniform(-4.0, 4.0, (1000, 3)).astype(np.float32)
        total_pts = mapper.add_scan(local_scan, wp)
        print(f" [Waypoint #{i}] Added scan at pose ({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}) -> Map points: {total_pts}")

    out_ply = Path("data/output/simulated_global_map.ply")
    mapper.save_ply(out_ply)
    print(f" Exported global disaster map to {out_ply} ({len(mapper.global_points)} points).")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if "--standalone" in sys.argv or "--demo" in sys.argv or not HAS_ROS2:
        run_standalone_demo()
        return

    if HAS_ROS2:
        rclpy.init(args=args)
        node = MappingNode()
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
