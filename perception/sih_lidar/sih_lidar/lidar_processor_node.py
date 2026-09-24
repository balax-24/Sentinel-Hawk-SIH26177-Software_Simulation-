"""LiDAR Point Cloud processing node for SIH Search & Rescue.

Filters ground plane, removes noise, and extracts 3D obstacles from /lidar/points.
Works in ROS 2 mode (rclpy) or standalone demonstration mode.
"""

from __future__ import annotations
import logging
import sys
import time
from typing import Optional, List, Tuple, Any
import numpy as np

logger = logging.getLogger("SIH_LiDAR")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from geometry_msgs.msg import Point, Vector3
    from std_msgs.msg import Header
    import sensor_msgs_py.point_cloud2 as pc2
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    PointCloud2 = Any
    Header = Any
    pc2 = None
    qos_profile_sensor_data = None

try:
    from sih_interfaces.msg import Obstacle, ObstacleArray
    HAS_OBSTACLES_MSG = True
except ImportError:
    HAS_OBSTACLES_MSG = False


class LidarProcessor:
    """Core LiDAR point cloud algorithmic processing without ROS dependencies."""

    def __init__(
        self,
        min_range: float = 0.5,
        max_range: float = 40.0,
        ground_threshold_z: float = 0.25,
        voxel_size: float = 0.1,
    ) -> None:
        self.min_range = min_range
        self.max_range = max_range
        self.ground_threshold_z = ground_threshold_z
        self.voxel_size = voxel_size

    def filter_points(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Separate raw LiDAR point cloud into obstacle points and ground returns.

        Args:
            points: (N, 3) or (N, 4) array of [X, Y, Z, (intensity)]

        Returns:
            Tuple of (obstacle_points, ground_points)
        """
        if len(points) == 0:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.float32)

        xyz = points[:, :3].astype(np.float32)
        dists = np.linalg.norm(xyz, axis=1)

        # 1. Range pass-through filter
        in_range = (dists >= self.min_range) & (dists <= self.max_range)
        pts_filtered = xyz[in_range]

        if len(pts_filtered) == 0:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.float32)

        # 2. Voxel grid downsampling
        if self.voxel_size > 0:
            voxel_idx = np.floor(pts_filtered / self.voxel_size).astype(np.int32)
            _, unique_idx = np.unique(voxel_idx, axis=0, return_index=True)
            pts_filtered = pts_filtered[unique_idx]

        # 3. Ground plane separation (height threshold relative to sensor frame)
        ground_mask = pts_filtered[:, 2] <= self.ground_threshold_z
        ground_pts = pts_filtered[ground_mask]
        obstacle_pts = pts_filtered[~ground_mask]

        return obstacle_pts, ground_pts

    def cluster_obstacles(
        self, obstacle_pts: np.ndarray, cluster_dist: float = 0.8, min_samples: int = 5
    ) -> List[Tuple[np.ndarray, np.ndarray, float]]:
        """Cluster obstacle points into discrete 3D bounding boxes.

        Returns list of (centroid, dimensions, distance_to_origin).
        """
        if len(obstacle_pts) < min_samples:
            return []

        unvisited = np.ones(len(obstacle_pts), dtype=bool)
        clusters = []

        for i in range(len(obstacle_pts)):
            if not unvisited[i]:
                continue

            queue = [i]
            unvisited[i] = False
            cluster_idx = []

            while queue:
                curr = queue.pop(0)
                cluster_idx.append(curr)
                dists = np.linalg.norm(obstacle_pts - obstacle_pts[curr], axis=1)
                neighbors = np.where((dists <= cluster_dist) & unvisited)[0]
                for n in neighbors:
                    unvisited[n] = False
                    queue.append(n)

            if len(cluster_idx) >= min_samples:
                c_pts = obstacle_pts[cluster_idx]
                centroid = np.mean(c_pts, axis=0)
                dims = np.ptp(c_pts, axis=0)
                dims = np.maximum(dims, 0.4)
                dist = float(np.linalg.norm(centroid))
                clusters.append((centroid, dims, dist))

        return clusters


def decode_pointcloud2_xyz(msg: PointCloud2) -> np.ndarray:
    """Fast, robust extraction of XYZ coordinates from PointCloud2 with any field layout."""
    if len(msg.data) == 0 or msg.point_step < 12:
        return np.empty((0, 3), dtype=np.float32)
    try:
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        pts = raw.reshape(-1, msg.point_step)[:, :12].copy().view(dtype=np.float32)
        finite_mask = np.isfinite(pts).all(axis=1)
        return pts[finite_mask]
    except Exception:
        try:
            pts_list = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))
            if len(pts_list) == 0:
                return np.empty((0, 3), dtype=np.float32)
            pts = np.array(pts_list, dtype=np.float32)
            finite_mask = np.isfinite(pts).all(axis=1)
            return pts[finite_mask]
        except Exception:
            return np.empty((0, 3), dtype=np.float32)


class LidarProcessorNode(Node if HAS_ROS2 else object):
    """ROS 2 Node subscribing to /lidar/points and publishing filtered obstacles."""

    def __init__(self) -> None:
        if HAS_ROS2:
            super().__init__("sih_lidar_node")

            self.declare_parameter("min_range", 0.5)
            self.declare_parameter("max_range", 40.0)
            self.declare_parameter("ground_threshold_z", -0.5)
            self.declare_parameter("voxel_size", 0.1)

            min_range = float(self.get_parameter("min_range").value)
            max_range = float(self.get_parameter("max_range").value)
            ground_threshold_z = float(self.get_parameter("ground_threshold_z").value)
            voxel_size = float(self.get_parameter("voxel_size").value)

            self.processor = LidarProcessor(
                min_range=min_range,
                max_range=max_range,
                ground_threshold_z=ground_threshold_z,
                voxel_size=voxel_size,
            )

            self.sub_points = self.create_subscription(
                PointCloud2, "/lidar/points", self._points_callback, qos_profile_sensor_data
            )

            # Primary output topic
            self.pub_processed_points = self.create_publisher(
                PointCloud2, "/lidar/processed_points", 10
            )
            # Secondary / alias topic for backward compatibility
            self.pub_filtered_points = self.create_publisher(
                PointCloud2, "/lidar/filtered_points", 10
            )
            # Ground returns topic
            self.pub_ground_points = self.create_publisher(
                PointCloud2, "/lidar/ground_points", 10
            )

            if HAS_OBSTACLES_MSG:
                self.pub_obstacles = self.create_publisher(
                    ObstacleArray, "/obstacles", 10
                )
            else:
                self.pub_obstacles = None

            self.scan_count = 0
            self.get_logger().info("sih_lidar_node initialized. Listening on /lidar/points...")
        else:
            self.processor = LidarProcessor()
            logger.info("LidarProcessor initialized in standalone mode.")

    def _points_callback(self, msg: PointCloud2) -> None:
        if not HAS_ROS2 or pc2 is None:
            return

        # 1. Decode PointCloud2 into finite NumPy array
        pts = decode_pointcloud2_xyz(msg)
        if len(pts) == 0:
            return

        # 2. Apply core LiDAR processing: range filter, voxel downsampling, ground separation
        obstacle_pts, ground_pts = self.processor.filter_points(pts)

        # 4. Construct output header
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        # 5. Publish processed points (/lidar/processed_points & /lidar/filtered_points)
        output_pts = obstacle_pts if len(obstacle_pts) > 0 else pts
        proc_msg = pc2.create_cloud_xyz32(header, output_pts)
        self.pub_processed_points.publish(proc_msg)
        self.pub_filtered_points.publish(proc_msg)

        # 6. Publish ground returns (/lidar/ground_points)
        if len(ground_pts) > 0:
            gnd_msg = pc2.create_cloud_xyz32(header, ground_pts)
            self.pub_ground_points.publish(gnd_msg)

        # 7. Cluster obstacle points and publish ObstacleArray if available
        if self.pub_obstacles is not None and len(obstacle_pts) > 0:
            clusters = self.processor.cluster_obstacles(obstacle_pts)
            obs_array_msg = ObstacleArray()
            obs_array_msg.header = header
            for i, (centroid, dims, dist) in enumerate(clusters):
                obs = Obstacle()
                obs.header = header
                obs.id = i
                obs.classification = "OBSTACLE"
                obs.position = Point(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2]))
                obs.dimensions = Vector3(x=float(dims[0]), y=float(dims[1]), z=float(dims[2]))
                obs.confidence = 0.95
                obs.distance_to_uav = float(dist)
                obs_array_msg.obstacles.append(obs)
            self.pub_obstacles.publish(obs_array_msg)

        self.scan_count += 1
        if self.scan_count % 20 == 1:
            self.get_logger().info(
                f"[LiDAR] Processed scan #{self.scan_count}: "
                f"raw={len(pts)}, obstacles={len(obstacle_pts)}, ground={len(ground_pts)}"
            )


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 LiDAR PROCESSING DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    proc = LidarProcessor(voxel_size=0.1)

    # Generate synthetic raw scan: ground + two distinct obstacle clusters
    gnd_pts = np.hstack([np.random.uniform(-10, 10, (1000, 2)), np.random.normal(0.05, 0.03, (1000, 1))]).astype(np.float32)
    obs1 = np.random.normal(loc=[4.0, 4.0, 1.2], scale=0.25, size=(120, 3)).astype(np.float32)
    obs2 = np.random.normal(loc=[-3.0, 5.0, 1.8], scale=0.25, size=(120, 3)).astype(np.float32)
    raw_pts = np.vstack([gnd_pts, obs1, obs2])

    obs, gnd = proc.filter_points(raw_pts)
    clusters = proc.cluster_obstacles(obs)

    print(f" Raw LiDAR Points:     {len(raw_pts)}")
    print(f" Extracted Ground:     {len(gnd)} points")
    print(f" Filtered Obstacles:   {len(obs)} points")
    print(f" Detected 3D Clusters: {len(clusters)} objects")
    for i, (c, d, dist) in enumerate(clusters[:3], 1):
        print(f"   * Obstacle #{i}: Center=({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f}m), Dist={dist:.2f}m")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if "--standalone" in sys.argv or "--demo" in sys.argv or not HAS_ROS2:
        run_standalone_demo()
        return

    if HAS_ROS2:
        rclpy.init(args=args)
        node = LidarProcessorNode()
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
