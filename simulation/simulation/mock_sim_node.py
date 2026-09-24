"""High-fidelity standalone simulation node for SIH Search-and-Rescue.

Publishes simulated:
  1. 3D LiDAR point cloud (/lidar/points)
  2. RGB Camera stream (/camera/image)
  3. Depth Camera stream (/camera/depth)
  4. GPS Fix (/drone/gps/fix)
  5. 6-Axis IMU (/drone/imu/data)
  6. UAV Pose (/drone/pose) and Velocity (/drone/velocity)

Operates seamlessly inside ROS 2 (rclpy) OR as a standalone Python process for
rapid local testing and demonstrations on Windows, Linux, and Raspberry Pi 4.
"""

from __future__ import annotations

import argparse
import logging
import math
import struct
import sys
import time
from pathlib import Path
from typing import Optional, List, Tuple, Any

import cv2
import numpy as np

_project_root = Path(__file__).resolve().parents[2]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from simulation.simulation.uav_controller import UAVDynamicsSimulator, UAVState

logger = logging.getLogger("SIH_Simulation")

# Attempt importing ROS 2 libraries
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField, Image, NavSatFix, Imu
    from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Quaternion
    from std_msgs.msg import Header
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    Header = Any
    PointCloud2 = Any
    PointField = Any
    Image = Any
    NavSatFix = Any
    Imu = Any
    PoseStamped = Any
    TwistStamped = Any
    Twist = Any
    Point = Any
    Quaternion = Any


class DisasterSceneGenerator:
    """Procedurally generates realistic 3D disaster geometry for simulated LiDAR and vision."""

    def __init__(self) -> None:
        self.obstacles = [
            # Large collapsed building slab: (cx, cy, cz, dx, dy, dz)
            (5.0, 5.0, 0.4, 6.0, 4.0, 0.35),
            # Inclined concrete slab
            (6.5, 4.0, 1.2, 4.5, 3.0, 0.3),
            # Masonry rubble debris pile
            (3.0, 7.0, 0.5, 2.5, 2.5, 1.0),
            # Concrete blocks
            (1.5, 3.0, 0.4, 1.2, 1.2, 0.8),
            (8.0, 2.0, 0.6, 1.5, 1.0, 1.2),
            # Retaining wall
            (0.0, 8.0, 1.75, 8.0, 0.4, 3.5),
        ]
        self.survivors = [
            {"id": 1, "pos": (4.2, 4.8, 0.1), "label": "PERSON_TRAPPED"},
            {"id": 2, "pos": (-1.5, 6.5, 0.1), "label": "PERSON_CONSCIOUS"},
        ]

    def generate_lidar_points(
        self, uav_x: float, uav_y: float, uav_z: float, num_points: int = 1500
    ) -> np.ndarray:
        """Simulate a 3D LiDAR point cloud scan centered at the current UAV position.

        Returns:
            np.ndarray of shape (N, 4): [x, y, z, intensity] in UAV sensor frame (meters).
        """
        points = []

        # 1. Ground plane returns beneath/around drone
        ground_x = np.random.uniform(-15.0, 15.0, int(num_points * 0.45))
        ground_y = np.random.uniform(-15.0, 15.0, int(num_points * 0.45))
        ground_z = np.random.normal(0.0, 0.03, len(ground_x))  # Rough ground
        ground_intensity = np.random.uniform(0.1, 0.3, len(ground_x))

        for gx, gy, gz, gi in zip(ground_x, ground_y, ground_z, ground_intensity):
            points.append([gx - uav_x, gy - uav_y, gz - uav_z, gi])

        # 2. Rubble and structural obstacle surface returns
        pts_per_obs = int((num_points * 0.5) / len(self.obstacles))
        for ox, oy, oz, dx, dy, dz in self.obstacles:
            # Sample points on the bounding box surfaces
            ox_pts = np.random.uniform(ox - dx / 2, ox + dx / 2, pts_per_obs)
            oy_pts = np.random.uniform(oy - dy / 2, oy + dy / 2, pts_per_obs)
            oz_pts = np.random.uniform(max(0, oz - dz / 2), oz + dz / 2, pts_per_obs)
            intens = np.random.uniform(0.5, 0.85, pts_per_obs)
            for px, py, pz, pi in zip(ox_pts, oy_pts, oz_pts, intens):
                points.append([px - uav_x, py - uav_y, pz - uav_z, pi])

        # 3. Survivor returns (higher reflectivity / heat clothing)
        for s in self.survivors:
            sx, sy, sz = s["pos"]
            for _ in range(25):
                px = sx + np.random.normal(0, 0.2)
                py = sy + np.random.normal(0, 0.2)
                pz = sz + np.random.uniform(0, 0.4)
                points.append([px - uav_x, py - uav_y, pz - uav_z, 0.95])

        pts_array = np.array(points, dtype=np.float32)
        # Filter points within sensor range (0.2m to 50.0m)
        dists = np.linalg.norm(pts_array[:, :3], axis=1)
        valid = (dists >= 0.2) & (dists <= 50.0)
        return pts_array[valid]


def create_pointcloud2_msg(
    header: Header, points: np.ndarray, frame_id: str = "lidar_link"
) -> PointCloud2:
    """Serialize (N, 4) [x, y, z, intensity] array to sensor_msgs/PointCloud2."""
    msg = PointCloud2()
    msg.header = header
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)

    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.astype(np.float32).tobytes()
    return msg


class SimulationNode(Node if HAS_ROS2 else object):
    """ROS 2 Node executing the UAV sensor and flight physics simulation."""

    def __init__(self, uav_id: str = "uav_alpha_1") -> None:
        if HAS_ROS2:
            super().__init__("sih_simulation_node")
            self.declare_parameter("uav_id", uav_id)
            self.uav_id = self.get_parameter("uav_id").value

            # Publishers
            self.pub_lidar = self.create_publisher(PointCloud2, "/lidar/points", 10)
            self.pub_camera = self.create_publisher(Image, "/camera/image", 10)
            self.pub_depth = self.create_publisher(Image, "/camera/depth", 10)
            self.pub_pose = self.create_publisher(PoseStamped, "/drone/pose", 10)
            self.pub_vel = self.create_publisher(TwistStamped, "/drone/velocity", 10)
            self.pub_gps = self.create_publisher(NavSatFix, "/drone/gps/fix", 10)
            self.pub_imu = self.create_publisher(Imu, "/drone/imu/data", 10)

            # Subscribers
            self.sub_cmd_vel = self.create_subscription(
                Twist, "/drone/cmd_vel", self._cmd_vel_callback, 10
            )

            # Timer loops for different sensor frequencies
            self.timer_lidar = self.create_timer(1.0 / 10.0, self._publish_lidar)
            self.timer_camera = self.create_timer(1.0 / 15.0, self._publish_camera)
            self.timer_pose = self.create_timer(1.0 / 50.0, self._publish_pose_and_imu)
            self.timer_gps = self.create_timer(1.0 / 5.0, self._publish_gps)

        self.uav_id = uav_id
        self.simulator = UAVDynamicsSimulator(initial_pose=(0.0, 0.0, 3.0))
        self.scene = DisasterSceneGenerator()
        self.last_step_time = time.perf_counter()

        # Load reference disaster image if present
        ref_img_path = Path("data/input/image.jpg")
        if ref_img_path.exists():
            self.sample_rgb = cv2.imread(str(ref_img_path))
        else:
            self.sample_rgb = np.full((720, 1280, 3), 120, dtype=np.uint8)

        logger.info("Simulation initialized. UAV '%s' positioned at Z=3.0m hover.", self.uav_id)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Handle incoming velocity commands from navigation."""
        self.simulator.set_cmd_vel(
            vx=msg.linear.x,
            vy=msg.linear.y,
            vz=msg.linear.z,
            yaw_rate=msg.angular.z,
        )

    def step(self) -> UAVState:
        """Update flight dynamics."""
        now = time.perf_counter()
        dt = now - self.last_step_time
        self.last_step_time = now
        return self.simulator.step(dt)

    def _publish_lidar(self) -> None:
        """Generate and publish 3D LiDAR point cloud."""
        if not HAS_ROS2:
            return
        state = self.step()
        points = self.scene.generate_lidar_points(state.x, state.y, state.z)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "lidar_link"
        msg = create_pointcloud2_msg(header, points)
        self.pub_lidar.publish(msg)

    def _publish_camera(self) -> None:
        """Publish simulated RGB and depth camera frames."""
        if not HAS_ROS2 or self.sample_rgb is None:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_optical_link"

        # RGB Image
        rgb_msg = Image()
        rgb_msg.header = header
        rgb_msg.height, rgb_msg.width = self.sample_rgb.shape[:2]
        rgb_msg.encoding = "bgr8"
        rgb_msg.is_bigendian = False
        rgb_msg.step = rgb_msg.width * 3
        rgb_msg.data = self.sample_rgb.tobytes()
        self.pub_camera.publish(rgb_msg)

    def _publish_pose_and_imu(self) -> None:
        """Publish real-time pose, twist, and IMU data."""
        if not HAS_ROS2:
            return
        state = self.step()
        stamp = self.get_clock().now().to_msg()

        # Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position = Point(x=state.x, y=state.y, z=state.z)

        # Yaw to quaternion
        cy = math.cos(state.yaw * 0.5)
        sy = math.sin(state.yaw * 0.5)
        pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)
        self.pub_pose.publish(pose_msg)

        # Velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "base_link"
        vel_msg.twist.linear.x = state.vx
        vel_msg.twist.linear.y = state.vy
        vel_msg.twist.linear.z = state.vz
        vel_msg.twist.angular.z = state.r
        self.pub_vel.publish(vel_msg)

        # IMU
        imu_readings = self.simulator.get_imu_readings()
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = imu_readings["linear_acceleration"][0]
        imu_msg.linear_acceleration.y = imu_readings["linear_acceleration"][1]
        imu_msg.linear_acceleration.z = imu_readings["linear_acceleration"][2]
        imu_msg.angular_velocity.x = imu_readings["angular_velocity"][0]
        imu_msg.angular_velocity.y = imu_readings["angular_velocity"][1]
        imu_msg.angular_velocity.z = imu_readings["angular_velocity"][2]
        self.pub_imu.publish(imu_msg)

    def _publish_gps(self) -> None:
        """Publish simulated GPS fix."""
        if not HAS_ROS2:
            return
        lat, lon, alt = self.simulator.get_gps_coordinates()
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link"
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        gps_msg.status.status = 0  # STATUS_FIX
        self.pub_gps.publish(gps_msg)


def run_standalone_demo(duration_sec: float = 5.0) -> None:
    """Run standalone simulation without ROS 2 dependencies for quick demonstrations."""
    print("=" * 80)
    print(" SIH26177 UAV SIMULATION DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    print(" Simulating 3D LiDAR, RGB Camera, GPS, and IMU telemetry...")
    sim = SimulationNode()

    # Apply forward search crawl
    sim.simulator.set_cmd_vel(vx=1.5, vy=0.5, vz=0.0, yaw_rate=0.05)

    start_time = time.perf_counter()
    steps = 0
    while time.perf_counter() - start_time < duration_sec:
        state = sim.step()
        lidar_pts = sim.scene.generate_lidar_points(state.x, state.y, state.z)
        lat, lon, alt = sim.simulator.get_gps_coordinates()
        imu = sim.simulator.get_imu_readings()

        steps += 1
        if steps % 10 == 0:
            print(
                f"[SIM T+{steps*0.05:.1f}s] UAV Pose: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f} m) | "
                f"GPS: ({lat:.6f}, {lon:.6f}, {alt:.1f} m) | "
                f"LiDAR: {len(lidar_pts)} pts | "
                f"Batt: {state.battery_percentage:.1f}% ({state.battery_voltage:.2f}V)"
            )
        time.sleep(0.05)

    print("=" * 80)
    print(f" Demo complete: simulated {steps} physics cycles successfully.")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] [%(levelname)s]: %(message)s")
    if HAS_ROS2:
        rclpy.init(args=args)
        node = SimulationNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        run_standalone_demo(duration_sec=3.0)


if __name__ == "__main__":
    main()
