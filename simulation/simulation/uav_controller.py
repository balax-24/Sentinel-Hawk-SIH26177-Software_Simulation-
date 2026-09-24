"""Simulated UAV flight kinematics and state estimation.

Independent of physics engine, providing realistic UAV movement and telemetry.
"""

from dataclasses import dataclass, field
import math
import time
from typing import Dict, Any, Tuple
import numpy as np


@dataclass
class UAVState:
    """Telemetry and flight state of the simulated UAV."""

    # 3D Position in local ENU frame (meters)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.3

    # Linear velocities (m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Euler angles (radians): roll, pitch, yaw
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Angular velocities (rad/s)
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

    # Flight controller mode
    armed: bool = True
    mode: str = "OFFBOARD"

    # Battery
    battery_percentage: float = 100.0
    battery_voltage: float = 16.8  # 4S LiPo

    # GPS Origin reference (lat, lon, alt)
    origin_lat: float = 28.613939
    origin_lon: float = 77.209021
    origin_alt: float = 216.0


class UAVDynamicsSimulator:
    """Calculates UAV kinematic updates based on velocity commands and flight time."""

    def __init__(
        self,
        initial_pose: Tuple[float, float, float] = (0.0, 0.0, 0.3),
        max_velocity: float = 5.0,
        battery_drain_rate_per_sec: float = 0.02,
    ) -> None:
        self.state = UAVState(x=initial_pose[0], y=initial_pose[1], z=initial_pose[2])
        self.max_vel = max_velocity
        self.battery_drain_rate = battery_drain_rate_per_sec
        self.last_update_time = time.perf_counter()

    def set_cmd_vel(
        self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0
    ) -> None:
        """Apply velocity setpoint from navigation or manual controller."""
        # Clamp velocities
        self.state.vx = float(np.clip(vx, -self.max_vel, self.max_vel))
        self.state.vy = float(np.clip(vy, -self.max_vel, self.max_vel))
        self.state.vz = float(np.clip(vz, -self.max_vel, self.max_vel))
        self.state.r = float(yaw_rate)

        # Approximate small tilt angles from acceleration/velocity
        self.state.pitch = float(np.clip(-self.state.vx * 0.08, -0.35, 0.35))
        self.state.roll = float(np.clip(self.state.vy * 0.08, -0.35, 0.35))

    def step(self, dt: float) -> UAVState:
        """Advance simulation by dt seconds."""
        if dt <= 0:
            return self.state

        # Update yaw
        self.state.yaw = (self.state.yaw + self.state.r * dt) % (2.0 * math.pi)

        # Coordinate transformation from body frame velocity to world frame
        cos_yaw = math.cos(self.state.yaw)
        sin_yaw = math.sin(self.state.yaw)

        # Assume cmd_vel is in local world/ENU frame for simplified high-level guidance
        self.state.x += self.state.vx * dt
        self.state.y += self.state.vy * dt
        self.state.z = max(0.0, self.state.z + self.state.vz * dt)

        # Battery discharge
        flight_power = 1.0 + 0.3 * (
            abs(self.state.vx) + abs(self.state.vy) + abs(self.state.vz)
        )
        self.state.battery_percentage = max(
            0.0, self.state.battery_percentage - self.battery_drain_rate * flight_power * dt
        )
        self.state.battery_voltage = 13.5 + (self.state.battery_percentage / 100.0) * 3.3

        return self.state

    def get_gps_coordinates(self) -> Tuple[float, float, float]:
        """Convert local metric (x, y, z) into WGS84 GPS (lat, lon, alt)."""
        # 1 deg latitude ~ 111,320 meters
        d_lat = self.state.y / 111320.0
        lat = self.state.origin_lat + d_lat

        # 1 deg longitude ~ 111,320 * cos(lat) meters
        lat_rad = math.radians(lat)
        d_lon = self.state.x / (111320.0 * max(0.1, math.cos(lat_rad)))
        lon = self.state.origin_lon + d_lon

        alt = self.state.origin_alt + self.state.z
        return lat, lon, alt

    def get_imu_readings(self) -> Dict[str, Tuple[float, float, float]]:
        """Return simulated IMU linear acceleration and angular velocity with noise."""
        # Gravity vector in world is (0, 0, -9.81)
        # When hovering upright, accelerometer senses specific force +9.81 upward in Z
        acc_z = 9.81 + np.random.normal(0, 0.02)
        acc_x = (self.state.vx * 0.1) + np.random.normal(0, 0.015)
        acc_y = (self.state.vy * 0.1) + np.random.normal(0, 0.015)

        ang_x = self.state.p + np.random.normal(0, 0.002)
        ang_y = self.state.q + np.random.normal(0, 0.002)
        ang_z = self.state.r + np.random.normal(0, 0.002)

        return {
            "linear_acceleration": (acc_x, acc_y, acc_z),
            "angular_velocity": (ang_x, ang_y, ang_z),
        }
