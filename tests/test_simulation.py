"""Unit tests for UAV kinematics, physics simulation, and synthetic sensors."""

import math
import numpy as np
import pytest

from simulation.simulation.uav_controller import UAVDynamicsSimulator, UAVState
from simulation.simulation.mock_sim_node import DisasterSceneGenerator


def test_uav_kinematics_and_movement():
    """Verify velocity commands correctly update UAV positions over time."""
    sim = UAVDynamicsSimulator(initial_pose=(0.0, 0.0, 3.0), max_velocity=5.0)

    # Command forward movement in X at 2.0 m/s for 1.0 second
    sim.set_cmd_vel(vx=2.0, vy=0.0, vz=0.0, yaw_rate=0.0)
    state = sim.step(dt=1.0)

    assert math.isclose(state.x, 2.0, abs_tol=1e-3)
    assert math.isclose(state.y, 0.0, abs_tol=1e-3)
    assert math.isclose(state.z, 3.0, abs_tol=1e-3)
    assert state.battery_percentage < 100.0  # Battery drained


def test_gps_coordinate_conversion():
    """Verify local metric offsets translate accurately to WGS84 coordinates."""
    sim = UAVDynamicsSimulator(initial_pose=(0.0, 0.0, 3.0))
    lat0, lon0, alt0 = sim.get_gps_coordinates()

    assert math.isclose(lat0, 28.613939, rel_tol=1e-6)
    assert math.isclose(lon0, 77.209021, rel_tol=1e-6)
    assert math.isclose(alt0, 219.0, rel_tol=1e-6)  # 216m ground + 3m hover

    # Move 111.32 meters North (+Y)
    sim.state.y = 111.32
    lat1, lon1, _ = sim.get_gps_coordinates()
    # 1 deg latitude ~ 111,320m => 111.32m should be ~ +0.001 deg
    assert math.isclose(lat1 - lat0, 0.001, abs_tol=1e-5)


def test_imu_readings():
    """Verify simulated IMU outputs gravity force when hovering."""
    sim = UAVDynamicsSimulator(initial_pose=(0.0, 0.0, 3.0))
    imu = sim.get_imu_readings()

    acc = imu["linear_acceleration"]
    gyro = imu["angular_velocity"]

    # When upright, Z acceleration is approximately +9.81 m/s^2 (specific force)
    assert math.isclose(acc[2], 9.81, abs_tol=0.2)
    assert math.isclose(acc[0], 0.0, abs_tol=0.2)
    assert math.isclose(gyro[0], 0.0, abs_tol=0.05)


def test_disaster_scene_lidar_generation():
    """Verify simulated 3D LiDAR point cloud returns points within sensor bounds."""
    scene = DisasterSceneGenerator()
    points = scene.generate_lidar_points(uav_x=0.0, uav_y=0.0, uav_z=3.0, num_points=1000)

    assert len(points) > 0
    assert points.shape[1] == 4  # X, Y, Z, Intensity

    dists = np.linalg.norm(points[:, :3], axis=1)
    assert np.all(dists >= 0.2)
    assert np.all(dists <= 50.0)
