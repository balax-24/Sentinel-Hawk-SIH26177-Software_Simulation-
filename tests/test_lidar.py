"""Unit tests for LiDAR point-cloud processing and obstacle clustering."""

import numpy as np
import pytest

from perception.sih_lidar.sih_lidar.lidar_processor_node import LidarProcessor


def test_lidar_ground_and_range_filtering():
    """Verify ground plane separation and distance truncation."""
    proc = LidarProcessor(min_range=1.0, max_range=20.0, ground_threshold_z=0.3, voxel_size=0.0)

    # 4 synthetic points:
    # Pt 1: Too close (0.5m) -> dropped
    # Pt 2: Too far (30m) -> dropped
    # Pt 3: Ground return (Z = 0.1m, dist = 5m) -> ground
    # Pt 4: Obstacle return (Z = 1.5m, dist = 5m) -> obstacle
    pts = np.array(
        [
            [0.5, 0.0, 0.0],
            [30.0, 0.0, 0.0],
            [5.0, 0.0, 0.1],
            [5.0, 0.0, 1.5],
        ],
        dtype=np.float32,
    )

    obs, gnd = proc.filter_points(pts)

    assert len(obs) == 1
    assert len(gnd) == 1
    assert obs[0, 2] == 1.5
    assert gnd[0, 2] == pytest.approx(0.1, abs=1e-4)


def test_lidar_obstacle_clustering():
    """Verify spatial grouping of obstacle points into 3D clusters."""
    proc = LidarProcessor()

    # Create a dense cluster of points around (3.0, 3.0, 1.0)
    rng = np.random.default_rng(42)
    cluster_pts = rng.normal(loc=[3.0, 3.0, 1.0], scale=0.1, size=(20, 3)).astype(np.float32)

    clusters = proc.cluster_obstacles(cluster_pts)
    assert len(clusters) == 1
    centroid, dims, dist = clusters[0]

    assert np.allclose(centroid, [3.0, 3.0, 1.0], atol=0.2)
    assert dist > 0
