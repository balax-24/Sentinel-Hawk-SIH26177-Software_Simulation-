"""Unit tests for 3D point cloud reconstruction and camera geometry."""

import math
import numpy as np
import pytest

from app.pointcloud.reconstruction import (
    CameraIntrinsics,
    PointCloudData,
    PointCloudReconstructor,
)


def test_camera_intrinsics_from_fov():
    """Verify focal length and principal point calculation from horizontal FOV."""
    fov_deg = 60.0
    width = 640
    height = 480
    intrinsics = CameraIntrinsics.from_fov(fov_deg, width, height)

    # Expected fx = (W / 2) / tan(fov / 2)
    expected_fx = (width / 2.0) / math.tan(math.radians(30.0))
    assert math.isclose(intrinsics.fx, expected_fx, rel_tol=1e-4)
    assert math.isclose(intrinsics.fy, expected_fx, rel_tol=1e-4)
    assert intrinsics.cx == 320.0
    assert intrinsics.cy == 240.0
    assert intrinsics.width == 640
    assert intrinsics.height == 480


def test_camera_intrinsics_from_config():
    """Verify explicit parameters override FOV."""
    cfg = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}
    intrinsics = CameraIntrinsics.from_config(cfg, width=640, height=480)
    assert intrinsics.fx == 500.0
    assert intrinsics.fy == 500.0
    assert intrinsics.cx == 320.0
    assert intrinsics.cy == 240.0


def test_reconstruction_pinhole_math():
    """Verify accurate 3D coordinate back-projection from known synthetic depth."""
    width = 100
    height = 100
    fov_deg = 90.0  # tan(45 deg) = 1.0 => fx = 50.0
    intrinsics = CameraIntrinsics.from_fov(fov_deg, width, height)
    reconstructor = PointCloudReconstructor(
        intrinsics=intrinsics,
        voxel_size=None,
        remove_outliers=False,
    )

    # Flat plane at depth Z = 2.0 meters
    depth_map = np.full((height, width), 2.0, dtype=np.float32)
    rgb_image = np.zeros((height, width, 3), dtype=np.uint8)
    rgb_image[:, :] = [255, 128, 64]  # Unique color

    pcd = reconstructor.reconstruct(rgb_image, depth_map)

    assert len(pcd) == width * height

    # Center pixel (cx, cy) = (50, 50) must back-project to X=0, Y=0, Z=2.0
    center_idx = 50 * width + 50
    center_pt = pcd.points[center_idx]
    assert math.isclose(center_pt[0], 0.0, abs_tol=1e-3)
    assert math.isclose(center_pt[1], 0.0, abs_tol=1e-3)
    assert math.isclose(center_pt[2], 2.0, abs_tol=1e-3)

    # Verify RGB preservation
    expected_rgb = np.array([255 / 255.0, 128 / 255.0, 64 / 255.0], dtype=np.float32)
    assert np.allclose(pcd.colors[center_idx], expected_rgb, atol=1e-3)


def test_reconstruction_invalid_depth_filtering():
    """Verify that zero, negative, NaN, and out-of-bounds depths are filtered out."""
    intrinsics = CameraIntrinsics.from_fov(60.0, 4, 4)
    reconstructor = PointCloudReconstructor(
        intrinsics=intrinsics,
        voxel_size=None,
        remove_outliers=False,
        depth_trunc=5.0,
    )

    depth_map = np.array(
        [
            [1.0, 2.0, 0.0, -1.0],     # 2 valid
            [np.nan, 3.0, 4.0, 6.0],   # 2 valid (6.0 exceeds depth_trunc=5.0)
            [1.5, 2.5, 3.5, 4.5],      # 4 valid
            [0.0, 0.0, 0.0, 0.0],      # 0 valid
        ],
        dtype=np.float32,
    )
    rgb_image = np.ones((4, 4, 3), dtype=np.uint8) * 200

    pcd = reconstructor.reconstruct(rgb_image, depth_map)
    # Total valid points: 2 + 2 + 4 + 0 = 8
    assert len(pcd) == 8
    assert np.all(pcd.points[:, 2] > 0.0)
    assert np.all(pcd.points[:, 2] <= 5.0)


def test_ply_saving(tmp_path):
    """Verify PLY serialization generates valid file with points and colors."""
    points = np.array(
        [[0.0, 0.0, 1.0], [1.0, 0.0, 2.0], [0.0, 1.0, 3.0]], dtype=np.float32
    )
    colors = np.array(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32
    )
    pcd = PointCloudData(points=points, colors=colors)

    # 1. Standard save_ply (uses Open3D if available)
    ply_path = tmp_path / "test.ply"
    pcd.save_ply(ply_path)

    assert ply_path.exists()
    assert ply_path.stat().st_size > 0

    with open(ply_path, "rb") as f:
        header_bytes = f.read(250).decode("latin-1")
        assert "ply" in header_bytes
        assert "element vertex 3" in header_bytes
        assert ("property double x" in header_bytes) or ("property float x" in header_bytes)
        assert "property uchar red" in header_bytes

    # 2. Native binary writer fallback test
    native_path = tmp_path / "test_native.ply"
    pcd._write_ply_native(native_path)
    assert native_path.exists()
    with open(native_path, "rb") as f:
        header_bytes = f.read(250).decode("latin-1")
        assert "property float x" in header_bytes
        assert "property uchar red" in header_bytes

