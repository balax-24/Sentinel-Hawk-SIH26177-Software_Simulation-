"""3D Point Cloud reconstruction from 2D RGB image and monocular depth map.

IMPORTANT TECHNICAL NOTE:
This module back-projects optical depth estimates via camera pinhole intrinsic geometry.
It constructs a 3D point cloud from perspective ray projection, NOT physical LiDAR pulses.
"""

from dataclasses import dataclass
import logging
import math
from pathlib import Path
from typing import Optional, Tuple, Dict, Any

import numpy as np

logger = logging.getLogger(__name__)

# Try importing open3d, but gracefully allow fallback
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    o3d = None
    HAS_OPEN3D = False
    logger.warning("Open3D is not installed. Native pure-Python/NumPy fallback will be used.")


@dataclass
class CameraIntrinsics:
    """Pinhole camera intrinsic parameters."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_fov(
        cls, fov_x_deg: float, width: int, height: int
    ) -> "CameraIntrinsics":
        """Compute intrinsics from horizontal field of view."""
        fov_rad = math.radians(fov_x_deg)
        fx = (width / 2.0) / math.tan(fov_rad / 2.0)
        fy = fx  # Square pixels
        cx = width / 2.0
        cy = height / 2.0
        return cls(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height)

    @classmethod
    def from_config(
        cls, camera_cfg: Dict[str, Any], width: int, height: int
    ) -> "CameraIntrinsics":
        """Construct intrinsics from configuration dictionary.

        If explicit fx, fy, cx, cy are provided, they are used; otherwise,
        parameters are calculated using horizontal FOV.
        """
        fx = camera_cfg.get("fx")
        fy = camera_cfg.get("fy")
        cx = camera_cfg.get("cx")
        cy = camera_cfg.get("cy")
        fov_x_deg = camera_cfg.get("fov_x_deg", 62.2)

        if fx is not None and fy is not None:
            cx_val = cx if cx is not None else width / 2.0
            cy_val = cy if cy is not None else height / 2.0
            return cls(
                fx=float(fx),
                fy=float(fy),
                cx=float(cx_val),
                cy=float(cy_val),
                width=width,
                height=height,
            )

        return cls.from_fov(fov_x_deg=float(fov_x_deg), width=width, height=height)

    def to_matrix(self) -> np.ndarray:
        """Return 3x3 intrinsic matrix K."""
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


@dataclass
class PointCloudData:
    """Container for 3D point cloud data."""

    points: np.ndarray  # (N, 3) float32 (X, Y, Z in meters)
    colors: np.ndarray  # (N, 3) float32 [0.0, 1.0] (R, G, B)

    def __len__(self) -> int:
        return len(self.points)

    def to_open3d(self):
        """Convert to an Open3D PointCloud object."""
        if not HAS_OPEN3D:
            raise RuntimeError("Open3D is required for to_open3d() but is not installed.")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(self.colors.astype(np.float64))
        return pcd

    def save_ply(self, file_path: Path) -> Path:
        """Save point cloud to PLY file.

        Uses Open3D when available; otherwise falls back to a native binary PLY writer.
        """
        file_path = Path(file_path)
        file_path.parent.mkdir(parents=True, exist_ok=True)

        if HAS_OPEN3D:
            try:
                pcd = self.to_open3d()
                success = o3d.io.write_point_cloud(str(file_path), pcd, write_ascii=False)
                if success:
                    logger.info("Saved point cloud via Open3D to %s (%d points)", file_path, len(self))
                    return file_path
            except Exception as e:
                logger.warning("Open3D PLY export failed (%s). Using native PLY writer.", e)

        # Native binary PLY writer fallback
        self._write_ply_native(file_path)
        logger.info("Saved point cloud via native PLY writer to %s (%d points)", file_path, len(self))
        return file_path

    def _write_ply_native(self, file_path: Path) -> None:
        """Native binary PLY writer preserving vertex colors without external libraries."""
        n_points = len(self.points)
        pts = self.points.astype(np.float32)
        # Convert float colors [0, 1] to uint8 [0, 255]
        cols = np.clip(self.colors * 255.0, 0, 255).astype(np.uint8)

        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {n_points}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "end_header\n"
        ).encode("ascii")

        # Combine into structured array for fast binary serialization
        structured_type = np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("red", "u1"),
                ("green", "u1"),
                ("blue", "u1"),
            ]
        )
        data = np.empty(n_points, dtype=structured_type)
        data["x"] = pts[:, 0]
        data["y"] = pts[:, 1]
        data["z"] = pts[:, 2]
        data["red"] = cols[:, 0]
        data["green"] = cols[:, 1]
        data["blue"] = cols[:, 2]

        with open(file_path, "wb") as f:
            f.write(header)
            f.write(data.tobytes())


class PointCloudReconstructor:
    """Reconstructs 3D point clouds from RGB images and estimated depth maps."""

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        voxel_size: Optional[float] = 0.02,
        remove_outliers: bool = True,
        nb_neighbors: int = 20,
        std_ratio: float = 2.0,
        depth_trunc: float = 10.0,
    ) -> None:
        """Initialize reconstructor with camera geometry and filtering parameters."""
        self.intrinsics = intrinsics
        self.voxel_size = voxel_size if (voxel_size and voxel_size > 0) else None
        self.remove_outliers = remove_outliers
        self.nb_neighbors = nb_neighbors
        self.std_ratio = std_ratio
        self.depth_trunc = depth_trunc

    def reconstruct(
        self, rgb_image: np.ndarray, depth_map: np.ndarray
    ) -> PointCloudData:
        """Generate a 3D point cloud from RGB image and depth map.

        Args:
            rgb_image: Input RGB image of shape (H, W, 3) in uint8 or float32.
            depth_map: Estimated depth map of shape (H, W) in float32 (meters).

        Returns:
            PointCloudData containing 3D coordinates (X, Y, Z) and RGB vertex colors.
        """
        h, w = depth_map.shape[:2]
        if rgb_image.shape[:2] != (h, w):
            raise ValueError(
                f"Image shape {rgb_image.shape[:2]} and depth shape {(h, w)} mismatch!"
            )

        # Coordinate grid
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))

        # Filter valid depth values
        valid_mask = (
            (depth_map > 0.0)
            & (depth_map <= self.depth_trunc)
            & np.isfinite(depth_map)
        )

        num_valid = int(np.count_nonzero(valid_mask))
        if num_valid == 0:
            logger.warning("No valid depth points found for 3D reconstruction!")
            return PointCloudData(
                points=np.empty((0, 3), dtype=np.float32),
                colors=np.empty((0, 3), dtype=np.float32),
            )

        # Pinhole inverse projection:
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = Z
        z = depth_map[valid_mask]
        x = (u[valid_mask] - self.intrinsics.cx) * z / self.intrinsics.fx
        y = (v[valid_mask] - self.intrinsics.cy) * z / self.intrinsics.fy

        # Camera frame: standard pinhole X right, Y down, Z forward
        # Invert Y to match conventional 3D world convention (Y up)
        y = -y

        points = np.stack([x, y, z], axis=-1).astype(np.float32)

        # Extract RGB colors
        rgb_flat = rgb_image[valid_mask]
        if rgb_flat.dtype == np.uint8:
            colors = (rgb_flat / 255.0).astype(np.float32)
        else:
            colors = np.clip(rgb_flat, 0.0, 1.0).astype(np.float32)

        raw_count = len(points)
        logger.info("Raw 3D point cloud projected: %d points", raw_count)

        # Apply filtering if Open3D is available
        if HAS_OPEN3D and raw_count > 0:
            points, colors = self._filter_open3d(points, colors)
        elif not HAS_OPEN3D and self.voxel_size and raw_count > 0:
            points, colors = self._voxel_downsample_numpy(points, colors, self.voxel_size)

        return PointCloudData(points=points, colors=colors)

    def _filter_open3d(
        self, points: np.ndarray, colors: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Apply voxel downsampling and statistical outlier removal with Open3D."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))

        # 1. Voxel downsampling
        if self.voxel_size:
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            logger.info("Points after voxel downsampling (size=%.3f): %d", self.voxel_size, len(pcd.points))

        # 2. Statistical outlier removal (removes isolated flying pixels on depth boundaries)
        if self.remove_outliers and len(pcd.points) > self.nb_neighbors:
            pcd, inliers = pcd.remove_statistical_outlier(
                nb_neighbors=self.nb_neighbors, std_ratio=self.std_ratio
            )
            logger.info("Points after statistical outlier removal: %d", len(pcd.points))

        filtered_pts = np.asarray(pcd.points, dtype=np.float32)
        filtered_cols = np.asarray(pcd.colors, dtype=np.float32)
        return filtered_pts, filtered_cols

    @staticmethod
    def _voxel_downsample_numpy(
        points: np.ndarray, colors: np.ndarray, voxel_size: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Pure NumPy voxel downsampling fallback."""
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        # Create a unique 1D hash or view for unique voxels
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        return points[unique_indices], colors[unique_indices]
