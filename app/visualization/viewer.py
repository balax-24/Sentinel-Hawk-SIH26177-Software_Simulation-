"""Interactive 3D Point Cloud visualization module.

Decoupled from depth inference and reconstruction processing.
Provides interactive inspection of the reconstructed 3D point cloud with headless safeguards.
"""

import logging
import os
import sys
from pathlib import Path
from typing import Optional, Tuple, Union, List, Any

import numpy as np

logger = logging.getLogger(__name__)

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    o3d = None
    HAS_OPEN3D = False


class PointCloudVisualizer:
    """Manages 3D point cloud interactive rendering and headless inspection."""

    def __init__(
        self,
        window_name: str = "SIH Search & Rescue - 3D Point Cloud Viewer",
        width: int = 1280,
        height: int = 720,
        point_size: float = 2.5,
        background_color: Tuple[float, float, float] = (0.05, 0.05, 0.05),
        show_coordinate_frame: bool = True,
        headless: bool = False,
    ) -> None:
        """Initialize visualizer settings.

        Args:
            window_name: Title bar text for the GUI window.
            width: Window width in pixels.
            height: Window height in pixels.
            point_size: Rendered point diameter in pixels.
            background_color: RGB float tuple [0.0, 1.0].
            show_coordinate_frame: Whether to display XYZ coordinate axes.
            headless: If True, disables opening interactive graphical windows.
        """
        self.window_name = window_name
        self.width = width
        self.height = height
        self.point_size = point_size
        self.background_color = background_color
        self.show_coordinate_frame = show_coordinate_frame
        self.headless = headless or self._detect_headless_environment()

    @staticmethod
    def _detect_headless_environment() -> bool:
        """Detect whether the current runtime lacks a graphical display."""
        if os.environ.get("CI") or os.environ.get("HEADLESS", "").lower() in ("1", "true"):
            return True
        # On Linux/Docker, check if DISPLAY or WAYLAND_DISPLAY is set
        if sys.platform.startswith("linux"):
            if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
                return True
        return False

    def view(
        self,
        pcd_input: Union[Path, str, "PointCloudData", Any],
        snapshot_path: Optional[Union[Path, str]] = None,
    ) -> bool:
        """Display the 3D point cloud in an interactive Open3D window.

        Args:
            pcd_input: Can be a file path (.ply), PointCloudData instance,
                       or an Open3D PointCloud geometry object.
            snapshot_path: Optional path to save an offscreen rendered image.

        Returns:
            True if visualization or rendering succeeded, False otherwise.
        """
        if self.headless:
            logger.info(
                "Headless mode detected or requested. Skipping interactive 3D window. "
                "Point cloud file is ready for inspection in MeshLab, CloudCompare, or Open3D."
            )
            if snapshot_path and HAS_OPEN3D:
                try:
                    pcd = self._resolve_open3d_pcd(pcd_input)
                    if pcd and len(pcd.points) > 0:
                        vis = o3d.visualization.Visualizer()
                        vis.create_window(visible=False, width=self.width, height=self.height)
                        vis.add_geometry(pcd)
                        render_opt = vis.get_render_option()
                        if render_opt:
                            render_opt.background_color = np.asarray(self.background_color)
                            render_opt.point_size = float(self.point_size)
                        vis.poll_events()
                        vis.update_renderer()
                        vis.capture_screen_image(str(snapshot_path), do_render=True)
                        vis.destroy_window()
                        logger.info("Saved offscreen 3D snapshot to %s", snapshot_path)
                except Exception as e:
                    logger.debug("Offscreen snapshot failed: %s", e)
            return True

        if not HAS_OPEN3D:
            logger.warning(
                "Open3D is not installed or available. Cannot open interactive viewer. "
                "The generated .ply file can be opened directly with MeshLab or CloudCompare."
            )
            return False

        # Load / resolve Open3D geometry
        pcd = self._resolve_open3d_pcd(pcd_input)
        if pcd is None or len(pcd.points) == 0:
            logger.warning("Point cloud is empty or could not be loaded for visualization.")
            return False

        geometries = [pcd]
        if self.show_coordinate_frame:
            # Create a coordinate frame at the origin (X=red, Y=green, Z=blue)
            bbox = pcd.get_axis_aligned_bounding_box()
            extent = max(bbox.get_extent()) if not bbox.is_empty() else 1.0
            coord_size = max(0.2, float(extent * 0.15))
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=coord_size, origin=[0, 0, 0]
            )
            geometries.append(coord_frame)

        logger.info(
            "Launching interactive 3D viewer '%s' with %d points. (Press 'Q' or 'ESC' to exit)",
            self.window_name,
            len(pcd.points),
        )

        try:
            vis = o3d.visualization.Visualizer()
            vis.create_window(
                window_name=self.window_name,
                width=self.width,
                height=self.height,
                visible=True,
            )

            for geom in geometries:
                vis.add_geometry(geom)

            render_opt = vis.get_render_option()
            if render_opt:
                render_opt.background_color = np.asarray(self.background_color)
                render_opt.point_size = float(self.point_size)
                render_opt.show_coordinate_frame = False

            # Reset view to fit all points nicely
            vis.reset_view_point(True)

            if snapshot_path:
                vis.poll_events()
                vis.update_renderer()
                vis.capture_screen_image(str(snapshot_path), do_render=True)
                logger.info("Saved viewer snapshot to %s", snapshot_path)

            vis.run()
            vis.destroy_window()
            return True

        except Exception as e:
            logger.error("Failed to run interactive Open3D viewer: %s", e)
            logger.info("Point cloud file remains safely persisted on disk.")
            return False

    def _resolve_open3d_pcd(self, pcd_input: Any) -> Optional[Any]:
        """Convert various input types to an Open3D PointCloud."""
        if hasattr(pcd_input, "to_open3d"):
            return pcd_input.to_open3d()

        if isinstance(pcd_input, (str, Path)):
            ply_path = Path(pcd_input)
            if not ply_path.exists():
                logger.error("PLY file not found: %s", ply_path)
                return None
            return o3d.io.read_point_cloud(str(ply_path))

        # Check if already an Open3D PointCloud instance
        if HAS_OPEN3D and isinstance(pcd_input, o3d.geometry.PointCloud):
            return pcd_input

        logger.error("Unsupported point cloud input type: %s", type(pcd_input))
        return None
