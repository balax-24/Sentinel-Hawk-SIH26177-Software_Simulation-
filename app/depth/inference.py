"""Depth inference pipeline and depth map post-processing.

Bridges the raw depth estimation model output with downstream consumption,
converting disparity to metric/pseudo-depth and generating visualizations and raw maps.
"""

from dataclasses import dataclass
import logging
from pathlib import Path
from typing import Dict, Tuple, Optional

import cv2
import numpy as np

from app.depth.model import BaseDepthModel

logger = logging.getLogger(__name__)


@dataclass
class DepthEstimationResult:
    """Container for depth estimation outputs."""

    # Pseudo-metric depth in meters (Z coordinate in camera frame, H x W float32)
    depth_map: np.ndarray

    # Normalized disparity [0.0, 1.0] (H x W float32, higher = closer)
    disparity_map: np.ndarray

    # Visual false-color representation (H x W x 3 uint8 BGR for cv2 saving)
    colored_depth: np.ndarray

    # Lossless 16-bit depth in millimeters (H x W uint16)
    raw_depth_16u: np.ndarray

    # Original input image dimensions (height, width)
    original_shape: Tuple[int, int]


class DepthEstimator:
    """High-level depth estimation pipeline manager.

    Processes single images or continuous video/camera frames and produces
    both metric-scaled depth representations and visual false-color maps.
    """

    COLORMAP_DICT = {
        "inferno": cv2.COLORMAP_INFERNO,
        "turbo": cv2.COLORMAP_TURBO,
        "viridis": cv2.COLORMAP_VIRIDIS,
        "magma": cv2.COLORMAP_MAGMA,
        "plasma": cv2.COLORMAP_PLASMA,
        "jet": cv2.COLORMAP_JET,
    }

    def __init__(
        self,
        model: BaseDepthModel,
        min_depth: float = 0.1,
        max_depth: float = 10.0,
        invert_depth: bool = True,
        colormap: str = "inferno",
    ) -> None:
        """Initialize the depth estimator.

        Args:
            model: Loaded depth model adhering to BaseDepthModel.
            min_depth: Minimum estimated depth in meters.
            max_depth: Maximum estimated depth in meters.
            invert_depth: If True, converts disparity (high=near) to depth (high=far).
            colormap: Name of OpenCV colormap for visualization.
        """
        self.model = model
        self.min_depth = max(1e-3, float(min_depth))
        self.max_depth = max(self.min_depth + 1e-2, float(max_depth))
        self.invert_depth = invert_depth
        self.cv2_colormap = self.COLORMAP_DICT.get(
            colormap.lower(), cv2.COLORMAP_INFERNO
        )

    def estimate(self, rgb_image: np.ndarray) -> DepthEstimationResult:
        """Run depth inference on an RGB image.

        Args:
            rgb_image: Input image array of shape (H, W, 3) in RGB order.

        Returns:
            DepthEstimationResult containing depth map, disparity, and visualizations.
        """
        if rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
            raise ValueError(
                f"Input image must have shape (H, W, 3), got shape {rgb_image.shape}"
            )

        orig_h, orig_w = rgb_image.shape[:2]

        # Predict normalized disparity [0.0, 1.0] (higher values = closer objects)
        norm_disparity = self.model.predict(rgb_image)

        # Ensure valid range
        norm_disparity = np.clip(norm_disparity, 0.0, 1.0).astype(np.float32)

        # Convert disparity to pseudo-metric depth (Z coordinate)
        # Monocular cameras exhibit scale ambiguity. We map disparity to depth
        # using the reciprocal optical relation: depth ~ 1 / disparity
        if self.invert_depth:
            # Physical perspective inverse: Z = min_depth * max_depth / (max_depth - disp * (max_depth - min_depth))
            # When disp = 1.0 (nearest), Z = min_depth
            # When disp = 0.0 (farthest), Z = max_depth
            denom = self.max_depth - norm_disparity * (self.max_depth - self.min_depth)
            denom = np.maximum(denom, 1e-4)
            depth_map = (self.min_depth * self.max_depth) / denom
        else:
            # Direct linear scaling
            depth_map = self.min_depth + norm_disparity * (self.max_depth - self.min_depth)

        depth_map = np.clip(depth_map, self.min_depth, self.max_depth).astype(np.float32)

        # Generate 16-bit unsigned depth in millimeters for lossless persistence
        raw_depth_16u = np.clip(depth_map * 1000.0, 0, 65535).astype(np.uint16)

        # Generate false-color visualization for human inspection
        # For visualization, we colormap disparity so that closer objects stand out warmly
        disp_uint8 = (norm_disparity * 255.0).astype(np.uint8)
        colored_depth = cv2.applyColorMap(disp_uint8, self.cv2_colormap)

        return DepthEstimationResult(
            depth_map=depth_map,
            disparity_map=norm_disparity,
            colored_depth=colored_depth,
            raw_depth_16u=raw_depth_16u,
            original_shape=(orig_h, orig_w),
        )

    def estimate_frame(
        self, frame: np.ndarray, is_bgr: bool = True
    ) -> DepthEstimationResult:
        """Process a video frame or camera stream buffer.

        Convenience wrapper for future Raspberry Pi camera or live RTSP streams.
        """
        if is_bgr:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            rgb_frame = frame
        return self.estimate(rgb_frame)

    def save_outputs(
        self,
        result: DepthEstimationResult,
        output_dir: Path,
        save_colormap: bool = True,
        save_raw: bool = True,
    ) -> Dict[str, Path]:
        """Save depth outputs to disk.

        Args:
            result: Result from estimate().
            output_dir: Target output directory path.
            save_colormap: Whether to save colormapped 8-bit PNG (depth.png).
            save_raw: Whether to save lossless 16-bit depth (depth_raw.png).

        Returns:
            Dictionary mapping output keys to saved file paths.
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        saved_paths: Dict[str, Path] = {}

        if save_colormap:
            depth_png_path = output_dir / "depth.png"
            success = cv2.imwrite(str(depth_png_path), result.colored_depth)
            if not success:
                raise IOError(f"Failed to write depth visualization to {depth_png_path}")
            saved_paths["depth_colormap"] = depth_png_path
            logger.info("Saved colormapped depth visualization to %s", depth_png_path)

        if save_raw:
            depth_raw_path = output_dir / "depth_raw.png"
            success = cv2.imwrite(str(depth_raw_path), result.raw_depth_16u)
            if not success:
                raise IOError(f"Failed to write 16-bit depth map to {depth_raw_path}")
            saved_paths["depth_raw"] = depth_raw_path
            logger.info("Saved 16-bit raw millimeter depth map to %s", depth_raw_path)

        return saved_paths
