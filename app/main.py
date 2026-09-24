"""Main CLI entrypoint for SIH Search-and-Rescue 3D Reconstruction.

Usage:
    python -m app.main --input data/input/image.jpg
    python -m app.main --input data/input/image.jpg --headless
    python -m app.main --config config/config.yaml

IMPORTANT TECHNICAL NOTE:
This system performs monocular depth estimation followed by 3D point-cloud reconstruction.
It is an optical AI reconstruction pipeline and does NOT claim to produce real hardware LiDAR data.
"""

import argparse
import logging
import os
from pathlib import Path
import sys
import time
from typing import Any, Dict

import cv2
import numpy as np
import yaml

from app.depth.model import get_depth_model
from app.depth.inference import DepthEstimator
from app.pointcloud.reconstruction import CameraIntrinsics, PointCloudReconstructor
from app.visualization.viewer import PointCloudVisualizer

BANNER = """
================================================================================
     SIH Search & Rescue: Monocular Depth & 3D Point-Cloud Reconstruction
================================================================================
 [SYSTEM NOTICE]
 Optical Monocular Depth Estimation + Pinhole Camera Back-Projection.
 This software reconstructs 3D geometry from 2D optical perspective cues.
 It is NOT physical hardware LiDAR (Light Detection and Ranging).
================================================================================
"""


def setup_logging(level_name: str = "INFO") -> None:
    """Configure system logging format."""
    numeric_level = getattr(logging, level_name.upper(), logging.INFO)
    logging.basicConfig(
        level=numeric_level,
        format="[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s",
        datefmt="%H:%M:%S",
    )


def load_config(config_path: Path) -> Dict[str, Any]:
    """Load YAML configuration file or return defaults."""
    if not config_path.exists():
        logging.warning("Config file %s not found. Using default parameters.", config_path)
        return {}
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
            return cfg or {}
    except Exception as e:
        logging.error("Failed to parse config file %s: %s. Using defaults.", config_path, e)
        return {}


def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="SIH Search-and-Rescue Monocular 3D Reconstruction Pipeline",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "--input",
        "-i",
        type=str,
        default="data/input/image.jpg",
        help="Path to input RGB image (jpg/png/bmp)",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default="data/output",
        help="Directory to save generated outputs (original.png, depth.png, pointcloud.ply)",
    )
    parser.add_argument(
        "--config",
        "-c",
        type=str,
        default="config/config.yaml",
        help="Path to pipeline configuration YAML file",
    )
    parser.add_argument(
        "--model-type",
        type=str,
        default=None,
        help="Depth model type ('midas_small' or 'dummy')",
    )
    parser.add_argument(
        "--model-path",
        type=str,
        default=None,
        help="Path to ONNX model weights",
    )
    parser.add_argument(
        "--fov",
        type=float,
        default=None,
        help="Override camera horizontal field-of-view in degrees",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=None,
        help="Override voxel downsampling size in meters (0 to disable)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=None,
        help="Run without opening interactive 3D visualizer window (for servers/Docker)",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity level",
    )

    return parser.parse_args()


def run_pipeline(
    input_path: Path,
    output_dir: Path,
    config: Dict[str, Any],
    headless: bool = False,
    model_type_override: str = None,
    model_path_override: str = None,
    fov_override: float = None,
    voxel_size_override: float = None,
) -> bool:
    """Execute the end-to-end depth estimation and point-cloud reconstruction pipeline."""
    logger = logging.getLogger("Pipeline")

    if not input_path.exists():
        logger.error("Input image file does not exist: %s", input_path.resolve())
        return False

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Read input RGB image
    logger.info("Loading input image: %s", input_path)
    bgr_image = cv2.imread(str(input_path))
    if bgr_image is None or bgr_image.size == 0:
        logger.error("Failed to read image at %s (unsupported or corrupted format).", input_path)
        return False

    rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    height, width = rgb_image.shape[:2]
    logger.info("Image loaded successfully: %d x %d pixels, 3 channels (RGB)", width, height)

    # 2. Extract configuration
    model_cfg = config.get("model", {})
    camera_cfg = config.get("camera", {})
    pcd_cfg = config.get("pointcloud", {})
    io_cfg = config.get("io", {})
    vis_cfg = config.get("visualization", {})

    model_type = model_type_override or model_cfg.get("type", "midas_small")
    model_path = model_path_override or model_cfg.get("path", "models/model-small.onnx")
    download_url = model_cfg.get("download_url")
    invert_depth = model_cfg.get("invert_depth", True)
    min_depth = float(model_cfg.get("min_depth", 0.1))
    max_depth = float(model_cfg.get("max_depth", 10.0))
    colormap = io_cfg.get("colormap", "inferno")

    if fov_override is not None:
        camera_cfg["fov_x_deg"] = fov_override
    if voxel_size_override is not None:
        pcd_cfg["voxel_size"] = voxel_size_override

    # 3. Initialize depth model & estimator
    logger.info("Initializing depth model: '%s'...", model_type)
    t0 = time.perf_counter()
    try:
        model = get_depth_model(
            model_type=model_type,
            model_path=model_path,
            download_url=download_url,
            auto_download=True,
        )
        model.load_model()
    except Exception as e:
        logger.error("Error loading depth model: %s", e)
        return False

    estimator = DepthEstimator(
        model=model,
        min_depth=min_depth,
        max_depth=max_depth,
        invert_depth=invert_depth,
        colormap=colormap,
    )
    t_model = time.perf_counter() - t0
    logger.info("Model loaded in %.2f ms", t_model * 1000)

    # 4. Perform depth estimation
    logger.info("Running monocular depth estimation on input image...")
    t0 = time.perf_counter()
    depth_result = estimator.estimate(rgb_image)
    t_depth = time.perf_counter() - t0
    logger.info(
        "Depth estimated in %.2f ms (Depth Range: %.2f m to %.2f m)",
        t_depth * 1000,
        float(np.min(depth_result.depth_map)),
        float(np.max(depth_result.depth_map)),
    )

    # 5. Build camera intrinsics & point-cloud reconstructor
    intrinsics = CameraIntrinsics.from_config(camera_cfg, width=width, height=height)
    logger.info(
        "Camera Intrinsics: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f (FOV_x=%.1f deg)",
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.cx,
        intrinsics.cy,
        camera_cfg.get("fov_x_deg", 62.2),
    )

    reconstructor = PointCloudReconstructor(
        intrinsics=intrinsics,
        voxel_size=pcd_cfg.get("voxel_size", 0.002),
        remove_outliers=pcd_cfg.get("remove_outliers", True),
        nb_neighbors=pcd_cfg.get("nb_neighbors", 20),
        std_ratio=pcd_cfg.get("std_ratio", 2.0),
        depth_trunc=pcd_cfg.get("depth_trunc", 10.0),
    )

    # 6. Reconstruct 3D Point Cloud
    logger.info("Reconstructing 3D point cloud via pinhole back-projection...")
    t0 = time.perf_counter()
    pcd_data = reconstructor.reconstruct(rgb_image, depth_result.depth_map)
    t_pcd = time.perf_counter() - t0
    logger.info(
        "Point cloud reconstructed in %.2f ms (Generated %d 3D points)",
        t_pcd * 1000,
        len(pcd_data),
    )

    # 7. Save outputs
    logger.info("Saving outputs to %s...", output_dir)
    # Output 1: original.png
    orig_output_path = output_dir / "original.png"
    cv2.imwrite(str(orig_output_path), bgr_image)
    logger.info("1/3 Saved original image: %s", orig_output_path)

    # Output 2: depth.png (and depth_raw.png)
    depth_paths = estimator.save_outputs(
        depth_result,
        output_dir=output_dir,
        save_colormap=io_cfg.get("save_depth_colormap", True),
        save_raw=io_cfg.get("save_depth_raw", True),
    )
    logger.info("2/3 Saved depth map: %s", depth_paths.get("depth_colormap"))

    # Output 3: pointcloud.ply
    ply_path = output_dir / "pointcloud.ply"
    pcd_data.save_ply(ply_path)
    logger.info("3/3 Saved 3D point cloud: %s", ply_path)

    # 8. Interactive 3D Visualization
    is_headless = headless
    if is_headless is None:
        is_headless = vis_cfg.get("headless", False)

    snapshot_path = output_dir / "pointcloud_preview.png"
    visualizer = PointCloudVisualizer(
        window_name="SIH Search & Rescue - 3D Reconstructed Point Cloud",
        point_size=vis_cfg.get("point_size", 2.5),
        background_color=tuple(vis_cfg.get("background_color", [0.05, 0.05, 0.05])),
        headless=is_headless,
    )

    if vis_cfg.get("enabled", True):
        visualizer.view(pcd_data, snapshot_path=snapshot_path)

    print("\n" + "=" * 80)
    print(" PIPELINE EXECUTION COMPLETED SUCCESSFULLY")
    print("=" * 80)
    print(f" * Input:       {input_path}")
    print(f" * Original:    {orig_output_path}")
    print(f" * Depth Map:   {output_dir / 'depth.png'}")
    print(f" * Point Cloud: {ply_path} ({len(pcd_data):,} vertices)")
    print("=" * 80 + "\n")

    return True


def main() -> None:
    """CLI Entry point."""
    print(BANNER)
    args = parse_arguments()
    setup_logging(args.log_level)

    config_path = Path(args.config)
    config = load_config(config_path)

    input_path = Path(args.input)
    output_dir = Path(args.output)

    headless_flag = args.headless
    if headless_flag is False and "--headless" not in sys.argv:
        headless_flag = None

    success = run_pipeline(
        input_path=input_path,
        output_dir=output_dir,
        config=config,
        headless=headless_flag,
        model_type_override=args.model_type,
        model_path_override=args.model_path,
        fov_override=args.fov,
        voxel_size_override=args.voxel_size,
    )

    if not success:
        sys.exit(1)


if __name__ == "__main__":
    main()
