"""Integration tests for end-to-end pipeline execution."""

from pathlib import Path
import cv2
import numpy as np

from app.main import run_pipeline


def test_end_to_end_pipeline(tmp_path):
    """Verify that run_pipeline generates original.png, depth.png, and pointcloud.ply."""
    input_img_path = tmp_path / "sample_input.jpg"
    output_dir = tmp_path / "output"

    # Create synthetic test image (e.g. gradient rescue scene)
    img = np.zeros((120, 160, 3), dtype=np.uint8)
    img[:60, :, 0] = 200  # Blue sky
    img[60:, :, 1] = 180  # Green ground
    img[40:80, 60:100, 2] = 250  # Red object/rubble
    cv2.imwrite(str(input_img_path), img)

    config = {
        "model": {
            "type": "dummy",
            "min_depth": 0.5,
            "max_depth": 10.0,
            "invert_depth": True,
        },
        "camera": {
            "fov_x_deg": 65.0,
        },
        "pointcloud": {
            "voxel_size": 0.05,
            "remove_outliers": False,
        },
        "io": {
            "save_original": True,
            "save_depth_colormap": True,
            "save_depth_raw": True,
            "save_pointcloud": True,
            "colormap": "inferno",
        },
        "visualization": {
            "enabled": False,
            "headless": True,
        },
    }

    success = run_pipeline(
        input_path=input_img_path,
        output_dir=output_dir,
        config=config,
        headless=True,
        model_type_override="dummy",
    )

    assert success is True

    # Check that required output files exist
    orig_file = output_dir / "original.png"
    depth_file = output_dir / "depth.png"
    ply_file = output_dir / "pointcloud.ply"

    assert orig_file.exists(), "original.png was not generated"
    assert depth_file.exists(), "depth.png was not generated"
    assert ply_file.exists(), "pointcloud.ply was not generated"

    assert orig_file.stat().st_size > 0
    assert depth_file.stat().st_size > 0
    assert ply_file.stat().st_size > 0
