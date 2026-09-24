"""Unit tests for depth estimation models and pipeline."""

import numpy as np
import pytest

from app.depth.model import DummyDepthModel, get_depth_model
from app.depth.inference import DepthEstimator


def test_dummy_depth_model_prediction():
    """Verify dummy depth model returns normalized disparity of correct shape."""
    model = DummyDepthModel()
    model.load_model()
    assert model.is_loaded

    h, w = 120, 160
    sample_img = np.random.randint(0, 256, (h, w, 3), dtype=np.uint8)
    disparity = model.predict(sample_img)

    assert disparity.shape == (h, w)
    assert disparity.dtype == np.float32
    assert float(np.min(disparity)) >= 0.0
    assert float(np.max(disparity)) <= 1.0


def test_depth_model_factory():
    """Verify factory returns appropriate models or raises for unknown types."""
    dummy = get_depth_model("dummy")
    assert isinstance(dummy, DummyDepthModel)

    with pytest.raises(ValueError):
        get_depth_model("non_existent_model_xyz")


def test_depth_estimator_scaling_and_outputs(tmp_path):
    """Verify depth estimator metric conversion and file output persistence."""
    model = DummyDepthModel()
    model.load_model()

    min_d = 0.5
    max_d = 8.0
    estimator = DepthEstimator(
        model=model,
        min_depth=min_d,
        max_depth=max_d,
        invert_depth=True,
    )

    h, w = 64, 64
    sample_img = np.zeros((h, w, 3), dtype=np.uint8)
    result = estimator.estimate(sample_img)

    assert result.depth_map.shape == (h, w)
    assert result.disparity_map.shape == (h, w)
    assert result.colored_depth.shape == (h, w, 3)
    assert result.raw_depth_16u.shape == (h, w)

    # Depth range must be within [min_d, max_d]
    assert np.min(result.depth_map) >= min_d - 1e-3
    assert np.max(result.depth_map) <= max_d + 1e-3

    # Test file saving
    saved = estimator.save_outputs(result, tmp_path, save_colormap=True, save_raw=True)
    assert "depth_colormap" in saved
    assert saved["depth_colormap"].exists()
    assert "depth_raw" in saved
    assert saved["depth_raw"].exists()
