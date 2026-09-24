"""Depth estimation subpackage.

Provides model interfaces, ONNX model inference, and depth map processing.
"""

from app.depth.model import (
    BaseDepthModel,
    MiDaSSmallONNX,
    DummyDepthModel,
    get_depth_model,
)
from app.depth.inference import (
    DepthEstimator,
    DepthEstimationResult,
)

__all__ = [
    "BaseDepthModel",
    "MiDaSSmallONNX",
    "DummyDepthModel",
    "get_depth_model",
    "DepthEstimator",
    "DepthEstimationResult",
]
