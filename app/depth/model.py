"""Monocular depth estimation model interfaces and implementations.

IMPORTANT TECHNICAL DISTINCTION:
This module performs optical monocular depth estimation using deep learning.
It infers relative or pseudo-metric depth from single 2D RGB perspective cues.
It is NOT a LiDAR hardware sensor, and outputs should be treated as optical approximations.
"""

from abc import ABC, abstractmethod
import logging
import os
from pathlib import Path
import urllib.request
from typing import Dict, Any, Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class BaseDepthModel(ABC):
    """Abstract Base Class for monocular depth estimation models.

    Allows plugging in different backends (MiDaS, Depth Anything, FastDepth, etc.)
    without altering downstream point-cloud reconstruction or visualization modules.
    """

    def __init__(self, model_name: str) -> None:
        self.model_name = model_name
        self.is_loaded = False

    @abstractmethod
    def load_model(self, model_path: Optional[str] = None) -> None:
        """Load model weights and initialize inference session."""
        pass

    @abstractmethod
    def predict(self, rgb_image: np.ndarray) -> np.ndarray:
        """Predict relative or pseudo-metric depth from an RGB image.

        Args:
            rgb_image: Input RGB image as numpy array of shape (H, W, 3), dtype uint8 or float32.

        Returns:
            Depth or disparity map as float32 numpy array of shape (H, W).
        """
        pass

    def get_info(self) -> Dict[str, Any]:
        """Return diagnostic and configuration metadata for the model."""
        return {
            "model_name": self.model_name,
            "is_loaded": self.is_loaded,
        }


class MiDaSSmallONNX(BaseDepthModel):
    """Lightweight MiDaS v2.1 Small model running via ONNX Runtime.

    Optimized for CPU inference on edge devices (e.g. Raspberry Pi 4, low-power drones)
    with a small memory footprint (~45 MB model file, 256x256 input tensor).
    """

    DEFAULT_DOWNLOAD_URL = (
        "https://github.com/isl-org/MiDaS/releases/download/v2_1/model-small.onnx"
    )

    def __init__(
        self,
        model_path: str = "models/model-small.onnx",
        download_url: Optional[str] = None,
        input_size: int = 256,
        auto_download: bool = True,
    ) -> None:
        super().__init__(model_name="MiDaS_v2.1_Small_ONNX")
        self.model_path = Path(model_path)
        self.download_url = download_url or self.DEFAULT_DOWNLOAD_URL
        self.input_size = input_size
        self.auto_download = auto_download
        self.session = None
        self.input_name = None
        self.output_name = None

    def load_model(self, model_path: Optional[str] = None) -> None:
        """Initialize the ONNX Runtime inference session with CPU fallback."""
        if model_path:
            self.model_path = Path(model_path)

        if not self.model_path.exists():
            if self.auto_download:
                logger.info(
                    "Model file not found at %s. Attempting download from %s...",
                    self.model_path,
                    self.download_url,
                )
                self._download_weights()
            else:
                raise FileNotFoundError(
                    f"Model file not found at {self.model_path} and auto_download is disabled."
                )

        try:
            import onnxruntime as ort
        except ImportError as e:
            raise ImportError(
                "onnxruntime is required for MiDaSSmallONNX. "
                "Install it using `pip install onnxruntime`."
            ) from e

        logger.info("Initializing ONNX Runtime session for %s...", self.model_path)
        # Prioritize CPU execution for deterministic edge/embedded compatibility
        providers = ["CPUExecutionProvider"]
        available_providers = ort.get_available_providers()
        if "CUDAExecutionProvider" in available_providers:
            providers.insert(0, "CUDAExecutionProvider")

        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        sess_options.intra_op_num_threads = max(1, os.cpu_count() or 1)

        self.session = ort.InferenceSession(
            str(self.model_path), sess_options=sess_options, providers=providers
        )
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.is_loaded = True
        logger.info(
            "Model loaded successfully with execution provider: %s",
            self.session.get_providers(),
        )

    def _download_weights(self) -> None:
        """Download model weights if missing."""
        self.model_path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self.model_path.with_suffix(".tmp")
        try:
            logger.info("Downloading pretrained weights to %s...", self.model_path)
            urllib.request.urlretrieve(self.download_url, temp_path)
            temp_path.replace(self.model_path)
            logger.info("Model download complete (%d bytes).", self.model_path.stat().st_size)
        except Exception as e:
            if temp_path.exists():
                temp_path.unlink()
            raise RuntimeError(
                f"Failed to download model weights from {self.download_url}: {e}"
            ) from e

    def _preprocess(self, rgb_image: np.ndarray) -> np.ndarray:
        """Preprocess RGB image to normalized CHW float32 tensor."""
        h, w = rgb_image.shape[:2]
        # Resize to input_size x input_size
        resized = cv2.resize(
            rgb_image, (self.input_size, self.input_size), interpolation=cv2.INTER_CUBIC
        )

        # Normalize to [0, 1]
        img_float = resized.astype(np.float32) / 255.0

        # MiDaS ImageNet normalization
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img_norm = (img_float - mean) / std

        # Transpose HWC -> CHW and add batch dimension -> NCHW
        tensor = np.transpose(img_norm, (2, 0, 1))
        tensor = np.expand_dims(tensor, axis=0)
        return tensor

    def predict(self, rgb_image: np.ndarray) -> np.ndarray:
        """Run monocular depth estimation on the input image.

        Args:
            rgb_image: Input image in RGB format (H, W, 3), uint8.

        Returns:
            Normalized disparity map (H, W) where higher values indicate closer proximity.
        """
        if not self.is_loaded or self.session is None:
            self.load_model()

        if rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
            raise ValueError(
                f"Expected RGB image of shape (H, W, 3), got shape {rgb_image.shape}"
            )

        orig_h, orig_w = rgb_image.shape[:2]
        input_tensor = self._preprocess(rgb_image)

        # Run ONNX inference
        outputs = self.session.run([self.output_name], {self.input_name: input_tensor})
        raw_output = outputs[0]  # Shape typically (1, 256, 256) or (1, 1, 256, 256)
        disparity_small = np.squeeze(raw_output)

        # Resize back to original image dimensions
        disparity = cv2.resize(
            disparity_small, (orig_w, orig_h), interpolation=cv2.INTER_CUBIC
        )

        # Normalize disparity map to [0, 1]
        d_min = float(np.min(disparity))
        d_max = float(np.max(disparity))
        if d_max > d_min:
            norm_disparity = (disparity - d_min) / (d_max - d_min)
        else:
            norm_disparity = np.zeros_like(disparity, dtype=np.float32)

        return norm_disparity.astype(np.float32)


class DummyDepthModel(BaseDepthModel):
    """Deterministic synthetic depth model for testing and offline development.

    Produces a smooth synthetic depth ramp with a foreground spherical feature,
    guaranteeing valid 3D points without requiring network access or external weights.
    """

    def __init__(self, model_name: str = "DummyDepthModel") -> None:
        super().__init__(model_name=model_name)

    def load_model(self, model_path: Optional[str] = None) -> None:
        self.is_loaded = True
        logger.info("DummyDepthModel initialized (no external weights required).")

    def predict(self, rgb_image: np.ndarray) -> np.ndarray:
        orig_h, orig_w = rgb_image.shape[:2]
        # Create a vertical depth gradient (sky far away, ground closer)
        y = np.linspace(0.1, 0.9, orig_h, dtype=np.float32)[:, None]
        base_disparity = np.repeat(y, orig_w, axis=1)

        # Add a central object feature
        cy, cx = orig_h // 2, orig_w // 2
        radius = min(orig_h, orig_w) // 4
        y_coords, x_coords = np.ogrid[:orig_h, :orig_w]
        dist_from_center = np.sqrt((x_coords - cx) ** 2 + (y_coords - cy) ** 2)
        sphere_mask = dist_from_center < radius
        sphere_bump = np.zeros_like(base_disparity)
        sphere_bump[sphere_mask] = 0.4 * (1.0 - (dist_from_center[sphere_mask] / radius))

        disparity = base_disparity + sphere_bump
        disparity = np.clip(disparity, 0.0, 1.0)
        return disparity.astype(np.float32)


def get_depth_model(
    model_type: str = "midas_small",
    model_path: str = "models/model-small.onnx",
    download_url: Optional[str] = None,
    auto_download: bool = True,
) -> BaseDepthModel:
    """Factory function to instantiate depth estimation models.

    Args:
        model_type: Type identifier ('midas_small', 'dummy').
        model_path: File system path to the model weights.
        download_url: Optional URL to fetch weights if missing.
        auto_download: Whether to fetch model if missing.

    Returns:
        Instantiated BaseDepthModel subclass.
    """
    model_type_lower = model_type.lower()
    if model_type_lower in ("midas_small", "midas", "onnx"):
        return MiDaSSmallONNX(
            model_path=model_path,
            download_url=download_url,
            auto_download=auto_download,
        )
    elif model_type_lower in ("dummy", "mock", "test"):
        return DummyDepthModel()
    else:
        raise ValueError(
            f"Unsupported depth model type '{model_type}'. "
            f"Supported options: 'midas_small', 'dummy'."
        )
