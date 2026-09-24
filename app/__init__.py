"""SIH Monocular Depth Estimation and 3D Point Cloud Reconstruction System.

This module provides end-to-end processing from RGB imagery to estimated depth maps
and 3D point cloud reconstructions.

Important Technical Note:
This application performs monocular depth estimation via deep learning and pinhole
camera back-projection. It is NOT hardware LiDAR, but an optical 3D reconstruction pipeline.
"""

__version__ = "0.1.0"
