# SIH Perception Package (`sih_perception`)

Provides AI visual perception for the SIH26177 UAV, including:
1. **Optical Monocular Depth Estimation (`optical_depth_node.py`)**: Runs lightweight deep learning on CPU (preserving and integrating the standalone `app/depth` pipeline) to produce depth maps and point clouds alongside simulated/physical LiDAR.
2. **Survivor Detection & Geotagging (`survivor_detector_node.py`)**: Identifies casualties/survivors in disaster rubble and calculates their WGS84 GPS latitude/longitude.

## Launch Commands

### 1. Launch Optical Monocular Depth Node
```bash
ros2 launch sih_perception optical_depth.launch.py
```

### 2. Launch Survivor Detection Node
```bash
ros2 launch sih_perception survivor_detection.launch.py
```

### 3. Standalone Demonstrations (No ROS 2 needed)
```bash
python -m perception.sih_perception.sih_perception.optical_depth_node
python -m perception.sih_perception.sih_perception.survivor_detector_node
```
