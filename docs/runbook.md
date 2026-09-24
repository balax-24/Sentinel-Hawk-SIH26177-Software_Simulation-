# SIH26177 UAV Prototype: Operator Runbook & Verification Manual

## 1. Quickstart: Execution Modes

The SIH26177 search-and-rescue UAV platform supports three distinct runtime environments:

1. **Standalone Direct Python Mode (Windows / macOS / Raspberry Pi OS)**:
   - Zero ROS 2 or Gazebo installation required.
   - Uses built-in standalone fallback simulators and mock generators.
   - Perfect for testing algorithms, demonstration on developer laptops, and CI/CD pipelines.
2. **Native ROS 2 Humble (Ubuntu 22.04 LTS)**:
   - Native microsecond inter-process communication using CycloneDDS / FastDDS.
   - Full RViz2 and Gazebo Fortress physics simulation.
3. **Docker Containerized Mode**:
   - Pre-packaged reproducible environment (`docker/Dockerfile.ros2` + `docker/docker-compose.yml`).

---

## 2. Standalone Direct Python Mode (Developer Laptops / Edge Testing)

Activate your Python virtual environment:
```powershell
# Windows
.\.venv\Scripts\Activate.ps1

# Linux / macOS
source .venv/bin/activate
```

Every module can be demonstrated independently:

### 2.1 Simulation & Telemetry
```powershell
python simulation/simulation/mock_sim_node.py
```
*Outputs: Generates simulated 3D disaster terrain point clouds, drone kinematics, GPS coordinates, and IMU measurements at 10 Hz.*

### 2.2 LiDAR Ground Filtering & Obstacle Clustering
```powershell
python perception/sih_lidar/sih_lidar/lidar_processor_node.py
```
*Outputs: Ingests 3D points, strips ground returns (Z < 0.2m), performs Euclidean spatial clustering, and outputs 3D obstacle bounding boxes.*

### 2.3 Optical Monocular Depth Estimation (AI)
```powershell
python perception/sih_perception/sih_perception/optical_depth_node.py
```
*Outputs: Runs MiDaS v2.1 Small ONNX model on CPU (with OpenCV fallback) on RGB images, yielding metric depth maps.*

### 2.4 Casualty Detection & Geotagging
```powershell
python perception/sih_perception/sih_perception/survivor_detector_node.py
```
*Outputs: Detects high-visibility orange/red thermal signatures, projects pixel centroids through depth and camera intrinsics into 3D camera coordinates, and transforms them into WGS84 GPS coordinates.*

### 2.5 3D Environment Map Reconstruction
```powershell
python mapping/sih_mapping/mapping_node.py
```
*Outputs: Accumulates filtered 3D LiDAR obstacle points into a global point cloud octree/voxel grid and periodically flushes to `data/output/map_disaster.ply`.*

### 2.6 Lawnmower Search Path Planner
```powershell
python navigation/sih_navigation/planner_node.py
```
*Outputs: Computes optimal boustrophedon (lawnmower) search tracks over the specified disaster bounding box with user-defined lane spacing and survey altitude.*

### 2.7 Obstacle Avoidance (Artificial Potential Field)
```powershell
python navigation/sih_navigation/obstacle_avoidance_node.py
```
*Outputs: Computes attractive target vectors towards waypoints and repulsive hyperbolic potential fields away from detected 3D obstacles, outputting safe 3D velocity vectors.*

### 2.8 Mission State Machine Coordinator
```powershell
python mission/sih_mission/mission_manager_node.py
```
*Outputs: Drives the autonomous operational phases: `PREFLIGHT` -> `TAKEOFF` -> `SURVEY` -> `PAYLOAD_DROP` -> `RTL` (Return to Launch).*

### 2.9 Emergency Supply Payload Dispenser
```powershell
python mission/sih_mission/payload_delivery_node.py
```
*Outputs: Manages dispenser bay arming, release interlocks, drop countdown, and actuation confirmation.*

### 2.10 Multi-UAV Swarm Partitioning
```powershell
python swarm/sih_swarm/swarm_manager_node.py
```
*Outputs: Decomposes a large disaster search perimeter into equal, non-overlapping geometric sectors across N active UAV drones.*

### 2.11 Tactical Situational Awareness Dashboard
```powershell
python dashboard/sih_dashboard/dashboard_node.py
```
*Outputs: Live console telemetry HUD showing drone coordinates, battery %, flight mode, detected survivors, and payload status.*

---

## 3. ROS 2 Native / Docker Deployment Commands

In an Ubuntu 22.04 LTS or Docker environment with ROS 2 Humble installed:

### 3.1 Build the Colcon Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

### 3.2 Launch Individual Modules Independently

#### Simulation Subsystem:
```bash
# Launch Gazebo disaster world with collapsed structures
ros2 launch sih_simulation disaster_world.launch.py

# Or launch the complete simulation stack (Gazebo + UAV spawn)
ros2 launch sih_simulation simulation.launch.py
```

#### Perception Subsystem:
```bash
# Launch 3D LiDAR processor
ros2 launch sih_lidar lidar.launch.py

# Launch optical monocular depth estimator
ros2 launch sih_perception optical_depth.launch.py

# Launch survivor detector and geotagger
ros2 launch sih_perception survivor_detection.launch.py
```

#### Mapping Subsystem:
```bash
# Launch 3D global point cloud accumulator
ros2 launch sih_mapping mapping.launch.py
```

#### Navigation Subsystem:
```bash
# Launch coverage path planner
ros2 launch sih_navigation planner.launch.py

# Launch reactive obstacle avoidance
ros2 launch sih_navigation obstacle_avoidance.launch.py
```

#### Mission & Swarm Subsystem:
```bash
# Launch mission coordinator
ros2 launch sih_mission mission_manager.launch.py

# Launch payload delivery bay controller
ros2 launch sih_mission payload_delivery.launch.py

# Launch multi-UAV swarm coordinator
ros2 launch sih_swarm swarm.launch.py
```

#### Ground Control Dashboard:
```bash
ros2 launch sih_dashboard dashboard.launch.py
```

---

## 4. Master End-to-End Mission Launch

To launch all 11 modular subsystems simultaneously in one integrated mission:
```bash
ros2 launch sih_system full_mission.launch.py
```

### With Docker Compose:
```bash
cd docker
docker-compose up --build
```

---

## 5. Verification & Live Inspection Recipes

### Check Active Topics:
```bash
ros2 topic list
```

### Inspect Raw LiDAR Point Cloud Stream:
```bash
ros2 topic hz /lidar/points
ros2 topic echo /lidar/filtered_points --once
```

### Verify Detected Obstacles:
```bash
ros2 topic echo /obstacles
```

### Verify Detected Survivors & GPS Coordinates:
```bash
ros2 topic echo /survivors
```

### Monitor Mission State Transitions:
```bash
ros2 topic echo /mission/state
```

### Trigger Emergency Payload Drop Manually:
```bash
ros2 service call /mission/trigger_payload interfaces/srv/TriggerPayload "{bay_id: 1, target_survivor_id: 'casualty-01'}"
```

### Inspect Swarm Sector Allocations:
```bash
ros2 topic echo /swarm/state
```

---

## 6. Automated Verification Test Suite

To run all 20 automated unit and integration tests across the entire codebase:
```powershell
# Windows
.\.venv\Scripts\pytest -v

# Linux
pytest -v
```

All 20 tests verify:
1. `tests/test_depth.py`: Depth model inference, preprocessing, metric scaling.
2. `tests/test_reconstruction.py`: Pinhole camera backprojection, voxel filtering, PLY writer.
3. `tests/test_lidar.py`: 3D LiDAR ground extraction, range clipping, Euclidean obstacle clustering.
4. `tests/test_simulation.py`: UAV kinematics, WGS84 GPS projections, IMU readings, Gazebo scene point generation.
5. `tests/test_navigation.py`: Lawnmower search grid coverage, potential field repulsion vectors.
6. `tests/test_mission.py`: Mission state machine phase transitions, payload dispenser actuation logic.
7. `tests/test_swarm.py`: Multi-UAV spatial area partitioning and sector containment.
8. `tests/test_pipeline.py`: End-to-end optical 3D reconstruction pipeline.
