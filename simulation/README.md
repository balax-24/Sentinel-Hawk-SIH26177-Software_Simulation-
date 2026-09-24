# SIH Simulation Package (`sih_simulation`)

Simulates a disaster search-and-rescue environment in Gazebo with an instrumented quadrotor UAV.

## Capabilities
1. **Gazebo Disaster World (`disaster_world.world`)**: Collapsed multi-story concrete slabs, inclined roofs, masonry debris piles, damaged perimeter walls, and simulated human casualty mannequins.
2. **UAV Platform (`model.sdf`)**: Realistic quadrotor airframe equipped with:
   * **3D LiDAR**: 16 channels, 512 horizontal samples, publishing `/lidar/points` (`sensor_msgs/PointCloud2`).
   * **RGB Camera**: 1280x720 @ 30 FPS, publishing `/camera/image` and `/camera/camera_info`.
   * **Depth Camera**: 640x480 @ 15 FPS, publishing `/camera/depth`.
   * **GPS / GNSS**: WGS84 coordinates publishing `/drone/gps/fix` (`sensor_msgs/NavSatFix`).
   * **6-Axis IMU**: Accelerometer + Gyroscope publishing `/drone/imu/data` (`sensor_msgs/Imu`).
   * **Flight Controller Kinematics**: Publishes `/drone/pose` and `/drone/velocity`.

## Launch Commands

### 1. Launch Disaster World
```bash
ros2 launch sih_simulation disaster_world.launch.py
```
*(Headless mode for Docker/servers: `ros2 launch sih_simulation disaster_world.launch.py gui:=false`)*

### 2. Spawn UAV into World
```bash
ros2 launch sih_simulation spawn_uav.launch.py x:=0.0 y:=0.0 z:=0.3
```

### 3. Full Simulation (World + UAV)
```bash
ros2 launch sih_simulation simulation.launch.py
```

### 4. Standalone Lightweight Runner (No GPU / Windows / Raspberry Pi)
```bash
python -m simulation.simulation.mock_sim_node
```

## ROS 2 Topics
* **Published**:
  * `/lidar/points` (`sensor_msgs/msg/PointCloud2`)
  * `/camera/image` (`sensor_msgs/msg/Image`)
  * `/camera/depth` (`sensor_msgs/msg/Image`)
  * `/drone/gps/fix` (`sensor_msgs/msg/NavSatFix`)
  * `/drone/imu/data` (`sensor_msgs/msg/Imu`)
  * `/drone/pose` (`geometry_msgs/msg/PoseStamped`)
  * `/drone/velocity` (`geometry_msgs/msg/TwistStamped`)
* **Subscribed**:
  * `/drone/cmd_vel` (`geometry_msgs/msg/Twist`)
