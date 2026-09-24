# Hardware Abstraction Layer (HAL) & Physical Deployment Guide

## 1. Overview and Design Philosophy

The SIH26177 UAV software stack is built with strict **Hardware Decoupling**. None of the algorithmic nodes (perception, mapping, navigation, mission management) know or care whether they are receiving sensor data from Gazebo Fortress, a lightweight mock simulator, or real physical sensors mounted on a carbon-fiber airframe.

The core principle:
> **Sensor Drivers publish standard ROS 2 message types to designated standard topics.**
> **Autopilot bridges translate standard `/drone/cmd_vel` velocity commands into actuator mixer outputs.**

```
+-----------------------------------------------------------------------------------+
|                            SIH26177 Core Software Stack                           |
|  (sih_lidar, sih_perception, sih_mapping, sih_navigation, sih_mission, sih_swarm)|
+-----------------------------------------------------------------------------------+
                                      ▲  |
             Standard ROS 2 Topics    |  |  Standard ROS 2 Commands
             (/lidar/points, etc.)    |  |  (/drone/cmd_vel, etc.)
                                      |  ▼
+-----------------------------------------------------------------------------------+
|                        Hardware Abstraction Layer (HAL)                           |
+-----------------------------------------+-----------------------------------------+
|          SIMULATION HAL                 |           PHYSICAL HARDWARE HAL         |
|  - Gazebo Fortress Sensors Plugins      |  - velodyne_driver / livox_ros_driver2  |
|  - disaster_world.world                 |  - v4l2_camera / libcamera ROS node    |
|  - mock_sim_node.py (pure Python)       |  - Micro-XRCE-DDS Agent (Pixhawk 6C)   |
|  - uav_controller.py                    |  - RPi GPIO / PWM Servo Dispenser      |
+-----------------------------------------+-----------------------------------------+
```

---

## 2. Sensor Mapping: Simulation vs. Physical Hardware

Every simulated sensor topic in `sih_simulation` maps 1-to-1 with off-the-shelf physical UAV hardware:

| Modality | Simulated Driver / Source | Physical Hardware Target | Physical ROS 2 Driver Package | Output Topic & Type |
| :--- | :--- | :--- | :--- | :--- |
| **3D LiDAR** | Gazebo Ray Plugin (`model.sdf`) | Velodyne VLP-16 Puck / Livox Mid-360 | `velodyne_driver` or `livox_ros_driver2` | `/lidar/points`<br>`sensor_msgs/msg/PointCloud2` |
| **RGB Camera** | Gazebo Camera Plugin (`model.sdf`) | Raspberry Pi Camera Module v2/v3 / Sony IMX477 | `v4l2_camera` or `camera_ros` | `/camera/image`<br>`sensor_msgs/msg/Image` |
| **Depth Camera** | Gazebo Depth Camera Plugin | Intel RealSense D435i **OR** `sih_optical_depth_node` | `realsense2_camera` or internal MiDaS node | `/camera/depth`<br>`sensor_msgs/msg/Image` |
| **Flight Pose & Odom**| Gazebo Odometry Plugin | Pixhawk 6C (PX4 EKF2) | `Micro-XRCE-DDS-Agent` / `px4_ros_com` | `/drone/pose`<br>`geometry_msgs/msg/PoseStamped` |
| **Body Velocity** | Gazebo Odometry Plugin | Pixhawk 6C (PX4 EKF2) | `Micro-XRCE-DDS-Agent` / `px4_ros_com` | `/drone/velocity`<br>`geometry_msgs/msg/TwistStamped` |
| **GNSS (GPS)** | Gazebo NavSat Plugin | U-Blox NEO-M8N / ZED-F9P RTK | PX4 bridged GNSS or `ublox_dgnss` | `/drone/gps/fix`<br>`sensor_msgs/msg/NavSatFix` |
| **IMU** | Gazebo IMU Plugin | InvenSense ICM-42688-P (Pixhawk 6C) | `Micro-XRCE-DDS-Agent` | `/drone/imu/data`<br>`sensor_msgs/msg/Imu` |
| **Velocity Control** | Gazebo Twist Controller | Pixhawk 6C Offboard Mode | `px4_ros_com` (Offboard velocity) | `/drone/cmd_vel`<br>`geometry_msgs/msg/Twist` |
| **Payload Release** | Mock Payload Node / Sim | 5V MG996R PWM Servo Dispenser | `rpi_gpio_controller` or Pixhawk AUX PWM | `/mission/trigger_payload`<br>`interfaces/srv/TriggerPayload` |

---

## 3. Physical Hardware Wiring & Interconnects

### Companion Computer: Raspberry Pi 4 Model B (4 GB RAM)
- **OS**: Ubuntu 22.04 LTS 64-bit Server + ROS 2 Humble.
- **Power**: 5V 3.5A clean DC supplied via a dedicated step-down BEC (Battery Eliminator Circuit) connected to the 4S/6S LiPo main battery.
- **Cooling**: Active aluminum heat-sink casing with a 5V fan to prevent thermal throttling under CPU-intensive computer vision.

### Flight Controller: Holybro Pixhawk 6C / Pixhawk 4
- **Firmware**: PX4 Autopilot v1.14+.
- **Communication Protocol**: High-speed UART (`/dev/ttyAMA0` or USB `TELEM2`) operating at 921,600 baud.
- **Bridge Software**: Micro-XRCE-DDS Client runs on Pixhawk NuttX, while Micro-XRCE-DDS Agent runs as a systemd service on the Raspberry Pi companion computer.

### LiDAR Interconnect
- **Velodyne VLP-16 Puck**:
  - Connected via 100BASE-T Ethernet to the Raspberry Pi RJ45 port.
  - Pi static IP: `192.168.1.100`, LiDAR static IP: `192.168.1.201`.
  - Data stream: 300,000 pts/sec over UDP port 2368.
- **Livox Mid-360 Alternative**:
  - Ethernet connected; uses Livox SDK2 and `livox_ros_driver2` publishing standard `sensor_msgs/PointCloud2`.

### RGB Camera Interconnect
- **CSI-2 Ribbon Cable**: Raspberry Pi Camera Module 3 (Sony IMX708 with autofocus) connected directly to the Pi Camera port.
- Driver: `libcamera` with hardware-accelerated ISP pipeline, streaming 640x480 RGB frames at 15-30 FPS with negligible CPU footprint.

### Emergency Supply Payload Dispenser
- **Actuator**: High-torque metal-gear digital servo (MG996R / MG90S).
- **Control Options**:
  1. **Direct GPIO**: Raspberry Pi Pin 18 (GPIO 18 / PWM0) driven by software PWM (`RPi.GPIO` or hardware PWM).
  2. **Pixhawk AUX1 PWM**: Triggered through MAVLink / PX4 Actuator testing topic (`/fmu/in/actuator_motors` or MAVLink `MAV_CMD_DO_SET_SERVO`).

---

## 4. Hardware Bridge Configurations

### 4.1 PX4 Micro-XRCE-DDS Bridge Launch
To bridge physical flight controller telemetry to ROS 2:
```bash
# On the Raspberry Pi Companion Computer
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```
This automatically exposes:
- `/fmu/out/vehicle_odometry` -> remapped to `/drone/pose` and `/drone/velocity`
- `/fmu/out/sensor_gps` -> remapped to `/drone/gps/fix`
- `/fmu/out/vehicle_attitude` -> remapped to `/drone/imu/data`
- `/fmu/in/trajectory_setpoint` <- fed by `/drone/cmd_vel` translator

### 4.2 Velodyne VLP-16 Physical Driver Launch
```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py \
  device_ip:=192.168.1.201 \
  port:=2368 \
  frame_id:=lidar_link
```
Remap driver output `/velodyne_points` to `/lidar/points`:
```bash
ros2 run topic_tools relay /velodyne_points /lidar/points
```

### 4.3 Raspberry Pi Camera Physical Driver Launch
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p image_size:="[640,480]" \
  -p pixel_format:="YUYV" \
  -r image_raw:=/camera/image
```

---

## 5. Edge Resource Budget on Raspberry Pi 4 (4 GB RAM)

The companion computer must balance sensory processing without dropping frames or triggering watchdog resets:

| Subsystem Node | Target Rate | CPU Usage (Cores) | Memory (RAM) | Optimization Strategy |
| :--- | :--- | :--- | :--- | :--- |
| **Pixhawk XRCE Bridge** | 50 Hz | ~5% of 1 core | ~30 MB | Direct binary DDS serialization |
| **`sih_lidar_node`** | 10 Hz | ~25% of 1 core | ~120 MB | Voxel downsampling (0.15m), passthrough filter, bounding box clustering |
| **`sih_optical_depth_node`** | 5-10 Hz | ~60% of 2 cores | ~250 MB | ONNX Runtime CPU with 2 threads, 256x256 input tensor resolution |
| **`sih_survivor_detector_node`** | 10 Hz | ~20% of 1 core | ~90 MB | HSV color gating + contour geometry (pure NumPy/OpenCV vectorization) |
| **`sih_mapping_node`** | 2 Hz | ~15% of 1 core | ~180 MB | Incremental map voxel accumulation, periodic disk flush |
| **`sih_planner_node`** | 1 Hz | ~5% of 1 core | ~40 MB | Deterministic coverage grid, lightweight waypoints |
| **`sih_obstacle_avoidance_node`** | 20 Hz | ~15% of 1 core | ~50 MB | Artificial potential field vector mathematics |
| **`sih_mission_manager_node`** | 5 Hz | ~5% of 1 core | ~35 MB | Finite State Machine transitions |
| **Total System Load** | - | **~2.2 / 4 Cores** | **~745 MB / 4096 MB** | **Safe thermal margin (< 65 deg C with fan)** |

---

## 6. Pre-Flight Hardware Checklist

1. **Power & Battery**: Main LiPo voltage >= 15.8V (4S) / 23.8V (6S). BEC output stable at 5.1V under load.
2. **LiDAR Link**: Link active on `eth0`; ping `192.168.1.201` succeeds; `ros2 topic hz /lidar/points` reports >= 9.5 Hz.
3. **Camera Alignment**: CSI camera lens clean; orientation matches vehicle body frame (X-forward, Y-left, Z-up or optical frame Z-forward).
4. **GNSS Lock**: Pixhawk reports 3D RTK/DGPS fix with >= 12 satellites visible; HDOP < 1.2.
5. **Payload Bay**: Servo arm calibrated; mechanical release pin securely seats survival kit package until `/mission/trigger_payload` service is called.
6. **E-Stop Check**: Manual RC transmitter configured with high-priority manual override switch (PX4 channel 5 POS 1: Manual Stabilized, POS 2: Position Hold, POS 3: Offboard Autonomous).
