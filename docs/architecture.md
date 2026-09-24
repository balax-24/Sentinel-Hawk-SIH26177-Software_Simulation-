# SIH26177 Autonomous Search-and-Rescue UAV Architecture

```mermaid
graph TD
    subgraph Simulation ["simulation/ (sih_simulation)"]
        GW["Gazebo Disaster World<br/>(disaster_world.world)"]
        UAV["Simulated UAV Platform<br/>(model.sdf / mock_sim_node)"]
        GW --> UAV
    end

    subgraph Sensors ["Sensory Topics"]
        LIDAR_RAW["/lidar/points<br/>sensor_msgs/PointCloud2"]
        RGB_RAW["/camera/image<br/>sensor_msgs/Image"]
        DEPTH_RAW["/camera/depth<br/>sensor_msgs/Image"]
        GPS_RAW["/drone/gps/fix<br/>sensor_msgs/NavSatFix"]
        IMU_RAW["/drone/imu/data<br/>sensor_msgs/Imu"]
        POSE_RAW["/drone/pose<br/>geometry_msgs/PoseStamped"]
    end

    UAV --> LIDAR_RAW
    UAV --> RGB_RAW
    UAV --> DEPTH_RAW
    UAV --> GPS_RAW
    UAV --> IMU_RAW
    UAV --> POSE_RAW

    subgraph Perception ["perception/ (sih_lidar & sih_perception)"]
        LP["sih_lidar_node<br/>Ground extraction & clustering"]
        OD["sih_optical_depth_node<br/>MiDaS ONNX CPU inference"]
        SD["sih_survivor_detector_node<br/>Casualty detection & geotagging"]
    end

    LIDAR_RAW --> LP
    RGB_RAW --> OD
    RGB_RAW --> SD
    DEPTH_RAW --> SD
    POSE_RAW --> SD
    GPS_RAW --> SD

    LP --> OBS["/obstacles<br/>interfaces/ObstacleArray"]
    LP --> LP_FILT["/lidar/filtered_points<br/>sensor_msgs/PointCloud2"]
    SD --> SURV["/survivors<br/>interfaces/SurvivorArray"]

    subgraph Mapping ["mapping/ (sih_mapping)"]
        MAP_NODE["sih_mapping_node<br/>Global 3D point cloud accumulator"]
    end

    LP_FILT --> MAP_NODE
    POSE_RAW --> MAP_NODE
    MAP_NODE --> GLOBAL_MAP["/map/pointcloud<br/>sensor_msgs/PointCloud2"]

    subgraph Navigation ["navigation/ (sih_navigation)"]
        PLANNER["sih_planner_node<br/>Search grid coverage planner"]
        AVOID["sih_obstacle_avoidance_node<br/>Artificial potential fields"]
    end

    POSE_RAW --> PLANNER
    PLANNER --> PATH["/navigation/path<br/>nav_msgs/Path"]
    PATH --> AVOID
    OBS --> AVOID
    POSE_RAW --> AVOID
    AVOID --> CMD_VEL["/drone/cmd_vel<br/>geometry_msgs/Twist"]
    CMD_VEL --> UAV

    subgraph Mission ["mission/ & swarm/ (sih_mission & sih_swarm)"]
        MM["sih_mission_manager_node<br/>State machine coordinator"]
        PD["sih_payload_delivery_node<br/>Emergency supply dispenser"]
        SWARM["sih_swarm_node<br/>Sector partition & allocation"]
    end

    SURV --> MM
    MM --> M_STATE["/mission/state<br/>std_msgs/String"]
    MM --> TRIG["/mission/trigger_payload<br/>interfaces/srv/TriggerPayload"]
    TRIG --> PD
    PD --> P_STAT["/payload/status<br/>interfaces/PayloadStatus"]

    subgraph Dashboard ["dashboard/ (sih_dashboard)"]
        DASH["sih_dashboard_node<br/>Situational awareness console"]
    end

    M_STATE --> DASH
    SURV --> DASH
    P_STAT --> DASH
    POSE_RAW --> DASH
    GPS_RAW --> DASH
```

## Architectural Decoupling Principles

1. **Separation of Physics and Computation**:
   Nodes subscribe only to standard sensory ROS 2 topics (`/lidar/points`, `/camera/image`, etc.). They have no hard-coded dependencies on Gazebo. Replacing Gazebo with real hardware is zero-code-change at the algorithm layer.
2. **CPU-First & Raspberry Pi 4 Ready**:
   All perception and navigation algorithms are designed for single-thread and multi-core CPU execution. Deep learning uses quantized/lightweight ONNX models (`MiDaS v2.1 Small`) with inference latencies under 100 ms on CPU.
3. **Independent Demonstrability**:
   Every package has a standalone executable mode and an independent ROS 2 launch file.
