# SIH26177 Topic, Service, and Action Matrix

## Standard ROS 2 Topics

| Topic Name | Message Type | Publisher(s) | Subscriber(s) | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | `sih_simulation` (Gazebo / Mock) | `sih_lidar` | Raw 3D point cloud from LiDAR sensor |
| `/lidar/filtered_points` | `sensor_msgs/msg/PointCloud2` | `sih_lidar` | `sih_mapping` | Outlier and ground-filtered 3D obstacle points |
| `/camera/image` | `sensor_msgs/msg/Image` | `sih_simulation` / Hardware | `sih_perception` | Raw RGB imagery from gimbaled camera |
| `/camera/depth` | `sensor_msgs/msg/Image` | `sih_simulation` / `sih_perception` | `sih_perception` | Depth map from sensor or monocular estimation |
| `/drone/pose` | `geometry_msgs/msg/PoseStamped` | `sih_simulation` / EKF | All nodes | Real-time 3D position and orientation |
| `/drone/velocity` | `geometry_msgs/msg/TwistStamped` | `sih_simulation` / Flight Controller | Navigation | Real-time linear and angular velocity |
| `/drone/gps/fix` | `sensor_msgs/msg/NavSatFix` | `sih_simulation` / U-Blox GPS | Mission, Dashboard | WGS84 latitude, longitude, and altitude |
| `/drone/imu/data` | `sensor_msgs/msg/Imu` | `sih_simulation` / Pixhawk IMU | State Estimator | Linear accelerations and angular rates |
| `/drone/cmd_vel` | `geometry_msgs/msg/Twist` | `sih_navigation` | Flight Controller / Sim | Commanded linear and angular velocity |
| `/navigation/path` | `nav_msgs/msg/Path` | `sih_navigation` (planner) | Obstacle Avoidance | Planned global search waypoints |
| `/map/pointcloud` | `sensor_msgs/msg/PointCloud2` | `sih_mapping` | Dashboard, Operators | Accumulated global 3D disaster terrain map |

## Custom Interfaces (`sih_interfaces`)

| Topic / Service / Action | Type | Provider / Publisher | Consumer / Subscriber | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/obstacles` | `interfaces/msg/ObstacleArray` | `sih_lidar` | `sih_navigation` | Bounding boxes and ranges of detected obstacles |
| `/survivors` | `interfaces/msg/SurvivorArray` | `sih_perception` | `sih_mission`, Dashboard | Identified casualties with geotagged GPS locations |
| `/mission/state` | `std_msgs/msg/String` | `sih_mission` | All nodes, Dashboard | Active phase (PREFLIGHT, SURVEY, DROP, RTL) |
| `/payload/status` | `interfaces/msg/PayloadStatus` | `sih_mission` (payload) | Dashboard, Ground Crew | Dispenser bay arming and release status |
| `/swarm/state` | `interfaces/msg/SwarmState` | `sih_swarm` | Dashboard | Swarm sector allocations and coverage % |
| `/mission/trigger_payload` | `interfaces/srv/TriggerPayload` | `sih_mission` (payload) | `sih_mission` (manager) | Actuates servo release mechanism |
| `/navigation/navigate_waypoint` | `interfaces/action/NavigateWaypoint`| `sih_navigation` | `sih_mission` | Trajectory action with obstacle feedback |
