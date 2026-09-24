# SIH LiDAR Processing Package (`sih_lidar`)

Processes raw 3D LiDAR point clouds (`/lidar/points`), separates ground returns from structural obstacles, and clusters 3D bounding obstacles for downstream navigation and mapping.

## Launch Command
```bash
ros2 launch sih_lidar lidar.launch.py
```

## Standalone Demo
```bash
python -m perception.sih_lidar.sih_lidar.lidar_processor_node
```

## ROS 2 Topics
* **Subscribed**:
  * `/lidar/points` (`sensor_msgs/msg/PointCloud2`)
* **Published**:
  * `/lidar/filtered_points` (`sensor_msgs/msg/PointCloud2`)
  * `/obstacles` (`interfaces/msg/ObstacleArray`)
