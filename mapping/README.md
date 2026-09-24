# SIH 3D Mapping Package (`sih_mapping`)

Maintains and accumulates the global 3D volumetric point cloud representation of the disaster environment as the UAV traverses the search area.

## Launch Command
```bash
ros2 launch sih_mapping mapping.launch.py
```

## Standalone Demo
```bash
python -m mapping.sih_mapping.mapping_node
```

## ROS 2 Topics
* **Subscribed**:
  * `/lidar/filtered_points` (`sensor_msgs/msg/PointCloud2`)
  * `/drone/pose` (`geometry_msgs/msg/PoseStamped`)
* **Published**:
  * `/map/pointcloud` (`sensor_msgs/msg/PointCloud2`)
