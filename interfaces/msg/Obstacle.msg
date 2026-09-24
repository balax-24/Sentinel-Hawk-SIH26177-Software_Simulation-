# Obstacle.msg
# Detected 3D obstacle in disaster search zone

std_msgs/Header header
uint32 id
string classification         # "WALL", "RUBBLE", "TREE", "POWER_LINE", "UNKNOWN"
geometry_msgs/Point position  # Centroid in camera/world frame (meters)
geometry_msgs/Vector3 dimensions # Bounding box size: width(dx), length(dy), height(dz)
geometry_msgs/Vector3 velocity   # Estimated velocity vector (m/s)
float32 confidence            # Detection confidence [0.0, 1.0]
float32 distance_to_uav       # Euclidian distance to UAV in meters
