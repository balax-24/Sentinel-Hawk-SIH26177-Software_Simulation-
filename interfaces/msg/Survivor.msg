# Survivor.msg
# Detected human survivor in disaster search-and-rescue operation

std_msgs/Header header
uint32 id
string classification         # "PERSON_CONSCIOUS", "PERSON_TRAPPED", "PERSON_UNCONSCIOUS", "THERMAL_HEAT_SIGNATURE"
float32 confidence            # Detection confidence score [0.0, 1.0]

# 3D Location in UAV sensor frame (meters)
geometry_msgs/Point local_position

# Geospatial coordinate for field rescue teams
sensor_msgs/NavSatFix gps_coordinate

# Estimated rescue urgency (1 = Critical, 2 = High, 3 = Moderate, 4 = Low)
uint8 priority_level

# 2D Bounding box on camera frame: [x_min, y_min, x_max, y_max] in pixels
int32[4] bounding_box
