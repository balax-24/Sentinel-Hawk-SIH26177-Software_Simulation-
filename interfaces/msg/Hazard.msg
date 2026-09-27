# Hazard.msg
# Detected disaster hazard or environmental danger in search area

std_msgs/Header header
uint32 id
string hazard_type              # "FIRE", "SMOKE", "FLOOD_WATER", "DEBRIS", "DAMAGED_STRUCTURE", "LANDSLIDE", "ELECTRICAL_HAZARD", "CHEMICAL_LEAK"
float32 confidence              # Detection confidence score [0.0, 1.0]
geometry_msgs/Point local_position # 3D location in UAV sensor frame (meters)
sensor_msgs/NavSatFix gps_coordinate # Geo-tagged coordinate for incident response
string severity                 # "CRITICAL", "HIGH", "MODERATE", "LOW"
string source_uav               # UAV ID that detected the hazard (e.g. "uav_1")
string description              # Diagnostic notes or thermal readings
