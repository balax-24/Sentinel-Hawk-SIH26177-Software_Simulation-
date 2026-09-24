# PayloadStatus.msg
# First aid / emergency supply dispenser mechanism telemetry

std_msgs/Header header
string bay_id                 # e.g., "bay_medkit_1", "bay_water_2"
string status                 # "LOADED", "ARMED", "RELEASED", "DELIVERED", "FAILED"
string payload_type           # "FIRST_AID_KIT", "WATER_RATION", "EPIRB_BEACON", "RADIO"
float32 payload_weight_kg     # Payload mass in kilograms
geometry_msgs/Point release_coordinates # Location where payload was dropped
bool gripper_closed           # Physical electro-mechanical servo state
