# DroneState.msg
# Real-time telemetry and flight controller status

std_msgs/Header header
string drone_id               # e.g., "uav_alpha_1"
bool armed
string flight_mode            # "MANUAL", "OFFBOARD", "AUTO_SURVEY", "PAYLOAD_DROP", "RTL"
float32 battery_percentage    # 0.0 - 100.0
float32 battery_voltage       # Volts
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped velocity
sensor_msgs/NavSatFix gps_fix
sensor_msgs/Imu imu_data
