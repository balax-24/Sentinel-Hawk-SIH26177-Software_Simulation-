# SwarmState.msg
# Coordinated multi-UAV swarm telemetry

std_msgs/Header header
string swarm_id               # e.g., "sih_search_swarm_alpha"
string mission_phase          # "INIT", "DISPERSAL", "GRID_SEARCH", "SURVIVOR_FOUND", "RELAY", "RETURN"
string leader_drone_id        # Active leader UAV ID
string[] active_drones        # List of active drone IDs
geometry_msgs/Point[] drone_positions # Estimated positions of each swarm member
string[] assigned_tasks       # Task description for each corresponding drone
uint32 total_search_area_sqm  # Target disaster area in square meters
float32 coverage_percentage   # Completed search coverage [0.0 - 100.0]
