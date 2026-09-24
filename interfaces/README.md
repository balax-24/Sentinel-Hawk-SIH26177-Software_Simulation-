# SIH Interfaces (`sih_interfaces`)

Standard and custom ROS 2 messages, services, and actions for the SIH26177 Search-and-Rescue system.

## Messages
* `DroneState.msg`: Battery, armed state, flight mode, 3D pose, linear/angular velocity, GPS, and IMU.
* `Obstacle.msg`: 3D obstacle position, bounding box dimensions, classification, and relative distance.
* `ObstacleArray.msg`: Aggregation of obstacles detected by LiDAR / optical depth.
* `Survivor.msg`: Detected human survivor classification, confidence, 3D local offset, GPS coordinates, and bounding box.
* `SurvivorArray.msg`: List of detected survivors in search area.
* `PayloadStatus.msg`: Emergency supply dispenser mechanism state (medical kit, water, beacon).
* `SwarmState.msg`: Multi-UAV swarm telemetry, active drones, area coverage percentage.

## Services
* `TriggerPayload.srv`: Service to actuate payload bay release mechanism.

## Actions
* `NavigateWaypoint.action`: Action for high-level waypoint trajectory navigation with obstacle avoidance feedback.
