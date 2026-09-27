"""Full system end-to-end integration and architectural tests for SIH26177."""

import json
from pathlib import Path
import numpy as np
import cv2
import pytest

from perception.sih_lidar.sih_lidar.lidar_processor_node import LidarProcessor
from mapping.sih_mapping.mapping_node import GlobalMapAccumulator
from navigation.sih_navigation.planner_node import LawnmoverGridPlanner
from navigation.sih_navigation.obstacle_avoidance_node import PotentialFieldAvoidance
from perception.sih_perception.sih_perception.survivor_detector_node import SurvivorDetector
from perception.sih_perception.sih_perception.hazard_detector_node import HazardDetectorNode
from mission.sih_mission.mission_manager_node import SARMissionCoordinator, MissionState
from mission.sih_mission.payload_delivery_node import PayloadMechanismSimulator
from swarm.sih_swarm.swarm_manager_node import SwarmCoordinator
from dashboard.sih_dashboard.dashboard_node import ConsoleDashboardViewer


def test_full_mission_architecture_integration():
    """Verify that all independent agents and pipelines interact harmoniously."""
    # 1. Swarm Manager partitions search zone
    swarm = SwarmCoordinator(drones=["uav_1", "uav_2", "uav_3"])
    sectors = swarm.partition_disaster_area(x_min=-10.0, x_max=20.0, y_min=-10.0, y_max=20.0)
    assert len(sectors) == 3

    # 2. UAV 1 Path Planning for Sector 1
    s1_bounds = sectors["uav_1"]["bounds"]
    planner = LawnmoverGridPlanner(altitude_m=3.0, lane_spacing_m=3.0)
    wps = planner.plan_search_grid(s1_bounds[0], s1_bounds[1], s1_bounds[2], s1_bounds[3])
    assert len(wps) > 0

    # 3. UAV 1 LiDAR Processing & Obstacle Detection
    lidar = LidarProcessor(min_range=0.5, max_range=30.0, ground_threshold_z=0.2)
    raw_scan = np.array([
        [0.0, 0.0, 0.0, 0.2],    # Ground
        [5.0, 0.0, 1.2, 0.8],    # Obstacle ahead
        [5.2, 0.1, 1.5, 0.9],    # Obstacle ahead
        [5.1, -0.1, 1.3, 0.85],  # Obstacle ahead
        [5.3, 0.2, 1.1, 0.9],    # Obstacle ahead
        [5.0, 0.3, 1.4, 0.8],    # Obstacle ahead
    ], dtype=np.float32)
    obs_pts, gnd_pts = lidar.filter_points(raw_scan)
    assert len(obs_pts) >= 5
    clusters = lidar.cluster_obstacles(obs_pts, cluster_dist=1.0, min_samples=3)
    assert len(clusters) == 1

    # 4. 3D Map Accumulation
    mapper = GlobalMapAccumulator(voxel_size=0.1)
    pts_count = mapper.add_scan(obs_pts, drone_pose=(0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0))
    assert pts_count > 0
    assert len(mapper.global_points) == pts_count

    # 5. Obstacle Avoidance Calculation
    avoider = PotentialFieldAvoidance(safety_radius_m=3.0, repulsion_gain=2.0)
    obs_pos = [clusters[0][0]]
    desired_vel = (1.5, 0.0, 0.0)
    safe_vel = avoider.compute_avoidance_velocity(desired_vel, (3.5, 0.0, 3.0), obs_pos)
    # Lateral deflection or deceleration verified
    assert safe_vel[0] < desired_vel[0] or abs(safe_vel[1]) > 0.01

    # 6. Survivor Detection & Geotagging
    detector = SurvivorDetector()
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    cv2.rectangle(frame, (300, 200), (350, 260), (20, 140, 245), -1)  # Rescue vest
    detections = detector.detect_in_frame(frame)
    assert len(detections) >= 1

    surv_lat, surv_lon, surv_alt = detector.calculate_gps_location(
        centroid_px=detections[0]["centroid_px"],
        estimated_depth_m=3.0,
        img_width=640,
        img_height=480,
        drone_pose=(3.5, 0.0, 3.0, 0.0),
        drone_gps=(28.613939, 77.209021, 219.5),
    )
    assert 28.0 < surv_lat < 29.0
    assert 77.0 < surv_lon < 78.0

    # 7. Hazard Detection
    hazard_node = HazardDetectorNode(uav_id="uav_1")
    haz_img = np.full((300, 300, 3), 80, dtype=np.uint8)
    cv2.rectangle(haz_img, (50, 50), (100, 100), (0, 215, 255), -1)  # Fire patch
    hazard_node._run_detection_pipeline(haz_img)
    assert len(hazard_node.tracked_hazards) >= 1

    # 8. Mission State Machine & Payload Drop
    coordinator = SARMissionCoordinator(uav_id="uav_1")
    coordinator.transition_to(MissionState.TAKEOFF)
    coordinator.transition_to(MissionState.GRID_SURVEY)
    coordinator.on_survivor_reported(detections[0]["id"])
    assert coordinator.current_state == MissionState.SURVIVOR_FOUND

    dispenser = PayloadMechanismSimulator()
    res = dispenser.trigger_drop("bay_1")
    assert res["success"] is True
    assert dispenser.bays["bay_1"]["status"] == "RELEASED"

    # 9. Dashboard Rendering
    viewer = ConsoleDashboardViewer()
    rendered = viewer.render_view({
        "swarm": {"id": "sih_swarm_alpha", "phase": "SURVIVOR_FOUND", "coverage_pct": 42.0},
        "uav_positions": {"uav_1": (3.5, 0.0, 3.0), "uav_2": (10.0, 5.0, 3.0)},
        "survivors": [{"id": 1, "classification": "PERSON_TRAPPED", "lat": surv_lat, "lon": surv_lon, "confidence": 0.92}],
        "hazards": [{"type": "FIRE", "severity": "CRITICAL", "confidence": 0.95, "source": "uav_1"}],
        "payloads": dispenser.bays,
        "map_info": {"scan_count": 1, "total_map_points": pts_count},
    })
    assert "SIH26177 SEARCH-AND-RESCUE" in rendered
    assert "SURVIVORS DETECTED" in rendered
    assert "HAZARDS DETECTED" in rendered
