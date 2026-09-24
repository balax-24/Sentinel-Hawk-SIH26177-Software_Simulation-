"""Unit tests for navigation path planning and potential field obstacle avoidance."""

import math
import numpy as np
import pytest

from navigation.sih_navigation.planner_node import LawnmoverGridPlanner
from navigation.sih_navigation.obstacle_avoidance_node import PotentialFieldAvoidance


def test_lawnmower_planner_coverage():
    """Verify generated search path covers specified rectangular boundaries."""
    planner = LawnmoverGridPlanner(altitude_m=4.0, lane_spacing_m=2.0)
    wps = planner.plan_search_grid(x_min=0.0, x_max=6.0, y_min=0.0, y_max=4.0)

    # 3 lanes (y=0, y=2, y=4), each with 2 waypoints => 6 waypoints
    assert len(wps) == 6
    assert np.all([wp[2] == 4.0 for wp in wps])  # Altitude constant
    assert wps[0] == (0.0, 0.0, 4.0)
    assert wps[1] == (6.0, 0.0, 4.0)
    assert wps[2] == (6.0, 2.0, 4.0)


def test_potential_field_repulsion():
    """Verify obstacle directly ahead pushes UAV laterally."""
    avoider = PotentialFieldAvoidance(safety_radius_m=2.0, repulsion_gain=2.0)

    drone_pos = (0.0, 0.0, 2.0)
    desired_vel = (1.0, 0.0, 0.0)  # Fly in +X
    obstacle_ahead = [(1.0, 0.1, 2.0)]  # Obstacle at X=1.0, Y=0.1

    vx, vy, vz = avoider.compute_avoidance_velocity(desired_vel, drone_pos, obstacle_ahead)

    # Repulsive vector must deflect Y in opposite direction of obstacle (+Y obstacle pushes to -Y)
    assert vy < 0.0
    assert math.hypot(vx, vy, vz) <= avoider.max_speed
