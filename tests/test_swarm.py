"""Unit tests for multi-UAV swarm search area partitioning."""

import pytest

from swarm.sih_swarm.swarm_manager_node import SwarmCoordinator


def test_swarm_area_partitioning():
    """Verify non-overlapping search sector allocation among UAVs."""
    swarm = SwarmCoordinator()
    sectors = swarm.partition_disaster_area(x_min=0.0, x_max=15.0, y_min=0.0, y_max=10.0)

    assert len(sectors) == 3

    # Check contiguous bounds
    uav1_bounds = sectors["uav_1"]["bounds"]
    uav2_bounds = sectors["uav_2"]["bounds"]
    uav3_bounds = sectors["uav_3"]["bounds"]

    assert uav1_bounds[0] == 0.0 and uav1_bounds[1] == 5.0
    assert uav2_bounds[0] == 5.0 and uav2_bounds[1] == 10.0
    assert uav3_bounds[0] == 10.0 and uav3_bounds[1] == 15.0
