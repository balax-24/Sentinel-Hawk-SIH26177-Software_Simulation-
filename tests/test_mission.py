"""Unit tests for mission state machine and payload delivery mechanism."""

import pytest

from mission.sih_mission.mission_manager_node import SARMissionCoordinator, MissionState
from mission.sih_mission.payload_delivery_node import PayloadMechanismSimulator


def test_mission_state_transitions():
    """Verify logical state machine progressions."""
    coord = SARMissionCoordinator()
    assert coord.current_state == MissionState.IDLE

    coord.transition_to(MissionState.TAKEOFF)
    assert coord.current_state == MissionState.TAKEOFF

    coord.on_survivor_reported(survivor_id=1)
    assert coord.current_state == MissionState.SURVIVOR_FOUND
    assert coord.survivors_located == 1


def test_payload_mechanism_drop():
    """Verify payload bay triggers and rejects double-release."""
    disp = PayloadMechanismSimulator()

    # Bay 1 drop
    res1 = disp.trigger_drop("bay_1")
    assert res1["success"] is True
    assert disp.bays["bay_1"]["status"] == "RELEASED"

    # Attempt second drop on empty bay
    res2 = disp.trigger_drop("bay_1")
    assert res2["success"] is False
