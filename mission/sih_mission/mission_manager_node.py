"""Forwarding wrapper for backward compatibility with mission.sih_mission imports."""

from .sih_mission.mission_manager_node import (
    MissionState,
    SARMissionCoordinator,
    MissionManagerNode,
    main,
)

__all__ = ["MissionState", "SARMissionCoordinator", "MissionManagerNode", "main"]

if __name__ == "__main__":
    main()
