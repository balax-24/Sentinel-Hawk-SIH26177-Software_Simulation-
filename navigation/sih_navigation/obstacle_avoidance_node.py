"""Forwarding wrapper for backward compatibility with navigation.sih_navigation imports."""

from .sih_navigation.obstacle_avoidance_node import (
    PotentialFieldAvoidance,
    ObstacleAvoidanceNode,
    main,
)

__all__ = ["PotentialFieldAvoidance", "ObstacleAvoidanceNode", "main"]

if __name__ == "__main__":
    main()
