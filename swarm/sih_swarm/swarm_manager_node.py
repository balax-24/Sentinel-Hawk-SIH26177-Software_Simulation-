"""Forwarding wrapper for backward compatibility with swarm.sih_swarm imports."""

from .sih_swarm.swarm_manager_node import (
    SwarmCoordinator,
    SwarmManagerNode,
    main,
)

__all__ = ["SwarmCoordinator", "SwarmManagerNode", "main"]

if __name__ == "__main__":
    main()
