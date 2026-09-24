"""Multi-UAV Swarm Management and Task Allocation Node for SIH26177.

Partitions disaster search zones among multiple UAVs, monitors swarm health,
and tracks cumulative search area coverage.
Publishes:
  /swarm/state (interfaces/msg/SwarmState)
"""

from __future__ import annotations
import logging
import sys
from typing import List, Dict, Any

logger = logging.getLogger("SwarmManager")

try:
    import rclpy
    from rclpy.node import Node
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object


class SwarmCoordinator:
    """Allocates search sectors to multiple drones in the swarm."""

    def __init__(self, swarm_id: str = "sih_swarm_alpha") -> None:
        self.swarm_id = swarm_id
        self.drones = ["uav_1", "uav_2", "uav_3"]
        self.sector_assignments = {}

    def partition_disaster_area(
        self, x_min: float, x_max: float, y_min: float, y_max: float
    ) -> Dict[str, Dict[str, float]]:
        """Divide bounding box into non-overlapping sub-sectors along X."""
        num_drones = len(self.drones)
        dx_per_drone = (x_max - x_min) / float(num_drones)

        for i, drone_id in enumerate(self.drones):
            sub_xmin = x_min + i * dx_per_drone
            sub_xmax = sub_xmin + dx_per_drone
            self.sector_assignments[drone_id] = {
                "sector_id": f"sector_{i+1}",
                "bounds": [sub_xmin, sub_xmax, y_min, y_max],
                "task": "GRID_SEARCH",
            }
        return self.sector_assignments


class SwarmManagerNode(Node if HAS_ROS2 else object):
    """ROS 2 Node coordinating multi-UAV swarm."""

    def __init__(self) -> None:
        if HAS_ROS2:
            super().__init__("sih_swarm_node")
            self.coordinator = SwarmCoordinator()
            logger.info("sih_swarm_node initialized.")
        else:
            self.coordinator = SwarmCoordinator()
            logger.info("SwarmCoordinator initialized in standalone mode.")


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 MULTI-UAV SWARM ALLOCATION DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    swarm = SwarmCoordinator()
    sectors = swarm.partition_disaster_area(x_min=-10.0, x_max=20.0, y_min=-10.0, y_max=20.0)

    print(f" Swarm ID: {swarm.swarm_id} ({len(swarm.drones)} UAVs active)")
    print(" Allocated Search Sectors:")
    for drone_id, info in sectors.items():
        b = info["bounds"]
        print(f"   * [{drone_id}]: Assigned to {info['sector_id']} -> X:[{b[0]:.1f}, {b[1]:.1f}]m, Y:[{b[2]:.1f}, {b[3]:.1f}]m")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if HAS_ROS2:
        rclpy.init(args=args)
        node = SwarmManagerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        run_standalone_demo()


if __name__ == "__main__":
    main()
