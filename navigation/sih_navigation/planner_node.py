"""Global and survey grid path planner for SIH Search & Rescue.

Generates coverage search paths and waypoint trajectories for the disaster area.
Subscribes to:
  /drone/pose (geometry_msgs/PoseStamped)
Publishes:
  /navigation/path (nav_msgs/Path)
"""

from __future__ import annotations
import logging
import math
import sys
from typing import List, Tuple, Any

logger = logging.getLogger("SIH_Planner")

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped, Point
    from std_msgs.msg import Header
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    Path = Any
    PoseStamped = Any
    Header = Any


class LawnmoverGridPlanner:
    """Computes an optimal lawnmower search coverage path over a disaster bounding box."""

    def __init__(self, altitude_m: float = 3.5, lane_spacing_m: float = 3.0) -> None:
        self.altitude = altitude_m
        self.lane_spacing = lane_spacing_m

    def plan_search_grid(
        self,
        x_min: float = -5.0,
        x_max: float = 12.0,
        y_min: float = -2.0,
        y_max: float = 12.0,
    ) -> List[Tuple[float, float, float]]:
        """Generate snake / lawnmower waypoints (X, Y, Z)."""
        waypoints = []
        y_lanes = np.arange(y_min, y_max + self.lane_spacing, self.lane_spacing)

        for i, y in enumerate(y_lanes):
            if i % 2 == 0:
                # Left to right
                waypoints.append((x_min, float(y), self.altitude))
                waypoints.append((x_max, float(y), self.altitude))
            else:
                # Right to left
                waypoints.append((x_max, float(y), self.altitude))
                waypoints.append((x_min, float(y), self.altitude))

        return waypoints


import numpy as np


class PlannerNode(Node if HAS_ROS2 else object):
    """ROS 2 Node publishing planned paths."""

    def __init__(self) -> None:
        if HAS_ROS2:
            super().__init__("sih_planner_node")
            self.planner = LawnmoverGridPlanner()
            self.pub_path = self.create_publisher(Path, "/navigation/path", 10)
            logger.info("sih_planner_node initialized.")
        else:
            self.planner = LawnmoverGridPlanner()
            logger.info("LawnmoverGridPlanner initialized in standalone mode.")


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 PATH PLANNER DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    planner = LawnmoverGridPlanner(altitude_m=3.5, lane_spacing_m=3.0)
    wps = planner.plan_search_grid(x_min=-2.0, x_max=10.0, y_min=0.0, y_max=10.0)
    print(f" Generated Survey Grid Path: {len(wps)} waypoints at altitude {planner.altitude}m")
    for i, pt in enumerate(wps[:6], 1):
        print(f"   * Waypoint #{i}: X={pt[0]:.1f}, Y={pt[1]:.1f}, Z={pt[2]:.1f}m")
    if len(wps) > 6:
        print(f"   ... ({len(wps) - 6} more waypoints covering full disaster perimeter)")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if HAS_ROS2:
        rclpy.init(args=args)
        node = PlannerNode()
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
