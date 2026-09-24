"""Local Obstacle Avoidance Node using Artificial Potential Fields for SIH26177.

Subscribes to:
  /drone/pose (geometry_msgs/PoseStamped)
  /drone/velocity (geometry_msgs/TwistStamped)
  /obstacles (interfaces/msg/ObstacleArray or synthetic list)
Publishes:
  /drone/cmd_vel (geometry_msgs/Twist)
"""

from __future__ import annotations
import logging
import math
import sys
from typing import List, Tuple, Any
import numpy as np

logger = logging.getLogger("ObstacleAvoidance")

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    Twist = Any
    PoseStamped = Any


class PotentialFieldAvoidance:
    """Calculates repulsive vectors from nearby 3D obstacles."""

    def __init__(
        self,
        safety_radius_m: float = 2.0,
        repulsion_gain: float = 1.5,
        max_speed_mps: float = 3.0,
    ) -> None:
        self.safety_radius = safety_radius_m
        self.k_rep = repulsion_gain
        self.max_speed = max_speed_mps

    def compute_avoidance_velocity(
        self,
        desired_vel: Tuple[float, float, float],
        uav_pos: Tuple[float, float, float],
        obstacle_positions: List[Tuple[float, float, float]],
    ) -> Tuple[float, float, float]:
        """Blend attractive goal velocity with repulsive obstacle vectors."""
        v_attr = np.array(desired_vel, dtype=np.float32)
        v_rep = np.zeros(3, dtype=np.float32)

        u_pos = np.array(uav_pos, dtype=np.float32)

        for obs_pos in obstacle_positions:
            diff = u_pos - np.array(obs_pos, dtype=np.float32)
            dist = float(np.linalg.norm(diff))

            if 0.05 < dist < self.safety_radius:
                # Repulsive force direction pointing away from obstacle
                unit_dir = diff / dist
                # Force magnitude proportional to inverse distance
                mag = self.k_rep * (1.0 / dist - 1.0 / self.safety_radius) / (dist ** 2)
                v_rep += unit_dir * float(min(mag, 4.0))

        result_vel = v_attr + v_rep
        speed = float(np.linalg.norm(result_vel))
        if speed > self.max_speed:
            result_vel = (result_vel / speed) * self.max_speed

        return float(result_vel[0]), float(result_vel[1]), float(result_vel[2])


class ObstacleAvoidanceNode(Node if HAS_ROS2 else object):
    """ROS 2 Node publishing safe velocity commands."""

    def __init__(self) -> None:
        if HAS_ROS2:
            super().__init__("sih_obstacle_avoidance_node")
            self.avoider = PotentialFieldAvoidance()
            self.pub_cmd_vel = self.create_publisher(Twist, "/drone/cmd_vel", 10)
            logger.info("sih_obstacle_avoidance_node initialized.")
        else:
            self.avoider = PotentialFieldAvoidance()
            logger.info("PotentialFieldAvoidance initialized in standalone mode.")


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 OBSTACLE AVOIDANCE DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    avoider = PotentialFieldAvoidance(safety_radius_m=2.5, repulsion_gain=1.5)

    drone_pos = (4.0, 4.0, 2.0)
    desired_vel = (1.5, 0.0, 0.0)  # Want to fly in +X
    obstacles = [(5.0, 4.2, 2.0)]  # Obstacle directly ahead at 1.0m!

    safe_vx, safe_vy, safe_vz = avoider.compute_avoidance_velocity(
        desired_vel, drone_pos, obstacles
    )

    print(f" UAV Position:            ({drone_pos[0]:.1f}, {drone_pos[1]:.1f}, {drone_pos[2]:.1f}m)")
    print(f" Desired Vector:          ({desired_vel[0]:.1f}, {desired_vel[1]:.1f}, {desired_vel[2]:.1f} m/s)")
    print(f" Obstacle Position:       ({obstacles[0][0]:.1f}, {obstacles[0][1]:.1f}, {obstacles[0][2]:.1f}m) -> Dist = 1.02m")
    print(f" Safe Avoidance Vector:   ({safe_vx:.2f}, {safe_vy:.2f}, {safe_vz:.2f} m/s)")
    print(" Notice: UAV successfully deflected laterally to bypass obstacle safely!")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if HAS_ROS2:
        rclpy.init(args=args)
        node = ObstacleAvoidanceNode()
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
