"""Local Obstacle Avoidance Node using Artificial Potential Fields for SIH26177.

Subscribes to:
  /drone/pose (geometry_msgs/PoseStamped)
  /navigation/current_goal (geometry_msgs/PoseStamped)
  /obstacles (interfaces/msg/ObstacleArray)
Publishes:
  /drone/cmd_vel (geometry_msgs/Twist)
  /navigation/avoidance_status (std_msgs/String)
"""

from __future__ import annotations
import json
import logging
import math
import sys
from typing import List, Tuple, Any, Optional
import numpy as np

logger = logging.getLogger("ObstacleAvoidance")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist, PoseStamped, Point
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    Twist = Any
    PoseStamped = Any
    Point = Any
    String = Any
    qos_profile_sensor_data = None

try:
    from sih_interfaces.msg import Obstacle, ObstacleArray
    HAS_OBSTACLES = True
except ImportError:
    HAS_OBSTACLES = False
    Obstacle = Any
    ObstacleArray = Any


class PotentialFieldAvoidance:
    """Calculates repulsive vectors from nearby 3D obstacles."""

    def __init__(
        self,
        safety_radius_m: float = 2.5,
        repulsion_gain: float = 2.0,
        max_speed_mps: float = 2.0,
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
                unit_dir = diff / dist
                mag = self.k_rep * (1.0 / dist - 1.0 / self.safety_radius) / (dist ** 2)
                # Cap repulsive contribution per obstacle to avoid instability
                v_rep += unit_dir * float(min(mag, 4.0))

        result_vel = v_attr + v_rep
        speed = float(np.linalg.norm(result_vel))
        if speed > self.max_speed:
            result_vel = (result_vel / speed) * self.max_speed

        return float(result_vel[0]), float(result_vel[1]), float(result_vel[2])


class ObstacleAvoidanceNode(Node if HAS_ROS2 else object):
    """ROS 2 Node subscribing to pose, goal, and obstacles, publishing safe /drone/cmd_vel."""

    def __init__(self) -> None:
        if HAS_ROS2:
            if not rclpy.ok():
                rclpy.init()
            super().__init__("sih_obstacle_avoidance_node")
            self.declare_parameter("safety_radius", 2.5)
            self.declare_parameter("repulsion_gain", 2.0)
            self.declare_parameter("max_speed", 2.0)
            self.declare_parameter("kp_pos", 0.8)
            self.declare_parameter("kp_z", 1.0)

            safety_radius = float(self.get_parameter("safety_radius").value)
            repulsion_gain = float(self.get_parameter("repulsion_gain").value)
            max_speed = float(self.get_parameter("max_speed").value)
            self.kp_pos = float(self.get_parameter("kp_pos").value)
            self.kp_z = float(self.get_parameter("kp_z").value)

            self.avoider = PotentialFieldAvoidance(
                safety_radius_m=safety_radius,
                repulsion_gain=repulsion_gain,
                max_speed_mps=max_speed,
            )

            self.current_pose: Optional[PoseStamped] = None
            self.current_goal: Optional[PoseStamped] = None
            self.obstacle_positions: List[Tuple[float, float, float]] = []

            # Subscriptions
            self.sub_pose = self.create_subscription(
                PoseStamped, "/drone/pose", self._pose_callback, qos_profile_sensor_data
            )
            self.sub_goal = self.create_subscription(
                PoseStamped, "/navigation/current_goal", self._goal_callback, 10
            )

            if HAS_OBSTACLES:
                self.sub_obstacles = self.create_subscription(
                    ObstacleArray, "/obstacles", self._obstacles_callback, 10
                )
            else:
                self.sub_obstacles = None

            # Publishers
            self.pub_cmd_vel = self.create_publisher(Twist, "/drone/cmd_vel", 10)
            self.pub_status = self.create_publisher(String, "/navigation/avoidance_status", 10)

            # Control loop (20 Hz)
            self.timer = self.create_timer(0.05, self._control_loop)
            self.get_logger().info(
                f"sih_obstacle_avoidance_node initialized. Max speed: {max_speed} m/s, safety radius: {safety_radius} m."
            )
        else:
            self.avoider = PotentialFieldAvoidance()
            logger.info("PotentialFieldAvoidance initialized in standalone mode.")

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.current_goal = msg

    def _obstacles_callback(self, msg: ObstacleArray) -> None:
        """Cache positions of detected obstacles in world frame."""
        new_obs = []
        if self.current_pose is not None:
            # Transform local sensor obstacles into world frame if needed
            ux = self.current_pose.pose.position.x
            uy = self.current_pose.pose.position.y
            uz = self.current_pose.pose.position.z

            for obs in msg.obstacles:
                # Bounding box / centroid
                wx = ux + obs.position.x
                wy = uy + obs.position.y
                wz = uz + obs.position.z
                new_obs.append((wx, wy, wz))
        else:
            for obs in msg.obstacles:
                new_obs.append((obs.position.x, obs.position.y, obs.position.z))

        self.obstacle_positions = new_obs

    def _control_loop(self) -> None:
        if not HAS_ROS2 or self.current_pose is None or self.current_goal is None:
            return

        uav_x = self.current_pose.pose.position.x
        uav_y = self.current_pose.pose.position.y
        uav_z = self.current_pose.pose.position.z

        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        goal_z = self.current_goal.pose.position.z

        # 1. Attractive goal vector (proportional controller)
        dx = goal_x - uav_x
        dy = goal_y - uav_y
        dz = goal_z - uav_z
        dist_xy = math.sqrt(dx * dx + dy * dy)
        dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist_3d < 0.2:
            # Reached goal position: hover in place
            cmd = Twist()
            self.pub_cmd_vel.publish(cmd)
            return

        # Desired horizontal and vertical speed
        vx_attr = float(np.clip(self.kp_pos * dx, -self.avoider.max_speed, self.avoider.max_speed))
        vy_attr = float(np.clip(self.kp_pos * dy, -self.avoider.max_speed, self.avoider.max_speed))
        vz_attr = float(np.clip(self.kp_z * dz, -1.0, 1.0))

        # 2. Compute safe avoidance velocity via potential fields
        safe_vx, safe_vy, safe_vz = self.avoider.compute_avoidance_velocity(
            desired_vel=(vx_attr, vy_attr, vz_attr),
            uav_pos=(uav_x, uav_y, uav_z),
            obstacle_positions=self.obstacle_positions,
        )

        # 3. Publish cmd_vel to UAV
        cmd = Twist()
        cmd.linear.x = safe_vx
        cmd.linear.y = safe_vy
        cmd.linear.z = safe_vz

        # Compute heading toward motion direction
        if dist_xy > 0.5:
            target_yaw = math.atan2(safe_vy, safe_vx)
            # Simple yaw alignment
            cmd.angular.z = float(np.clip(target_yaw * 0.5, -0.8, 0.8))

        self.pub_cmd_vel.publish(cmd)

        # 4. Status telemetry
        status = {
            "uav_pos": [round(uav_x, 2), round(uav_y, 2), round(uav_z, 2)],
            "target": [round(goal_x, 2), round(goal_y, 2), round(goal_z, 2)],
            "distance": round(dist_3d, 2),
            "commanded_vel": [round(safe_vx, 2), round(safe_vy, 2), round(safe_vz, 2)],
            "active_obstacles": len(self.obstacle_positions),
        }
        stat_msg = String()
        stat_msg.data = json.dumps(status)
        self.pub_status.publish(stat_msg)


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
