"""Global and survey grid path planner for SIH Search & Rescue.

Generates coverage search paths and waypoint trajectories for the disaster area.
Subscribes to:
  /drone/pose (geometry_msgs/PoseStamped)
  /navigation/target_waypoint (geometry_msgs/PoseStamped)
Publishes:
  /navigation/path (nav_msgs/Path)
  /navigation/current_goal (geometry_msgs/PoseStamped)
  /navigation/status (std_msgs/String)
"""

from __future__ import annotations
import json
import logging
import math
import sys
from typing import List, Tuple, Any, Optional
import numpy as np

logger = logging.getLogger("SIH_Planner")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped, Point
    from std_msgs.msg import Header, String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    Path = Any
    PoseStamped = Any
    Point = Any
    Header = Any
    String = Any
    qos_profile_sensor_data = None


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
                waypoints.append((float(x_min), float(y), float(self.altitude)))
                waypoints.append((float(x_max), float(y), float(self.altitude)))
            else:
                # Right to left
                waypoints.append((float(x_max), float(y), float(self.altitude)))
                waypoints.append((float(x_min), float(y), float(self.altitude)))

        return waypoints


class PlannerNode(Node if HAS_ROS2 else object):
    """ROS 2 Node publishing planned paths and tracking waypoint progression."""

    def __init__(self) -> None:
        if HAS_ROS2:
            if not rclpy.ok():
                rclpy.init()
            super().__init__("sih_planner_node")
            self.declare_parameter("altitude", 3.0)
            self.declare_parameter("lane_spacing", 4.0)
            self.declare_parameter("x_min", -5.0)
            self.declare_parameter("x_max", 20.0)
            self.declare_parameter("y_min", -5.0)
            self.declare_parameter("y_max", 20.0)
            self.declare_parameter("acceptance_radius", 1.5)
            self.declare_parameter("frame_id", "disaster_world")

            alt = float(self.get_parameter("altitude").value)
            spacing = float(self.get_parameter("lane_spacing").value)
            self.acceptance_radius = float(self.get_parameter("acceptance_radius").value)
            self.frame_id = str(self.get_parameter("frame_id").value)

            self.planner = LawnmoverGridPlanner(altitude_m=alt, lane_spacing_m=spacing)

            x_min = float(self.get_parameter("x_min").value)
            x_max = float(self.get_parameter("x_max").value)
            y_min = float(self.get_parameter("y_min").value)
            y_max = float(self.get_parameter("y_max").value)

            self.waypoints = self.planner.plan_search_grid(x_min, x_max, y_min, y_max)
            self.current_wp_idx = 0
            self.latest_pose: Optional[PoseStamped] = None

            # Subscriptions
            self.sub_pose = self.create_subscription(
                PoseStamped, "/drone/pose", self._pose_callback, qos_profile_sensor_data
            )
            self.sub_target = self.create_subscription(
                PoseStamped, "/navigation/target_waypoint", self._target_waypoint_callback, 10
            )

            # Publishers
            self.pub_path = self.create_publisher(Path, "/navigation/path", 10)
            self.pub_goal = self.create_publisher(PoseStamped, "/navigation/current_goal", 10)
            self.pub_status = self.create_publisher(String, "/navigation/status", 10)

            # Timer loop for waypoint progression & goal dispatch (10 Hz)
            self.timer = self.create_timer(0.1, self._timer_callback)
            self.get_logger().info(
                f"sih_planner_node initialized with {len(self.waypoints)} waypoints in [{x_min},{x_max}]x[{y_min},{y_max}]."
            )
        else:
            self.planner = LawnmoverGridPlanner()
            self.waypoints = self.planner.plan_search_grid()
            self.current_wp_idx = 0
            logger.info("LawnmoverGridPlanner initialized in standalone mode.")

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _target_waypoint_callback(self, msg: PoseStamped) -> None:
        """Dynamically divert to a high-priority waypoint (e.g. survivor drop or base)."""
        target = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if HAS_ROS2:
            self.get_logger().info(f"Received high-priority target waypoint: {target}")
        self.waypoints.insert(self.current_wp_idx, target)

    def _timer_callback(self) -> None:
        if not HAS_ROS2 or len(self.waypoints) == 0:
            return

        # 1. Publish current full path
        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = self.frame_id

        for wp in self.waypoints:
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = wp[0]
            ps.pose.position.y = wp[1]
            ps.pose.position.z = wp[2]
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.pub_path.publish(path_msg)

        # 2. Check if current waypoint is reached
        target_wp = self.waypoints[self.current_wp_idx]
        dist_to_wp = 999.0

        if self.latest_pose is not None:
            px = self.latest_pose.pose.position.x
            py = self.latest_pose.pose.position.y
            pz = self.latest_pose.pose.position.z
            dist_to_wp = math.sqrt(
                (px - target_wp[0]) ** 2 + (py - target_wp[1]) ** 2 + (pz - target_wp[2]) ** 2
            )

            if dist_to_wp < self.acceptance_radius:
                if self.current_wp_idx < len(self.waypoints) - 1:
                    self.current_wp_idx += 1
                    target_wp = self.waypoints[self.current_wp_idx]
                    self.get_logger().info(
                        f"Reached Waypoint! Advancing to #{self.current_wp_idx+1}/{len(self.waypoints)}: {target_wp}"
                    )
                else:
                    self.get_logger().info("Completed all search waypoints in survey grid.")

        # 3. Publish active goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = now
        goal_msg.header.frame_id = self.frame_id
        goal_msg.pose.position.x = target_wp[0]
        goal_msg.pose.position.y = target_wp[1]
        goal_msg.pose.position.z = target_wp[2]
        goal_msg.pose.orientation.w = 1.0
        self.pub_goal.publish(goal_msg)

        # 4. Publish navigation status
        status = {
            "waypoint_index": self.current_wp_idx + 1,
            "total_waypoints": len(self.waypoints),
            "target": target_wp,
            "distance_to_target": round(dist_to_wp, 2) if dist_to_wp < 900.0 else None,
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.pub_status.publish(status_msg)


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
