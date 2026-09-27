"""Emergency payload delivery dispenser node for SIH Search & Rescue.

Handles dropping medical kits / water rations near identified survivor targets.
Subscribes to:
  /drone/pose (geometry_msgs/PoseStamped)
Publishes:
  /payload/status (interfaces/msg/PayloadStatus)
Provides Service:
  /mission/trigger_payload (interfaces/srv/TriggerPayload)
"""

from __future__ import annotations
import logging
import sys
import time
from typing import Dict, Any, Optional

logger = logging.getLogger("PayloadDelivery")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, Point
    from std_msgs.msg import Header
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    PoseStamped = Any
    Point = Any
    Header = Any
    qos_profile_sensor_data = None

try:
    from sih_interfaces.msg import PayloadStatus
    from sih_interfaces.srv import TriggerPayload
    HAS_PAYLOAD_MSGS = True
except ImportError:
    HAS_PAYLOAD_MSGS = False
    PayloadStatus = Any
    TriggerPayload = Any


class PayloadMechanismSimulator:
    """Simulates multi-bay electro-mechanical payload servo releases."""

    def __init__(self) -> None:
        self.bays = {
            "bay_1": {
                "type": "FIRST_AID_KIT",
                "weight_kg": 0.5,
                "status": "LOADED",
                "servo_pin": 18,
            },
            "bay_2": {
                "type": "WATER_RATION",
                "weight_kg": 0.6,
                "status": "LOADED",
                "servo_pin": 19,
            },
        }

    def trigger_drop(self, bay_id: str) -> Dict[str, Any]:
        """Actuate servo gripper release."""
        if bay_id not in self.bays:
            return {"success": False, "msg": f"Unknown bay '{bay_id}'."}

        bay = self.bays[bay_id]
        if bay["status"] != "LOADED":
            return {"success": False, "msg": f"Bay '{bay_id}' is already {bay['status']}."}

        # Actuate servo
        bay["status"] = "RELEASED"
        logger.info("Servo on pin %d actuated: Dropped %s!", bay["servo_pin"], bay["type"])
        return {
            "success": True,
            "msg": f"Successfully released {bay['type']} from {bay_id}.",
            "payload_type": bay["type"],
            "weight_kg": bay["weight_kg"],
        }


class PayloadDeliveryNode(Node if HAS_ROS2 else object):
    """ROS 2 Node managing emergency payload drop services."""

    def __init__(self) -> None:
        if HAS_ROS2:
            if not rclpy.ok():
                rclpy.init()
            super().__init__("sih_payload_delivery_node")
            self.dispenser = PayloadMechanismSimulator()
            self.latest_pose: Optional[PoseStamped] = None

            # Subscriptions
            self.sub_pose = self.create_subscription(
                PoseStamped, "/drone/pose", self._pose_callback, qos_profile_sensor_data
            )

            # Publishers & Services
            if HAS_PAYLOAD_MSGS:
                self.pub_status = self.create_publisher(PayloadStatus, "/payload/status", 10)
                self.srv_trigger = self.create_service(
                    TriggerPayload, "/mission/trigger_payload", self._handle_trigger_payload
                )
            else:
                self.pub_status = None
                self.srv_trigger = None

            # Periodic status broadcast (1 Hz)
            self.timer = self.create_timer(1.0, self._broadcast_status)
            self.get_logger().info("sih_payload_delivery_node initialized. Service: /mission/trigger_payload")
        else:
            self.dispenser = PayloadMechanismSimulator()
            logger.info("PayloadMechanismSimulator initialized in standalone mode.")

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _handle_trigger_payload(
        self, request: TriggerPayload.Request, response: TriggerPayload.Response
    ) -> TriggerPayload.Response:
        bay_id = request.bay_id if request.bay_id else "bay_1"
        self.get_logger().info(f"Received payload release request for {bay_id}")

        res = self.dispenser.trigger_drop(bay_id)
        response.success = res["success"]
        response.status_message = res["msg"]
        response.release_time = self.get_clock().now().to_msg()

        # Immediate status publish
        self._broadcast_status()
        return response

    def _broadcast_status(self) -> None:
        if not HAS_ROS2 or self.pub_status is None:
            return

        now = self.get_clock().now().to_msg()
        for bay_id, info in self.dispenser.bays.items():
            msg = PayloadStatus()
            msg.header.stamp = now
            msg.header.frame_id = "base_link"
            msg.bay_id = bay_id
            msg.status = info["status"]
            msg.payload_type = info["type"]
            msg.payload_weight_kg = float(info["weight_kg"])
            msg.gripper_closed = (info["status"] == "LOADED")

            if self.latest_pose is not None:
                p = self.latest_pose.pose.position
                msg.release_coordinates = Point(x=p.x, y=p.y, z=p.z)
            else:
                msg.release_coordinates = Point(x=0.0, y=0.0, z=0.0)

            self.pub_status.publish(msg)


def run_standalone_demo():
    print("=" * 80)
    print(" SIH26177 PAYLOAD DELIVERY DISPENSER DEMO (STANDALONE RUNNER)")
    print("=" * 80)
    dispenser = PayloadMechanismSimulator()
    print(" Initial Payload State:")
    for bay_id, info in dispenser.bays.items():
        print(f"   * [{bay_id}]: {info['type']} ({info['weight_kg']} kg) - {info['status']}")

    print("\n Triggering Emergency First Aid Drop for Bay 1...")
    res = dispenser.trigger_drop("bay_1")
    print(f" Result: {res['msg']}")

    print("\n Final Payload State:")
    for bay_id, info in dispenser.bays.items():
        print(f"   * [{bay_id}]: {info['type']} - Status: {info['status']}")
    print("=" * 80)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    if HAS_ROS2:
        rclpy.init(args=args)
        node = PayloadDeliveryNode()
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
