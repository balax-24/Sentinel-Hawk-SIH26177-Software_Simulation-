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
from typing import Dict, Any

logger = logging.getLogger("PayloadDelivery")

try:
    import rclpy
    from rclpy.node import Node
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object


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
            super().__init__("sih_payload_delivery_node")
            self.dispenser = PayloadMechanismSimulator()
            logger.info("sih_payload_delivery_node initialized.")
        else:
            self.dispenser = PayloadMechanismSimulator()
            logger.info("PayloadMechanismSimulator initialized in standalone mode.")


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
