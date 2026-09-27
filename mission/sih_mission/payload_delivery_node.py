"""Forwarding wrapper for backward compatibility with mission.sih_mission imports."""

from .sih_mission.payload_delivery_node import (
    PayloadMechanismSimulator,
    PayloadDeliveryNode,
    main,
)

__all__ = ["PayloadMechanismSimulator", "PayloadDeliveryNode", "main"]

if __name__ == "__main__":
    main()
