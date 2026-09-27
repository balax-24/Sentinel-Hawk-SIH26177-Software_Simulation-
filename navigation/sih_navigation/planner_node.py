"""Forwarding wrapper for backward compatibility with navigation.sih_navigation imports."""

from .sih_navigation.planner_node import (
    LawnmoverGridPlanner,
    PlannerNode,
    main,
)

__all__ = ["LawnmoverGridPlanner", "PlannerNode", "main"]

if __name__ == "__main__":
    main()
