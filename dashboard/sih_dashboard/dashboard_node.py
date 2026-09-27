"""Forwarding wrapper for backward compatibility with dashboard.sih_dashboard imports."""

from .sih_dashboard.dashboard_node import (
    ConsoleDashboardViewer,
    DashboardNode,
    main,
)

__all__ = ["ConsoleDashboardViewer", "DashboardNode", "main"]

if __name__ == "__main__":
    main()
