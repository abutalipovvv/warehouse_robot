"""Compatibility facade for the traffic-graph API."""

from .traffic_graph_models import (
    TrafficGraph,
    TrafficLane,
    TrafficVertex,
    lane_id,
)

__all__ = ["TrafficVertex", "TrafficLane", "TrafficGraph", "lane_id"]
