"""Small, dependency-free mathematical building blocks.

The classes exported here are intentionally independent from map loading,
planning and runtime code.  They provide one vocabulary that those layers can
adopt incrementally.
"""

from .geometry import (
    Pose2D,
    Vector2,
    normalize_angle,
    shortest_angular_distance,
)
from .intervals import (
    TimeInterval,
    merge_intervals,
    subtract_intervals,
)
from .polygons import Polygon2D, Projection, distance_to_segment

__all__ = [
    "Pose2D",
    "Polygon2D",
    "Projection",
    "TimeInterval",
    "Vector2",
    "distance_to_segment",
    "merge_intervals",
    "normalize_angle",
    "shortest_angular_distance",
    "subtract_intervals",
]
