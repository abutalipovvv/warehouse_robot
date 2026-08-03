"""Backward-compatible access to the mathematical toolkit in ``core.math``."""

from pathlib import Path
import sys

from fleet_manager.core.math import curves as _curves
from fleet_manager.core.math import geometry as _geometry
from fleet_manager.core.math import intervals as _intervals
from fleet_manager.core.math import polygons as _polygons
from fleet_manager.core.math.geometry import (
    Pose2D,
    Vector2,
    normalize_angle,
    normalize_angle_rounded,
    shortest_angular_distance,
)
from fleet_manager.core.math.curves import (
    bezier_point,
    cubic_bezier_derivative,
    cubic_bezier_length,
    cubic_bezier_point,
)
from fleet_manager.core.math.intervals import (
    TimeInterval,
    closed_intervals_overlap,
    merge_intervals,
    subtract_intervals,
)
from fleet_manager.core.math.polygons import (
    Polygon2D,
    Projection,
    distance_to_segment,
)

# Keep historical deep imports working without a second implementation.
__path__ = [str(Path(__file__).with_name("core") / "algorithms" / "math")]
sys.modules[f"{__name__}.curves"] = _curves
sys.modules[f"{__name__}.geometry"] = _geometry
sys.modules[f"{__name__}.intervals"] = _intervals
sys.modules[f"{__name__}.polygons"] = _polygons

__all__ = [
    "Pose2D",
    "Polygon2D",
    "Projection",
    "TimeInterval",
    "Vector2",
    "bezier_point",
    "closed_intervals_overlap",
    "cubic_bezier_derivative",
    "cubic_bezier_length",
    "cubic_bezier_point",
    "distance_to_segment",
    "merge_intervals",
    "normalize_angle",
    "normalize_angle_rounded",
    "shortest_angular_distance",
    "subtract_intervals",
]
