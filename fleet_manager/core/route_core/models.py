from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Sequence


@dataclass(frozen=True)
class WorldPoint:
    x: float
    y: float

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y}


@dataclass(frozen=True)
class Landmark:
    name: str
    x: float
    y: float
    properties: Mapping[str, object] = field(default_factory=dict)
    ignore_dir: object | None = None

    def to_point(self) -> WorldPoint:
        return WorldPoint(x=self.x, y=self.y)

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "x": self.x,
            "y": self.y,
            "properties": dict(self.properties),
            "ignoreDir": self.ignore_dir,
        }


@dataclass(frozen=True)
class EdgeGeometry:
    geometry: str
    control_points: Sequence[WorldPoint]
    curve_type: str

    def to_dict(self) -> dict[str, object]:
        return {
            "geometry": self.geometry,
            "control_points": [point.to_dict() for point in self.control_points],
            "curve_type": self.curve_type,
        }


@dataclass(frozen=True)
class GraphEdge:
    from_name: str
    to_name: str
    length: float
    kind: str
    edge_type: str
    world_points: Sequence[WorldPoint]
    geometry: EdgeGeometry | None = None
    properties: Mapping[str, object] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        motion_code = self.motion_direction_code()
        payload: dict[str, object] = {
            "from": self.from_name,
            "to": self.to_name,
            "length": self.length,
            "kind": self.kind,
            "type": self.edge_type,
            "world_points": [point.to_dict() for point in self.world_points],
            "properties": dict(self.properties),
            "motionDirectionCode": motion_code,
            "motionDirection": self.motion_direction_label(motion_code),
        }
        if self.geometry is not None:
            payload.update(self.geometry.to_dict())
        return payload

    def motion_direction_code(self) -> int:
        raw = self.properties.get("direction", -1)
        if isinstance(raw, str):
            label = raw.strip().lower().replace("-", "_")
            if label == "forward":
                return 0
            if label == "backward":
                return 1
            if label in {"not_specified", "unspecified", "none", ""}:
                return -1
        try:
            value = int(raw)
        except (TypeError, ValueError):
            return -1
        return value if value in {0, 1} else -1

    @staticmethod
    def motion_direction_label(code: int) -> str:
        if code == 0:
            return "forward"
        if code == 1:
            return "backward"
        return "not_specified"


@dataclass(frozen=True)
class MapMetadata:
    map_name: str
    width: int
    height: int
    resolution: float
    ros_origin: Sequence[float]
    image_data_url: str
    view_padding: int = 36
    coordinate_frame: str = "map_top_left"

    @property
    def origin(self) -> tuple[float, float, float]:
        return (0.0, 0.0, 0.0)

    @property
    def height_m(self) -> float:
        return float(self.height) * float(self.resolution)

    def ros_to_map_point(self, point: WorldPoint) -> WorldPoint:
        origin = [float(value) for value in self.ros_origin]
        yaw = float(origin[2] if len(origin) > 2 else 0.0)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        dx = float(point.x) - float(origin[0] if origin else 0.0)
        dy = float(point.y) - float(origin[1] if len(origin) > 1 else 0.0)
        local_x = (cos_yaw * dx) + (sin_yaw * dy)
        local_y = (-sin_yaw * dx) + (cos_yaw * dy)
        return WorldPoint(x=local_x, y=self.height_m - local_y)

    def map_to_ros_point(self, point: WorldPoint) -> WorldPoint:
        origin = [float(value) for value in self.ros_origin]
        yaw = float(origin[2] if len(origin) > 2 else 0.0)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = float(point.x)
        local_y = self.height_m - float(point.y)
        return WorldPoint(
            x=float(origin[0] if origin else 0.0) + (cos_yaw * local_x) - (sin_yaw * local_y),
            y=float(origin[1] if len(origin) > 1 else 0.0) + (sin_yaw * local_x) + (cos_yaw * local_y),
        )

    def ros_yaw_to_map(self, yaw: float) -> float:
        origin = [float(value) for value in self.ros_origin]
        origin_yaw = float(origin[2] if len(origin) > 2 else 0.0)
        return math.atan2(math.sin(origin_yaw - float(yaw)), math.cos(origin_yaw - float(yaw)))

    def map_yaw_to_ros(self, yaw: float) -> float:
        origin = [float(value) for value in self.ros_origin]
        origin_yaw = float(origin[2] if len(origin) > 2 else 0.0)
        return math.atan2(math.sin(origin_yaw - float(yaw)), math.cos(origin_yaw - float(yaw)))

    def to_dict(self) -> dict[str, object]:
        view_width = self.width + (self.view_padding * 2)
        view_height = self.height + (self.view_padding * 2)
        return {
            "width": self.width,
            "height": self.height,
            "viewPadding": self.view_padding,
            "viewWidth": view_width,
            "viewHeight": view_height,
            "resolution": self.resolution,
            "origin": [0.0, 0.0, 0.0],
            "rosOrigin": [float(value) for value in self.ros_origin],
            "coordinateFrame": self.coordinate_frame,
            "imageDataUrl": self.image_data_url,
        }


@dataclass(frozen=True)
class LoadedMapData:
    map_dir: Path
    map_metadata: MapMetadata
    landmarks: dict[str, Landmark]
    edges: list[GraphEdge]


@dataclass(frozen=True)
class PlannedRoute:
    nodes: list[str]
    edges: list[GraphEdge]
    length: float

    def to_dict(self) -> dict[str, object]:
        return {
            "nodes": list(self.nodes),
            "length": self.length,
        }
