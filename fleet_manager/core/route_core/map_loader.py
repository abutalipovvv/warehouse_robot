from __future__ import annotations

import base64
import struct
import zlib
from pathlib import Path

import yaml

from fleet_manager.core.traffic.corridors import compile_controlled_corridor_zones
from fleet_manager.map_data.pgm import PgmImage

from .models import (
    EdgeGeometry,
    GraphEdge,
    Landmark,
    LoadedMapData,
    MapMetadata,
    TrafficZone,
    WorldPoint,
)


MAP_SUPPORT_YAML_FILES = {
    "LMs.yaml",
    "graphs.yaml",
    "graph_edges_lengths.yaml",
    "traffic_zones.yaml",
}


class WarehouseMapLoader:
    def __init__(
        self,
        map_dir: Path,
        *,
        compile_traffic_policies: bool = True,
    ) -> None:
        self.map_dir = map_dir.resolve()
        self.compile_traffic_policies = bool(compile_traffic_policies)

    def load(self) -> LoadedMapData:
        if not self.map_dir.is_dir():
            raise FileNotFoundError(f"Map directory does not exist: {self.map_dir}")

        ros_map_yaml = self.find_ros_map_yaml()
        ros_map = self.read_yaml(ros_map_yaml)
        if not isinstance(ros_map, dict):
            raise ValueError(f"Unexpected ROS map file format: {ros_map_yaml}")

        image_path = (self.map_dir / str(ros_map["image"])).resolve()
        width, height, pixels = self.load_pgm(image_path)
        png_bytes = self._build_grayscale_png(width, height, pixels)
        image_png_base64 = base64.b64encode(png_bytes).decode("ascii")

        map_name = str(ros_map.get("image", image_path.stem)).replace(".pgm", "")
        map_metadata = MapMetadata(
            map_name=map_name,
            width=width,
            height=height,
            resolution=float(ros_map["resolution"]),
            ros_origin=ros_map["origin"],
            image_data_url=f"data:image/png;base64,{image_png_base64}",
        )
        landmarks = self._load_landmarks(self.map_dir / "LMs.yaml", map_metadata)
        edges = self._load_edges(self.map_dir / "graph_edges_lengths.yaml", landmarks, map_metadata)
        traffic_zones = self._load_traffic_zones(
            self.map_dir / "traffic_zones.yaml",
            map_metadata,
        )
        if self.compile_traffic_policies:
            landmarks, edges = compile_controlled_corridor_zones(
                landmarks,
                edges,
                traffic_zones,
            )
        return LoadedMapData(
            map_dir=self.map_dir,
            map_metadata=map_metadata,
            landmarks=landmarks,
            edges=edges,
            traffic_zones=traffic_zones,
        )

    def read_yaml(self, path: Path) -> object:
        return yaml.safe_load(path.read_text(encoding="utf-8"))

    def find_ros_map_yaml(self) -> Path:
        candidates = sorted(
            path
            for path in self.map_dir.glob("*.yaml")
            if path.name not in MAP_SUPPORT_YAML_FILES
        )
        if not candidates:
            raise FileNotFoundError(f"No ROS map yaml found in {self.map_dir}")
        return candidates[0]

    def load_pgm(self, path: Path) -> tuple[int, int, bytes]:
        image = PgmImage.read(path)
        return image.width, image.height, image.pixels

    # Compatibility aliases for older callers. New code should use the
    # public methods above instead of reaching into loader internals.
    def _read_yaml(self, path: Path) -> object:
        return self.read_yaml(path)

    def _find_ros_map_yaml(self) -> Path:
        return self.find_ros_map_yaml()

    def _load_pgm(self, path: Path) -> tuple[int, int, bytes]:
        return self.load_pgm(path)

    def _build_grayscale_png(self, width: int, height: int, pixels: bytes) -> bytes:
        if len(pixels) != width * height:
            raise ValueError("Pixel buffer size does not match image dimensions.")

        def chunk(kind: bytes, payload: bytes) -> bytes:
            crc = zlib.crc32(kind + payload) & 0xFFFFFFFF
            return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", crc)

        rows = []
        row_size = width
        for y in range(height):
            start = y * row_size
            rows.append(b"\x00" + pixels[start : start + row_size])

        compressed = zlib.compress(b"".join(rows), level=9)
        png = bytearray()
        png.extend(b"\x89PNG\r\n\x1a\n")
        png.extend(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)))
        png.extend(chunk(b"IDAT", compressed))
        png.extend(chunk(b"IEND", b""))
        return bytes(png)

    def _coordinate_frame_is_map_top_left(self, payload: object) -> bool:
        return (
            isinstance(payload, dict)
            and str(payload.get("coordinateFrame") or payload.get("coordinate_frame") or "").strip() == "map_top_left"
        )

    def _map_point_from_payload(
        self,
        point: object,
        map_metadata: MapMetadata,
        *,
        already_map_frame: bool,
    ) -> WorldPoint | None:
        if not isinstance(point, dict):
            return None
        try:
            raw = WorldPoint(x=float(point["x"]), y=float(point["y"]))
        except (KeyError, TypeError, ValueError):
            return None
        return raw if already_map_frame else map_metadata.ros_to_map_point(raw)

    def _load_landmarks(self, path: Path, map_metadata: MapMetadata) -> dict[str, Landmark]:
        payload = self._read_yaml(path)
        if not isinstance(payload, dict) or "LMs" not in payload:
            raise ValueError(f"Unexpected LM file format: {path}")
        already_map_frame = self._coordinate_frame_is_map_top_left(payload)

        landmarks: dict[str, Landmark] = {}
        for item in payload["LMs"]:
            point = self._map_point_from_payload(item, map_metadata, already_map_frame=already_map_frame)
            if point is None:
                continue
            landmark = Landmark(
                name=str(item["name"]),
                x=point.x,
                y=point.y,
                properties=dict(item.get("properties") or {}),
                ignore_dir=item.get("ignoreDir"),
            )
            landmarks[landmark.name] = landmark
        return landmarks

    def _load_traffic_zones(
        self,
        path: Path,
        map_metadata: MapMetadata,
    ) -> list[TrafficZone]:
        if not path.exists():
            return []
        payload = self._read_yaml(path)
        if not isinstance(payload, dict):
            raise ValueError(f"Unexpected traffic zone file format: {path}")
        raw_zones = payload.get("zones") or payload.get("trafficZones") or []
        if not isinstance(raw_zones, list):
            raise ValueError(f"traffic zone list is invalid: {path}")
        already_map_frame = self._coordinate_frame_is_map_top_left(payload)
        zones: list[TrafficZone] = []
        seen: set[str] = set()
        for index, item in enumerate(raw_zones):
            if not isinstance(item, dict):
                continue
            zone_id = str(item.get("id") or item.get("zoneId") or f"corridor:{index + 1}").strip()
            if not zone_id or zone_id in seen:
                raise ValueError(f"duplicate or empty traffic zone id: {zone_id!r}")
            shape = str(item.get("shape") or "rectangle").strip().lower()
            if shape != "rectangle":
                continue
            bounds = item.get("bounds")
            if not isinstance(bounds, dict):
                continue
            try:
                first = WorldPoint(
                    x=float(bounds.get("minX", bounds.get("min_x"))),
                    y=float(bounds.get("minY", bounds.get("min_y"))),
                )
                second = WorldPoint(
                    x=float(bounds.get("maxX", bounds.get("max_x"))),
                    y=float(bounds.get("maxY", bounds.get("max_y"))),
                )
            except (TypeError, ValueError):
                continue
            if not already_map_frame:
                first = map_metadata.ros_to_map_point(first)
                second = map_metadata.ros_to_map_point(second)
            seen.add(zone_id)
            zones.append(
                TrafficZone(
                    zone_id=zone_id,
                    kind=str(item.get("kind") or "controlled_corridor"),
                    min_x=min(first.x, second.x),
                    min_y=min(first.y, second.y),
                    max_x=max(first.x, second.x),
                    max_y=max(first.y, second.y),
                    capacity=max(1, int(item.get("capacity") or 1)),
                    properties=dict(item.get("properties") or {}),
                )
            )
        return zones

    def _load_edges(
        self,
        path: Path,
        landmarks: dict[str, Landmark],
        map_metadata: MapMetadata,
    ) -> list[GraphEdge]:
        payload = self._read_yaml(path)
        if not isinstance(payload, list):
            raise ValueError(f"Unexpected edge file format: {path}")

        geometries, primitive_edges, primitive_properties = self._load_graph_geometries(
            path.parent / "graphs.yaml",
            landmarks,
            map_metadata,
        )
        edges: list[GraphEdge] = []
        for item in payload:
            start = str(item["from"])
            goal = str(item["to"])
            if start not in landmarks or goal not in landmarks:
                continue
            if primitive_edges and (start, goal) not in primitive_edges:
                continue

            # ``graphs.yaml`` owns geometry while
            # ``graph_edges_lengths.yaml`` owns the routable edge record.
            # Editors normally keep their metadata identical, but generated
            # and imported maps can legitimately add traffic properties (for
            # example ``controlled_region``) only to the latter.  Choosing
            # one non-empty mapping discarded those properties silently.
            # Merge both representations and retain the historical primitive
            # precedence when the same key exists in both files.
            properties = {
                **dict(item.get("properties") or {}),
                **dict(primitive_properties.get((start, goal)) or {}),
            }
            edges.append(
                GraphEdge(
                    from_name=start,
                    to_name=goal,
                    length=float(item["length"]),
                    kind=str(item.get("kind", "unknown")),
                    edge_type=str(item.get("type", "unknown")),
                    world_points=(landmarks[start].to_point(), landmarks[goal].to_point()),
                    geometry=geometries.get((start, goal)),
                    properties=properties,
                )
            )
        return edges

    def _load_graph_geometries(
        self,
        path: Path,
        landmarks: dict[str, Landmark],
        map_metadata: MapMetadata,
    ) -> tuple[
        dict[tuple[str, str], EdgeGeometry],
        set[tuple[str, str]],
        dict[tuple[str, str], dict[str, object]],
    ]:
        if not path.exists():
            return {}, set(), {}

        payload = self._read_yaml(path)
        if not isinstance(payload, dict):
            return {}, set(), {}
        already_map_frame = self._coordinate_frame_is_map_top_left(payload)

        primitives = payload.get("primitives", [])
        if not isinstance(primitives, list):
            return {}, set(), {}

        geometries: dict[tuple[str, str], EdgeGeometry] = {}
        primitive_edges: set[tuple[str, str]] = set()
        primitive_properties: dict[tuple[str, str], dict[str, object]] = {}
        for primitive in primitives:
            if not isinstance(primitive, dict):
                continue

            kind = str(primitive.get("kind", ""))
            endpoint_payload = primitive.get("curve") if kind == "curve" else primitive
            if not isinstance(endpoint_payload, dict):
                continue

            start_name = endpoint_payload.get("start_name")
            end_name = endpoint_payload.get("end_name")
            start_point = self._map_point_from_payload(
                endpoint_payload.get("start"),
                map_metadata,
                already_map_frame=already_map_frame,
            )
            end_point = self._map_point_from_payload(
                endpoint_payload.get("end"),
                map_metadata,
                already_map_frame=already_map_frame,
            )
            key = self._primitive_edge_key(
                start_name=start_name,
                end_name=end_name,
                start_point=start_point,
                end_point=end_point,
                landmarks=landmarks,
            )
            if key is None:
                continue
            primitive_edges.add(key)
            primitive_properties[key] = dict(primitive.get("properties") or {})

            if kind != "curve":
                continue

            point_keys = ("start", "control1", "control2", "end")
            try:
                control_points = tuple(
                    self._map_point_from_payload(
                        endpoint_payload[key],
                        map_metadata,
                        already_map_frame=already_map_frame,
                    )
                    for key in point_keys
                )
            except KeyError:
                continue
            if any(point is None for point in control_points):
                continue

            geometry = EdgeGeometry(
                geometry="bezier",
                control_points=control_points,
                curve_type=str(primitive.get("curve_type", "Bezier")),
            )
            geometries[key] = geometry

        return geometries, primitive_edges, primitive_properties

    def _primitive_edge_key(
        self,
        start_name: object,
        end_name: object,
        start_point: WorldPoint | None,
        end_point: WorldPoint | None,
        landmarks: dict[str, Landmark],
    ) -> tuple[str, str] | None:
        start_key = str(start_name) if start_name is not None else ""
        end_key = str(end_name) if end_name is not None else ""
        start = start_key if start_key in landmarks else ""
        end = end_key if end_key in landmarks else ""
        if not start:
            start = self._nearest_landmark_name(start_point, landmarks)
        if not end:
            end = self._nearest_landmark_name(end_point, landmarks)
        if not start or not end or start == end:
            return None
        return start, end

    def _nearest_landmark_name(
        self,
        point: WorldPoint | None,
        landmarks: dict[str, Landmark],
        max_radius_m: float = 0.75,
    ) -> str:
        if point is None:
            return ""
        best = min(
            landmarks.values(),
            key=lambda landmark: ((landmark.x - point.x) ** 2) + ((landmark.y - point.y) ** 2),
            default=None,
        )
        if best is None:
            return ""
        distance = ((best.x - point.x) ** 2 + (best.y - point.y) ** 2) ** 0.5
        return best.name if distance <= max_radius_m else ""
