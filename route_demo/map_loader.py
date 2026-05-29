from __future__ import annotations

import base64
import struct
import zlib
from pathlib import Path

import yaml

from .models import EdgeGeometry, GraphEdge, Landmark, LoadedMapData, MapMetadata, WorldPoint


class WarehouseMapLoader:
    def __init__(self, map_dir: Path) -> None:
        self.map_dir = map_dir.resolve()

    def load(self) -> LoadedMapData:
        if not self.map_dir.is_dir():
            raise FileNotFoundError(f"Map directory does not exist: {self.map_dir}")

        ros_map_yaml = self._find_ros_map_yaml()
        ros_map = self._read_yaml(ros_map_yaml)
        if not isinstance(ros_map, dict):
            raise ValueError(f"Unexpected ROS map file format: {ros_map_yaml}")

        image_path = (self.map_dir / str(ros_map["image"])).resolve()
        width, height, pixels = self._load_pgm(image_path)
        png_bytes = self._build_grayscale_png(width, height, pixels)
        image_png_base64 = base64.b64encode(png_bytes).decode("ascii")

        landmarks = self._load_landmarks(self.map_dir / "LMs.yaml")
        edges = self._load_edges(self.map_dir / "graph_edges_lengths.yaml", landmarks)
        map_name = str(ros_map.get("image", image_path.stem)).replace(".pgm", "")
        map_metadata = MapMetadata(
            map_name=map_name,
            width=width,
            height=height,
            resolution=float(ros_map["resolution"]),
            origin=ros_map["origin"],
            image_data_url=f"data:image/png;base64,{image_png_base64}",
        )
        return LoadedMapData(
            map_dir=self.map_dir,
            map_metadata=map_metadata,
            landmarks=landmarks,
            edges=edges,
        )

    def _read_yaml(self, path: Path) -> object:
        return yaml.safe_load(path.read_text(encoding="utf-8"))

    def _find_ros_map_yaml(self) -> Path:
        candidates = sorted(
            path
            for path in self.map_dir.glob("*.yaml")
            if path.name not in {"LMs.yaml", "graphs.yaml", "graph_edges_lengths.yaml"}
        )
        if not candidates:
            raise FileNotFoundError(f"No ROS map yaml found in {self.map_dir}")
        return candidates[0]

    def _read_pgm_token(self, data: bytes, index: int) -> tuple[bytes, int]:
        length = len(data)
        while index < length:
            byte = data[index]
            if byte == 35:
                while index < length and data[index] not in (10, 13):
                    index += 1
            elif chr(byte).isspace():
                index += 1
            else:
                break

        start = index
        while index < length and not chr(data[index]).isspace():
            index += 1

        return data[start:index], index

    def _load_pgm(self, path: Path) -> tuple[int, int, bytes]:
        raw = path.read_bytes()
        magic, index = self._read_pgm_token(raw, 0)
        if magic not in {b"P5", b"P2"}:
            raise ValueError(f"Unsupported PGM format in {path}: {magic!r}")

        width_token, index = self._read_pgm_token(raw, index)
        height_token, index = self._read_pgm_token(raw, index)
        max_value_token, index = self._read_pgm_token(raw, index)
        width = int(width_token)
        height = int(height_token)
        max_value = int(max_value_token)

        while index < len(raw) and chr(raw[index]).isspace():
            index += 1

        if magic == b"P5":
            if max_value > 255:
                raise ValueError("Only 8-bit binary PGM files are supported.")
            pixels = raw[index : index + (width * height)]
            if len(pixels) != width * height:
                raise ValueError("PGM pixel data is shorter than expected.")
            return width, height, pixels

        text_values = raw[index:].split()
        if len(text_values) < width * height:
            raise ValueError("PGM ascii pixel data is shorter than expected.")

        scale = 255 / max_value if max_value else 1.0
        pixels = bytes(int(round(int(token) * scale)) for token in text_values[: width * height])
        return width, height, pixels

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

    def _load_landmarks(self, path: Path) -> dict[str, Landmark]:
        payload = self._read_yaml(path)
        if not isinstance(payload, dict) or "LMs" not in payload:
            raise ValueError(f"Unexpected LM file format: {path}")

        landmarks: dict[str, Landmark] = {}
        for item in payload["LMs"]:
            landmark = Landmark(
                name=str(item["name"]),
                x=float(item["x"]),
                y=float(item["y"]),
            )
            landmarks[landmark.name] = landmark
        return landmarks

    def _load_edges(
        self,
        path: Path,
        landmarks: dict[str, Landmark],
    ) -> list[GraphEdge]:
        payload = self._read_yaml(path)
        if not isinstance(payload, list):
            raise ValueError(f"Unexpected edge file format: {path}")

        geometries = self._load_graph_geometries(path.parent / "graphs.yaml")
        edges: list[GraphEdge] = []
        for item in payload:
            start = str(item["from"])
            goal = str(item["to"])
            if start not in landmarks or goal not in landmarks:
                continue

            geometry = geometries.get((start, goal))
            edges.append(
                GraphEdge(
                    from_name=start,
                    to_name=goal,
                    length=float(item["length"]),
                    kind=str(item.get("kind", "unknown")),
                    edge_type=str(item.get("type", "unknown")),
                    world_points=(landmarks[start].to_point(), landmarks[goal].to_point()),
                    geometry=geometry,
                )
            )
        return edges

    def _load_graph_geometries(self, path: Path) -> dict[tuple[str, str], EdgeGeometry]:
        if not path.exists():
            return {}

        payload = self._read_yaml(path)
        if not isinstance(payload, dict):
            return {}

        primitives = payload.get("primitives", [])
        if not isinstance(primitives, list):
            return {}

        geometries: dict[tuple[str, str], EdgeGeometry] = {}
        for primitive in primitives:
            if not isinstance(primitive, dict) or primitive.get("kind") != "curve":
                continue

            curve = primitive.get("curve")
            if not isinstance(curve, dict):
                continue

            start_name = curve.get("start_name")
            end_name = curve.get("end_name")
            if not start_name or not end_name:
                continue

            point_keys = ("start", "control1", "control2", "end")
            try:
                control_points = tuple(
                    WorldPoint(
                        x=float(curve[key]["x"]),
                        y=float(curve[key]["y"]),
                    )
                    for key in point_keys
                )
            except (KeyError, TypeError, ValueError):
                continue

            geometry = EdgeGeometry(
                geometry="bezier",
                control_points=control_points,
                curve_type=str(primitive.get("curve_type", "Bezier")),
            )
            geometries[(str(start_name), str(end_name))] = geometry
            geometries[(str(end_name), str(start_name))] = EdgeGeometry(
                geometry=geometry.geometry,
                control_points=tuple(reversed(control_points)),
                curve_type=geometry.curve_type,
            )

        return geometries
