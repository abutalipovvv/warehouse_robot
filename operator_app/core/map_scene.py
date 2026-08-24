"""Build the static 3D scene payload for an editable warehouse map."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import yaml

from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader
from fleet_manager.core.mapping.maps.models import LoadedMapData


MAP_SUPPORT_YAML_FILES = {
    "LMs.yaml",
    "graphs.yaml",
    "graph_edges_lengths.yaml",
    "traffic_zones.yaml",
}

WALL_STRIDE_STEPS = (1, 2, 3, 4, 6, 8, 12, 16, 24, 32)


@dataclass(slots=True)
class MapSceneBuilder:
    """Cache static map geometry and merge occupied pixels into wall boxes."""

    loaded_map: LoadedMapData
    maximum_wall_rectangles: int = 1500
    target_wall_grid_cells: int = 180_000
    _cached_payload: dict[str, Any] | None = None
    _cached_light_payload: dict[str, Any] | None = None

    @property
    def map_dir(self) -> Path:
        return self.loaded_map.map_dir.resolve()

    def build(
        self,
        *,
        wall_height: float = 1.8,
        include_walls: bool = True,
    ) -> dict[str, Any]:
        cached = (
            self._cached_payload
            if include_walls
            else self._cached_light_payload
        )
        if cached is not None:
            return cached

        metadata = self.loaded_map.map_metadata
        payload = {
            "ok": True,
            "mapName": self.map_dir.stem.replace(".smap", ""),
            "mapDir": str(self.map_dir),
            "coordinateFrame": "map_top_left",
            "floor": {
                "width": metadata.width * metadata.resolution,
                "depth": metadata.height * metadata.resolution,
                "resolution": metadata.resolution,
                "imageDataUrl": metadata.image_data_url,
            },
            "bounds": {
                "minX": 0.0,
                "minZ": 0.0,
                "maxX": metadata.width * metadata.resolution,
                "maxZ": metadata.height * metadata.resolution,
            },
            "walls": (
                self.wall_rectangles(wall_height=wall_height)
                if include_walls
                else []
            ),
            "wallHeight": wall_height,
            "lms": [
                self.loaded_map.landmarks[name].to_dict()
                for name in sorted(self.loaded_map.landmarks)
            ],
            "edges": [edge.to_dict() for edge in self.loaded_map.edges],
        }
        if include_walls:
            self._cached_payload = payload
        else:
            self._cached_light_payload = payload
        return payload

    def wall_rectangles(
        self,
        *,
        wall_height: float,
    ) -> list[dict[str, Any]]:
        ros_map_yaml = self.find_ros_map_yaml()
        ros_map = yaml.safe_load(ros_map_yaml.read_text(encoding="utf-8"))
        if not isinstance(ros_map, dict):
            raise ValueError(
                f"Unexpected ROS map file format: {ros_map_yaml}"
            )
        image_path = (
            self.map_dir / str(ros_map["image"])
        ).resolve()
        width, height, pixels = WarehouseMapLoader(
            self.map_dir
        ).load_pgm(image_path)
        occupied_threshold = float(
            ros_map.get("occupied_thresh", 0.65) or 0.65
        )
        negate = int(ros_map.get("negate", 0) or 0)
        # Starting at full PGM resolution is needlessly expensive for large
        # maps: the 3D walls are visualization geometry, not navigation
        # collision data.  Select the first stride from the raster area and
        # only increase it if the merged instance budget is still exceeded.
        # At 0.02 m/px, stride 3 preserves a 0.06 m visual resolution while
        # reducing the 22.05.26 map from 9,565 boxes to about 1,400.
        target_cells = max(1, int(self.target_wall_grid_cells))
        initial_stride = max(
            1,
            math.ceil(math.sqrt((width * height) / target_cells)),
        )
        strides = [initial_stride]
        strides.extend(
            stride
            for stride in WALL_STRIDE_STEPS
            if stride > initial_stride
        )
        if strides[-1] < WALL_STRIDE_STEPS[-1]:
            strides.append(WALL_STRIDE_STEPS[-1])

        for index, stride in enumerate(strides):
            rectangles = self.merge_wall_rectangles(
                width,
                height,
                pixels,
                occupied_threshold=occupied_threshold,
                negate=negate,
                stride=stride,
                wall_height=wall_height,
            )
            if (
                len(rectangles) <= self.maximum_wall_rectangles
                or index == len(strides) - 1
            ):
                return rectangles
        return []

    def merge_wall_rectangles(
        self,
        width: int,
        height: int,
        pixels: bytes,
        *,
        occupied_threshold: float,
        negate: int,
        stride: int,
        wall_height: float,
    ) -> list[dict[str, Any]]:
        """Merge equal horizontal runs across rows into large wall boxes."""

        grid_width = math.ceil(width / stride)
        grid_height = math.ceil(height / stride)
        active: dict[tuple[int, int], dict[str, int]] = {}
        completed: list[dict[str, int]] = []

        def block_is_occupied(cell_x: int, cell_y: int) -> bool:
            start_x = cell_x * stride
            start_y = cell_y * stride
            end_x = min(width, start_x + stride)
            end_y = min(height, start_y + stride)
            for pixel_y in range(start_y, end_y):
                row_offset = pixel_y * width
                for pixel_x in range(start_x, end_x):
                    value = pixels[row_offset + pixel_x]
                    occupancy = (
                        value / 255.0
                        if negate
                        else (255 - value) / 255.0
                    )
                    if occupancy > occupied_threshold:
                        return True
            return False

        for cell_y in range(grid_height):
            runs = horizontal_runs(
                grid_width,
                lambda cell_x: block_is_occupied(cell_x, cell_y),
            )
            next_active: dict[tuple[int, int], dict[str, int]] = {}
            for start_x, run_width in runs:
                key = (start_x, run_width)
                rectangle = active.pop(key, None)
                if rectangle is None:
                    rectangle = {
                        "x": start_x,
                        "y": cell_y,
                        "w": run_width,
                        "h": 1,
                    }
                else:
                    rectangle["h"] += 1
                next_active[key] = rectangle
            completed.extend(active.values())
            active = next_active
        completed.extend(active.values())

        resolution = float(self.loaded_map.map_metadata.resolution)
        return [
            rectangle_payload(
                rectangle,
                width=width,
                height=height,
                stride=stride,
                resolution=resolution,
                wall_height=wall_height,
            )
            for rectangle in completed
        ]

    def find_ros_map_yaml(self) -> Path:
        candidates = sorted(
            path
            for path in self.map_dir.glob("*.yaml")
            if path.name not in MAP_SUPPORT_YAML_FILES
        )
        if not candidates:
            raise FileNotFoundError(
                f"No ROS map yaml found in {self.map_dir}"
            )
        return candidates[0]


def horizontal_runs(
    width: int,
    occupied_at,
) -> list[tuple[int, int]]:
    """Return ``(start, length)`` runs for one boolean raster row."""

    runs: list[tuple[int, int]] = []
    run_start: int | None = None
    for x in range(width):
        if occupied_at(x):
            if run_start is None:
                run_start = x
        elif run_start is not None:
            runs.append((run_start, x - run_start))
            run_start = None
    if run_start is not None:
        runs.append((run_start, width - run_start))
    return runs


def rectangle_payload(
    rectangle: dict[str, int],
    *,
    width: int,
    height: int,
    stride: int,
    resolution: float,
    wall_height: float,
) -> dict[str, Any]:
    pixel_x = rectangle["x"] * stride
    pixel_y = rectangle["y"] * stride
    pixel_width = min(width - pixel_x, rectangle["w"] * stride)
    pixel_height = min(height - pixel_y, rectangle["h"] * stride)
    return {
        "x": round((pixel_x + (pixel_width / 2.0)) * resolution, 4),
        "z": round((pixel_y + (pixel_height / 2.0)) * resolution, 4),
        "width": round(pixel_width * resolution, 4),
        "depth": round(pixel_height * resolution, 4),
        "height": wall_height,
        "stride": stride,
    }
