"""Footprint, obstacle and occupancy collision geometry."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader
from fleet_manager.core.mapping.maps.models import MapMetadata
from fleet_manager.core.math.geometry import Pose2D, Vector2
from fleet_manager.core.math.polygons import Polygon2D, distance_to_segment


class FleetCollisionChecker:
    def __init__(
        self,
        params: dict[str, Any],
        map_dir: Path | None,
        map_metadata: MapMetadata | None,
    ) -> None:
        self.params = params
        self.map_metadata = map_metadata
        self.map_pixels: bytes | None = None
        self.map_width = 0
        self.map_height = 0
        self._footprint_cache: list[dict[str, float]] | None = None
        self._footprint_polygon_cache: Polygon2D | None = None
        self._footprint_diameter_cache: float | None = None
        self._pose_sample_local_points_cache: list[dict[str, float]] | None = None
        self._rotation_clear_cache: dict[
            tuple[float, float, float, float],
            bool,
        ] = {}
        self._full_rotation_clear_cache: dict[tuple[float, float], bool] = {}
        if map_dir is not None:
            self._load_map_pixels(map_dir)

    def set_params(self, params: dict[str, Any]) -> None:
        self.params = params
        self._footprint_cache = None
        self._footprint_polygon_cache = None
        self._footprint_diameter_cache = None
        self._pose_sample_local_points_cache = None
        self._rotation_clear_cache.clear()
        self._full_rotation_clear_cache.clear()

    def lookahead_time(self) -> float:
        navigation = self._dict_param("navigation")
        fleet = self._dict_param("fleet")
        speed = max(0.05, float(navigation.get("route_speed", 0.35) or 0.35))
        stop_distance = max(0.08, float(navigation.get("stop_distance", 0.40) or 0.40))
        footprint_lookahead = max(
            stop_distance,
            float(navigation.get("footprint_lookahead", 0.80) or 0.80),
        )
        try:
            configured = float(fleet.get("runtime_collision_lookahead_sec") or 0.0)
        except (TypeError, ValueError):
            configured = 0.0
        if configured > 0.0:
            return max(0.25, min(4.0, configured))

        # Auto mode covers both the configured spatial preview and a full
        # braking interval. At warehouse speed this is intentionally much
        # earlier than the final footprint-overlap guard.
        try:
            acceleration = max(
                0.10,
                float(navigation.get("route_acceleration", 0.60) or 0.60),
            )
        except (TypeError, ValueError):
            acceleration = 0.60
        braking_time = speed / acceleration
        distance_time = footprint_lookahead / speed
        reaction_time = max(
            0.05,
            float(fleet.get("continuous_collision_step_sec", 0.10) or 0.10),
        )
        return max(0.35, min(3.0, max(braking_time, distance_time) + reaction_time))

    def sample_time_step(self) -> float:
        manual = self._dict_param("manual")
        return max(0.04, min(0.14, float(manual.get("prediction_step", 0.10) or 0.10)))

    def blocked_reason(
        self,
        pose: dict[str, float],
        obstacles: list[dict[str, float]],
        obstacle_areas: list[dict[str, float]],
    ) -> str:
        points = self.pose_sample_points(pose)
        for point in points:
            if self.map_occupied(point):
                return "map occupancy under footprint"
            for area in obstacle_areas:
                if self.area_contains(area, point):
                    return "obstacle area under footprint"
        for obstacle in obstacles:
            if self.obstacle_hits_pose(obstacle, pose):
                return "point obstacle hits footprint"
        return ""

    def dynamic_blocked_reason(
        self,
        pose: dict[str, float],
        obstacles: list[dict[str, float]],
        obstacle_areas: list[dict[str, float]],
    ) -> str:
        points = self.pose_sample_points(pose)
        for point in points:
            for area in obstacle_areas:
                if self.area_contains(area, point):
                    return "obstacle area under footprint"
        for obstacle in obstacles:
            if self.obstacle_hits_pose(obstacle, pose):
                return "point obstacle hits footprint"
        return ""

    def rotation_is_clear(
        self,
        pose: dict[str, float],
        target_yaw: float,
        *,
        angular_step: float = math.radians(2.0),
    ) -> bool:
        """Return whether an in-place turn clears the static occupancy map.

        Checking only both endpoint orientations misses the swept corners of
        a rectangular robot. Runtime collision checks would then stop an
        already-started turn and leave the robot blocking an aisle. Sampling
        the same shortest angular path before planning prevents that state.
        """

        if self.map_pixels is None or self.map_metadata is None:
            return True
        x = float(pose.get("x", 0.0) or 0.0)
        y = float(pose.get("y", 0.0) or 0.0)
        start_yaw = float(pose.get("yaw", 0.0) or 0.0)
        target_yaw = float(target_yaw or 0.0)
        delta = math.atan2(
            math.sin(target_yaw - start_yaw),
            math.cos(target_yaw - start_yaw),
        )
        if abs(delta) < math.radians(2.0):
            return True

        key = (
            round(x, 5),
            round(y, 5),
            round(start_yaw, 5),
            round(start_yaw + delta, 5),
        )
        cached = self._rotation_clear_cache.get(key)
        if cached is not None:
            return cached

        full_rotation_clear = self._full_rotation_is_clear(x, y)
        if full_rotation_clear:
            self._rotation_clear_cache[key] = True
            return True
        # A quarter-turn sweeps practically the complete circumscribed body
        # envelope on the orthogonal warehouse graph. The disk test is
        # conservative for the small footprint asymmetry and much cheaper
        # than transforming hundreds of footprint samples at every angle.
        if abs(delta) >= math.radians(89.0):
            self._rotation_clear_cache[key] = False
            return False

        step = max(math.radians(0.5), float(angular_step))
        sample_count = max(1, int(math.ceil(abs(delta) / step)))
        clear = True
        for index in range(sample_count + 1):
            yaw = start_yaw + (delta * index / sample_count)
            if self.blocked_reason({"x": x, "y": y, "yaw": yaw}, [], []):
                clear = False
                break
        self._rotation_clear_cache[key] = clear
        return clear

    def _full_rotation_is_clear(self, x: float, y: float) -> bool:
        if (
            self.map_pixels is None
            or self.map_metadata is None
            or self.map_width <= 0
            or self.map_height <= 0
        ):
            return False
        key = (round(x, 5), round(y, 5))
        cached = self._full_rotation_clear_cache.get(key)
        if cached is not None:
            return cached

        resolution = max(0.001, float(self.map_metadata.resolution))
        radius = max(
            (
                math.hypot(float(point["x"]), float(point["y"]))
                for point in self.footprint()
            ),
            default=0.22,
        ) + self.collision_margin()
        center_x = round(x / resolution)
        center_y = round(y / resolution)
        pixel_radius = int(math.ceil(radius / resolution))
        radius_sq = (radius + (resolution * 0.5)) ** 2
        clear = True
        for pixel_y in range(center_y - pixel_radius, center_y + pixel_radius + 1):
            world_y = pixel_y * resolution
            for pixel_x in range(center_x - pixel_radius, center_x + pixel_radius + 1):
                world_x = pixel_x * resolution
                if ((world_x - x) ** 2) + ((world_y - y) ** 2) > radius_sq:
                    continue
                if (
                    pixel_x < 0
                    or pixel_y < 0
                    or pixel_x >= self.map_width
                    or pixel_y >= self.map_height
                    or self.map_pixels[(pixel_y * self.map_width) + pixel_x] < 82
                ):
                    clear = False
                    break
            if not clear:
                break
        self._full_rotation_clear_cache[key] = clear
        return clear

    def footprints_overlap(self, first_pose: dict[str, float], second_pose: dict[str, float]) -> bool:
        margin = self.collision_margin()
        if not self._footprint_bounds_may_overlap(first_pose, second_pose, margin):
            return False
        footprint = self._footprint_polygon()
        return footprint.overlaps_positioned(
            float(first_pose.get("x", 0.0) or 0.0),
            float(first_pose.get("y", 0.0) or 0.0),
            float(first_pose.get("yaw", 0.0) or 0.0),
            footprint,
            float(second_pose.get("x", 0.0) or 0.0),
            float(second_pose.get("y", 0.0) or 0.0),
            float(second_pose.get("yaw", 0.0) or 0.0),
            margin=margin,
        )

    def robot_footprints_conflict(self, first_pose: dict[str, float], second_pose: dict[str, float]) -> bool:
        margin = self.robot_collision_margin()
        if not self._footprint_bounds_may_overlap(first_pose, second_pose, margin):
            return False
        footprint = self._footprint_polygon()
        return footprint.overlaps_positioned(
            float(first_pose.get("x", 0.0) or 0.0),
            float(first_pose.get("y", 0.0) or 0.0),
            float(first_pose.get("yaw", 0.0) or 0.0),
            footprint,
            float(second_pose.get("x", 0.0) or 0.0),
            float(second_pose.get("y", 0.0) or 0.0),
            float(second_pose.get("yaw", 0.0) or 0.0),
            margin=margin,
        )

    def _footprint_bounds_may_overlap(
        self,
        first_pose: dict[str, float],
        second_pose: dict[str, float],
        margin: float,
    ) -> bool:
        dx = float(first_pose.get("x", 0.0) or 0.0) - float(
            second_pose.get("x", 0.0) or 0.0
        )
        dy = float(first_pose.get("y", 0.0) or 0.0) - float(
            second_pose.get("y", 0.0) or 0.0
        )
        threshold = self._footprint_diameter() + max(0.0, margin)
        return (dx * dx) + (dy * dy) <= threshold * threshold

    def robot_broadphase_distance(self) -> float:
        return self._footprint_diameter() + self.robot_collision_margin()

    def physical_broadphase_distance(self) -> float:
        return self._footprint_diameter() + self.collision_margin()

    def _footprint_diameter(self) -> float:
        if self._footprint_diameter_cache is not None:
            return self._footprint_diameter_cache
        radius = max(
            (point.length for point in self._footprint_polygon().points),
            default=0.22,
        )
        self._footprint_diameter_cache = radius * 2.0
        return self._footprint_diameter_cache

    def pose_sample_points(self, pose: dict[str, float]) -> list[dict[str, float]]:
        return self._local_points_to_world(pose, self._pose_sample_local_points())

    def _pose_sample_local_points(self) -> list[dict[str, float]]:
        if self._pose_sample_local_points_cache is not None:
            return self._pose_sample_local_points_cache
        footprint = self.footprint()
        margin = self.collision_margin()
        min_x = min(point["x"] for point in footprint) - margin
        max_x = max(point["x"] for point in footprint) + margin
        min_y = min(point["y"] for point in footprint) - margin
        max_y = max(point["y"] for point in footprint) + margin
        resolution = self.map_metadata.resolution if self.map_metadata is not None else 0.02
        step = max(0.04, resolution * 2.0)
        points: list[dict[str, float]] = []
        x = min_x
        while x <= max_x + 0.000001:
            y = min_y
            while y <= max_y + 0.000001:
                local = {"x": x, "y": y}
                if self.point_in_polygon(local, footprint) or self.distance_to_polygon(local, footprint) <= margin:
                    points.append(local)
                y += step
            x += step
        points.append({"x": 0.0, "y": 0.0})
        for point in footprint:
            points.append(point)
        self._pose_sample_local_points_cache = points
        return self._pose_sample_local_points_cache

    def obstacle_hits_pose(self, obstacle: dict[str, float], pose: dict[str, float]) -> bool:
        local = self.world_to_local(pose, obstacle)
        radius = float(obstacle.get("radius", 0.08) or 0.08) + self.collision_margin()
        footprint = self.footprint()
        return self.point_in_polygon(local, footprint) or self.distance_to_polygon(local, footprint) <= radius

    def map_occupied(self, point: dict[str, float]) -> bool:
        if self.map_pixels is None or self.map_metadata is None:
            return False
        pixel = self.world_to_image(point)
        if pixel["x"] < 0 or pixel["y"] < 0 or pixel["x"] >= self.map_width or pixel["y"] >= self.map_height:
            return True
        value = self.map_pixels[(pixel["y"] * self.map_width) + pixel["x"]]
        return value < 82

    def area_contains(self, area: dict[str, float], point: dict[str, float]) -> bool:
        x1 = min(area["x1"], area["x2"])
        x2 = max(area["x1"], area["x2"])
        y1 = min(area["y1"], area["y2"])
        y2 = max(area["y1"], area["y2"])
        return x1 <= point["x"] <= x2 and y1 <= point["y"] <= y2

    def footprint_corners(self, pose: dict[str, float]) -> list[dict[str, float]]:
        return [
            self._point_payload(point)
            for point in self._footprint_at(pose).points
        ]

    @staticmethod
    def _local_points_to_world(
        pose: dict[str, float],
        points: list[dict[str, float]],
    ) -> list[dict[str, float]]:
        world_pose = FleetCollisionChecker._pose(pose)
        return [
            FleetCollisionChecker._point_payload(
                world_pose.transform_local_vector(
                    FleetCollisionChecker._point(point)
                )
            )
            for point in points
        ]

    def local_to_world(self, pose: dict[str, float], point: dict[str, float]) -> dict[str, float]:
        return self._point_payload(
            self._pose(pose).transform_local_vector(self._point(point))
        )

    def world_to_local(self, pose: dict[str, float], point: dict[str, float]) -> dict[str, float]:
        return self._point_payload(
            self._pose(pose).relative_vector_to(self._point(point))
        )

    def world_to_image(self, point: dict[str, float]) -> dict[str, int]:
        assert self.map_metadata is not None
        resolution = self.map_metadata.resolution
        return {
            "x": round(point["x"] / resolution),
            "y": round(point["y"] / resolution),
        }

    def polygons_overlap(
        self,
        first: list[dict[str, float]],
        second: list[dict[str, float]],
        margin: float,
    ) -> bool:
        return self._polygon(first).overlaps(
            self._polygon(second),
            margin=margin,
        )

    def polygon_axes(self, polygon: list[dict[str, float]]) -> list[dict[str, float]]:
        return [
            self._point_payload(axis)
            for axis in self._polygon(polygon).axes()
        ]

    def project_polygon(self, polygon: list[dict[str, float]], axis: dict[str, float]) -> dict[str, float]:
        projection = self._polygon(polygon).project(self._point(axis))
        return {"min": projection.minimum, "max": projection.maximum}

    def point_in_polygon(self, point: dict[str, float], polygon: list[dict[str, float]]) -> bool:
        return self._polygon(polygon).contains(self._point(point))

    def distance_to_polygon(self, point: dict[str, float], polygon: list[dict[str, float]]) -> float:
        return self._polygon(polygon).distance_to(self._point(point))

    def distance_to_segment(
        self,
        point: dict[str, float],
        start: dict[str, float],
        end: dict[str, float],
    ) -> float:
        return distance_to_segment(
            self._point(point),
            self._point(start),
            self._point(end),
        )

    def footprint(self) -> list[dict[str, float]]:
        if self._footprint_cache is not None:
            return self._footprint_cache
        robot_model = self._dict_param("robot_model")
        raw_footprint = robot_model.get("footprint")
        if not isinstance(raw_footprint, list) or len(raw_footprint) < 3:
            raw_footprint = [
                {"x": 0.220000, "y": 0.000000},
                {"x": 0.203253, "y": 0.084190},
                {"x": 0.155563, "y": 0.155563},
                {"x": 0.084190, "y": 0.203253},
                {"x": 0.000000, "y": 0.220000},
                {"x": -0.084190, "y": 0.203253},
                {"x": -0.155563, "y": 0.155563},
                {"x": -0.203253, "y": 0.084190},
                {"x": -0.220000, "y": 0.000000},
                {"x": -0.203253, "y": -0.084190},
                {"x": -0.155563, "y": -0.155563},
                {"x": -0.084190, "y": -0.203253},
                {"x": 0.000000, "y": -0.220000},
                {"x": 0.084190, "y": -0.203253},
                {"x": 0.155563, "y": -0.155563},
                {"x": 0.203253, "y": -0.084190},
            ]
        self._footprint_cache = [
            {
                "x": float(point.get("x", 0.0) or 0.0),
                "y": float(point.get("y", 0.0) or 0.0),
            }
            for point in raw_footprint
            if isinstance(point, dict)
        ]
        return self._footprint_cache

    def _footprint_polygon(self) -> Polygon2D:
        if self._footprint_polygon_cache is None:
            self._footprint_polygon_cache = self._polygon(self.footprint())
        return self._footprint_polygon_cache

    def _footprint_at(self, pose: dict[str, float]) -> Polygon2D:
        return self._footprint_polygon().transformed(self._pose(pose))

    @staticmethod
    def _pose(payload: dict[str, float]) -> Pose2D:
        return Pose2D.from_xy(
            float(payload.get("x", 0.0) or 0.0),
            float(payload.get("y", 0.0) or 0.0),
            float(payload.get("yaw", 0.0) or 0.0),
        )

    @staticmethod
    def _point(payload: dict[str, float]) -> Vector2:
        return Vector2(float(payload["x"]), float(payload["y"]))

    @classmethod
    def _polygon(cls, payload: list[dict[str, float]]) -> Polygon2D:
        return Polygon2D(cls._point(point) for point in payload)

    @staticmethod
    def _point_payload(point: Vector2) -> dict[str, float]:
        return {"x": point.x, "y": point.y}

    def collision_margin(self) -> float:
        navigation = self._dict_param("navigation")
        return max(0.0, float(navigation.get("collision_margin", 0.04) or 0.04))

    def robot_collision_margin(self) -> float:
        fleet = self._dict_param("fleet")
        try:
            configured = fleet.get("robot_clearance_m", 0.35)
            clearance = 0.35 if configured is None else float(configured)
        except (TypeError, ValueError):
            clearance = 0.35
        return self.collision_margin() + max(0.0, clearance)

    def _dict_param(self, key: str) -> dict[str, Any]:
        value = self.params.get(key, {})
        return value if isinstance(value, dict) else {}

    def _load_map_pixels(self, map_dir: Path) -> None:
        try:
            loader = WarehouseMapLoader(map_dir)
            ros_map_yaml = loader.find_ros_map_yaml()
            ros_map = loader.read_yaml(ros_map_yaml)
            if not isinstance(ros_map, dict):
                return
            image_path = (map_dir / str(ros_map["image"])).resolve()
            width, height, pixels = loader.load_pgm(image_path)
        except Exception:
            return
        self.map_width = width
        self.map_height = height
        self.map_pixels = pixels
