from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int64]
StringArray = NDArray[np.str_]


class TrajectoryMath:
    """Vectorized geometry used by trajectory planning and control.

    This focused class is the single home for numerical trajectory operations.
    It deliberately does not contain planning policy or mutable controller state.
    """

    @staticmethod
    def normalize_angles(values: Any) -> Any:
        """Normalize a scalar or NumPy array to the [-pi, pi] interval."""

        return np.arctan2(np.sin(values), np.cos(values))

    @staticmethod
    def normalize_angle(value: float) -> float:
        return float(TrajectoryMath.normalize_angles(float(value)))

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return float(np.clip(float(value), float(low), float(high)))

    @staticmethod
    def map_pose_from_odom_anchor(
        map_anchor: Sequence[float],
        odom_anchor: Sequence[float],
        odom_pose: Sequence[float],
    ) -> tuple[float, float, float]:
        """Propagate a map pose with smooth local odometry.

        Warehouse graph coordinates use a downward-positive Y axis, while ROS
        odometry is REP-103 right-handed.  The two small matrices explicitly
        convert the odom displacement through the anchor body frame instead of
        mixing the two yaw conventions in controller code.
        """

        map_value = np.asarray(map_anchor, dtype=np.float64)
        odom_origin = np.asarray(odom_anchor, dtype=np.float64)
        odom_value = np.asarray(odom_pose, dtype=np.float64)
        if (
            map_value.shape != (3,)
            or odom_origin.shape != (3,)
            or odom_value.shape != (3,)
        ):
            raise ValueError("map and odom poses must have shape (3,)")
        if not all(
            np.all(np.isfinite(value))
            for value in (map_value, odom_origin, odom_value)
        ):
            raise ValueError("map and odom poses must contain finite values")

        odom_yaw = float(odom_origin[2])
        odom_to_body = np.asarray(
            (
                (np.cos(odom_yaw), np.sin(odom_yaw)),
                (-np.sin(odom_yaw), np.cos(odom_yaw)),
            ),
            dtype=np.float64,
        )
        forward_left = odom_to_body @ (odom_value[:2] - odom_origin[:2])

        map_yaw = float(map_value[2])
        body_to_map = np.asarray(
            (
                (np.cos(map_yaw), np.sin(map_yaw)),
                (np.sin(map_yaw), -np.cos(map_yaw)),
            ),
            dtype=np.float64,
        )
        map_xy = map_value[:2] + body_to_map @ forward_left
        odom_yaw_delta = TrajectoryMath.normalize_angle(
            float(odom_value[2] - odom_origin[2])
        )
        return (
            float(map_xy[0]),
            float(map_xy[1]),
            TrajectoryMath.normalize_angle(map_yaw - odom_yaw_delta),
        )

    @staticmethod
    def polyline_length(xy: Sequence[Sequence[float]] | FloatArray) -> float:
        points = np.asarray(xy, dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (2,):
            raise ValueError(f"xy must have shape (n, 2), got {points.shape}")
        if points.shape[0] < 2:
            return 0.0
        return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())

    @staticmethod
    def sample_cubic_bezier(
        control_points: Sequence[Sequence[float]],
        steps: int,
    ) -> tuple[FloatArray, FloatArray]:
        """Evaluate cubic Bezier positions and derivatives with matrix products."""

        controls = np.asarray(control_points, dtype=np.float64)
        if controls.shape != (4, 2):
            raise ValueError(
                "cubic Bezier control points must have shape (4, 2), "
                f"got {controls.shape}"
            )

        sample_count = max(2, int(steps)) + 1
        t = np.linspace(0.0, 1.0, sample_count, dtype=np.float64)
        return TrajectoryMath._evaluate_cubic_bezier(controls, t)

    @staticmethod
    def sample_cubic_bezier_by_arc_length(
        control_points: Sequence[Sequence[float]],
        sample_distance: float,
    ) -> tuple[FloatArray, FloatArray]:
        """Sample a cubic Bezier at approximately uniform metric spacing."""

        controls = np.asarray(control_points, dtype=np.float64)
        if controls.shape != (4, 2):
            raise ValueError(
                "cubic Bezier control points must have shape (4, 2), "
                f"got {controls.shape}"
            )
        distance = max(0.001, float(sample_distance))
        control_polygon_length = float(
            np.linalg.norm(np.diff(controls, axis=0), axis=1).sum()
        )
        dense_steps = max(
            64,
            int(math.ceil(control_polygon_length / distance)) * 8,
        )
        dense_t = np.linspace(0.0, 1.0, dense_steps + 1, dtype=np.float64)
        dense_xy, _ = TrajectoryMath._evaluate_cubic_bezier(
            controls,
            dense_t,
        )
        dense_s = TrajectoryMath.path_distances(dense_xy)
        total_length = float(dense_s[-1])
        if total_length <= 1e-12:
            return dense_xy[[0, -1]], np.zeros((2, 2), dtype=np.float64)

        segment_count = max(1, int(math.ceil(total_length / distance)))
        target_s = np.linspace(
            0.0,
            total_length,
            segment_count + 1,
            dtype=np.float64,
        )
        target_t = np.interp(target_s, dense_s, dense_t)
        return TrajectoryMath._evaluate_cubic_bezier(controls, target_t)

    @staticmethod
    def _evaluate_cubic_bezier(
        controls: FloatArray,
        t: FloatArray,
    ) -> tuple[FloatArray, FloatArray]:
        omt = 1.0 - t
        basis = np.column_stack(
            (
                omt**3,
                3.0 * omt * omt * t,
                3.0 * omt * t * t,
                t**3,
            )
        )
        derivative_basis = np.column_stack(
            (
                -3.0 * omt * omt,
                3.0 * omt * omt - 6.0 * omt * t,
                6.0 * omt * t - 3.0 * t * t,
                3.0 * t * t,
            )
        )
        return basis @ controls, derivative_basis @ controls

    @staticmethod
    def path_distances(xy: FloatArray) -> FloatArray:
        distances = np.zeros(xy.shape[0], dtype=np.float64)
        if xy.shape[0] > 1:
            distances[1:] = np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))
        return distances

    @staticmethod
    def path_curvature(
        xy: FloatArray,
        edge_ids: StringArray,
        edge_start_indices: IntArray,
        edge_end_indices: IntArray,
    ) -> FloatArray:
        curvature = np.zeros(xy.shape[0], dtype=np.float64)
        if xy.shape[0] < 3:
            return curvature

        first = xy[1:-1] - xy[:-2]
        second = xy[2:] - xy[1:-1]
        chord = xy[2:] - xy[:-2]
        twice_area = 2.0 * (
            first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]
        )
        denominator = (
            np.linalg.norm(first, axis=1)
            * np.linalg.norm(second, axis=1)
            * np.linalg.norm(chord, axis=1)
        )
        middle_curvature = np.divide(
            twice_area,
            denominator,
            out=np.zeros_like(twice_area),
            where=denominator > 1e-12,
        )
        same_edge = (
            (edge_ids[:-2] == edge_ids[1:-1])
            & (edge_ids[1:-1] == edge_ids[2:])
        )
        curvature[1:-1] = np.where(same_edge, middle_curvature, 0.0)

        edge_starts = np.flatnonzero(
            np.arange(xy.shape[0], dtype=np.int64) == edge_start_indices
        )
        for start in edge_starts.tolist():
            end = int(edge_end_indices[start])
            if end - start < 2:
                continue
            curvature[start] = curvature[start + 1]
            curvature[end] = curvature[end - 1]
        return curvature

    @staticmethod
    def edge_spans(edge_ids: StringArray) -> tuple[IntArray, IntArray]:
        starts = np.flatnonzero(
            np.concatenate(
                (
                    np.asarray((True,), dtype=np.bool_),
                    edge_ids[1:] != edge_ids[:-1],
                )
            )
        ).astype(np.int64)
        ends = np.concatenate(
            (starts[1:] - 1, np.asarray((edge_ids.shape[0] - 1,), dtype=np.int64))
        )
        counts = ends - starts + 1
        return np.repeat(starts, counts), np.repeat(ends, counts)


@dataclass(frozen=True, slots=True)
class PathProjection:
    index: int
    s: float
    x: float
    y: float
    yaw: float
    cross_track: float
    distance: float
    edge_id: str

    def to_dict(self) -> dict[str, float | int | str]:
        return {
            "index": self.index,
            "s": self.s,
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "cross_track": self.cross_track,
            "distance": self.distance,
            "edge_id": self.edge_id,
        }


@dataclass(frozen=True, slots=True)
class TrajectorySample:
    x: float
    y: float
    yaw: float
    s: float
    curvature: float
    edge_id: str
    motion_direction: str
    not_before: float


@dataclass(frozen=True, slots=True)
class TrajectoryArray:
    """Numeric structure-of-arrays representation of a robot trajectory."""

    xy: FloatArray
    yaw: FloatArray
    s: FloatArray
    curvature: FloatArray
    edge_ids: StringArray
    motion_directions: StringArray
    not_before: FloatArray
    edge_start_indices: IntArray
    edge_end_indices: IntArray

    @classmethod
    def from_route_points(cls, points: Sequence[Any]) -> "TrajectoryArray":
        if not points:
            raise ValueError("trajectory must contain at least one point")

        xy = np.asarray([(float(point.x), float(point.y)) for point in points], dtype=np.float64)
        yaw = TrajectoryMath.normalize_angles(
            np.asarray([float(point.yaw) for point in points], dtype=np.float64)
        )
        edge_ids = np.asarray([str(point.edge_id) for point in points], dtype=np.str_)
        motion_directions = np.asarray(
            [str(point.motion_direction or "forward") for point in points],
            dtype=np.str_,
        )
        not_before = np.asarray(
            [float(point.not_before) for point in points],
            dtype=np.float64,
        )
        s = TrajectoryMath.path_distances(xy)
        edge_start_indices, edge_end_indices = TrajectoryMath.edge_spans(edge_ids)
        curvature = TrajectoryMath.path_curvature(
            xy,
            edge_ids,
            edge_start_indices,
            edge_end_indices,
        )

        for array in (
            xy,
            yaw,
            s,
            curvature,
            edge_ids,
            motion_directions,
            not_before,
            edge_start_indices,
            edge_end_indices,
        ):
            array.setflags(write=False)
        return cls(
            xy=xy,
            yaw=yaw,
            s=s,
            curvature=curvature,
            edge_ids=edge_ids,
            motion_directions=motion_directions,
            not_before=not_before,
            edge_start_indices=edge_start_indices,
            edge_end_indices=edge_end_indices,
        )

    @property
    def size(self) -> int:
        return int(self.xy.shape[0])

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def edge_span(self, index: int) -> tuple[int, int]:
        """Return the inclusive point-index span of the contiguous active edge."""

        point_index = max(0, min(self.size - 1, int(index)))
        return (
            int(self.edge_start_indices[point_index]),
            int(self.edge_end_indices[point_index]),
        )

    def max_abs_curvature_between(self, start_s: float, end_s: float) -> float:
        """Return peak sampled curvature inside an arc-length interval."""

        low = float(np.clip(min(start_s, end_s), 0.0, self.length))
        high = float(np.clip(max(start_s, end_s), 0.0, self.length))
        start = max(0, int(np.searchsorted(self.s, low, side="left")) - 1)
        stop = min(self.size, int(np.searchsorted(self.s, high, side="right")) + 1)
        if stop <= start:
            return abs(float(self.curvature[min(start, self.size - 1)]))
        return float(np.max(np.abs(self.curvature[start:stop])))

    def project(
        self,
        pose_xy: Sequence[float],
        hint_index: int = 0,
        *,
        search_back: int = 4,
        search_forward: int = 72,
        fallback_distance: float = 0.75,
    ) -> PathProjection:
        pose = np.asarray(pose_xy, dtype=np.float64)
        if pose.shape != (2,):
            raise ValueError(f"pose_xy must have shape (2,), got {pose.shape}")
        if self.size == 1:
            delta = pose - self.xy[0]
            distance = float(np.linalg.norm(delta))
            return PathProjection(
                index=0,
                s=0.0,
                x=float(self.xy[0, 0]),
                y=float(self.xy[0, 1]),
                yaw=float(self.yaw[0]),
                cross_track=distance,
                distance=distance,
                edge_id=str(self.edge_ids[0]),
            )

        segment_count = self.size - 1
        start = max(0, min(segment_count - 1, int(hint_index) - max(0, int(search_back))))
        stop = min(segment_count, int(hint_index) + max(1, int(search_forward)))
        if stop <= start:
            stop = min(segment_count, start + 1)
        best = self.project_range(pose, start, stop)
        if best is None or abs(best.cross_track) > max(0.0, float(fallback_distance)):
            fallback = self.project_range(pose, 0, segment_count)
            if fallback is not None:
                best = fallback
        if best is None:  # All trajectories with at least two points have a segment.
            raise ValueError("trajectory contains no projectable segment")
        return best

    def project_range(
        self,
        pose_xy: Sequence[float] | FloatArray,
        start_index: int,
        stop_index: int,
    ) -> PathProjection | None:
        """Project onto [start_index, stop_index) segments in one vector operation."""

        if self.size < 2:
            return None
        pose = np.asarray(pose_xy, dtype=np.float64)
        segment_count = self.size - 1
        start = min(max(0, int(start_index)), segment_count - 1)
        stop = min(max(start + 1, int(stop_index)), segment_count)

        first = self.xy[start:stop]
        segments = self.xy[start + 1 : stop + 1] - first
        segment_length_sq = np.einsum("ij,ij->i", segments, segments)
        relative = pose[None, :] - first
        numerator = np.einsum("ij,ij->i", relative, segments)
        ratios = np.divide(
            numerator,
            segment_length_sq,
            out=np.zeros_like(numerator),
            where=segment_length_sq > 1e-12,
        )
        ratios = np.clip(ratios, 0.0, 1.0)
        projected = first + ratios[:, None] * segments
        offsets = pose[None, :] - projected
        distance_sq = np.einsum("ij,ij->i", offsets, offsets)

        local_index = int(np.argmin(distance_sq))
        segment_index = start + local_index
        ratio = float(ratios[local_index])
        yaw_delta = float(TrajectoryMath.normalize_angles(self.yaw[segment_index + 1] - self.yaw[segment_index]))
        projected_yaw = float(TrajectoryMath.normalize_angles(self.yaw[segment_index] + yaw_delta * ratio))
        offset = offsets[local_index]
        cross_track = float(
            (-np.sin(projected_yaw) * offset[0])
            + (np.cos(projected_yaw) * offset[1])
        )
        segment_length = float(np.sqrt(segment_length_sq[local_index]))
        point_index = segment_index if ratio < 0.5 else min(segment_index + 1, self.size - 1)
        first_edge_id = str(self.edge_ids[segment_index])
        second_edge_id = str(self.edge_ids[segment_index + 1])
        edge_id = second_edge_id if ratio > 0.5 and second_edge_id else first_edge_id
        return PathProjection(
            index=point_index,
            s=float(self.s[segment_index] + segment_length * ratio),
            x=float(projected[local_index, 0]),
            y=float(projected[local_index, 1]),
            yaw=projected_yaw,
            cross_track=cross_track,
            distance=float(np.sqrt(distance_sq[local_index])),
            edge_id=edge_id,
        )

    def sample_at(self, target_s: float) -> TrajectorySample:
        target = float(np.clip(target_s, 0.0, self.length))
        if self.size == 1 or target <= 0.0:
            return self._sample_from_index(0, 0.0)
        if target >= self.length:
            return self._sample_from_index(self.size - 1, self.length)

        index = int(np.searchsorted(self.s, target, side="right") - 1)
        index = max(0, min(index, self.size - 2))
        span = float(self.s[index + 1] - self.s[index])
        ratio = 0.0 if span <= 1e-12 else float(np.clip((target - self.s[index]) / span, 0.0, 1.0))
        xy = self.xy[index] + ratio * (self.xy[index + 1] - self.xy[index])
        yaw_delta = float(TrajectoryMath.normalize_angles(self.yaw[index + 1] - self.yaw[index]))
        yaw = float(TrajectoryMath.normalize_angles(self.yaw[index] + yaw_delta * ratio))
        curvature = float(self.curvature[index] + ratio * (self.curvature[index + 1] - self.curvature[index]))
        metadata_index = index + 1 if ratio > 0.5 else index
        edge_id = str(self.edge_ids[metadata_index] or self.edge_ids[index])
        return TrajectorySample(
            x=float(xy[0]),
            y=float(xy[1]),
            yaw=yaw,
            s=target,
            curvature=curvature,
            edge_id=edge_id,
            motion_direction=str(self.motion_directions[metadata_index]),
            not_before=float(self.not_before[metadata_index]),
        )

    def _sample_from_index(self, index: int, target_s: float) -> TrajectorySample:
        return TrajectorySample(
            x=float(self.xy[index, 0]),
            y=float(self.xy[index, 1]),
            yaw=float(self.yaw[index]),
            s=float(target_s),
            curvature=float(self.curvature[index]),
            edge_id=str(self.edge_ids[index]),
            motion_direction=str(self.motion_directions[index]),
            not_before=float(self.not_before[index]),
        )
