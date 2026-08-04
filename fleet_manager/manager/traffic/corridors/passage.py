"""Controlled-corridor passage geometry and live occupancy."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

from fleet_manager.robot.model import FleetRobot
from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorOccupancy,
    CorridorResourceWindow,
    CorridorSlot,
)


@dataclass(slots=True)
class _CorridorPassageScan:
    """Mutable result accumulated while scanning one committed trajectory."""

    entry: dict[str, Any] | None = None
    regions: list[str] = field(default_factory=list)
    resource_windows: dict[str, dict[str, Any]] = field(
        default_factory=dict
    )
    occupied_passage_ended: bool = False


_LiveOccupancyProjection = tuple[
    float,
    tuple[str, ...],
    str,
    str,
    str,
    float,
    dict[str, CorridorResourceWindow],
    float,
    float,
]


class ControlledCorridorPassageMixin:
    """Describe authored corridor passages from route and footprint state."""

    def _controlled_corridor_param(self, key: str, default: float) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        return self._positive_float_param(fleet, key, default)

    def _controlled_corridor_pose_is_at_lm(
        self,
        pose: dict[str, Any] | None,
        lm_name: str,
    ) -> bool:
        if pose is None:
            return False
        landmark = self.landmarks.get(lm_name)
        if landmark is None:
            return False
        # Admission ownership is stricter than the normal replanning
        # tolerance. A robot 5-10 cm beyond the entry LM is already inside and
        # must keep the token until its complete footprint reaches the exit.
        return math.hypot(
            landmark.x - float(pose.get("x", 0.0) or 0.0),
            landmark.y - float(pose.get("y", 0.0) or 0.0),
        ) <= 0.03

    def _controlled_regions_for_robot(self, robot: FleetRobot) -> set[str]:
        graph = self._controlled_corridor_graph
        if graph is None:
            return set()
        regions: set[str] = set()
        current_lm = self._traffic_lm_for_robot(robot)
        vertex = graph.vertices.get(current_lm)
        if vertex is not None:
            regions.update(vertex.controlled_region_ids)

        if not robot.trajectory:
            return regions
        edge = self._parse_edge_id(
            self._edge_id_at_trajectory(robot.trajectory, robot.route_clock)
        )
        if edge is None:
            return regions
        segment_index = self._trajectory_segment_index(
            robot.trajectory,
            robot.route_clock,
            boundary_belongs_to_previous=True,
        )
        segment_start = float(
            robot.trajectory[segment_index].get("t", 0.0) or 0.0
        )
        segment_end = float(
            robot.trajectory[segment_index + 1].get("t", segment_start)
            or segment_start
        )
        src, dst = edge
        lane_lookup = getattr(graph, "lane_for", None)
        if not callable(lane_lookup):
            # Lightweight test/integration graph adapters may expose only
            # vertex annotations. The vertex region is still authoritative;
            # lane occupancy simply cannot add anything in that adapter.
            return regions
        lane = lane_lookup(src, dst)
        if lane is None:
            return regions
        src_vertex = graph.vertices.get(src)
        dst_vertex = graph.vertices.get(dst)
        for region_id in lane.controlled_region_ids:
            at_entry_boundary = (
                (
                    robot.route_clock <= segment_start + 0.000001
                    or self._controlled_corridor_pose_is_at_lm(
                        robot.pose,
                        src,
                    )
                )
                and (
                    src_vertex is None
                    or region_id not in src_vertex.controlled_region_ids
                )
            )
            at_exit_boundary = (
                (
                    robot.route_clock >= segment_end - 0.000001
                    or self._controlled_corridor_pose_is_at_lm(
                        robot.pose,
                        dst,
                    )
                )
                and (
                    dst_vertex is None
                    or region_id not in dst_vertex.controlled_region_ids
                )
            )
            if not at_entry_boundary and not at_exit_boundary:
                regions.add(region_id)
        return regions

    def _controlled_regions_intersecting_footprint(
        self,
        robot: FleetRobot,
    ) -> set[str]:
        """Return authored regions touched by the complete physical body."""
        bounds_by_region = getattr(
            self,
            "_controlled_corridor_region_bounds",
            {},
        )
        route_regions = self._controlled_regions_for_robot(robot)
        if robot.pose is None or not bounds_by_region:
            return route_regions
        corners = self.collision.footprint_corners(robot.pose)
        if not corners:
            return route_regions
        min_x = min(float(point["x"]) for point in corners)
        max_x = max(float(point["x"]) for point in corners)
        min_y = min(float(point["y"]) for point in corners)
        max_y = max(float(point["y"]) for point in corners)
        geometric_regions = {
            str(region_id)
            for region_id, bounds in bounds_by_region.items()
            if (
                max_x >= float(bounds[0])
                and min_x <= float(bounds[2])
                and max_y >= float(bounds[1])
                and min_y <= float(bounds[3])
            )
        }
        # Auto-detected corridors have no authored rectangle to intersect;
        # their ``A<=>B`` region spans the complete inferred lane chain.
        # Explicit edge-only regions can likewise lack annotated vertices.
        geometric_regions.update(
            region_id
            for region_id in route_regions
            if (
                region_id not in bounds_by_region
                or "<=>" in region_id
            )
        )
        return geometric_regions

    def _controlled_corridor_staging_lm(
        self,
        robot: FleetRobot,
        *,
        portal_lm: str,
        portal_clock: float,
    ) -> tuple[str, float]:
        """Return the closest upstream LM that leaves the portal clear.

        The graph endpoint immediately outside a controlled corridor is also
        the exit point for traffic travelling in the opposite direction.  A
        red light on that endpoint can therefore prevent the current owner
        from completing its turn out of the corridor.  Walk backwards along
        the committed trajectory and place the stop line at the first safe LM
        whose centre is outside the robot/robot broadphase around the portal.

        Maps with no earlier graph LM keep the portal as a compatibility
        fallback.  They remain capacity safe, but cannot provide the extra
        exit pocket without an upstream waiting point in the graph.
        """
        graph = self._controlled_corridor_graph
        portal = self.landmarks.get(portal_lm)
        if graph is None or portal is None:
            return portal_lm, portal_clock

        clearance = max(0.0, self.collision.robot_broadphase_distance())
        fallback = (portal_lm, portal_clock)
        seen: set[str] = set()
        for sample in reversed(robot.trajectory):
            sample_clock = float(sample.get("t", 0.0) or 0.0)
            if sample_clock > portal_clock + 0.000001:
                continue
            lm_name = str(sample.get("lm") or "").strip()
            if not lm_name or lm_name in seen:
                continue
            seen.add(lm_name)
            vertex = graph.vertices.get(lm_name)
            landmark = self.landmarks.get(lm_name)
            edge = self._parse_edge_id(str(sample.get("edgeId") or ""))
            incoming_lane = graph.lane_for(*edge) if edge is not None else None
            crosses_controlled_resource = bool(
                (vertex is not None and vertex.controlled_region_ids)
                or (
                    incoming_lane is not None
                    and incoming_lane.controlled_region_ids
                )
            )
            if (
                vertex is None
                or landmark is None
                or not vertex.can_wait
                or vertex.controlled_region_ids
            ):
                if crosses_controlled_resource:
                    break
                continue
            fallback = (lm_name, sample_clock)
            if crosses_controlled_resource:
                # This is the graph-safe exit of a preceding narrow passage.
                # A stop line before that passage is not reachable from the
                # current approach leg and can already lie behind the robot.
                break
            distance = math.hypot(
                float(landmark.x) - float(portal.x),
                float(landmark.y) - float(portal.y),
            )
            if distance + 0.000001 >= clearance:
                return lm_name, sample_clock
        return fallback

    def _next_controlled_corridor_entry(
        self,
        robot: FleetRobot,
        *,
        lookahead_sec: float | None = None,
    ) -> dict[str, Any] | None:
        """Return the complete no-wait passage approaching ``robot``.

        Geometric corridor rectangles may touch through an edge whose two
        endpoints are both non-waitable.  A per-rectangle traffic light is
        unsafe there: after entering region A the robot can be denied region B
        with nowhere legal to stop.  Admission is therefore computed from the
        last external safe LM through the first following external safe LM and
        contains every controlled resource crossed in between.
        """
        graph = self._controlled_corridor_graph
        if graph is None or len(robot.trajectory) < 2:
            if self.robots.get(robot.name) is robot:
                self._controlled_corridor_entry_cache.pop(
                    robot.name,
                    None,
                )
            return None
        lookahead = (
            self._controlled_corridor_entry_lookahead()
            if lookahead_sec is None
            else max(0.0, float(lookahead_sec))
        )
        cache_name, cache_key = self._controlled_corridor_entry_cache_key(
            robot,
            lookahead,
        )
        if cache_name:
            cached = self._controlled_corridor_entry_cache.get(cache_name)
            if (
                cached is not None
                and cached[0] is robot.trajectory
                and cached[1] == cache_key
            ):
                return cached[2]

        inside = self._controlled_regions_for_robot(robot)
        scan = self._scan_controlled_corridor_passage(
            robot,
            lookahead=lookahead,
            inside_regions=inside,
        )
        entry = (
            None
            if scan.occupied_passage_ended
            else self._finalize_controlled_corridor_entry(robot, scan)
        )
        if cache_name:
            self._controlled_corridor_entry_cache[cache_name] = (
                robot.trajectory,
                cache_key,
                entry,
            )
        return entry

    def _controlled_corridor_entry_cache_key(
        self,
        robot: FleetRobot,
        lookahead: float,
    ) -> tuple[str, tuple[Any, ...]]:
        cache_name = (
            robot.name
            if self.robots.get(robot.name) is robot
            else ""
        )
        pose = robot.pose if isinstance(robot.pose, dict) else {}
        return cache_name, (
            int(robot.route_revision),
            len(robot.trajectory),
            float(robot.route_clock),
            str(robot.current_lm or ""),
            float(pose.get("x", 0.0) or 0.0),
            float(pose.get("y", 0.0) or 0.0),
            float(pose.get("yaw", 0.0) or 0.0),
            float(lookahead),
        )

    def _scan_controlled_corridor_passage(
        self,
        robot: FleetRobot,
        *,
        lookahead: float,
        inside_regions: set[str],
    ) -> _CorridorPassageScan:
        """Locate a no-wait passage and collect its resource windows."""

        graph = self._controlled_corridor_graph
        scan = _CorridorPassageScan()
        if graph is None:
            return scan
        first_index = max(
            0,
            self._trajectory_segment_index(
                robot.trajectory,
                robot.route_clock,
                boundary_belongs_to_previous=True,
            ) - 1,
        )
        for index in range(first_index, len(robot.trajectory) - 1):
            start = robot.trajectory[index]
            end = robot.trajectory[index + 1]
            start_time = float(start.get("t", 0.0) or 0.0)
            end_time = float(end.get("t", start_time) or start_time)
            if end_time + 0.000001 < robot.route_clock:
                continue
            eta = max(0.0, start_time - robot.route_clock)
            if scan.entry is None and eta > lookahead + 0.000001:
                break
            edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
            edge = self._parse_edge_id(edge_id)
            if edge is None:
                continue
            src, dst = edge
            lane = graph.lane_for(src, dst)
            if lane is None:
                continue
            # A rendered/commanded trajectory contains several interpolated
            # samples with the same ``src->dst`` edge id.  The edge's graph
            # destination is reached only by the sample explicitly tagged with
            # that LM; treating the first interpolation sample as arrival made
            # the central calendar release a corridor almost an edge too early.
            reaches_dst_lm = str(end.get("lm") or "").strip() == dst
            src_vertex = graph.vertices.get(src)
            dst_vertex = graph.vertices.get(dst)
            lane_regions = tuple(lane.controlled_region_ids)
            if (
                scan.entry is None
                and inside_regions
                and reaches_dst_lm
                and dst_vertex is not None
                and dst_vertex.can_wait
                and not dst_vertex.controlled_region_ids
            ):
                # The currently occupied passage ends at this graph-safe
                # vertex.  A later route segment may enter the same or another
                # controlled corridor again, but that is a separate admission
                # transaction.  Looking through this safe exit bundled the
                # future re-entry with the passage being cleared and could
                # make its owner wait for its own lease before it had even
                # reached the intervening turn.
                scan.occupied_passage_ended = True
                return scan
            if scan.entry is None:
                new_regions = [
                    region_id
                    for region_id in lane_regions
                    if region_id not in inside_regions
                    and not (
                        self._controlled_corridor_pose_is_at_lm(
                            robot.pose,
                            dst,
                        )
                        and (
                            dst_vertex is None
                            or region_id
                            not in dst_vertex.controlled_region_ids
                        )
                    )
                ]
                if not new_regions:
                    continue
                source_is_safe = bool(
                    src_vertex is not None
                    and src_vertex.can_wait
                    and not src_vertex.controlled_region_ids
                )
                if not source_is_safe:
                    # The robot is already inside an older/legacy passage.
                    # It must keep moving to the next safe exit; creating a
                    # new red light here would stop it in the narrow space.
                    continue
                scan.entry = self._controlled_corridor_entry_from_segment(
                    robot,
                    src=src,
                    dst=dst,
                    start_time=start_time,
                    end_time=end_time,
                    eta=eta,
                    first_region=new_regions[0],
                )
            if self._record_controlled_corridor_segment(
                scan,
                src=src,
                dst=dst,
                start_time=start_time,
                end_time=end_time,
                lane_regions=lane_regions,
                dst_vertex=dst_vertex,
                reaches_dst_lm=reaches_dst_lm,
            ):
                break
        return scan

    def _controlled_corridor_entry_from_segment(
        self,
        robot: FleetRobot,
        *,
        src: str,
        dst: str,
        start_time: float,
        end_time: float,
        eta: float,
        first_region: str,
    ) -> dict[str, Any]:
        staging_lm, staging_clock = self._controlled_corridor_staging_lm(
            robot,
            portal_lm=src,
            portal_clock=start_time,
        )
        at_staging = self._controlled_corridor_pose_is_at_lm(
            robot.pose,
            staging_lm,
        )
        return {
            "region": first_region,
            "src": src,
            "dst": dst,
            "holding_lm": staging_lm,
            "staging_clock": staging_clock,
            "exit_lm": "",
            "eta": eta,
            "entry_clock": start_time,
            "exit_clock": end_time,
            "at_boundary": self._controlled_corridor_pose_is_at_lm(
                robot.pose,
                src,
            ),
            "at_staging": at_staging,
            "passed_staging": bool(
                robot.route_clock > staging_clock + 0.000001
                and not at_staging
            ),
        }

    def _record_controlled_corridor_segment(
        self,
        scan: _CorridorPassageScan,
        *,
        src: str,
        dst: str,
        start_time: float,
        end_time: float,
        lane_regions: tuple[str, ...],
        dst_vertex: Any,
        reaches_dst_lm: bool,
    ) -> bool:
        """Merge one trajectory segment into the current passage."""

        local_direction = self._controlled_corridor_lane_direction(src, dst)
        segment_regions = tuple(
            dict.fromkeys(
                (
                    *lane_regions,
                    *(
                        dst_vertex.controlled_region_ids
                        if dst_vertex is not None
                        else ()
                    ),
                )
            )
        )
        for region_id in segment_regions:
            window = scan.resource_windows.get(region_id)
            if window is None:
                scan.resource_windows[region_id] = {
                    "entry_clock": start_time,
                    "exit_clock": end_time,
                    "direction": local_direction,
                    "directions": [local_direction],
                }
                continue
            window["entry_clock"] = min(
                float(window["entry_clock"]),
                start_time,
            )
            window["exit_clock"] = max(
                float(window["exit_clock"]),
                end_time,
            )
            directions = window.setdefault(
                "directions",
                [str(window["direction"])],
            )
            if not directions or directions[-1] != local_direction:
                directions.append(local_direction)
            window["direction"] = (
                str(directions[0])
                if len(directions) == 1
                else "flow:path:"
                + ">".join(
                    str(direction).removeprefix("flow:")
                    for direction in directions
                )
            )

        for region_id in lane_regions:
            if region_id not in scan.regions:
                scan.regions.append(region_id)
        if dst_vertex is None:
            return False
        for region_id in dst_vertex.controlled_region_ids:
            if region_id not in scan.regions:
                scan.regions.append(region_id)
        if (
            not reaches_dst_lm
            or not dst_vertex.can_wait
            or dst_vertex.controlled_region_ids
        ):
            return False
        assert scan.entry is not None
        scan.entry["exit_lm"] = dst
        scan.entry["exit_clock"] = end_time
        scan.entry["direction"] = f"{scan.entry.get('src') or src}->{dst}"
        return True

    def _finalize_controlled_corridor_entry(
        self,
        robot: FleetRobot,
        scan: _CorridorPassageScan,
    ) -> dict[str, Any] | None:
        entry = scan.entry
        if entry is None or not scan.regions:
            return None
        entry["regions"] = tuple(scan.regions)
        entry["passage"] = "|".join(scan.regions)
        staging_clock = float(
            entry.get("staging_clock", entry.get("entry_clock", 0.0))
            or 0.0
        )
        passage_duration = max(
            self._runtime_motion_step(),
            float(entry.get("exit_clock", staging_clock) or staging_clock)
            - staging_clock,
        )
        exit_clock = float(
            entry.get("exit_clock", staging_clock) or staging_clock
        )
        passage_samples = [
            sample
            for sample in robot.trajectory
            if (
                isinstance(sample, dict)
                and staging_clock - 0.000001
                <= float(sample.get("t", 0.0) or 0.0)
                <= exit_clock + 0.000001
            )
        ]
        entry["no_wait_lms"] = tuple(
            dict.fromkeys(
                lm_name
                for sample in passage_samples
                if (
                    float(sample.get("t", 0.0) or 0.0)
                    > staging_clock + 0.000001
                    and float(sample.get("t", 0.0) or 0.0)
                    < exit_clock - 0.000001
                    and (lm_name := str(sample.get("lm") or "").strip())
                )
            )
        )
        entry["has_wait_after_staging"] = self._passage_has_wait(
            passage_samples,
            staging_clock,
        )
        entry["resource_windows"] = tuple(
            CorridorResourceWindow(
                region_id=region_id,
                entry_offset_sec=max(
                    0.0,
                    float(
                        scan.resource_windows[region_id]["entry_clock"]
                    )
                    - staging_clock,
                ),
                exit_offset_sec=min(
                    passage_duration,
                    max(
                        self._runtime_motion_step(),
                        float(
                            scan.resource_windows[region_id]["exit_clock"]
                        )
                        - staging_clock,
                    ),
                ),
                direction=str(
                    scan.resource_windows[region_id]["direction"]
                ),
            )
            for region_id in scan.regions
            if region_id in scan.resource_windows
        )
        return entry

    @staticmethod
    def _passage_has_wait(
        samples: list[dict[str, Any]],
        staging_clock: float,
    ) -> bool:
        return any(
            (
                float(second.get("t", 0.0) or 0.0)
                > float(first.get("t", 0.0) or 0.0) + 0.000001
                and float(first.get("t", 0.0) or 0.0)
                > staging_clock + 0.000001
                and math.hypot(
                    float(second.get("x", 0.0) or 0.0)
                    - float(first.get("x", 0.0) or 0.0),
                    float(second.get("y", 0.0) or 0.0)
                    - float(first.get("y", 0.0) or 0.0),
                )
                <= 0.000001
                and abs(
                    math.atan2(
                        math.sin(
                            float(second.get("yaw", 0.0) or 0.0)
                            - float(first.get("yaw", 0.0) or 0.0)
                        ),
                        math.cos(
                            float(second.get("yaw", 0.0) or 0.0)
                            - float(first.get("yaw", 0.0) or 0.0)
                        ),
                    )
                )
                <= 0.000001
            )
            for first, second in zip(
                samples,
                samples[1:],
            )
        )

    @staticmethod
    def _controlled_corridor_entry_regions(
        entry: dict[str, Any] | None,
    ) -> tuple[str, ...]:
        if not isinstance(entry, dict):
            return ()
        raw = entry.get("regions")
        if isinstance(raw, (list, tuple)):
            regions = tuple(str(item) for item in raw if str(item))
            if regions:
                return tuple(dict.fromkeys(regions))
        region = str(entry.get("region") or "")
        return (region,) if region else ()

    def _controlled_corridor_queue_predecessor(
        self,
        robot: FleetRobot,
        entry: dict[str, Any],
        regions: tuple[str, ...],
        direction: str,
    ) -> str:
        """Return a same-direction robot physically ahead at this portal."""
        dependency_name = (
            str(robot.wait_for_robot or "").strip()
            or self._robot_name_from_conflict_reason(robot.last_reason)
        )
        if not dependency_name or dependency_name == robot.name:
            return ""
        dependency = self.robots.get(dependency_name)
        if (
            dependency is None
            or not dependency.trajectory
            or dependency.pose is None
            or robot.pose is None
        ):
            return ""
        dependency_entry = self._next_controlled_corridor_entry(
            dependency
        )
        if dependency_entry is None:
            return ""
        dependency_regions = set(
            self._controlled_corridor_entry_regions(dependency_entry)
        )
        dependency_direction = self._controlled_corridor_flow_direction(
            dependency_entry
        )
        portal_lm = str(entry.get("src") or "")
        if (
            not set(regions).intersection(dependency_regions)
            or dependency_direction != direction
            or str(dependency_entry.get("src") or "") != portal_lm
        ):
            return ""
        portal = self.landmarks.get(portal_lm)
        if portal is None:
            return ""
        robot_distance = math.hypot(
            float(robot.pose.get("x", 0.0)) - float(portal.x),
            float(robot.pose.get("y", 0.0)) - float(portal.y),
        )
        dependency_distance = math.hypot(
            float(dependency.pose.get("x", 0.0)) - float(portal.x),
            float(dependency.pose.get("y", 0.0)) - float(portal.y),
        )
        return (
            dependency.name
            if dependency_distance + 0.001 < robot_distance
            else ""
        )

    def _controlled_corridor_flow_direction(
        self,
        entry: dict[str, Any],
    ) -> str:
        """Return a stable flow phase shared by equal travel directions."""
        resource_windows = entry.get("resource_windows")
        if isinstance(resource_windows, (list, tuple)):
            first_window = next(
                (
                    window
                    for window in resource_windows
                    if isinstance(window, CorridorResourceWindow)
                ),
                None,
            )
            if first_window is not None:
                return first_window.direction
        src_name = str(entry.get("src") or "")
        dst_name = str(entry.get("exit_lm") or "")
        fallback = str(
            entry.get("direction")
            or f"{src_name}->{dst_name}"
        )
        return self._controlled_corridor_lane_direction(
            src_name,
            dst_name,
            fallback=fallback,
        )

    def _controlled_corridor_lane_direction(
        self,
        src_name: str,
        dst_name: str,
        *,
        fallback: str = "",
    ) -> str:
        """Quantize one local corridor resource traversal direction."""
        src = self.landmarks.get(str(src_name or ""))
        dst = self.landmarks.get(str(dst_name or ""))
        if src is None or dst is None:
            return str(fallback or f"{src_name}->{dst_name}")
        dx = float(dst.x) - float(src.x)
        dy = float(dst.y) - float(src.y)
        if abs(dx) <= 0.000001 and abs(dy) <= 0.000001:
            return str(fallback or f"{src.name}->{dst.name}")
        if abs(dx) >= abs(dy) * 2.0:
            return "flow:east" if dx > 0.0 else "flow:west"
        if abs(dy) >= abs(dx) * 2.0:
            return "flow:south" if dy > 0.0 else "flow:north"
        horizontal = "east" if dx > 0.0 else "west"
        vertical = "south" if dy > 0.0 else "north"
        return f"flow:{vertical}-{horizontal}"

    def _controlled_corridor_live_occupancy(
        self,
        robot: FleetRobot,
        *,
        physical_regions: set[str],
        previous_slot: CorridorSlot | None,
        entry: dict[str, Any],
        previous_passage: dict[str, Any],
        now: float,
    ) -> CorridorOccupancy:
        """Project one physical owner from immutable trajectory-clock data.

        Calendar slots are outputs of admission control. Their duration and
        offsets may move when a robot is delayed, so feeding those values back
        into trajectory time creates an accumulating positive feedback loop.
        ``route_resource_windows`` is the immutable template captured when the
        passage was first discovered. Every runtime tick rebases that template
        from the robot's current route clock.
        """
        projection = self._live_occupancy_projection(
            robot,
            physical_regions=physical_regions,
            previous_slot=previous_slot,
            entry=entry,
            previous_passage=previous_passage,
            now=now,
        )
        (
            _,
            _,
            exit_lm,
            staging_lm,
            direction,
            _,
            _,
            _,
            _,
        ) = projection
        occupancy_windows = self._project_live_occupancy_windows(
            projection,
            physical_regions=physical_regions,
            previous_slot=previous_slot,
        )
        expected_exit = now + max(
            window.exit_offset_sec
            for window in occupancy_windows
        )
        return CorridorOccupancy(
            robot_id=robot.name,
            regions=tuple(
                window.region_id
                for window in occupancy_windows
            ),
            direction=direction,
            entered_at=now,
            expected_exit_time=expected_exit,
            exit_lm=exit_lm,
            route_revision=int(robot.route_revision),
            staging_lm=staging_lm,
            resource_windows=occupancy_windows,
        )

    def _live_occupancy_projection(
        self,
        robot: FleetRobot,
        *,
        physical_regions: set[str],
        previous_slot: CorridorSlot | None,
        entry: dict[str, Any],
        previous_passage: dict[str, Any],
        now: float,
    ) -> _LiveOccupancyProjection:
        """Resolve stable identity, timing template and fallback horizon."""
        motion_step = max(
            self._runtime_motion_step(),
            self._controlled_corridor_param(
                "controlled_corridor_occupancy_recheck_sec",
                0.1,
            ),
        )
        current_lm = self._traffic_lm_for_robot(robot)
        candidate_regions = tuple(dict.fromkeys(
            (
                *(
                    previous_slot.regions
                    if previous_slot is not None
                    else tuple(
                        str(item)
                        for item in previous_passage.get("regions", ())
                        if str(item)
                    )
                ),
                *sorted(physical_regions),
            )
        ))
        exit_lm = str(
            (previous_slot.exit_lm if previous_slot is not None else "")
            or entry.get("exit_lm")
            or previous_passage.get("exit_lm")
            or current_lm
        )
        staging_lm = str(
            (previous_slot.staging_lm if previous_slot is not None else "")
            or entry.get("holding_lm")
            or previous_passage.get("staging_lm")
            or current_lm
        )
        direction = str(
            (previous_slot.direction if previous_slot is not None else "")
            or (
                self._controlled_corridor_flow_direction(entry)
                if entry
                else ""
            )
            or previous_passage.get("direction")
            or f"occupied:{robot.name}"
        )
        route_staging_clock = float(
            entry.get(
                "staging_clock",
                previous_passage.get(
                    "staging_clock",
                    robot.route_clock,
                ),
            )
            or 0.0
        )
        route_templates = tuple(
            window
            for window in (
                entry.get("resource_windows", ())
                or previous_passage.get("route_resource_windows", ())
                or (
                    previous_slot.resource_windows
                    if (
                        previous_slot is not None
                        and not previous_slot.physically_observed
                    )
                    else ()
                )
            )
            if isinstance(window, CorridorResourceWindow)
        )
        route_clock = float(robot.route_clock)
        fallback_remaining = max(
            motion_step,
            self._controlled_corridor_physical_exit_time(
                robot,
                exit_lm,
                now,
            )
            - now,
        )
        return (
            motion_step,
            candidate_regions,
            exit_lm,
            staging_lm,
            direction,
            route_staging_clock,
            {
                window.region_id: window
                for window in route_templates
            },
            route_clock,
            fallback_remaining,
        )

    @staticmethod
    def _project_live_occupancy_windows(
        projection: _LiveOccupancyProjection,
        *,
        physical_regions: set[str],
        previous_slot: CorridorSlot | None,
    ) -> tuple[CorridorResourceWindow, ...]:
        """Rebase immutable route windows onto the robot's live clock."""
        (
            motion_step,
            candidate_regions,
            _,
            _,
            direction,
            route_staging_clock,
            templates_by_region,
            route_clock,
            fallback_remaining,
        ) = projection
        live_windows: list[CorridorResourceWindow] = []
        for region_id in candidate_regions:
            template = templates_by_region.get(region_id)
            if template is None:
                if (
                    region_id not in physical_regions
                    and previous_slot is None
                ):
                    continue
                entry_offset = 0.0
                exit_offset = fallback_remaining
                local_direction = direction
            else:
                route_entry = (
                    route_staging_clock
                    + template.entry_offset_sec
                )
                route_exit = (
                    route_staging_clock
                    + template.exit_offset_sec
                )
                if (
                    region_id not in physical_regions
                    and route_exit <= route_clock + 0.000001
                ):
                    # This resource is behind the complete footprint. Do not
                    # repaint a released corridor red while the owner advances
                    # through a later rectangle in the same atomic passage.
                    continue
                entry_offset = (
                    0.0
                    if region_id in physical_regions
                    else max(
                        0.0,
                        route_entry - route_clock,
                    )
                )
                exit_offset = max(
                    entry_offset + motion_step,
                    motion_step,
                    route_exit - route_clock,
                )
                local_direction = template.direction
            live_windows.append(
                CorridorResourceWindow(
                    region_id=region_id,
                    entry_offset_sec=entry_offset,
                    exit_offset_sec=exit_offset,
                    direction=local_direction,
                )
            )

        # ``physical_regions`` is non-empty for callers, but a malformed
        # legacy passage may have no template. Preserve physical truth with a
        # bounded recheck window instead of returning an invalid empty claim.
        covered = {window.region_id for window in live_windows}
        for region_id in sorted(physical_regions - covered):
            live_windows.append(
                CorridorResourceWindow(
                    region_id=region_id,
                    entry_offset_sec=0.0,
                    exit_offset_sec=fallback_remaining,
                    direction=direction,
                )
            )
        return tuple(live_windows)
