"""Traffic-zone demand indexing and phased admission."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from fleet_manager.core.fleet.domain.models import FleetRobot


@dataclass(frozen=True, slots=True)
class _TrafficZonePolicy:
    capacity: int
    batch_size: int
    phase_duration: float
    lease_duration: float
    starvation_after: float


class TrafficZoneAdmissionMixin:
    """Coordinate capacity and direction phases for traffic zones."""

    def _traffic_zone_control_enabled(self) -> bool:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return True
        value = fleet.get("traffic_zone_control_enabled", True)
        if isinstance(value, str):
            return value.strip().lower() not in {"0", "false", "no", "off"}
        return bool(value)

    def _traffic_zone_param(self, key: str, default: float) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        return self._positive_float_param(fleet, key, default)

    def _edge_has_explicit_corridor_authority(self, src: str, dst: str) -> bool:
        """Avoid two independent admission gates on one physical edge.

        Dynamic traffic zones regulate fleet-wide demand. A Traffic Editor
        controlled corridor is the more precise local authority for its tagged
        edges, so the coarse zone light must not choose a different winner on
        the same transition.
        """
        graph = self._controlled_corridor_graph
        if graph is None:
            return False
        lane = graph.lane_for(src, dst)
        return bool(lane is not None and lane.controlled_region_ids)

    def _build_traffic_zone_index(self) -> dict[str, str]:
        if not self._traffic_zone_control_enabled() or not self.landmarks:
            return {}
        zone_size = self._traffic_zone_param("traffic_zone_size_m", 6.0)
        if zone_size <= 0.0:
            return {}
        origin_x = min(float(landmark.x) for landmark in self.landmarks.values())
        origin_y = min(float(landmark.y) for landmark in self.landmarks.values())
        zones: dict[str, str] = {}
        for name, landmark in self.landmarks.items():
            column = int(math.floor((float(landmark.x) - origin_x) / zone_size))
            row = int(math.floor((float(landmark.y) - origin_y) / zone_size))
            zones[name] = f"flow:{column}:{row}"
        return zones

    def _traffic_zone_route_demand(self) -> dict[str, int]:
        owners_by_zone: dict[str, set[str]] = {}
        for robot in self._runtime_robots():
            order = self._active_order_for_robot(robot)
            if order is None or len(order.spatial_route_nodes) < 2:
                continue
            route_nodes = [
                str(node)
                for node in order.spatial_route_nodes
                if str(node) in self._traffic_zone_by_lm
            ]
            current = self._traffic_lm_for_robot(robot)
            if current in route_nodes:
                route_nodes = route_nodes[route_nodes.index(current):]
            for zone_id in {
                self._traffic_zone_by_lm[node]
                for node in route_nodes
                if node in self._traffic_zone_by_lm
            }:
                owners_by_zone.setdefault(zone_id, set()).add(robot.name)
        return {
            zone_id: len(owners)
            for zone_id, owners in owners_by_zone.items()
        }

    def _next_traffic_zone_transition(
        self,
        robot: FleetRobot,
    ) -> tuple[str, str, str, str, float] | None:
        if not robot.trajectory:
            return None
        lookahead = self._traffic_zone_param(
            "traffic_zone_entry_lookahead_sec",
            3.0,
        )
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
            if start_time - robot.route_clock > lookahead + 0.000001:
                break
            edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
            parsed = self._parse_edge_id(edge_id)
            if parsed is None:
                continue
            src, dst = parsed
            if self._edge_has_explicit_corridor_authority(src, dst):
                continue
            src_zone = self._traffic_zone_by_lm.get(src, "")
            dst_zone = self._traffic_zone_by_lm.get(dst, "")
            if not src_zone or not dst_zone or src_zone == dst_zone:
                continue
            first = self.landmarks.get(src)
            second = self.landmarks.get(dst)
            if first is None or second is None:
                continue
            dx = float(second.x) - float(first.x)
            dy = float(second.y) - float(first.y)
            if abs(dx) >= abs(dy):
                phase = "E" if dx >= 0.0 else "W"
            else:
                phase = "S" if dy >= 0.0 else "N"
            return src, dst, dst_zone, phase, max(0.0, start_time - robot.route_clock)
        return None

    def _prepare_traffic_zone_admissions(self, now: float) -> None:
        self._traffic_zone_tick_now = now
        self._traffic_zone_winners = {}
        self._traffic_zone_queues = {}
        if (
            not self._traffic_zone_by_lm
            or not self._traffic_zone_control_enabled()
        ):
            self._traffic_zone_demand = {}
            self._traffic_zone_occupancy = {}
            return

        demand = self._traffic_zone_route_demand()
        self._traffic_zone_demand = demand
        threshold = max(
            1,
            int(self._traffic_zone_param("traffic_zone_demand_threshold", 6.0)),
        )
        hot_zones = {
            zone_id
            for zone_id, value in demand.items()
            if value >= threshold
        }
        occupancy_owners = self._traffic_zone_occupancy_owners()
        self._traffic_zone_occupancy = {
            zone_id: len(owners)
            for zone_id, owners in occupancy_owners.items()
        }
        self._expire_traffic_zone_leases(now)
        candidates_by_zone, candidate_keys = (
            self._collect_traffic_zone_candidates(
                now,
                demand=demand,
                hot_zones=hot_zones,
            )
        )
        for key in list(self._traffic_zone_wait_since):
            if key not in candidate_keys:
                self._traffic_zone_wait_since.pop(key, None)

        policy = self._traffic_zone_policy()
        for zone_id, candidates in candidates_by_zone.items():
            self._schedule_traffic_zone(
                zone_id,
                candidates,
                occupancy_owners=occupancy_owners,
                policy=policy,
                now=now,
            )

    def _traffic_zone_occupancy_owners(
        self,
    ) -> dict[str, set[str]]:
        occupancy_owners: dict[str, set[str]] = {}
        for robot in self._runtime_robots():
            current_lm = self._traffic_lm_for_robot(robot)
            zone_id = self._traffic_zone_by_lm.get(current_lm, "")
            if zone_id:
                occupancy_owners.setdefault(zone_id, set()).add(robot.name)
        return occupancy_owners

    def _expire_traffic_zone_leases(self, now: float) -> None:
        for key, expiry in list(self._traffic_zone_leases.items()):
            zone_id, robot_name = key
            robot = self.robots.get(robot_name)
            current_zone = (
                self._traffic_zone_by_lm.get(
                    self._traffic_lm_for_robot(robot),
                    "",
                )
                if robot is not None
                else ""
            )
            if expiry <= now or robot is None or current_zone == zone_id:
                self._traffic_zone_leases.pop(key, None)

    def _collect_traffic_zone_candidates(
        self,
        now: float,
        *,
        demand: dict[str, int],
        hot_zones: set[str],
    ) -> tuple[
        dict[str, list[dict[str, Any]]],
        set[tuple[str, str]],
    ]:
        candidates_by_zone: dict[str, list[dict[str, Any]]] = {}
        candidate_keys: set[tuple[str, str]] = set()
        for robot in self._runtime_robots():
            if robot.status not in {"MOVING", "WAITING"} or robot.is_remote():
                continue
            if robot.traffic_priority_until > now:
                continue
            transition = self._next_traffic_zone_transition(robot)
            if transition is None:
                continue
            src, dst, target_zone, phase, eta = transition
            source_zone = self._traffic_zone_by_lm.get(src, "")
            if target_zone not in hot_zones:
                continue
            # Admit only when moving up the demand gradient. Equal/downhill
            # transitions drain congestion freely, so neighbouring zone gates
            # cannot form a circular wait around the busy region.
            if demand.get(source_zone, 0) >= demand.get(target_zone, 0):
                continue
            key = (target_zone, robot.name)
            candidate_keys.add(key)
            wait_since = self._traffic_zone_wait_since.setdefault(key, now)
            order = self._active_order_for_robot(robot)
            candidates_by_zone.setdefault(target_zone, []).append({
                "robot": robot,
                "src": src,
                "dst": dst,
                "phase": phase,
                "eta": eta,
                "wait_since": wait_since,
                "priority": int(order.priority if order is not None else 0),
            })
        return candidates_by_zone, candidate_keys

    def _traffic_zone_policy(self) -> _TrafficZonePolicy:
        return _TrafficZonePolicy(
            capacity=max(
                1,
                int(
                    self._traffic_zone_param(
                        "traffic_zone_capacity",
                        3.0,
                    )
                ),
            ),
            batch_size=max(
                1,
                int(
                    self._traffic_zone_param(
                        "traffic_zone_batch_size",
                        3.0,
                    )
                ),
            ),
            phase_duration=max(
                0.5,
                self._traffic_zone_param("traffic_zone_phase_sec", 3.0),
            ),
            lease_duration=max(
                0.5,
                self._traffic_zone_param(
                    "traffic_zone_admission_lease_sec",
                    4.0,
                ),
            ),
            starvation_after=max(
                1.0,
                self._traffic_zone_param(
                    "traffic_zone_starvation_sec",
                    8.0,
                ),
            ),
        )

    def _schedule_traffic_zone(
        self,
        zone_id: str,
        candidates: list[dict[str, Any]],
        *,
        occupancy_owners: dict[str, set[str]],
        policy: _TrafficZonePolicy,
        now: float,
    ) -> None:
        occupied = set(occupancy_owners.get(zone_id, set()))
        leased = {
            robot_name
            for (lease_zone, robot_name), expiry
            in self._traffic_zone_leases.items()
            if lease_zone == zone_id and expiry > now
        }
        for robot_name in leased:
            self._traffic_zone_winners[robot_name] = zone_id
        slots = max(0, policy.capacity - len(occupied | leased))
        if slots <= 0:
            self._schedule_full_traffic_zone(
                zone_id,
                candidates,
                leased=leased,
                policy=policy,
                now=now,
            )
            return

        candidates.sort(
            key=lambda item: (
                item["wait_since"],
                -item["priority"],
                item["eta"],
                item["robot"].name,
            )
        )
        available = [
            item
            for item in candidates
            if item["robot"].name not in leased
        ]
        if not available:
            return
        selected_phase = self._select_traffic_zone_phase(
            zone_id,
            available,
            policy=policy,
            now=now,
        )
        compatible = [
            item for item in available if item["phase"] == selected_phase
        ]
        selected = compatible[:min(slots, policy.batch_size)]
        for item in selected:
            robot_name = item["robot"].name
            key = (zone_id, robot_name)
            if key not in self._traffic_zone_leases:
                self.traffic_metrics["zoneAdmissionsGranted"] += 1
            self._traffic_zone_leases[key] = now + policy.lease_duration
            self._traffic_zone_winners[robot_name] = zone_id
        selected_names = {item["robot"].name for item in selected}
        self._traffic_zone_queues[zone_id] = [
            item["robot"].name
            for item in available
            if item["robot"].name not in selected_names
        ]

    def _schedule_full_traffic_zone(
        self,
        zone_id: str,
        candidates: list[dict[str, Any]],
        *,
        leased: set[str],
        policy: _TrafficZonePolicy,
        now: float,
    ) -> None:
        available = sorted(
            (
                item
                for item in candidates
                if item["robot"].name not in leased
            ),
            key=lambda item: (
                item["wait_since"],
                -item["priority"],
                item["eta"],
                item["robot"].name,
            ),
        )
        starved = [
            item
            for item in available
            if now - float(item["wait_since"])
            >= policy.starvation_after
        ]
        emergency_until = self._traffic_zone_emergency_until.get(
            zone_id,
            0.0,
        )
        selected_name = ""
        if starved and emergency_until <= now:
            selected = starved[0]
            selected_name = selected["robot"].name
            self._traffic_zone_leases[(zone_id, selected_name)] = (
                now + policy.lease_duration
            )
            self._traffic_zone_winners[selected_name] = zone_id
            self._traffic_zone_phase[zone_id] = (
                selected["phase"],
                now + policy.phase_duration,
            )
            self._traffic_zone_emergency_until[zone_id] = (
                now + policy.phase_duration
            )
            self.traffic_metrics["zoneAdmissionsGranted"] += 1
        self._traffic_zone_queues[zone_id] = [
            item["robot"].name
            for item in available
            if item["robot"].name != selected_name
        ]

    def _select_traffic_zone_phase(
        self,
        zone_id: str,
        available: list[dict[str, Any]],
        *,
        policy: _TrafficZonePolicy,
        now: float,
    ) -> str:
        starved = [
            item
            for item in available
            if now - float(item["wait_since"])
            >= policy.starvation_after
        ]
        active_phase, phase_until = self._traffic_zone_phase.get(
            zone_id,
            ("", 0.0),
        )
        if starved:
            selected_phase = starved[0]["phase"]
            self._traffic_zone_phase[zone_id] = (
                selected_phase,
                now + policy.phase_duration,
            )
            return selected_phase
        if (
            active_phase
            and phase_until > now
            and any(
                item["phase"] == active_phase for item in available
            )
        ):
            return active_phase
        selected_phase = available[0]["phase"]
        self._traffic_zone_phase[zone_id] = (
            selected_phase,
            now + policy.phase_duration,
        )
        return selected_phase

    def _traffic_zone_admission_reason(
        self,
        robot: FleetRobot,
        check_clock: float,
    ) -> str:
        if (
            not self._traffic_zone_by_lm
            or not self._traffic_zone_control_enabled()
            or robot.status == "RETREATING"
            or robot.traffic_priority_until > self._traffic_zone_tick_now
        ):
            return ""
        edge = self._parse_edge_id(
            self._edge_id_at_trajectory(robot.trajectory, check_clock)
        )
        if edge is None:
            return ""
        src, dst = edge
        if self._edge_has_explicit_corridor_authority(src, dst):
            return ""
        source_zone = self._traffic_zone_by_lm.get(src, "")
        target_zone = self._traffic_zone_by_lm.get(dst, "")
        if not source_zone or not target_zone or source_zone == target_zone:
            return ""
        threshold = max(
            1,
            int(self._traffic_zone_param("traffic_zone_demand_threshold", 6.0)),
        )
        if self._traffic_zone_demand.get(target_zone, 0) < threshold:
            return ""
        if self._traffic_zone_demand.get(source_zone, 0) >= self._traffic_zone_demand.get(
            target_zone,
            0,
        ):
            return ""
        lease = self._traffic_zone_leases.get((target_zone, robot.name), 0.0)
        if lease > self._traffic_zone_tick_now:
            return ""
        # Hold on the graph vertex outside the zone, never midway along the
        # entering edge merely because far-lookahead noticed the closed gate.
        if robot.pose is None or not self._pose_is_at_lm(robot.pose, src):
            return ""
        reason = f"traffic admission wait at {src} for {target_zone}"
        if robot.last_reason != reason:
            self.traffic_metrics["zoneAdmissionWaits"] += 1
        return reason

    def _traffic_flow_payload(self) -> dict[str, Any]:
        zones = []
        corridor_schedule = self._controlled_corridor_schedule
        threshold = max(
            1,
            int(self._traffic_zone_param("traffic_zone_demand_threshold", 6.0)),
        )
        for zone_id in sorted(
            set(self._traffic_zone_demand)
            | set(self._traffic_zone_occupancy)
            | set(self._traffic_zone_queues)
        ):
            demand = int(self._traffic_zone_demand.get(zone_id, 0))
            queue = list(self._traffic_zone_queues.get(zone_id, []))
            if demand < threshold and not queue:
                continue
            phase, phase_until = self._traffic_zone_phase.get(zone_id, ("", 0.0))
            zones.append({
                "id": zone_id,
                "demand": demand,
                "occupancy": int(self._traffic_zone_occupancy.get(zone_id, 0)),
                "queue": queue,
                "phase": phase,
                "phaseUntil": phase_until,
            })
        return {
            "enabled": bool(
                self._traffic_zone_by_lm
                or self._controlled_corridor_graph is not None
            ),
            "controlledCorridorsEnabled": bool(
                self._controlled_corridor_graph is not None
            ),
            "zones": zones,
            "controlledCorridors": [
                {
                    "id": region_id,
                    "occupancy": list(
                        self._controlled_corridor_occupancy.get(region_id, [])
                    ),
                    "queue": list(
                        self._controlled_corridor_queues.get(region_id, [])
                    ),
                    "winner": next(
                        (
                            robot_name
                            for robot_name, winner_region in
                            self._controlled_corridor_winners.items()
                            if winner_region == region_id
                        ),
                        str(
                            self._controlled_corridor_leases.get(
                                region_id,
                                ("", 0.0),
                            )[0]
                            or ""
                        ),
                    ),
                }
                for region_id in sorted(
                    set(self._controlled_corridor_occupancy)
                    | set(self._controlled_corridor_queues)
                    | set(self._controlled_corridor_leases)
                )
            ],
            "controlledCorridorSchedule": {
                "epoch": (
                    corridor_schedule.epoch
                    if corridor_schedule is not None
                    else 0
                ),
                "generatedAt": (
                    corridor_schedule.generated_at
                    if corridor_schedule is not None
                    else 0.0
                ),
                "horizonEnd": (
                    corridor_schedule.horizon_end
                    if corridor_schedule is not None
                    else 0.0
                ),
                "slots": [
                    {
                        "robot": slot.robot_id,
                        "regions": list(slot.regions),
                        "direction": slot.direction,
                        "entryTime": slot.entry_time,
                        "exitTime": slot.exit_time,
                        "stagingLm": slot.staging_lm,
                        "exitLm": slot.exit_lm,
                        "state": slot.state.value,
                        "pastCommitPoint": slot.past_commit_point,
                        "physicallyObserved": slot.physically_observed,
                        "routeRevision": slot.route_revision,
                        "resourceWindows": [
                            {
                                "region": window.region_id,
                                "direction": window.direction,
                                "entryTime": (
                                    slot.entry_time
                                    + window.entry_offset_sec
                                ),
                                "exitTime": (
                                    slot.entry_time
                                    + window.exit_offset_sec
                                ),
                            }
                            for window in slot.resource_windows
                        ],
                    }
                    for slot in (
                        corridor_schedule.slots
                        if corridor_schedule is not None
                        else ()
                    )
                ],
            },
        }
