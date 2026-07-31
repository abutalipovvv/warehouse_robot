"""Topology and spatial policies used by fleet benchmarks."""

from __future__ import annotations

import math
import random
from typing import Any


class BenchmarkTopologyService:
    """Graph, corridor and spatial selection for benchmark starts and goals."""

    def __init__(self, owner: Any) -> None:
        self.owner = owner

    def _dynamic_goal_hop_window(self) -> tuple[int, int]:
        fleet = self.owner.manager.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 30, 160
        try:
            minimum = max(2, int(fleet.get("dynamic_order_min_hops", 30) or 30))
        except (TypeError, ValueError):
            minimum = 30
        try:
            maximum = max(minimum, int(fleet.get("dynamic_order_max_hops", 160) or 160))
        except (TypeError, ValueError):
            maximum = 160
        return minimum, maximum

    def _far_dynamic_goal(self, origin: str, candidates: list[str]) -> str:
        start = self.owner.loaded_map.landmarks[origin]
        ranked = sorted(
            candidates,
            key=lambda name: math.hypot(
                self.owner.loaded_map.landmarks[name].x - start.x,
                self.owner.loaded_map.landmarks[name].y - start.y,
            ),
        )
        fleet = self.owner.manager.params.get("fleet", {})
        try:
            fraction = float(fleet.get("dynamic_order_far_fraction", 0.08) or 0.08)
        except (AttributeError, TypeError, ValueError):
            fraction = 0.08
        fraction = max(0.05, min(1.0, fraction))
        pool_size = max(1, int(math.ceil(len(ranked) * fraction)))
        return self.owner._dynamic_rng.choice(ranked[-pool_size:])

    def _package_wave_assignments(
        self,
        robots: list[Any],
        wave_index: int,
    ) -> list[tuple[Any, str]]:
        peripheral = self.owner._benchmark_peripheral_lms(len(robots))
        if not peripheral:
            return []
        perimeter_rank = {name: index for index, name in enumerate(peripheral)}
        occupied_lms = {
            str(robot.current_lm)
            for robot in self.owner._benchmark_sim_robots()
            if str(robot.current_lm) in self.owner.loaded_map.landmarks
        }
        used_goals = {
            str(order.target_lm)
            for order in self.owner.manager.orders.values()
            if order.status not in {"COMPLETED", "FAILED", "CANCELED"}
            and str(order.target_lm) in self.owner.loaded_map.landmarks
        }
        assignments: list[tuple[Any, str]] = []
        min_hops, max_hops = self.owner._dynamic_goal_hop_window()
        robot_count = max(1, len(robots))
        # A full-fleet wave coordinates *when* robots depart, but it cannot
        # make two adjacent physical footprints disappear atomically.  In the
        # previous permutation model one robot could complete at another
        # member's start portal while that member was still waiting to rotate
        # and depart.  The completed robot then became a permanent obstacle
        # and the waiting member could never execute its first WAIT/ROTATE.
        # Every current fleet LM is therefore a hard destination exclusion,
        # including the starts of this departure cohort.  A robot's own origin
        # remains traversable to the graph search, but is never a wave target.
        excluded_goal_lms = set(occupied_lms)
        # Half-slot rotation prevents every wave from assigning the same edge
        # cells to the same robot while keeping targets uniformly distributed.
        wave_phase = (max(0, wave_index - 1) * 0.5) % robot_count
        for index, robot in enumerate(sorted(robots, key=lambda item: str(item.name))):
            origin = self.owner._dynamic_order_origin(robot.name) or str(robot.current_lm)
            if origin not in self.owner.loaded_map.landmarks:
                continue
            reachable = self.owner._forward_benchmark_goals(
                origin,
                used_goals,
                excluded_goal_lms,
                self.owner._dynamic_rng,
                min_hops=min_hops,
                max_hops=max_hops,
            )
            candidates = [
                name
                for name in reachable
                if (
                    name in perimeter_rank
                    and self.owner._package_goal_is_clear_of_occupied_lms(
                        name,
                        occupied_lms,
                    )
                )
            ]
            if not candidates:
                reachable = self.owner._forward_benchmark_goals(
                    origin,
                    used_goals,
                    excluded_goal_lms,
                    self.owner._dynamic_rng,
                    min_hops=2,
                    max_hops=min(300, max(max_hops, len(self.owner.loaded_map.landmarks))),
                )
                candidates = [
                    name
                    for name in reachable
                    if (
                        name in perimeter_rank
                        and self.owner._package_goal_is_clear_of_occupied_lms(
                            name,
                            occupied_lms,
                        )
                    )
                ]
            if not candidates:
                continue

            desired_fraction = ((index + wave_phase) % robot_count) / robot_count
            desired_rank = desired_fraction * len(peripheral)
            origin_lm = self.owner.loaded_map.landmarks[origin]

            def candidate_key(name: str) -> tuple[float, float, str]:
                rank = float(perimeter_rank[name])
                rank_distance = abs(rank - desired_rank)
                circular_distance = min(
                    rank_distance,
                    len(peripheral) - rank_distance,
                )
                target = self.owner.loaded_map.landmarks[name]
                distance = math.hypot(target.x - origin_lm.x, target.y - origin_lm.y)
                return circular_distance, -distance, name

            target_lm = min(candidates, key=candidate_key)
            assignments.append((robot, target_lm))
            used_goals.add(target_lm)
        return assignments

    def _package_goal_is_clear_of_occupied_lms(
        self,
        candidate: str,
        occupied_lms: set[str],
    ) -> bool:
        """Keep a package destination clear of the fleet's start footprints.

        ``robot_footprints_conflict`` checks the configured footprint in one
        orientation.  Package destinations also execute a terminal rotation,
        so use the circumscribed broad-phase diameter as the minimum centre
        distance as well.  This guarantees that a robot parked at the target
        cannot touch or overlap a neighbour that has not departed yet.
        """
        if candidate in occupied_lms or candidate not in self.owner.loaded_map.landmarks:
            return False
        if not self.owner._lm_is_separated_from(candidate, occupied_lms):
            return False
        landmark = self.owner.loaded_map.landmarks[candidate]
        minimum = max(
            self.owner._benchmark_min_separation(),
            self.owner.manager.collision.robot_broadphase_distance(),
        )
        for occupied_name in occupied_lms:
            occupied = self.owner.loaded_map.landmarks.get(occupied_name)
            if occupied is None:
                continue
            if math.hypot(
                landmark.x - occupied.x,
                landmark.y - occupied.y,
            ) + 0.000001 < minimum:
                return False
        return True

    def _benchmark_peripheral_lms(self, robot_count: int) -> list[str]:
        names = [
            name
            for name in self.owner._largest_benchmark_component()
            if (
                name in self.owner.loaded_map.landmarks
                and self.owner._benchmark_goal_lm_is_safe(name)
            )
        ]
        if not names:
            return []
        landmarks = self.owner.loaded_map.landmarks
        min_x = min(landmarks[name].x for name in names)
        max_x = max(landmarks[name].x for name in names)
        min_y = min(landmarks[name].y for name in names)
        max_y = max(landmarks[name].y for name in names)
        width = max(0.001, max_x - min_x)
        height = max(0.001, max_y - min_y)

        def edge_distance(name: str) -> float:
            lm = landmarks[name]
            return min(
                lm.x - min_x,
                max_x - lm.x,
                lm.y - min_y,
                max_y - lm.y,
            )

        # Use a broad outer ring so 100-robot waves still have enough unique,
        # footprint-separated destinations after excluding occupied cells.
        pool_size = min(len(names), max(64, int(robot_count) * 8))
        by_edge_distance = sorted(
            names,
            key=lambda name: (edge_distance(name), name),
        )
        distance_limit = edge_distance(by_edge_distance[pool_size - 1])
        # Include the complete distance band at the cutoff. Without this, a
        # lexicographic tie on the outermost row could select the top edge but
        # accidentally omit the bottom edge from small waves.
        outer_ring = [
            name
            for name in by_edge_distance
            if edge_distance(name) <= distance_limit + 0.000001
        ]

        def perimeter_position(name: str) -> float:
            lm = landmarks[name]
            distances = (
                (abs(lm.y - min_y), lm.x - min_x),
                (abs(lm.x - max_x), width + (lm.y - min_y)),
                (abs(lm.y - max_y), width + height + (max_x - lm.x)),
                (abs(lm.x - min_x), (2.0 * width) + height + (max_y - lm.y)),
            )
            return min(distances, key=lambda item: item[0])[1]

        return sorted(outer_ring, key=lambda name: (perimeter_position(name), name))

    def _benchmark_spawn_lms(self, count: int, seed: int) -> list[str]:
        names = [
            name
            for name in self.owner._largest_benchmark_component()
            if (
                self.owner._benchmark_spawn_lm_is_safe(name)
                and self.owner._benchmark_wait_lm_is_safe(name)
            )
        ]
        if len(names) < count:
            raise ValueError(
                f"add robots needs at least {count} collision-safe connected LMs; "
                f"largest component has {len(names)} safe spawn positions"
            )
        rng = random.Random(seed + 7919)
        shuffled = self.owner._corridor_safe_benchmark_lms(names, rng)
        spaced = self.owner._spatially_separated_lms(shuffled, count)
        if len(spaced) < count:
            raise ValueError(
                f"map can safely place only {len(spaced)} of {count} robots "
                f"with {self.owner._benchmark_min_separation():.2f} m center spacing"
            )
        return spaced

    def _benchmark_spawn_lm_is_safe(self, name: str) -> bool:
        landmark = self.owner.loaded_map.landmarks.get(name)
        if landmark is None:
            return False
        return not self.owner.manager.collision.blocked_reason(
            {
                "x": float(landmark.x),
                "y": float(landmark.y),
                "yaw": 0.0,
            },
            self.owner.manager.obstacles,
            self.owner.manager.obstacle_areas,
        )

    def _benchmark_corridor_region(self, name: str) -> str:
        graph = getattr(self.owner.manager, "_controlled_corridor_graph", None)
        vertex = graph.vertices.get(name) if graph is not None else None
        if vertex is None or not vertex.controlled_region_ids:
            return ""
        return sorted(vertex.controlled_region_ids)[0]

    def _benchmark_goal_lm_is_safe(self, name: str) -> bool:
        """Keep benchmark parking destinations out of traffic bottlenecks."""
        wait_lm_is_safe = self.owner.__dict__.get(
            "_benchmark_wait_lm_is_safe",
            self._benchmark_wait_lm_is_safe,
        )
        return self._goal_lm_is_safe_with_wait_policy(
            name,
            wait_lm_is_safe=wait_lm_is_safe,
        )

    def _goal_lm_is_safe_with_wait_policy(
        self,
        name: str,
        *,
        wait_lm_is_safe: Any,
    ) -> bool:
        if not wait_lm_is_safe(name):
            return False
        graph = getattr(self.owner.manager, "_controlled_corridor_graph", None)
        vertex = graph.vertices.get(name) if graph is not None else None
        if vertex is None:
            return True
        # On Kiva maps, no-wait aisle chains terminate at shared junctions.
        # Parking a completed wave robot on an internal four-way junction
        # removes a transit vertex and can disconnect every remaining route.
        # Degree-three perimeter portals are still graph-safe wait points and
        # leave the inner cross-aisles available to unfinished orders.
        neighbours = self.owner.manager.planner.graph.get(name, {})
        return len(neighbours) <= 3

    def _benchmark_wait_lm_is_safe(self, name: str) -> bool:
        graph = getattr(self.owner.manager, "_controlled_corridor_graph", None)
        vertex = graph.vertices.get(name) if graph is not None else None
        if vertex is None:
            return True
        # Auto-controlled corridors expose capacity-one stop-line LMs just
        # before each junction. They are legal holding/spawn points because a
        # robot there owns the whole corridor mutex; the benchmark's
        # corridor-safe selector below still places at most one robot in each
        # such region.
        return bool(vertex.can_wait)

    def _corridor_safe_benchmark_lms(
        self,
        names: list[str],
        rng: random.Random,
    ) -> list[str]:
        """Prefer holding points and place at most one robot inside a corridor."""
        if getattr(self.owner.manager, "_controlled_corridor_graph", None) is None:
            shuffled = list(names)
            rng.shuffle(shuffled)
            return shuffled
        holding: list[str] = []
        inside_by_region: dict[str, list[str]] = {}
        corridor_region = self.owner.__dict__.get(
            "_benchmark_corridor_region",
            self._benchmark_corridor_region,
        )
        for name in names:
            region_id = corridor_region(name)
            if region_id:
                inside_by_region.setdefault(region_id, []).append(name)
            else:
                holding.append(name)
        rng.shuffle(holding)
        region_ids = list(inside_by_region)
        rng.shuffle(region_ids)
        representatives: list[str] = []
        for region_id in region_ids:
            candidates = inside_by_region[region_id]
            rng.shuffle(candidates)
            representatives.append(candidates[0])
        return holding + representatives

    def _next_benchmark_robot_index(self) -> int:
        max_index = 0
        for name in self.owner.manager.robots:
            value = str(name)
            if not value.startswith("bench_"):
                continue
            try:
                max_index = max(max_index, int(value.split("_", 1)[1]))
            except (IndexError, ValueError):
                continue
        return max_index + 1

    def _benchmark_min_separation(self) -> float:
        fleet = self.owner.manager.params.get("fleet", {})
        if isinstance(fleet, dict):
            configured = fleet.get("mapf_min_robot_center_distance_m")
            if configured is not None:
                try:
                    return max(0.0, float(configured))
                except (TypeError, ValueError):
                    pass
        return self.owner.manager.collision.robot_broadphase_distance()

    def _forward_benchmark_goals(
        self,
        start_lm: str,
        used_goals: set[str],
        excluded_goals: set[str],
        rng: random.Random,
        *,
        min_hops: int = 3,
        max_hops: int = 15,
    ) -> list[str]:
        owner = self.owner
        loaded_map = owner.loaded_map
        landmarks = loaded_map.landmarks
        overrides = owner.__dict__
        corridor_region = overrides.get(
            "_benchmark_corridor_region",
            self._benchmark_corridor_region,
        )
        goal_is_safe = overrides.get(
            "_benchmark_goal_lm_is_safe"
        )
        if goal_is_safe is None:
            wait_lm_is_safe = overrides.get(
                "_benchmark_wait_lm_is_safe",
                self._benchmark_wait_lm_is_safe,
            )

            def goal_is_safe(name: str) -> bool:
                return self._goal_lm_is_safe_with_wait_policy(
                    name,
                    wait_lm_is_safe=wait_lm_is_safe,
                )

        is_separated = overrides.get("_lm_is_separated_from")
        if is_separated is None:
            minimum = overrides.get(
                "_benchmark_min_separation",
                self._benchmark_min_separation,
            )()

            def is_separated(
                candidate: str,
                selected: set[str] | list[str],
            ) -> bool:
                return self._lm_is_separated_with_minimum(
                    candidate,
                    selected,
                    minimum=minimum,
                )
        adjacency: dict[str, list[str]] = {
            name: [] for name in landmarks
        }
        for edge in loaded_map.edges:
            if edge.from_name in adjacency and edge.to_name in adjacency:
                adjacency[edge.from_name].append(edge.to_name)
        for neighbors in adjacency.values():
            neighbors.sort()

        queue: list[tuple[str, int, int]] = [(start_lm, 0, 0)]
        best_path: dict[str, tuple[int, int]] = {start_lm: (0, 0)}
        candidates: list[tuple[int, int, float, int, str]] = []
        used_goal_regions = {
            region_id
            for region_id in (
                corridor_region(name)
                for name in used_goals
            )
            if region_id
        }
        sequence = 0
        start = landmarks[start_lm]
        while queue:
            node, hops, occupied_starts = queue.pop(0)
            if best_path.get(node) != (hops, occupied_starts):
                continue
            if (
                hops >= min_hops
                and node not in used_goals
                and node not in excluded_goals
                and goal_is_safe(node)
                and is_separated(node, used_goals)
                and (
                    not corridor_region(node)
                    or corridor_region(node)
                    not in used_goal_regions
                )
            ):
                landmark = landmarks[node]
                distance_sq = ((landmark.x - start.x) ** 2) + ((landmark.y - start.y) ** 2)
                candidates.append((occupied_starts, hops, distance_sq, sequence, node))
                sequence += 1
            if hops >= max_hops:
                continue
            neighbors = list(adjacency.get(node, ()))
            rng.shuffle(neighbors)
            for neighbor in neighbors:
                next_hops = hops + 1
                next_occupied = occupied_starts + int(
                    neighbor in excluded_goals and neighbor != start_lm
                )
                previous = best_path.get(neighbor)
                if previous is not None and previous <= (next_hops, next_occupied):
                    continue
                best_path[neighbor] = (next_hops, next_occupied)
                queue.append((neighbor, next_hops, next_occupied))
        candidates.sort()
        return [item[4] for item in candidates]

    def _lm_is_separated_from(self, candidate: str, selected: set[str] | list[str]) -> bool:
        minimum = self.owner.__dict__.get(
            "_benchmark_min_separation",
            self._benchmark_min_separation,
        )()
        return self._lm_is_separated_with_minimum(
            candidate,
            selected,
            minimum=minimum,
        )

    def _lm_is_separated_with_minimum(
        self,
        candidate: str,
        selected: set[str] | list[str],
        *,
        minimum: float,
    ) -> bool:
        owner = self.owner
        landmarks = owner.loaded_map.landmarks
        collision = owner.manager.collision
        landmark = landmarks[candidate]
        candidate_pose = {"x": landmark.x, "y": landmark.y, "yaw": 0.0}
        for name in selected:
            if name not in landmarks:
                continue
            other = landmarks[name]
            if math.hypot(
                landmark.x - other.x,
                landmark.y - other.y,
            ) < minimum:
                return False
            if collision.robot_footprints_conflict(
                candidate_pose,
                {"x": other.x, "y": other.y, "yaw": 0.0},
            ):
                return False
        return True

    def _spatially_separated_lms(self, candidates: list[str], count: int) -> list[str]:
        selected: list[str] = []
        for name in candidates:
            if name not in self.owner.loaded_map.landmarks:
                continue
            if not self.owner._lm_is_separated_from(name, selected):
                continue
            selected.append(name)
            if len(selected) >= count:
                break
        return selected

    def _largest_benchmark_component(self) -> list[str]:
        adjacency: dict[str, set[str]] = {name: set() for name in self.owner.loaded_map.landmarks}
        for edge in self.owner.loaded_map.edges:
            if edge.from_name in adjacency and edge.to_name in adjacency:
                adjacency[edge.from_name].add(edge.to_name)
                adjacency[edge.to_name].add(edge.from_name)
        visited: set[str] = set()
        components: list[list[str]] = []
        for name in sorted(adjacency):
            if name in visited:
                continue
            stack = [name]
            visited.add(name)
            component: list[str] = []
            while stack:
                current = stack.pop()
                component.append(current)
                for neighbor in adjacency.get(current, set()):
                    if neighbor in visited:
                        continue
                    visited.add(neighbor)
                    stack.append(neighbor)
            components.append(sorted(component))
        return max(components, key=len, default=[])
