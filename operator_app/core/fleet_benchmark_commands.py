"""One-shot benchmark commands for the operator fleet facade."""

from __future__ import annotations

from dataclasses import dataclass
import random
from time import perf_counter
from typing import Any

from fleet_manager.core.mapping.navigation.params import load_route_params


@dataclass(slots=True)
class BenchmarkOptions:
    count: int
    seed: int
    speed: float
    acceleration: float
    rotate: bool
    turn_speed: float
    action: str
    add_only: bool
    plan_existing: bool
    package_existing: bool
    stop_dynamic: bool
    set_time_scale: bool
    reset: bool
    fast: bool


@dataclass(slots=True)
class BenchmarkRun:
    result: dict[str, Any]
    requests: list[dict[str, Any]]
    elapsed_ms: float
    allocation_attempts: int
    stress_profile: int
    used_traffic_stress: bool


class FleetBenchmarkCommandService:
    """One-shot benchmark commands, robot setup and plan statistics."""

    def __init__(self, owner: Any) -> None:
        self.owner = owner

    def benchmark_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self.owner._sync_manager_mode()
        if self.owner.mode != "simulation":
            raise ValueError(
                "benchmark is only available in Fleet Manager Sim"
            )
        options = self._parse_options(payload)
        if options.reset:
            self.owner._clear_simulation_runtime()
        immediate = self._immediate_action(payload, options)
        if immediate is not None:
            return immediate
        return self._run_planning_benchmark(payload, options)

    def _parse_options(
        self,
        payload: dict[str, Any],
    ) -> BenchmarkOptions:
        raw_count = payload.get("count", 20)
        count = max(
            0,
            min(300, int(20 if raw_count is None else raw_count)),
        )
        seed = int(payload.get("seed", 42) or 42)
        speed = float(
            payload.get("speed")
            or payload.get("routeSpeed")
            or 0.0
        )
        acceleration = float(
            payload.get("acceleration")
            or payload.get("routeAcceleration")
            or 0.0
        )
        rotate = bool(
            payload.get("rotate")
            or payload.get("simulateRotation")
            or payload.get("simulate_rotation")
        )
        turn_speed = float(
            payload.get("turnSpeed")
            or payload.get("turn_speed")
            or payload.get("rotationSpeed")
            or 0.0
        )
        if speed <= 0.0:
            params = load_route_params(
                self.owner.params_path,
                create=True,
            )
            navigation = (
                params.get("navigation", {})
                if isinstance(params, dict)
                else {}
            )
            speed = float(navigation.get("route_speed", 1.0) or 1.0)
            acceleration = float(
                navigation.get("route_acceleration", acceleration)
                or acceleration
                or 0.0
            )
            rotate = bool(
                navigation.get("simulate_rotation", rotate)
            )
            turn_speed = float(
                navigation.get("turn_speed", turn_speed)
                or turn_speed
                or 0.0
            )

        action = str(payload.get("action") or "").strip().lower()
        add_only = (
            action in {"add", "add_robots", "ensure"}
            or bool(payload.get("addOnly", False))
        )
        plan_existing = (
            action in {"plan", "plan_existing"}
            or bool(payload.get("planExisting", False))
        )
        package_existing = action in {
            "package",
            "package_orders",
            "package_waves",
        }
        stop_dynamic = action in {
            "stop",
            "stop_dynamic",
            "pause_dynamic",
        }
        set_time_scale = action in {
            "time_scale",
            "set_time_scale",
        }
        reset_default = not (
            add_only
            or plan_existing
            or package_existing
            or stop_dynamic
            or set_time_scale
        )
        return BenchmarkOptions(
            count=count,
            seed=seed,
            speed=speed,
            acceleration=acceleration,
            rotate=rotate,
            turn_speed=turn_speed,
            action=action,
            add_only=add_only,
            plan_existing=plan_existing,
            package_existing=package_existing,
            stop_dynamic=stop_dynamic,
            set_time_scale=set_time_scale,
            reset=bool(payload.get("reset", reset_default)),
            fast=bool(payload.get("fast", False)),
        )

    def _immediate_action(
        self,
        payload: dict[str, Any],
        options: BenchmarkOptions,
    ) -> dict[str, Any] | None:
        if options.set_time_scale:
            return self._set_time_scale(payload)
        if options.count <= 0:
            return self.owner._state_with_context(
                {
                    **self.owner.manager.state(
                        include_trajectories=True
                    ),
                    "benchmark": {
                        "count": 0,
                        "planned": 0,
                        "elapsedMs": 0.0,
                        "seed": options.seed,
                        "cleared": True,
                    },
                }
            )
        if options.add_only:
            return self.owner._ensure_benchmark_robots_payload(
                count=options.count,
                seed=options.seed,
            )
        if options.stop_dynamic:
            return self.owner._stop_dynamic_benchmark_payload()
        if options.package_existing:
            return self._start_dynamic(
                payload,
                options,
                generation_mode="package_waves",
            )
        if options.plan_existing:
            return self._start_dynamic(
                payload,
                options,
                generation_mode="continuous",
            )
        return None

    def _set_time_scale(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        self.owner._pump_dynamic_benchmark()
        self.owner.manager.advance_runtime()
        scale = self.owner.manager.set_simulation_time_scale(
            payload.get("timeScale", payload.get("scale", 1.0))
        )
        return self.owner._state_with_context(
            {
                **self.owner.manager.snapshot(
                    include_trajectories=True
                ),
                "benchmark": {
                    "action": "time_scale",
                    "timeScale": scale,
                },
            }
        )

    def _start_dynamic(
        self,
        payload: dict[str, Any],
        options: BenchmarkOptions,
        *,
        generation_mode: str,
    ) -> dict[str, Any]:
        package_waves = generation_mode == "package_waves"
        return self.owner._start_dynamic_benchmark_payload(
            count=options.count,
            seed=options.seed,
            horizon_sec=float(
                payload.get("horizonSec", 10.0) or 10.0
            ),
            order_interval_sec=(
                0.0
                if package_waves
                else float(
                    payload.get("orderIntervalSec", 3.0) or 3.0
                )
            ),
            queue_depth=(
                1
                if package_waves
                else int(payload.get("queueDepth", 2) or 2)
            ),
            speed=options.speed,
            acceleration=options.acceleration,
            rotate=options.rotate,
            turn_speed=options.turn_speed,
            generation_mode=generation_mode,
        )

    def _run_planning_benchmark(
        self,
        payload: dict[str, Any],
        options: BenchmarkOptions,
    ) -> dict[str, Any]:
        stress = bool(
            payload.get("stress", options.plan_existing)
        )
        stress_profile = 0
        requests = self._allocate_requests(
            options,
            seed=options.seed,
            stress=stress,
            stress_profile=stress_profile,
        )
        if options.plan_existing:
            self.owner.manager.orders.clear()
        run = self._plan_with_retries(
            options,
            requests=requests,
            stress=stress,
            stress_profile=stress_profile,
        )
        return self._format_planning_result(
            options,
            run,
            stress=stress,
        )

    def _allocate_requests(
        self,
        options: BenchmarkOptions,
        *,
        seed: int,
        stress: bool,
        stress_profile: int,
    ) -> list[dict[str, Any]]:
        allocator = (
            self.owner._benchmark_requests_for_existing
            if options.plan_existing
            else self.owner._benchmark_requests
        )
        return allocator(
            count=options.count,
            seed=seed,
            stress=stress,
            stress_profile=stress_profile,
        )

    def _plan_with_retries(
        self,
        options: BenchmarkOptions,
        *,
        requests: list[dict[str, Any]],
        stress: bool,
        stress_profile: int,
    ) -> BenchmarkRun:
        restore_fleet_params = (
            self.owner._apply_fast_benchmark_params(options.count)
        )
        started = perf_counter()
        allocation_attempts = 1
        used_traffic_stress = stress
        try:
            result = self._plan_requests(requests, options)
            while (
                not result.get("ok")
                and allocation_attempts < (4 if stress else 6)
            ):
                retry_seed = options.seed + allocation_attempts
                stress_profile = allocation_attempts
                use_traffic_stress = (
                    stress and stress_profile < 3
                )
                requests = self._allocate_requests(
                    options,
                    seed=(
                        options.seed
                        if use_traffic_stress
                        else retry_seed
                    ),
                    stress=use_traffic_stress,
                    stress_profile=stress_profile,
                )
                used_traffic_stress = use_traffic_stress
                allocation_attempts += 1
                result = self._plan_requests(requests, options)
        finally:
            if restore_fleet_params is not None:
                self.owner._restore_fleet_params(
                    restore_fleet_params
                )
        return BenchmarkRun(
            result=result,
            requests=requests,
            elapsed_ms=(perf_counter() - started) * 1000.0,
            allocation_attempts=allocation_attempts,
            stress_profile=stress_profile,
            used_traffic_stress=used_traffic_stress,
        )

    def _plan_requests(
        self,
        requests: list[dict[str, Any]],
        options: BenchmarkOptions,
    ) -> dict[str, Any]:
        return self.owner.manager.plan(
            {
                "robots": requests,
                "speed": options.speed,
                "acceleration": options.acceleration,
                "rotate": options.rotate,
                "turnSpeed": options.turn_speed,
                "stretchMotionToReservationTicks": True,
            }
        )

    def _format_planning_result(
        self,
        options: BenchmarkOptions,
        run: BenchmarkRun,
        *,
        stress: bool,
    ) -> dict[str, Any]:
        result = run.result
        plans = result.get("plans", [])
        if not isinstance(plans, list):
            plans = []
        debug = result.get("debug", {})
        if not isinstance(debug, dict):
            debug = {}
        plan_stats = self.owner._benchmark_plan_stats(
            plans,
            float(result.get("timeStepSec", 1.0) or 1.0),
        )
        result["benchmark"] = {
            "count": options.count,
            "planned": len(plans),
            "elapsedMs": round(run.elapsed_ms, 3),
            "seed": options.seed,
            "speed": options.speed,
            "acceleration": options.acceleration,
            "rotate": options.rotate,
            "turnSpeed": options.turn_speed,
            "mapName": self.owner.map_dir.stem.replace(".smap", ""),
            "plannerBackend": debug.get("plannerBackend", ""),
            "reason": debug.get("reason", ""),
            "expandedNodes": debug.get("expandedNodes", 0),
            "highLevelNodes": debug.get("highLevelNodes", 0),
            "fast": options.fast,
            "allocationAttempts": run.allocation_attempts,
            "action": (
                "plan"
                if options.plan_existing
                else "benchmark"
            ),
            "scenario": (
                "traffic_stress"
                if run.used_traffic_stress
                else "balanced_fallback"
                if stress
                else "balanced"
            ),
            "stressProfile": (
                run.stress_profile
                if run.used_traffic_stress
                else None
            ),
            "resolvedPriorityConflicts": int(
                debug.get("conflictsResolved", 0) or 0
            ),
            **plan_stats,
        }
        if isinstance(result.get("fleetState"), dict):
            result["state"] = result["fleetState"]
        return self.owner._result_with_context(result)

    def _clear_simulation_runtime(self) -> None:
        if self.owner.mode != "simulation":
            return
        self.owner.manager.robots.clear()
        self.owner.manager.orders.clear()
        self.owner.manager.events.clear()
        self.owner.manager.reset_planning_runtime_state()
        for key in getattr(self.owner.manager, "traffic_metrics", {}):
            self.owner.manager.traffic_metrics[key] = 0
        self.owner.manager._last_runtime_safety_rollback = None
        self.owner._reset_dynamic_benchmark()

    def _benchmark_sim_robots(self) -> list[Any]:
        owner = self.owner
        return sorted(
            [
                robot
                for robot in owner.manager.robots.values()
                if not robot.is_remote()
            ],
            key=lambda robot: str(robot.name),
        )

    def _ensure_benchmark_robots_payload(self, *, count: int, seed: int) -> dict[str, Any]:
        existing = self.owner._benchmark_sim_robots()
        target_count = max(0, count)
        if len(existing) >= target_count:
            state = self.owner.manager.state(include_trajectories=True)
            return self.owner._state_with_context({
                **state,
                "benchmark": {
                    "action": "add",
                    "target": target_count,
                    "robots": len(existing),
                    "added": 0,
                    "seed": seed,
                },
            })

        spawn_lms = self.owner._benchmark_spawn_lms(target_count, seed)
        used_lms = {
            str(robot.current_lm)
            for robot in existing
            if str(robot.current_lm)
        }
        existing_names = set(self.owner.manager.robots)
        next_index = self.owner._next_benchmark_robot_index()
        added = 0
        for lm_name in spawn_lms:
            if len(existing) + added >= target_count:
                break
            if lm_name in used_lms:
                continue
            while f"bench_{next_index:03d}" in existing_names:
                next_index += 1
            name = f"bench_{next_index:03d}"
            self.owner.manager.add_robot({
                "name": name,
                "spawnLm": lm_name,
                "mode": "simulated",
            })
            existing_names.add(name)
            used_lms.add(lm_name)
            next_index += 1
            added += 1

        robots = self.owner._benchmark_sim_robots()
        state = self.owner.manager.state(include_trajectories=True)
        return self.owner._state_with_context({
            **state,
            "benchmark": {
                "action": "add",
                "target": target_count,
                "robots": len(robots),
                "added": added,
                "seed": seed,
            },
        })

    def _apply_fast_benchmark_params(self, count: int = 0) -> dict[str, Any] | None:
        params = getattr(self.owner.manager, "params", None)
        if not isinstance(params, dict):
            return None
        fleet = params.setdefault("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
            params["fleet"] = fleet
        previous = {
            "batch_collision_horizon_sec": fleet.get("batch_collision_horizon_sec"),
            "batch_wait_max_iterations": fleet.get("batch_wait_max_iterations"),
            "continuous_collision_step_sec": fleet.get("continuous_collision_step_sec"),
            "stretch_motion_to_reservation_ticks": fleet.get("stretch_motion_to_reservation_ticks"),
            "_planner_backend": getattr(getattr(self.owner.manager, "planner", None), "planner_backend", None),
            "_planner_stretch_motion_to_reservation_ticks": getattr(
                getattr(self.owner.manager, "planner", None),
                "stretch_motion_to_reservation_ticks",
                None,
            ),
        }
        if count >= 100:
            horizon = 8.0
            iterations = 8
        elif count >= 50:
            horizon = 10.0
            iterations = 10
        else:
            # The interactive 20-robot traffic scenario uses long routes.
            # Validate their complete time horizon so late corridor conflicts
            # cannot hide behind the benchmark speed optimization.
            horizon = None
            iterations = 60
        fleet["batch_collision_horizon_sec"] = horizon
        fleet["batch_wait_max_iterations"] = iterations
        fleet["continuous_collision_step_sec"] = 0.10
        fleet["stretch_motion_to_reservation_ticks"] = True
        planner = getattr(self.owner.manager, "planner", None)
        if planner is not None:
            planner.planner_backend = "hybrid"
            planner.stretch_motion_to_reservation_ticks = True
        return previous

    def _restore_fleet_params(self, previous: dict[str, Any]) -> None:
        params = getattr(self.owner.manager, "params", None)
        if not isinstance(params, dict):
            return
        fleet = params.setdefault("fleet", {})
        if not isinstance(fleet, dict):
            return
        for key, value in previous.items():
            if key == "_planner_backend":
                planner = getattr(self.owner.manager, "planner", None)
                if planner is not None and value is not None:
                    planner.planner_backend = value
                continue
            if key == "_planner_stretch_motion_to_reservation_ticks":
                planner = getattr(self.owner.manager, "planner", None)
                if planner is not None and value is not None:
                    planner.stretch_motion_to_reservation_ticks = value
                continue
            if value is None:
                fleet.pop(key, None)
            else:
                fleet[key] = value

    def _benchmark_requests(
        self,
        *,
        count: int,
        seed: int,
        stress: bool = False,
        stress_profile: int = 0,
    ) -> list[dict[str, Any]]:
        names = self.owner._largest_benchmark_component()
        if len(names) < count * 2:
            raise ValueError(
                f"benchmark needs at least {count * 2} connected LMs; "
                f"largest component has {len(names)}"
            )
        rng = random.Random(seed + count)
        shuffled = self.owner._corridor_safe_benchmark_lms(names, rng)
        starts = self.owner._spatially_separated_lms(shuffled, count)
        if len(starts) < count:
            raise ValueError(
                f"benchmark can safely place only {len(starts)} of {count} robots"
            )
        used_goals: set[str] = set()
        requests: list[dict[str, Any]] = []
        start_set = set(starts)
        for index, start_name in enumerate(starts, start=1):
            candidates = self.owner._forward_benchmark_goals(
                start_name,
                used_goals,
                start_set,
                random.Random(seed + (index * 1009)),
                **self.owner._traffic_goal_window(stress, stress_profile, count),
            )
            if not candidates:
                raise ValueError("not enough physically separated benchmark goals")
            if stress:
                goal_name = self.owner._traffic_goal_from_candidates(
                    candidates,
                    stress_profile,
                    count,
                )
            else:
                goal_name = candidates[0]
            used_goals.add(goal_name)
            requests.append({
                "name": f"bench_{index:03d}",
                "startLm": start_name,
                "goalLm": goal_name,
            })
        return requests

    def _benchmark_requests_for_existing(
        self,
        *,
        count: int,
        seed: int,
        stress: bool = False,
        stress_profile: int = 0,
    ) -> list[dict[str, Any]]:
        robots = self.owner._benchmark_sim_robots()
        if count > 0:
            robots = robots[:count]
        if not robots:
            raise ValueError("add robots before planning")

        names = self.owner._largest_benchmark_component()
        if len(names) < len(robots):
            raise ValueError(
                f"planning needs at least {len(robots)} connected LMs; "
                f"largest component has {len(names)}"
            )
        name_set = set(names)
        planning_starts: dict[str, str] = {}
        for robot in robots:
            start_lm = self.owner.manager._safe_replan_start_lm(robot)
            if start_lm not in name_set:
                raise ValueError(
                    f"{robot.name} is between graph landmarks; "
                    "wait until it reaches the next LM before planning"
                )
            planning_starts[str(robot.name)] = start_lm
        start_lms = set(planning_starts.values())
        avoid_start_goals = len(names) >= len(robots) * 2
        used_goals: set[str] = set()
        requests: list[dict[str, Any]] = []
        for index, robot in enumerate(robots, start=1):
            start_lm = planning_starts[str(robot.name)]
            candidates = self.owner._forward_benchmark_goals(
                start_lm,
                used_goals,
                start_lms if avoid_start_goals else set(),
                random.Random(seed + (index * 1009)),
                start_yaw=(
                    float(robot.pose.get("yaw", 0.0) or 0.0)
                    if isinstance(robot.pose, dict)
                    else 0.0
                ),
                **self.owner._traffic_goal_window(
                    stress,
                    stress_profile,
                    len(robots),
                ),
            )
            if not candidates:
                candidates = self.owner._forward_benchmark_goals(
                    start_lm,
                    used_goals,
                    start_lms if avoid_start_goals else set(),
                    random.Random(seed + (index * 1009)),
                    min_hops=1,
                    max_hops=len(names),
                    start_yaw=(
                        float(robot.pose.get("yaw", 0.0) or 0.0)
                        if isinstance(robot.pose, dict)
                        else 0.0
                    ),
                )
            if not candidates:
                raise ValueError(f"no target LM available for {robot.name}")
            if stress:
                goal_lm = self.owner._traffic_goal_from_candidates(
                    candidates,
                    stress_profile,
                    len(robots),
                )
            else:
                goal_lm = candidates[0]
            used_goals.add(goal_lm)
            request: dict[str, Any] = {
                "name": str(robot.name),
                "startLm": start_lm,
                "goalLm": goal_lm,
            }
            if isinstance(robot.pose, dict):
                request["startPose"] = dict(robot.pose)
            requests.append(request)
        return requests

    def _traffic_goal_window(
        self,
        stress: bool,
        profile: int,
        count: int,
    ) -> dict[str, int]:
        if not stress:
            return {"min_hops": 3, "max_hops": 15}
        profiles = (
            ((5, 20), (6, 20), (4, 18))
            if count <= 30
            else ((4, 16), (5, 18), (3, 15))
        )
        minimum, maximum = profiles[min(max(0, profile), len(profiles) - 1)]
        return {"min_hops": minimum, "max_hops": maximum}

    def _traffic_goal_from_candidates(
        self,
        candidates: list[str],
        profile: int,
        count: int,
    ) -> str:
        fractions = (0.50, 0.60, 0.40) if count <= 30 else (0.35, 0.45, 0.30)
        fraction = fractions[min(max(0, profile), len(fractions) - 1)]
        index = min(len(candidates) - 1, max(0, int(len(candidates) * fraction)))
        return candidates[index]

    def _benchmark_plan_stats(
        self,
        plans: list[Any],
        time_step_sec: float,
    ) -> dict[str, Any]:
        waiting_robots = 0
        total_wait_ticks = 0
        max_wait_ticks = 0
        route_steps: list[int] = []
        for plan in plans:
            if not isinstance(plan, dict):
                continue
            nodes = [str(node) for node in plan.get("nodes", [])]
            times = [int(value) for value in plan.get("times", [])]
            robot_wait_ticks = 0
            move_steps = 0
            for index in range(1, min(len(nodes), len(times))):
                duration = max(0, times[index] - times[index - 1])
                if nodes[index] == nodes[index - 1]:
                    robot_wait_ticks += duration
                else:
                    move_steps += 1
            if robot_wait_ticks > 0:
                waiting_robots += 1
            total_wait_ticks += robot_wait_ticks
            max_wait_ticks = max(max_wait_ticks, robot_wait_ticks)
            route_steps.append(move_steps)
        return {
            "plannedWaitingRobots": waiting_robots,
            "plannedWaitTicks": total_wait_ticks,
            "plannedWaitSec": round(
                total_wait_ticks * max(0.0, time_step_sec),
                3,
            ),
            "maxPlannedWaitTicks": max_wait_ticks,
            "averageRouteSteps": (
                round(sum(route_steps) / len(route_steps), 2)
                if route_steps
                else 0.0
            ),
            "maxRouteSteps": max(route_steps, default=0),
        }
