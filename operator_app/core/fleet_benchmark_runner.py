"""Headless long-running benchmarks for the operator fleet simulator."""

from __future__ import annotations

import argparse
from collections import deque
import cProfile
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
from time import monotonic, sleep
from typing import Any

from operator_app.core.fleet_context import FLEET_ROOT, FLEET_MAPS_OUT_ROOT
from operator_app.core.fleet_manager import (
    FLEET_MANAGER_SIM_ID,
    OperatorFleetManager,
)


@dataclass(frozen=True, slots=True)
class FleetBenchmarkScenario:
    name: str
    robot_count: int
    duration_sec: float
    map_name: str
    traffic_zones: bool
    controlled_corridors: bool
    planning_horizon_sec: float = 75.0


SCENARIOS = {
    "open-50": FleetBenchmarkScenario(
        "open-50",
        50,
        30.0 * 60.0,
        "benchmark_open_kiva.smap",
        False,
        False,
    ),
    "open-100": FleetBenchmarkScenario(
        "open-100",
        100,
        30.0 * 60.0,
        "benchmark_open_kiva_rds360.smap",
        False,
        False,
    ),
    "zones-100": FleetBenchmarkScenario(
        "zones-100",
        100,
        30.0 * 60.0,
        "smart_kiva_large_w_mode.smap",
        True,
        False,
    ),
    "corridors-100": FleetBenchmarkScenario(
        "corridors-100",
        100,
        30.0 * 60.0,
        "smart_kiva_large_w_mode.smap",
        True,
        True,
    ),
}


class FleetBenchmarkRunner:
    """Drive the real operator simulation without a browser or WebSocket."""

    def __init__(
        self,
        scenario: FleetBenchmarkScenario,
        *,
        seed: int = 42,
        deterministic: bool = False,
        tick_sec: float = 0.1,
        profile_path: Path | None = None,
    ) -> None:
        self.scenario = scenario
        self.seed = int(seed)
        self.deterministic = bool(deterministic)
        self.tick_sec = max(0.01, float(tick_sec))
        self.profile_path = profile_path
        self._tick_duration_samples: deque[float] = deque(maxlen=8192)

    def run(self) -> dict[str, Any]:
        service = self._create_service()
        fake_clock = self._install_fake_clock(service) if self.deterministic else None
        manager = service.manager
        initial_memory = self._resident_memory_mib()
        warm_memory = initial_memory
        warmup_recorded = False
        collision_pairs: set[tuple[str, str]] = set()
        collision_count = 0
        started_sim = manager.simulation_time()
        started_wall = monotonic()
        warmup_at = started_sim + min(60.0, self.scenario.duration_sec * 0.20)
        next_progress_at = started_sim + 60.0
        profiler: cProfile.Profile | None = None

        try:
            service.benchmark_payload({
                "action": "add",
                "count": self.scenario.robot_count,
                "seed": self.seed,
                "reset": False,
            })
            service.benchmark_payload({
                "action": "plan",
                "count": self.scenario.robot_count,
                "seed": self.seed,
                "reset": False,
                # Benchmark duration and prepared-route horizon are different
                # controls. Coupling them made a 20-second smoke run use tiny
                # chunks and a long acceptance run silently use the 120-second
                # UI cap, so the two runs measured different systems.
                "horizonSec": self.scenario.planning_horizon_sec,
                "orderIntervalSec": 1.0,
                "queueDepth": 1,
            })

            while manager.simulation_time() - started_sim < self.scenario.duration_sec:
                if fake_clock is not None:
                    fake_clock[0] += self.tick_sec
                tick_started = monotonic()
                service.tick_payload({})
                self._tick_duration_samples.append(
                    (monotonic() - tick_started) * 1000.0
                )
                current_pairs = self._physical_collision_pairs(manager)
                collision_count += len(current_pairs - collision_pairs)
                collision_pairs = current_pairs
                sim_now = manager.simulation_time()
                if sim_now >= warmup_at and not warmup_recorded:
                    warm_memory = self._resident_memory_mib()
                    warmup_recorded = True
                    self._tick_duration_samples.clear()
                    metrics = manager.planning_state.rolling_metrics
                    metrics.queue_wait_samples.clear()
                    metrics.solver_duration_samples.clear()
                    metrics.route_buffer_samples.clear()
                    if self.profile_path is not None:
                        profiler = cProfile.Profile()
                        profiler.enable()
                if sim_now >= next_progress_at:
                    next_progress_at += 60.0
                    print(
                        json.dumps(
                            self._summary(
                                service,
                                started_sim=started_sim,
                                started_wall=started_wall,
                                collision_count=collision_count,
                                warm_memory=warm_memory,
                            ),
                            sort_keys=True,
                        ),
                        file=sys.stderr,
                        flush=True,
                    )
                sleep(0.001 if fake_clock is not None else self.tick_sec)

            return self._summary(
                service,
                started_sim=started_sim,
                started_wall=started_wall,
                collision_count=collision_count,
                warm_memory=warm_memory,
            )
        finally:
            if profiler is not None and self.profile_path is not None:
                profiler.disable()
                profiler.dump_stats(str(self.profile_path))
            service.close()

    def _create_service(self) -> OperatorFleetManager:
        map_dir = FLEET_MAPS_OUT_ROOT / self.scenario.map_name
        service = OperatorFleetManager(
            map_dir,
            FLEET_ROOT / "config" / "params.yaml",
            manager_id=FLEET_MANAGER_SIM_ID,
            mode="simulation",
            params_overrides={
                "fleet": {
                    "traffic_zone_control_enabled": (
                        self.scenario.traffic_zones
                    ),
                    "controlled_corridors_enabled": (
                        self.scenario.controlled_corridors
                    ),
                    "simulation_time_scale": 1.0,
                }
            },
        )
        return service

    @staticmethod
    def _install_fake_clock(service: OperatorFleetManager) -> list[float]:
        manager = service.manager
        clock = [0.0]
        manager._clock = lambda: clock[0]
        with manager._simulation_clock_lock:
            manager._simulation_clock = 0.0
            manager._simulation_clock_wall_at = 0.0
            manager._simulation_time_scale = 1.0
        return clock

    def _summary(
        self,
        service: OperatorFleetManager,
        *,
        started_sim: float,
        started_wall: float,
        collision_count: int,
        warm_memory: float,
    ) -> dict[str, Any]:
        manager = service.manager
        benchmark = service._dynamic_benchmark_payload()
        rolling_metrics = manager.planning_state.rolling_metrics
        current_memory = self._resident_memory_mib()
        elapsed_sim = max(0.0, manager.simulation_time() - started_sim)
        completed_orders = int(benchmark.get("ordersCompleted", 0) or 0)
        rolling_problems = self._rolling_problem_robots(manager)
        problem_names = {
            str(problem.get("robot") or "")
            for problem in rolling_problems
            if str(problem.get("robot") or "")
        }
        return {
            **benchmark,
            "scenario": self.scenario.name,
            "robots": self.scenario.robot_count,
            "durationSimSec": round(elapsed_sim, 3),
            "durationWallSec": round(monotonic() - started_wall, 3),
            "throughputOrdersPerMin": round(
                (
                    completed_orders * 60.0 / elapsed_sim
                    if elapsed_sim > 0.0
                    else 0.0
                ),
                3,
            ),
            "physicalCollisions": collision_count,
            "deadlockCycles": int(
                manager.traffic_metrics.get("waitCyclesDetected", 0)
            ),
            "memoryRssMiB": round(current_memory, 3),
            "memoryGrowthAfterWarmupMiB": round(
                max(0.0, current_memory - warm_memory),
                3,
            ),
            "rollingProblemRobots": rolling_problems,
            "rollingProblemEventTail": [
                str(event.message)
                for event in manager.events
                if any(
                    robot_name in str(event.message)
                    for robot_name in problem_names
                )
            ][-30:],
            "rollingUnderrunEventTail": list(
                rolling_metrics.underrun_events
            )[-12:],
            "runtimeSafetyLastPairs": self._runtime_safety_last_pairs(
                manager
            ),
            "deadlockEventTail": self._deadlock_event_tail(manager),
            "waitingReasons": self._waiting_reasons(manager),
            "p99RuntimeTickMs": round(
                self._percentile(self._tick_duration_samples, 0.99),
                3,
            ),
        }

    @staticmethod
    def _percentile(values: deque[float], ratio: float) -> float:
        if not values:
            return 0.0
        ordered = sorted(values)
        index = min(
            len(ordered) - 1,
            max(0, int(math.ceil(len(ordered) * ratio)) - 1),
        )
        return float(ordered[index])

    @staticmethod
    def _rolling_problem_robots(manager: Any) -> list[dict[str, Any]]:
        """Return bounded diagnostics only for continuations near failure."""

        policy = manager._rolling_buffer_policy()
        problems: list[dict[str, Any]] = []
        for robot in sorted(manager.robots.values(), key=lambda item: item.name):
            final_goal = str(robot.route_final_lm or "")
            needs_continuation = bool(
                robot.active_order_id
                and robot.route_chunk_goal_lm
                and final_goal
                and robot.route_chunk_goal_lm != final_goal
            )
            if not needs_continuation:
                continue
            buffer_sec = robot.route_buffer_seconds
            if (
                buffer_sec > policy.critical_sec
                and not manager._robot_waits_at_rolling_boundary(robot)
            ):
                continue
            trajectory_end = (
                float(robot.trajectory[-1].get("t", 0.0) or 0.0)
                if robot.trajectory
                else 0.0
            )
            problems.append({
                "robot": robot.name,
                "status": robot.status,
                "bufferSec": round(buffer_sec, 3),
                "routeClock": round(float(robot.route_clock), 3),
                "trajectoryEnd": round(trajectory_end, 3),
                "chunkGoal": robot.route_chunk_goal_lm,
                "finalGoal": final_goal,
                "routeRevision": int(robot.route_revision),
                "refillStatus": robot.rolling_refill_status,
                "refillJobId": robot.rolling_refill_job_id,
                "appendStatus": robot.rolling_append_status,
                "lastReason": robot.last_reason,
                "controlledCorridorWait": (
                    manager._robot_waits_for_controlled_corridor(robot)
                ),
                "waitFor": robot.wait_for_robot,
                "retryAt": round(float(
                    manager.planning_state.rolling_prefetch_retry_at.get(
                        robot.name,
                        0.0,
                    )
                ), 3),
                "lastAttemptAt": round(float(
                    manager.planning_state.rolling_prefetch_last_attempt_at.get(
                        robot.name,
                        0.0,
                    )
                ), 3),
                "failures": int(
                    manager.planning_state.rolling_prefetch_failures.get(
                        robot.name,
                        0,
                    )
                ),
                "blockers": manager.planning_state.rolling_prefetch_blockers.get(
                    robot.name,
                    {},
                ),
            })
            if len(problems) >= 20:
                break
        return problems

    @staticmethod
    def _runtime_safety_last_pairs(manager: Any) -> list[dict[str, Any]]:
        rollback = manager._last_runtime_safety_rollback
        if not isinstance(rollback, dict):
            return []
        pairs = rollback.get("pairs", [])
        return [
            {
                "robots": list(pair.get("robots", [])),
                "kind": str(pair.get("kind") or ""),
                "beforeEdges": {
                    name: str(context.get("edgeId") or "")
                    for name, context in pair.get("before", {}).items()
                    if isinstance(context, dict)
                },
                "proposedEdges": {
                    name: str(context.get("edgeId") or "")
                    for name, context in pair.get("proposed", {}).items()
                    if isinstance(context, dict)
                },
            }
            for pair in pairs[:10]
            if isinstance(pair, dict)
        ]

    @staticmethod
    def _waiting_reasons(manager: Any) -> dict[str, int]:
        reasons: dict[str, int] = {}
        for robot in manager.robots.values():
            if robot.status != "WAITING":
                continue
            reason = str(robot.last_reason or "unknown")
            reasons[reason] = reasons.get(reason, 0) + 1
        return dict(sorted(reasons.items()))

    @staticmethod
    def _deadlock_event_tail(manager: Any) -> list[str]:
        messages = [
            str(event.message)
            for event in manager.events
            if (
                "wait cycle" in str(event.message).lower()
                or "deadlock" in str(event.message).lower()
            )
        ]
        return messages[-12:]

    @staticmethod
    def _physical_collision_pairs(manager: Any) -> set[tuple[str, str]]:
        robots = [
            robot
            for robot in manager.robots.values()
            if robot.status not in {"DISCONNECTED", "OFFLINE"}
        ]
        pairs: set[tuple[str, str]] = set()
        for index, first in enumerate(robots):
            for second in robots[index + 1:]:
                if manager.collision.footprints_overlap(
                    first.pose,
                    second.pose,
                ):
                    pairs.add(tuple(sorted((first.name, second.name))))
        return pairs

    @staticmethod
    def _resident_memory_mib() -> float:
        statm = Path("/proc/self/statm")
        try:
            resident_pages = int(statm.read_text(encoding="utf-8").split()[1])
        except (OSError, IndexError, TypeError, ValueError):
            return 0.0
        return (resident_pages * os.sysconf("SC_PAGE_SIZE")) / (1024.0 * 1024.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), default="open-50")
    parser.add_argument("--duration-sec", type=float)
    parser.add_argument("--robots", type=int)
    parser.add_argument("--planning-horizon-sec", type=float)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--deterministic", action="store_true")
    parser.add_argument("--profile-path", type=Path)
    args = parser.parse_args()
    configured = SCENARIOS[args.scenario]
    scenario = FleetBenchmarkScenario(
        name=configured.name,
        robot_count=max(1, args.robots or configured.robot_count),
        duration_sec=max(0.1, args.duration_sec or configured.duration_sec),
        map_name=configured.map_name,
        traffic_zones=configured.traffic_zones,
        controlled_corridors=configured.controlled_corridors,
        planning_horizon_sec=max(
            1.0,
            args.planning_horizon_sec or configured.planning_horizon_sec,
        ),
    )
    result = FleetBenchmarkRunner(
        scenario,
        seed=args.seed,
        deterministic=args.deterministic,
        profile_path=args.profile_path,
    ).run()
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
