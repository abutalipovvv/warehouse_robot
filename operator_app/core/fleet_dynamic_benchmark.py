"""Dynamic benchmark runtime state machine."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import random
from time import perf_counter, time
from typing import Any


@dataclass(slots=True)
class DynamicBenchmarkSettings:
    requested_horizon_sec: float
    horizon_sec: float
    generation_mode: str
    order_interval_sec: float
    queue_depth: int


class DynamicBenchmarkRuntime:
    """State machine for continuous orders and package-wave benchmarks."""

    def __init__(
        self,
        owner: Any,
        *,
        benchmark_sim_robots: (
            Callable[[], list[Any]] | None
        ) = None,
    ) -> None:
        self.owner = owner
        if benchmark_sim_robots is None:
            benchmark_commands = getattr(
                owner,
                "_benchmark_commands",
                None,
            )
            benchmark_sim_robots = getattr(
                benchmark_commands,
                "_benchmark_sim_robots",
                None,
            )
        self._benchmark_sim_robots = benchmark_sim_robots

    def _start_dynamic_benchmark_payload(
        self,
        *,
        count: int,
        seed: int,
        horizon_sec: float,
        order_interval_sec: float,
        queue_depth: int,
        speed: float,
        acceleration: float,
        rotate: bool,
        turn_speed: float,
        generation_mode: str = "continuous",
    ) -> dict[str, Any]:
        robots = self._prepare_start_robots(count)
        settings = self._start_settings(
            horizon_sec=horizon_sec,
            order_interval_sec=order_interval_sec,
            queue_depth=queue_depth,
            generation_mode=generation_mode,
            robot_count=len(robots),
        )
        self._prepare_manager(settings.horizon_sec)
        now = self.owner._runtime_now()
        self.owner._dynamic_rng = random.Random(seed)
        self.owner._dynamic_benchmark = self._new_dynamic_config(
            settings,
            seed=seed,
            now=now,
            speed=speed,
            acceleration=acceleration,
            rotate=rotate,
            turn_speed=turn_speed,
        )
        if settings.generation_mode == "continuous":
            next_order_at = self.owner._dynamic_benchmark[
                "nextOrderAt"
            ]
            for robot in robots:
                next_order_at[robot.name] = now
        return self._start_response(
            robots,
            settings=settings,
            now=now,
        )

    def _prepare_start_robots(self, count: int) -> list[Any]:
        robots = self.owner._benchmark_sim_robots()
        if count > 0:
            robots = robots[:count]
        if not robots:
            raise ValueError("add robots before starting dynamic orders")
        pending = [
            order
            for order in self.owner.manager.orders.values()
            if order.order_id.startswith("dynamic-")
            and order.status
            not in {"COMPLETED", "FAILED", "CANCELED"}
        ]
        if pending:
            raise ValueError(
                f"wait for {len(pending)} active benchmark order(s) "
                "to finish before starting a new generator"
            )
        for order_id, order in list(
            self.owner.manager.orders.items()
        ):
            if (
                order_id.startswith("dynamic-")
                and order.status
                in {"COMPLETED", "FAILED", "CANCELED"}
            ):
                self.owner.manager.orders.pop(order_id, None)
        return robots

    def _start_settings(
        self,
        *,
        horizon_sec: float,
        order_interval_sec: float,
        queue_depth: int,
        generation_mode: str,
        robot_count: int,
    ) -> DynamicBenchmarkSettings:
        requested_horizon = max(
            1.0,
            min(120.0, float(horizon_sec)),
        )
        safe_horizon = self.owner._safe_dynamic_rolling_horizon(
            requested_horizon,
            robot_count=robot_count,
        )
        normalized_mode = (
            "package_waves"
            if str(generation_mode).strip().lower()
            == "package_waves"
            else "continuous"
        )
        interval = (
            0.0
            if normalized_mode == "package_waves"
            else max(
                0.25,
                min(120.0, float(order_interval_sec)),
            )
        )
        return DynamicBenchmarkSettings(
            requested_horizon_sec=requested_horizon,
            horizon_sec=safe_horizon,
            generation_mode=normalized_mode,
            order_interval_sec=interval,
            queue_depth=max(1, min(5, int(queue_depth))),
        )

    def _prepare_manager(self, horizon_sec: float) -> None:
        fleet = self.owner.manager.params.setdefault("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
            self.owner.manager.params["fleet"] = fleet
        fleet["rolling_horizon_sec"] = horizon_sec
        self.owner.manager.reset_planning_runtime_state()
        for key in getattr(
            self.owner.manager,
            "traffic_metrics",
            {},
        ):
            self.owner.manager.traffic_metrics[key] = 0
        self.owner.manager._last_runtime_safety_rollback = None

    @staticmethod
    def _new_dynamic_config(
        settings: DynamicBenchmarkSettings,
        *,
        seed: int,
        now: float,
        speed: float,
        acceleration: float,
        rotate: bool,
        turn_speed: float,
    ) -> dict[str, Any]:
        return {
            "active": True,
            "generationMode": settings.generation_mode,
            "scenario": (
                "package_order_waves"
                if settings.generation_mode == "package_waves"
                else "continuous_random_orders"
            ),
            "seed": seed,
            "startedAt": now,
            "stoppedAt": 0.0,
            "horizonSec": settings.horizon_sec,
            "horizonRequestedSec": (
                settings.requested_horizon_sec
            ),
            "orderIntervalSec": settings.order_interval_sec,
            "queueDepth": settings.queue_depth,
            "ordersGenerated": 0,
            "ordersCompleted": 0,
            "ordersTerminated": 0,
            "generationFailures": 0,
            "generatedDistanceMTotal": 0.0,
            "lastOrderDistanceM": 0.0,
            "dispatchElapsedMs": 0.0,
            "lastOrderAt": 0.0,
            "lastPumpAt": now,
            "nextOrderAt": {},
            "orderSequence": 0,
            "sessionId": int(now * 1000),
            "countedTerminalOrders": set(),
            "completedDurationsSec": [],
            "completedDurationTotalSec": 0.0,
            "lastTerminalAt": 0.0,
            "measurementFinishedAt": 0.0,
            "speed": speed,
            "acceleration": acceleration,
            "rotate": rotate,
            "turnSpeed": turn_speed,
            "initialWaveQueued": False,
            "waveOrderIds": set(),
            "waveIndex": 0,
            "wavesStarted": 0,
            "wavesCompleted": 0,
            "waveStartedAt": 0.0,
            "lastWaveDurationSec": 0.0,
            "waveDurationTotalSec": 0.0,
            "packageWaveOrderIds": {},
            "packageWaveRobots": {},
            "packageWaveStartedAt": {},
            "packageCompletedWaves": set(),
            "packageRobotRounds": {},
        }

    def _start_response(
        self,
        robots: list[Any],
        *,
        settings: DynamicBenchmarkSettings,
        now: float,
    ) -> dict[str, Any]:
        started = perf_counter()
        generated = self.owner._pump_dynamic_benchmark(now=now)
        elapsed_ms = (perf_counter() - started) * 1000.0
        benchmark = self.owner._dynamic_benchmark_payload()
        benchmark.update(
            {
                "action": (
                    "start_package_waves"
                    if settings.generation_mode == "package_waves"
                    else "start_dynamic"
                ),
                "scenario": self.owner._dynamic_benchmark[
                    "scenario"
                ],
                "count": len(robots),
                "planned": len(robots),
                "generatedNow": generated,
                "elapsedMs": round(elapsed_ms, 3),
            }
        )
        state = self.owner._state_with_context(
            self.owner.manager.state(include_trajectories=True)
        )
        return self.owner._result_with_context(
            {
                "ok": True,
                "benchmark": benchmark,
                "state": state,
            }
        )

    def _safe_dynamic_rolling_horizon(
        self,
        requested: float,
        *,
        robot_count: int | None = None,
    ) -> float:
        """Return the operator's committed planning window.

        Corridor scheduling lookahead and MAPF commitment are independent
        horizons. ``_rolling_planning_goal`` already extends a boundary that
        lands inside a no-wait passage to its next external safe LM, while the
        low-level budget separately includes the longest corridor. The old
        sum turned a requested 10 seconds into 66.7 seconds (120 at 4x).
        """

        _ = robot_count  # Kept for API compatibility with benchmark callers.
        return max(1.0, min(120.0, float(requested)))

    def _stop_dynamic_benchmark_payload(self) -> dict[str, Any]:
        if self.owner._dynamic_benchmark.get("active"):
            self.owner._dynamic_benchmark["active"] = False
            self.owner._dynamic_benchmark["stoppedAt"] = self.owner._runtime_now()
        benchmark = self.owner._dynamic_benchmark_payload()
        benchmark["action"] = "stop_dynamic"
        state = self.owner._state_with_context(self.owner.manager.state(include_trajectories=True))
        return self.owner._result_with_context({
            "ok": True,
            "benchmark": benchmark,
            "state": state,
        })

    def _pump_dynamic_benchmark(self, now: float | None = None) -> int:
        owner = self.owner
        if owner.mode != "simulation" or not hasattr(owner, "manager"):
            return 0
        manager = owner.manager
        overrides = owner.__dict__
        prune_history = overrides.get(
            "_prune_dynamic_order_history",
            self._prune_dynamic_order_history,
        )
        if now is None:
            runtime_now = overrides.get("_runtime_now", self._runtime_now)
            now = runtime_now()
        else:
            now = float(now)
        config = getattr(owner, "_dynamic_benchmark", {})
        prune_history()
        if str(config.get("generationMode") or "continuous") == "package_waves":
            finish_waves = overrides.get(
                "_finish_terminal_package_waves",
                self._finish_terminal_package_waves,
            )
            finish_waves(now)
            if not config.get("active"):
                config["lastPumpAt"] = now
                return 0
            top_up_orders = overrides.get(
                "_top_up_package_orders",
                self._top_up_package_orders,
            )
            generated = top_up_orders(now)
            config["lastPumpAt"] = now
            return generated
        if not config.get("active"):
            config["lastPumpAt"] = now
            return 0
        benchmark_robots = overrides.get("_benchmark_sim_robots")
        if benchmark_robots is None:
            benchmark_robots = self._benchmark_sim_robots
        robots = benchmark_robots()
        next_order_at = config.setdefault("nextOrderAt", {})
        interval = float(config.get("orderIntervalSec", 3.0) or 3.0)
        queue_depth = int(config.get("queueDepth", 2) or 2)
        order_depth = overrides.get("_dynamic_order_depth")
        if order_depth is None:
            depths = self._dynamic_order_depths(
                robots,
                manager.orders.values(),
            )
        else:
            depths = {
                robot.name: order_depth(robot.name)
                for robot in robots
            }
        due = [
            robot for robot in robots
            if depths[robot.name] < queue_depth
            and (
                depths[robot.name] == 0
                or now >= float(next_order_at.get(robot.name, now))
            )
        ]
        # Coverage is the hard invariant. Queue-depth prefill may stay
        # throttled, but every robot with zero orders is replenished in this
        # same pump instead of waiting behind a two-robot batch limit.
        due.sort(
            key=lambda robot: (
                depths[robot.name] != 0,
                float(next_order_at.get(robot.name, now)),
                robot.name,
            )
        )
        generated = 0
        initial_wave = not bool(config.get("initialWaveQueued", False))
        uncovered_count = sum(depths[robot.name] == 0 for robot in due)
        batch_limit = (
            len(due)
            if initial_wave
            else min(
                len(due),
                uncovered_count
                + overrides.get(
                    "_dynamic_generation_batch_size",
                    self._dynamic_generation_batch_size,
                )(),
            )
        )
        next_order_payload = overrides.get(
            "_next_dynamic_order_payload",
            self._next_dynamic_order_payload,
        )
        record_order = overrides.get(
            "_record_generated_dynamic_order",
            self._record_generated_dynamic_order,
        )
        for robot in due[:batch_limit]:
            started = perf_counter()
            try:
                order_payload = next_order_payload(
                    robot,
                    now,
                )
                if order_payload is None:
                    config["generationFailures"] = int(config.get("generationFailures", 0) or 0) + 1
                    manager._event(
                        "warn",
                        f"dynamic order pending for {robot.name}: no free reachable LM",
                    )
                else:
                    manager.set_order(order_payload, dispatch=False)
                    generated += 1
                    record_order(
                        order_payload,
                        now,
                    )
            except (RuntimeError, ValueError) as exc:
                config["generationFailures"] = int(config.get("generationFailures", 0) or 0) + 1
                manager._event(
                    "warn",
                    f"dynamic order pending for {robot.name}: {exc}",
                )
            finally:
                config["dispatchElapsedMs"] = float(config.get("dispatchElapsedMs", 0.0) or 0.0) + (
                    (perf_counter() - started) * 1000.0
                )
            jitter = owner._dynamic_rng.uniform(0.70, 1.30)
            next_order_at[robot.name] = now + (interval * jitter)
        if initial_wave:
            config["initialWaveQueued"] = True
        config["lastPumpAt"] = now
        prune_history()
        return generated

    @staticmethod
    def _dynamic_order_depths(
        robots: list[Any],
        orders: Any,
    ) -> dict[str, int]:
        """Count active orders for every robot in one queue scan."""
        counts = {robot.name: 0 for robot in robots}
        terminal_statuses = {"COMPLETED", "FAILED", "CANCELED"}
        for order in orders:
            if order.status in terminal_statuses:
                continue
            vehicle = order.vehicle
            assigned_robot = order.assigned_robot
            if vehicle in counts:
                counts[vehicle] += 1
            if assigned_robot != vehicle and assigned_robot in counts:
                counts[assigned_robot] += 1
        return counts

    def _finish_terminal_package_waves(self, now: float) -> int:
        config = self.owner._dynamic_benchmark
        wave_orders = config.setdefault("packageWaveOrderIds", {})
        wave_robots = config.setdefault("packageWaveRobots", {})
        wave_started = config.setdefault("packageWaveStartedAt", {})
        completed_waves = config.setdefault("packageCompletedWaves", set())
        completed_wave_indices: set[int] = set()
        for raw_index in completed_waves:
            try:
                completed_wave_indices.add(int(raw_index))
            except (TypeError, ValueError):
                continue
        completed_total = int(config.get("wavesCompleted", 0) or 0)
        expected_robots = len(self.owner._benchmark_sim_robots())
        completed_now = 0
        finished_indices: set[int] = set()
        for raw_index in sorted(wave_orders, key=int):
            wave_index = int(raw_index)
            if wave_index in completed_wave_indices:
                finished_indices.add(wave_index)
                continue
            order_ids = set(wave_orders.get(raw_index, set()))
            robot_names = set(wave_robots.get(raw_index, set()))
            if (
                expected_robots <= 0
                or len(order_ids) != expected_robots
                or len(robot_names) != expected_robots
            ):
                continue
            if any(
                order_id in self.owner.manager.orders
                and self.owner.manager.orders[order_id].status
                not in {"COMPLETED", "FAILED", "CANCELED"}
                for order_id in order_ids
            ):
                continue
            started_at = float(wave_started.get(raw_index, now) or now)
            duration = max(0.0, now - started_at)
            completed_waves.add(wave_index)
            completed_wave_indices.add(wave_index)
            finished_indices.add(wave_index)
            completed_now += 1
            config["lastWaveDurationSec"] = duration
            config["waveDurationTotalSec"] = float(
                config.get("waveDurationTotalSec", 0.0) or 0.0
            ) + duration
            self.owner.manager._event(
                "info",
                f"package wave {wave_index} completed: "
                f"{len(order_ids)} orders in {duration:.1f} simulated seconds",
            )
        config["wavesCompleted"] = completed_total + completed_now
        # The aggregate counters above are the benchmark record. Keeping all
        # per-wave order-id/name sets forever made this 10 Hz method scan the
        # complete run history after hours of operation.
        for wave_index in finished_indices:
            for mapping in (wave_orders, wave_robots, wave_started):
                mapping.pop(wave_index, None)
                mapping.pop(str(wave_index), None)
            completed_waves.discard(wave_index)
            completed_waves.discard(str(wave_index))
        return completed_now

    def _top_up_package_orders(self, now: float) -> int:
        config = self.owner._dynamic_benchmark
        robots = self.owner._benchmark_sim_robots()
        if not robots:
            return 0
        wave_orders = config.setdefault("packageWaveOrderIds", {})
        wave_robots = config.setdefault("packageWaveRobots", {})

        # Package mode is a barrier workload: exactly one order is issued to
        # every fleet robot, early finishers stay at the perimeter, and only
        # after the complete wave terminates is the next full wave generated.
        # The former per-robot top-up created overlapping 1/1 "waves" and sent
        # early finishers back through aisles that the previous wave was still
        # clearing, which steadily amplified stationary departure conflicts.
        active_indices = sorted(
            {
                int(raw_index)
                for raw_index, order_ids in wave_orders.items()
                if any(
                    order_id in self.owner.manager.orders
                    and self.owner.manager.orders[order_id].status
                    not in {"COMPLETED", "FAILED", "CANCELED"}
                    for order_id in set(order_ids)
                )
                or len(set(order_ids)) < len(robots)
            }
        )
        if active_indices:
            wave_index = active_indices[0]
            present = set(
                wave_robots.get(
                    wave_index,
                    wave_robots.get(str(wave_index), set()),
                )
            )
            missing = [
                robot
                for robot in robots
                if robot.name not in present
                and self.owner._dynamic_order_depth(robot.name) == 0
            ]
            if not missing:
                return 0
            return self.owner._generate_package_orders_for_wave(
                missing,
                wave_index,
                now,
            )

        # Do not overlap an untracked/manual outstanding command with a new
        # benchmark barrier. This also makes recovery from a partially loaded
        # workspace deterministic.
        if any(self.owner._dynamic_order_depth(robot.name) > 0 for robot in robots):
            return 0
        wave_index = max(
            int(config.get("waveIndex", 0) or 0),
            int(config.get("wavesStarted", 0) or 0),
        ) + 1
        return self.owner._generate_package_orders_for_wave(
            robots,
            wave_index,
            now,
        )

    def _generate_package_order_wave(self, now: float) -> int:
        """Backward-compatible entry point for package coverage generation."""
        return self.owner._top_up_package_orders(now)

    def _generate_package_orders_for_wave(
        self,
        robots: list[Any],
        wave_index: int,
        now: float,
    ) -> int:
        config = self.owner._dynamic_benchmark
        assignments = self.owner._package_wave_assignments(robots, wave_index)
        if len(assignments) != len(robots):
            last_failure = float(config.get("lastWaveFailureAt", 0.0) or 0.0)
            if now - last_failure >= 5.0:
                self.owner.manager._event(
                    "warn",
                    f"package wave {wave_index} coverage pending: assigned "
                    f"{len(assignments)}/{len(robots)} peripheral goals",
                )
                config["lastWaveFailureAt"] = now
            config["generationFailures"] = int(
                config.get("generationFailures", 0) or 0
            ) + max(1, len(robots) - len(assignments))
            return 0

        wave_orders = config.setdefault("packageWaveOrderIds", {})
        wave_robots = config.setdefault("packageWaveRobots", {})
        wave_started = config.setdefault("packageWaveStartedAt", {})
        robot_rounds = config.setdefault("packageRobotRounds", {})
        order_ids = wave_orders.setdefault(wave_index, set())
        robot_names = wave_robots.setdefault(wave_index, set())
        new_wave = not order_ids
        generated = 0
        started = perf_counter()
        for index, (robot, target_lm) in enumerate(assignments):
            priority = (wave_index + index) % 3
            order_payload = self.owner._dynamic_order_payload(
                robot,
                target_lm,
                now,
                priority=priority,
                external_prefix=f"package-wave-{wave_index}",
            )
            try:
                self.owner.manager.set_order(order_payload, dispatch=False)
            except (RuntimeError, ValueError) as exc:
                config["generationFailures"] = int(
                    config.get("generationFailures", 0) or 0
                ) + 1
                self.owner.manager._event(
                    "warn",
                    f"package order pending for {robot.name}: {exc}",
                )
                continue
            order_ids.add(str(order_payload["id"]))
            robot_names.add(str(robot.name))
            robot_rounds[str(robot.name)] = wave_index
            generated += 1
            self.owner._record_generated_dynamic_order(order_payload, now)
        config["dispatchElapsedMs"] = float(
            config.get("dispatchElapsedMs", 0.0) or 0.0
        ) + ((perf_counter() - started) * 1000.0)
        if generated != len(robots):
            # Keep the partial wave authoritative. It must drain before a new
            # wave is attempted; duplicate orders are never generated for the
            # robots that were successfully queued.
            config["generationFailures"] = int(
                config.get("generationFailures", 0) or 0
            ) + (len(robots) - generated)
        if generated and new_wave:
            wave_started[wave_index] = now
            config["wavesStarted"] = int(config.get("wavesStarted", 0) or 0) + 1
        config["waveIndex"] = max(
            int(config.get("waveIndex", 0) or 0),
            wave_index,
        )
        config["waveStartedAt"] = float(wave_started.get(wave_index, now) or now)
        config["waveOrderIds"] = set(order_ids)
        config["initialWaveQueued"] = True
        if generated:
            self.owner.manager._event(
                "info",
                f"package wave {wave_index} coverage queued: "
                f"{generated}/{len(robots)} orders to map perimeter",
            )
        return generated

    def _next_dynamic_order_payload(self, robot: Any, now: float) -> dict[str, Any] | None:
        config = self.owner._dynamic_benchmark
        origin = self.owner._dynamic_order_origin(robot.name) or str(robot.current_lm)
        if origin not in self.owner.loaded_map.landmarks:
            return None
        used_goals = {
            str(order.target_lm)
            for order in self.owner.manager.orders.values()
            if order.status not in {"COMPLETED", "FAILED", "CANCELED"}
            and str(order.target_lm) in self.owner.loaded_map.landmarks
        }
        occupied_lms = {
            str(item.current_lm)
            for item in self.owner._benchmark_sim_robots()
            if item.name != robot.name and str(item.current_lm) in self.owner.loaded_map.landmarks
        }
        min_hops, max_hops = self.owner._dynamic_goal_hop_window()
        candidates = self.owner._forward_benchmark_goals(
            origin,
            used_goals,
            occupied_lms,
            self.owner._dynamic_rng,
            min_hops=min_hops,
            max_hops=max_hops,
        )
        if not candidates:
            candidates = self.owner._forward_benchmark_goals(
                origin,
                used_goals,
                occupied_lms,
                self.owner._dynamic_rng,
                min_hops=max(2, min_hops // 3),
                max_hops=min(200, max(max_hops, len(self.owner.loaded_map.landmarks))),
            )
        if not candidates:
            return None
        target_lm = self.owner._far_dynamic_goal(origin, candidates)
        sequence = int(config.get("orderSequence", 0) or 0) + 1
        priority = self.owner._dynamic_rng.choice((0, 0, 1, 1, 2, 3))
        if sequence % 10 == 0:
            priority = 5
        return self.owner._dynamic_order_payload(
            robot,
            target_lm,
            now,
            priority=priority,
            external_prefix="benchmark",
        )

    def _dynamic_order_payload(
        self,
        robot: Any,
        target_lm: str,
        now: float,
        *,
        priority: int,
        external_prefix: str,
    ) -> dict[str, Any]:
        config = self.owner._dynamic_benchmark
        origin = self.owner._dynamic_order_origin(robot.name) or str(robot.current_lm)
        origin_lm = self.owner.loaded_map.landmarks[origin]
        target = self.owner.loaded_map.landmarks[target_lm]
        distance_m = math.hypot(target.x - origin_lm.x, target.y - origin_lm.y)
        sequence = int(config.get("orderSequence", 0) or 0) + 1
        config["orderSequence"] = sequence
        return {
            "id": f"dynamic-{int(config.get('sessionId', 0) or 0)}-{sequence:07d}-{robot.name}",
            "vehicle": robot.name,
            "targetLm": target_lm,
            "priority": priority,
            "speed": float(config.get("speed", 0.0) or 0.0),
            "acceleration": float(config.get("acceleration", 0.0) or 0.0),
            "rotate": bool(config.get("rotate", False)),
            "turnSpeed": float(config.get("turnSpeed", 0.0) or 0.0),
            "stretchMotionToReservationTicks": True,
            "externalId": f"{external_prefix}-{int(now * 1000)}-{sequence}",
            "benchmarkDistanceM": round(distance_m, 3),
        }

    def _record_generated_dynamic_order(
        self,
        order_payload: dict[str, Any],
        now: float,
    ) -> None:
        config = self.owner._dynamic_benchmark
        config["ordersGenerated"] = int(config.get("ordersGenerated", 0) or 0) + 1
        distance_m = float(order_payload.get("benchmarkDistanceM", 0.0) or 0.0)
        config["generatedDistanceMTotal"] = float(
            config.get("generatedDistanceMTotal", 0.0) or 0.0
        ) + distance_m
        config["lastOrderDistanceM"] = distance_m
        config["lastOrderAt"] = now
        config["measurementFinishedAt"] = 0.0

    def _dynamic_order_origin(self, robot_name: str) -> str:
        orders = [
            order for order in self.owner.manager.orders.values()
            if (order.vehicle == robot_name or order.assigned_robot == robot_name)
            and order.status not in {"COMPLETED", "FAILED", "CANCELED"}
        ]
        if not orders:
            robot = self.owner.manager.robots.get(robot_name)
            return str(robot.route_final_lm or robot.current_lm) if robot is not None else ""
        orders.sort(key=lambda order: (order.created_at, order.order_id))
        return str(orders[-1].target_lm)

    def _dynamic_order_depth(self, robot_name: str) -> int:
        orders = self.owner.manager.orders
        return sum(
            1 for order in orders.values()
            if (order.vehicle == robot_name or order.assigned_robot == robot_name)
            and order.status not in {"COMPLETED", "FAILED", "CANCELED"}
        )

    def _dynamic_generation_batch_size(self) -> int:
        # Bound synchronous MAPF work per web tick. Orders still arrive for
        # every robot, but the status stream never blocks on a fleet-wide burst.
        return 2

    def _prune_dynamic_order_history(self) -> None:
        owner = self.owner
        manager = owner.manager
        config = owner._dynamic_benchmark
        counted = config.setdefault("countedTerminalOrders", set())
        session_prefix = (
            f"dynamic-{int(config.get('sessionId', 0) or 0)}-"
        )
        terminal = [
            order for order in manager.orders.values()
            if order.order_id.startswith(session_prefix)
            and order.status in {"COMPLETED", "FAILED", "CANCELED"}
        ]
        terminal.sort(key=lambda order: (order.updated_at, order.order_id), reverse=True)
        for order in terminal:
            if order.order_id in counted:
                continue
            counted.add(order.order_id)
            if order.status == "COMPLETED":
                config["ordersCompleted"] = int(config.get("ordersCompleted", 0) or 0) + 1
                duration = max(0.0, float(order.updated_at) - float(order.created_at))
                durations = config.setdefault("completedDurationsSec", [])
                if isinstance(durations, list):
                    durations.append(duration)
                    del durations[:-1000]
                config["completedDurationTotalSec"] = float(
                    config.get("completedDurationTotalSec", 0.0) or 0.0
                ) + duration
            else:
                config["ordersTerminated"] = int(config.get("ordersTerminated", 0) or 0) + 1
            config["lastTerminalAt"] = max(
                float(config.get("lastTerminalAt", 0.0) or 0.0),
                float(order.updated_at),
            )
        # The operator queue exposes at most 120 records. Retaining additional
        # terminal benchmark orders only increases every lifecycle scan.
        for order in terminal[120:]:
            manager.orders.pop(order.order_id, None)
            counted.discard(order.order_id)

    def _dynamic_benchmark_payload(self) -> dict[str, Any]:
        owner = self.owner
        overrides = owner.__dict__
        has_manager = hasattr(owner, "manager")
        manager = owner.manager if has_manager else None
        config = getattr(owner, "_dynamic_benchmark", {})
        generated = int(config.get("ordersGenerated", 0) or 0)
        session_prefix = (
            f"dynamic-{int(config.get('sessionId', 0) or 0)}-"
        )
        dynamic_orders = [
            order for order in manager.orders.values()
            if order.order_id.startswith(session_prefix)
        ] if manager is not None else []
        overrides.get(
            "_prune_dynamic_order_history",
            self._prune_dynamic_order_history,
        )()
        completed = int(config.get("ordersCompleted", 0) or 0)
        queued = sum(order.status == "QUEUED" for order in dynamic_orders)
        executing = sum(order.status not in {"QUEUED", "COMPLETED", "FAILED", "CANCELED"} for order in dynamic_orders)
        benchmark_sim_robots = overrides.get(
            "_benchmark_sim_robots"
        )
        if benchmark_sim_robots is None:
            benchmark_sim_robots = self._benchmark_sim_robots
        benchmark_robots = (
            benchmark_sim_robots()
            if manager is not None
            else []
        )
        waiting = sum(robot.status == "WAITING" for robot in benchmark_robots)
        robots_with_orders = {
            str(order.vehicle or order.assigned_robot)
            for order in dynamic_orders
            if order.status not in {"COMPLETED", "FAILED", "CANCELED"}
            and str(order.vehicle or order.assigned_robot)
        }
        robots_with_orders &= {str(robot.name) for robot in benchmark_robots}
        traffic = (
            dict(getattr(manager, "traffic_metrics", {}))
            if manager is not None
            else {}
        )
        dispatch_ms = float(config.get("dispatchElapsedMs", 0.0) or 0.0)
        distance_total = float(config.get("generatedDistanceMTotal", 0.0) or 0.0)
        terminated = int(config.get("ordersTerminated", 0) or 0)
        outstanding = max(0, generated - completed - terminated)
        started_at = float(config.get("startedAt", 0.0) or 0.0)
        now = (
            overrides.get("_runtime_now", self._runtime_now)()
            if started_at > 0.0
            else 0.0
        )
        if not config.get("active") and outstanding <= 0 and generated > 0:
            finished_at = float(config.get("measurementFinishedAt", 0.0) or 0.0)
            if finished_at <= 0.0:
                finished_at = max(
                    started_at,
                    float(config.get("lastTerminalAt", 0.0) or 0.0),
                    float(config.get("stoppedAt", 0.0) or 0.0),
                )
                config["measurementFinishedAt"] = finished_at
        else:
            finished_at = 0.0
        elapsed_sim_sec = max(
            0.0,
            (finished_at or now or started_at) - started_at,
        ) if started_at > 0.0 else 0.0
        throughput = (
            (completed * 60.0) / elapsed_sim_sec
            if completed > 0 and elapsed_sim_sec > 0.000001
            else 0.0
        )
        completed_durations = [
            float(value)
            for value in config.get("completedDurationsSec", [])
            if isinstance(value, (int, float))
        ]
        completed_durations.sort()
        p95_duration = (
            completed_durations[
                min(
                    len(completed_durations) - 1,
                    max(0, int(math.ceil(len(completed_durations) * 0.95)) - 1),
                )
            ]
            if completed_durations
            else 0.0
        )
        waves_completed = int(config.get("wavesCompleted", 0) or 0)
        return {
            "active": bool(config.get("active", False)),
            "generationMode": str(config.get("generationMode") or "continuous"),
            "scenario": str(config.get("scenario") or "continuous_random_orders"),
            "seed": int(config.get("seed", 42) or 42),
            "horizonSec": float(config.get("horizonSec", 10.0) or 10.0),
            "effectiveHorizonSec": round(
                float(manager._rolling_horizon()),
                3,
            ),
            "horizonRequestedSec": float(
                config.get(
                    "horizonRequestedSec",
                    config.get("horizonSec", 10.0),
                )
                or 10.0
            ),
            "orderIntervalSec": float(config.get("orderIntervalSec", 3.0) or 3.0),
            "queueDepth": int(config.get("queueDepth", 2) or 2),
            "ordersGenerated": generated,
            "ordersCompleted": completed,
            "ordersQueued": queued,
            "ordersExecuting": executing,
            "ordersOutstanding": outstanding,
            "robotsWithOrders": len(robots_with_orders),
            "robotsWithoutOrders": max(
                0,
                len(benchmark_robots) - len(robots_with_orders),
            ),
            "waitingRobots": waiting,
            "generationFailures": int(config.get("generationFailures", 0) or 0),
            "averageDispatchMs": round(dispatch_ms / generated, 3) if generated else 0.0,
            "averageOrderDistanceM": round(distance_total / generated, 2) if generated else 0.0,
            "lastOrderDistanceM": round(float(config.get("lastOrderDistanceM", 0.0) or 0.0), 2),
            "elapsedSimSec": round(elapsed_sim_sec, 3),
            "throughputOrdersPerMin": round(throughput, 3),
            "averageOrderDurationSec": round(
                float(config.get("completedDurationTotalSec", 0.0) or 0.0) / completed,
                3,
            ) if completed else 0.0,
            "p95OrderDurationSec": round(p95_duration, 3),
            "waveIndex": int(config.get("waveIndex", 0) or 0),
            "wavesStarted": int(config.get("wavesStarted", 0) or 0),
            "wavesCompleted": waves_completed,
            "waveOrders": len(set(config.get("waveOrderIds", set()))),
            "lastWaveDurationSec": round(
                float(config.get("lastWaveDurationSec", 0.0) or 0.0),
                3,
            ),
            "averageWaveDurationSec": round(
                float(config.get("waveDurationTotalSec", 0.0) or 0.0)
                / waves_completed,
                3,
            ) if waves_completed else 0.0,
            "timeScale": (
                manager.simulation_time_scale()
                if manager is not None
                else 1.0
            ),
            **traffic,
        }

    def _runtime_now(self) -> float:
        owner = self.owner
        if owner.mode == "simulation" and hasattr(owner, "manager"):
            return owner.manager.simulation_time()
        return time()

    def _reset_dynamic_benchmark(self) -> None:
        self.owner._dynamic_rng = random.Random(42)
        self.owner._dynamic_benchmark = {
            "active": False,
            "generationMode": "continuous",
            "scenario": "continuous_random_orders",
            "seed": 42,
            "horizonSec": 10.0,
            "orderIntervalSec": 3.0,
            "queueDepth": 2,
            "ordersGenerated": 0,
            "ordersCompleted": 0,
            "ordersTerminated": 0,
            "generationFailures": 0,
            "generatedDistanceMTotal": 0.0,
            "lastOrderDistanceM": 0.0,
            "dispatchElapsedMs": 0.0,
            "nextOrderAt": {},
            "orderSequence": 0,
            "sessionId": 0,
            "countedTerminalOrders": set(),
            "completedDurationsSec": [],
            "completedDurationTotalSec": 0.0,
            "lastTerminalAt": 0.0,
            "measurementFinishedAt": 0.0,
            "initialWaveQueued": False,
            "waveOrderIds": set(),
            "waveIndex": 0,
            "wavesStarted": 0,
            "wavesCompleted": 0,
            "waveStartedAt": 0.0,
            "lastWaveDurationSec": 0.0,
            "waveDurationTotalSec": 0.0,
            "packageWaveOrderIds": {},
            "packageWaveRobots": {},
            "packageWaveStartedAt": {},
            "packageCompletedWaves": set(),
            "packageRobotRounds": {},
        }
