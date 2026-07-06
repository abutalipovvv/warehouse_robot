from __future__ import annotations

import math
import random
from pathlib import Path
from time import perf_counter
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
FLEET_ROOT = PROJECT_ROOT / "fleet_manager"
FLEET_MAP_DATA_ROOT = FLEET_ROOT / "map_data"
FLEET_MAPS_OUT_ROOT = FLEET_MAP_DATA_ROOT / "maps_out"
DEFAULT_FLEET_MAP_DIR = FLEET_MAPS_OUT_ROOT / "22.05.26_smap.smap"
DEFAULT_FLEET_SIM_MAP_DIR = FLEET_MAPS_OUT_ROOT / "benchmark_open_kiva.smap"
FLEET_MANAGER_ID = "__fleet_manager__"
FLEET_MANAGER_SIM_ID = "__fleet_manager_sim__"
FLEET_MANAGER_IDS = {FLEET_MANAGER_ID, FLEET_MANAGER_SIM_ID}

from fleet_manager.web_simulator import FleetManager
from fleet_manager.route_core import (
    WarehouseMapLoader,
    build_editable_map_bundle_payload,
    build_editable_map_payload,
    list_editable_maps,
    load_route_params,
    save_route_params,
    save_editable_map,
)


class OperatorFleetManager:
    def __init__(
        self,
        map_dir: Path,
        params_path: Path,
        remote_adapter: Any | None = None,
        *,
        manager_id: str = FLEET_MANAGER_ID,
        display_name: str = "Fleet Manager",
        mode: str = "robots",
    ) -> None:
        self.params_path = Path(params_path).expanduser().resolve()
        self.remote_adapter = remote_adapter
        mode = str(mode or "").strip().lower()
        if mode not in {"robots", "simulation"}:
            raise ValueError("mode must be robots or simulation")
        self.manager_id = manager_id
        self.display_name = display_name
        self.mode = mode
        self.map_dir = self.resolve_map_dir(map_dir)
        self.maps_root = self.map_dir.parent
        self._scene3d_cache: dict[str, Any] | None = None
        self._load_context(self.map_dir)

    def sidebar_payload(self, include_runtime: bool = True) -> dict[str, Any]:
        robots = []
        if include_runtime:
            state = self.state_payload(include_trajectories=False)
            robots = state.get("robots", [])
        return {
            "id": self.manager_id,
            "name": self.display_name,
            "type": "fleet_manager",
            "online": True,
            "host": "local",
            "port": 0,
            "baseUrl": "",
            "identity": {
                "robotId": "fleet-manager-sim" if self.mode == "simulation" else "fleet-manager",
                "mapId": self.map_dir.stem.replace(".smap", ""),
                "type": "fleet_manager",
                "mode": self.mode,
            },
            "status": {
                "state": self.mode.upper(),
                "robots": len(robots) if isinstance(robots, list) else 0,
            },
            "runtimeFresh": include_runtime,
            "error": "",
        }

    def mode_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "id": self.manager_id,
            "mode": self.mode,
            "mapName": self.map_dir.stem.replace(".smap", ""),
        }

    def set_mode_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        mode = str(payload.get("mode") or "").strip().lower()
        if mode not in {"simulation", "robots"}:
            raise ValueError("mode must be simulation or robots")
        if mode != self.mode:
            raise ValueError(f"{self.display_name} mode is fixed to {self.mode}")
        return self.mode_payload()

    def params_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "path": str(self.params_path),
            "params": load_route_params(self.params_path, create=True),
        }

    def save_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        params_payload = payload.get("params")
        if not isinstance(params_payload, dict):
            params_payload = payload
        save_route_params(params_payload, self.params_path)
        self._load_context(self.map_dir)
        return self.params_payload()

    def map_payload(self) -> dict[str, Any]:
        params = load_route_params(self.params_path, create=True)
        return build_editable_map_payload(self.map_dir, params=params)

    def scene3d_payload(self) -> dict[str, Any]:
        self._sync_manager_mode()
        static_scene = self._static_scene3d_payload()
        state = self.manager.state(include_trajectories=True)
        robots = state.get("robots", []) if isinstance(state, dict) else []
        return {
            **static_scene,
            "robots": robots if isinstance(robots, list) else [],
            "mode": self.mode,
            "managerId": self.manager_id,
            "managerName": self.display_name,
        }

    def maps_active_payload(self) -> dict[str, Any]:
        payload = self.map_payload()
        return {
            "ok": True,
            "mapName": str(payload.get("mapName") or self.map_dir.stem.replace(".smap", "")),
            "mapDir": str(self.map_dir),
            "signature": str(payload.get("signature") or ""),
        }

    def maps_list_payload(self) -> dict[str, Any]:
        return list_editable_maps(self.maps_root, active_map_dir=self.map_dir)

    def pull_map_payload(self, map_name: str = "") -> dict[str, Any]:
        target = self._resolve_map_dir_by_name(map_name)
        params = load_route_params(self.params_path, create=True)
        return build_editable_map_bundle_payload(target, params=params)

    def push_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        target_name = str(payload.get("mapName") or "").strip()
        source_name = str(payload.get("sourceMapName") or target_name).strip()
        source_dir = self._resolve_map_dir_by_name(source_name)
        output_name = str(payload.get("outputName") or "").strip()
        if not output_name and target_name and source_name and target_name != source_name:
            output_name = target_name
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        loaded_map = save_editable_map(
            source_dir,
            editable_map,
            output_name=output_name,
            overwrite_output=bool(payload.get("overwriteOutput", False)),
        )
        params = load_route_params(self.params_path, create=True)
        return {
            **build_editable_map_payload(loaded_map.map_dir, params=params),
            "savedAs": bool(output_name),
        }

    def load_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = str(payload.get("mapName") or payload.get("folder") or "").strip()
        if not map_name:
            raise ValueError("mapName is required")
        safe_name = Path(map_name).name
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        target = (self.maps_root / safe_name).resolve()
        if self.maps_root.resolve() not in target.parents and target != self.maps_root.resolve():
            raise ValueError("map must stay inside maps_out")
        if not target.is_dir():
            raise ValueError(f"map not found: {safe_name}")
        self._load_context(target)
        return {
            "ok": True,
            "mapName": self.map_dir.stem.replace(".smap", ""),
            "mapDir": str(self.map_dir),
            "mode": self.mode,
        }

    def save_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            editable_map = payload
        output_name = str(payload.get("outputName") or "").strip()
        loaded_map = save_editable_map(
            self.map_dir,
            editable_map,
            output_name=output_name,
            overwrite_output=bool(payload.get("overwriteOutput", False)),
        )
        self._load_context(loaded_map.map_dir)
        return {
            "ok": True,
            "mapName": self.map_dir.stem.replace(".smap", ""),
            "mapDir": str(self.map_dir),
            "map": self.map_payload(),
            "maps": self.maps_list_payload(),
        }

    def state_payload(self, include_trajectories: bool = True) -> dict[str, Any]:
        self._sync_manager_mode()
        state = self.manager.state(include_trajectories=include_trajectories)
        return self._state_with_context(state)

    def plan_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        result = self.manager.plan(payload)
        if isinstance(result.get("fleetState"), dict):
            result["state"] = result["fleetState"]
        return self._result_with_context(result)

    def benchmark_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        if self.mode != "simulation":
            raise ValueError("benchmark is only available in Fleet Manager Sim")
        raw_count = payload.get("count", 20)
        count = max(0, min(300, int(20 if raw_count is None else raw_count)))
        seed = int(payload.get("seed", 42) or 42)
        speed = float(payload.get("speed") or payload.get("routeSpeed") or 0.0)
        acceleration = float(payload.get("acceleration") or payload.get("routeAcceleration") or 0.0)
        rotate = bool(payload.get("rotate") or payload.get("simulateRotation") or payload.get("simulate_rotation"))
        turn_speed = float(payload.get("turnSpeed") or payload.get("turn_speed") or payload.get("rotationSpeed") or 0.0)
        if speed <= 0.0:
            params = load_route_params(self.params_path, create=True)
            navigation = params.get("navigation", {}) if isinstance(params, dict) else {}
            speed = float(navigation.get("route_speed", 1.0) or 1.0)
            acceleration = float(navigation.get("route_acceleration", acceleration) or acceleration or 0.0)
            rotate = bool(navigation.get("simulate_rotation", rotate))
            turn_speed = float(navigation.get("turn_speed", turn_speed) or turn_speed or 0.0)
        action = str(payload.get("action") or "").strip().lower()
        add_only = action in {"add", "add_robots", "ensure"} or bool(payload.get("addOnly", False))
        plan_existing = action in {"plan", "plan_existing"} or bool(payload.get("planExisting", False))
        reset_default = not (add_only or plan_existing)
        if payload.get("reset", reset_default):
            self._clear_simulation_runtime()
        if count <= 0:
            return self._state_with_context({
                **self.manager.state(include_trajectories=True),
                "benchmark": {
                    "count": 0,
                    "planned": 0,
                    "elapsedMs": 0.0,
                    "seed": seed,
                    "cleared": True,
                },
            })

        if add_only:
            return self._ensure_benchmark_robots_payload(count=count, seed=seed)

        requests = (
            self._benchmark_requests_for_existing(
                count=count,
                seed=seed,
                stress=bool(payload.get("stress", False)),
            )
            if plan_existing
            else self._benchmark_requests(
                count=count,
                seed=seed,
                stress=bool(payload.get("stress", False)),
            )
        )
        if plan_existing:
            self.manager.orders.clear()
        fast = bool(payload.get("fast", False))
        restore_fleet_params = self._apply_fast_benchmark_params(count)
        started = perf_counter()
        try:
            result = self.manager.plan({
                "robots": requests,
                "speed": speed,
                "acceleration": acceleration,
                "rotate": rotate,
                "turnSpeed": turn_speed,
                "stretchMotionToReservationTicks": True,
            })
        finally:
            if restore_fleet_params is not None:
                self._restore_fleet_params(restore_fleet_params)
        elapsed_ms = (perf_counter() - started) * 1000.0
        plans = result.get("plans", [])
        if not isinstance(plans, list):
            plans = []
        debug = result.get("debug", {})
        if not isinstance(debug, dict):
            debug = {}
        result["benchmark"] = {
            "count": count,
            "planned": len(plans),
            "elapsedMs": round(elapsed_ms, 3),
            "seed": seed,
            "speed": speed,
            "acceleration": acceleration,
            "rotate": rotate,
            "turnSpeed": turn_speed,
            "mapName": self.map_dir.stem.replace(".smap", ""),
            "plannerBackend": debug.get("plannerBackend", ""),
            "reason": debug.get("reason", ""),
            "expandedNodes": debug.get("expandedNodes", 0),
            "highLevelNodes": debug.get("highLevelNodes", 0),
            "fast": fast,
            "action": "plan" if plan_existing else "benchmark",
        }
        if isinstance(result.get("fleetState"), dict):
            result["state"] = result["fleetState"]
        return self._result_with_context(result)

    def orders_payload(self) -> dict[str, Any]:
        self._sync_manager_mode()
        result = self.manager.orders_payload()
        return self._result_with_context(result)

    def set_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.set_order(payload))

    def dispatch_orders_payload(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.dispatch_orders(payload or {}))

    def cancel_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.cancel_order(payload))

    def pause_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.pause_order(payload))

    def resume_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.resume_order(payload))

    def clear_orders_payload(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.clear_orders(payload or {}))

    def tick_payload(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._sync_manager_mode()
        state = self.manager.tick(payload or {})
        return self._state_with_context(state)

    def world_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.update_world(payload))

    def check_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self.manager.check_path(payload)

    def manual_step_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        name = str(payload.get("name") or "").strip()
        if not name:
            raise ValueError("robot name is required")
        if self.mode == "robots":
            result = self.manager.teleop_robot(
                {
                    "name": name,
                    "linear": float(payload.get("linear", 0.0) or 0.0),
                    "angular": float(payload.get("angular", 0.0) or 0.0),
                    "timeoutMs": int(payload.get("timeoutMs", 350) or 350),
                }
            )
            return {
                "ok": True,
                "blocked": False,
                "reason": "",
                "robot": result.get("robot"),
                "state": self._state_with_context(result.get("state")),
            }
        poses = payload.get("poses", [])
        check = self.manager.check_path({"name": name, "poses": poses})
        update_payload = {
            "name": name,
            "status": "MANUAL_BLOCKED" if check.get("blocked") else "MANUAL",
            "targetLm": "",
            "currentLm": str(
                payload.get("blockedCurrentLm" if check.get("blocked") else "currentLm")
                or payload.get("currentLm")
                or ""
            ),
        }
        pose_key = "blockedPose" if check.get("blocked") else "nextPose"
        pose = payload.get(pose_key)
        if isinstance(pose, dict):
            update_payload["pose"] = pose
        result = self.manager.update_robot(update_payload)
        return {
            "ok": True,
            "blocked": bool(check.get("blocked")),
            "reason": str(check.get("reason") or ""),
            "index": check.get("index"),
            "pose": check.get("pose"),
            "robot": result.get("robot"),
            "state": self._state_with_context(result.get("state")),
        }

    def manual_stop_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        name = str(payload.get("name") or "").strip()
        if not name:
            raise ValueError("robot name is required")
        if self.mode == "robots":
            result = self.manager.teleop_stop_robot({"name": name})
            return {
                "ok": True,
                "robot": result.get("robot"),
                "state": self._state_with_context(result.get("state")),
            }

        update_payload = {
            "name": name,
            "status": "IDLE",
            "targetLm": "",
            "currentLm": str(payload.get("currentLm") or ""),
        }
        pose = payload.get("pose")
        if isinstance(pose, dict):
            update_payload["pose"] = pose
        result = self.manager.update_robot(update_payload)
        return {
            "ok": True,
            "robot": result.get("robot"),
            "state": self._state_with_context(result.get("state")),
        }

    def add_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        payload = dict(payload)
        payload["mode"] = "remote" if self.mode == "robots" else "simulated"
        return self._result_with_context(self.manager.add_robot(payload))

    def remove_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.remove_robot(payload))

    def update_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.update_robot(payload))

    def stop_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.stop_robot(payload))

    def reset_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.reset_robot(payload))

    def _clear_simulation_runtime(self) -> None:
        if self.mode != "simulation":
            return
        self.manager.robots.clear()
        self.manager.orders.clear()
        self.manager.events.clear()

    def _benchmark_sim_robots(self) -> list[Any]:
        return sorted(
            [
                robot
                for robot in self.manager.robots.values()
                if not robot.is_remote()
            ],
            key=lambda robot: str(robot.name),
        )

    def _ensure_benchmark_robots_payload(self, *, count: int, seed: int) -> dict[str, Any]:
        existing = self._benchmark_sim_robots()
        target_count = max(0, count)
        if len(existing) >= target_count:
            state = self.manager.state(include_trajectories=True)
            return self._state_with_context({
                **state,
                "benchmark": {
                    "action": "add",
                    "target": target_count,
                    "robots": len(existing),
                    "added": 0,
                    "seed": seed,
                },
            })

        spawn_lms = self._benchmark_spawn_lms(target_count, seed)
        used_lms = {
            str(robot.current_lm)
            for robot in existing
            if str(robot.current_lm)
        }
        existing_names = set(self.manager.robots)
        next_index = self._next_benchmark_robot_index()
        added = 0
        for lm_name in spawn_lms:
            if len(existing) + added >= target_count:
                break
            if lm_name in used_lms:
                continue
            while f"bench_{next_index:03d}" in existing_names:
                next_index += 1
            name = f"bench_{next_index:03d}"
            self.manager.add_robot({
                "name": name,
                "spawnLm": lm_name,
                "mode": "simulated",
            })
            existing_names.add(name)
            used_lms.add(lm_name)
            next_index += 1
            added += 1

        robots = self._benchmark_sim_robots()
        state = self.manager.state(include_trajectories=True)
        return self._state_with_context({
            **state,
            "benchmark": {
                "action": "add",
                "target": target_count,
                "robots": len(robots),
                "added": added,
                "seed": seed,
            },
        })

    def _benchmark_spawn_lms(self, count: int, seed: int) -> list[str]:
        names = self._largest_benchmark_component()
        if len(names) < count:
            raise ValueError(
                f"add robots needs at least {count} connected LMs; "
                f"largest component has {len(names)}"
            )
        rng = random.Random(seed + 7919)
        shuffled = list(names)
        rng.shuffle(shuffled)
        return shuffled

    def _next_benchmark_robot_index(self) -> int:
        max_index = 0
        for name in self.manager.robots:
            value = str(name)
            if not value.startswith("bench_"):
                continue
            try:
                max_index = max(max_index, int(value.split("_", 1)[1]))
            except (IndexError, ValueError):
                continue
        return max_index + 1

    def _apply_fast_benchmark_params(self, count: int = 0) -> dict[str, Any] | None:
        params = getattr(self.manager, "params", None)
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
            "_planner_backend": getattr(getattr(self.manager, "planner", None), "planner_backend", None),
            "_planner_stretch_motion_to_reservation_ticks": getattr(
                getattr(self.manager, "planner", None),
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
            horizon = 12.0
            iterations = 12
        fleet["batch_collision_horizon_sec"] = horizon
        fleet["batch_wait_max_iterations"] = iterations
        fleet["continuous_collision_step_sec"] = 0.10
        fleet["stretch_motion_to_reservation_ticks"] = True
        planner = getattr(self.manager, "planner", None)
        if planner is not None:
            planner.planner_backend = "rolling_sipp"
            planner.stretch_motion_to_reservation_ticks = True
        return previous

    def _restore_fleet_params(self, previous: dict[str, Any]) -> None:
        params = getattr(self.manager, "params", None)
        if not isinstance(params, dict):
            return
        fleet = params.setdefault("fleet", {})
        if not isinstance(fleet, dict):
            return
        for key, value in previous.items():
            if key == "_planner_backend":
                planner = getattr(self.manager, "planner", None)
                if planner is not None and value is not None:
                    planner.planner_backend = value
                continue
            if key == "_planner_stretch_motion_to_reservation_ticks":
                planner = getattr(self.manager, "planner", None)
                if planner is not None and value is not None:
                    planner.stretch_motion_to_reservation_ticks = value
                continue
            if value is None:
                fleet.pop(key, None)
            else:
                fleet[key] = value

    def _benchmark_requests(self, *, count: int, seed: int, stress: bool = False) -> list[dict[str, Any]]:
        names = self._largest_benchmark_component()
        if len(names) < count * 2:
            raise ValueError(
                f"benchmark needs at least {count * 2} connected LMs; "
                f"largest component has {len(names)}"
            )
        rng = random.Random(seed + count)
        shuffled = list(names)
        rng.shuffle(shuffled)
        starts = shuffled[:count]
        used_goals: set[str] = set()
        requests: list[dict[str, Any]] = []
        landmarks = self.loaded_map.landmarks
        for index, start_name in enumerate(starts, start=1):
            start = landmarks[start_name]
            candidates = [
                name
                for name in names
                if name != start_name and name not in used_goals
            ]
            candidates.sort(
                key=lambda name: ((start.x - landmarks[name].x) ** 2) + ((start.y - landmarks[name].y) ** 2)
            )
            if stress:
                goal_name = candidates[-1]
            else:
                low = max(0, len(candidates) // 8)
                high = max(low + 1, len(candidates) // 3)
                goal_name = candidates[rng.randrange(low, high)]
            used_goals.add(goal_name)
            requests.append({
                "name": f"bench_{index:03d}",
                "startLm": start_name,
                "goalLm": goal_name,
            })
        return requests

    def _benchmark_requests_for_existing(self, *, count: int, seed: int, stress: bool = False) -> list[dict[str, Any]]:
        robots = self._benchmark_sim_robots()
        if count > 0:
            robots = robots[:count]
        if not robots:
            raise ValueError("add robots before planning")

        names = self._largest_benchmark_component()
        if len(names) < len(robots):
            raise ValueError(
                f"planning needs at least {len(robots)} connected LMs; "
                f"largest component has {len(names)}"
            )
        name_set = set(names)
        landmarks = self.loaded_map.landmarks
        rng = random.Random(seed + len(robots) + 3571)
        start_lms = {
            str(robot.current_lm)
            for robot in robots
            if str(robot.current_lm) in name_set
        }
        avoid_start_goals = len(names) >= len(robots) * 2
        used_goals: set[str] = set()
        requests: list[dict[str, Any]] = []
        for robot in robots:
            start_lm = str(robot.current_lm or "").strip()
            if start_lm not in name_set:
                start_lm = names[0]
            start = landmarks[start_lm]
            candidates = [
                name
                for name in names
                if name != start_lm
                and name not in used_goals
                and (not avoid_start_goals or name not in start_lms)
            ]
            if not candidates:
                candidates = [
                    name
                    for name in names
                    if name != start_lm and name not in used_goals
                ]
            if not candidates:
                raise ValueError(f"no target LM available for {robot.name}")
            candidates.sort(
                key=lambda name: ((start.x - landmarks[name].x) ** 2) + ((start.y - landmarks[name].y) ** 2)
            )
            if stress:
                goal_lm = candidates[-1]
            else:
                low = max(0, len(candidates) // 20)
                high = max(low + 1, len(candidates) // 8)
                goal_lm = candidates[rng.randrange(low, min(high, len(candidates)))]
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

    def _largest_benchmark_component(self) -> list[str]:
        adjacency: dict[str, set[str]] = {name: set() for name in self.loaded_map.landmarks}
        for edge in self.loaded_map.edges:
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

    def resolve_map_dir(self, map_dir: Path) -> Path:
        candidate = Path(map_dir).expanduser()
        safe_name = Path(candidate).name
        maps_root = FLEET_MAPS_OUT_ROOT.resolve()
        if candidate.is_absolute():
            resolved = candidate.resolve()
            if resolved.exists() and (resolved == maps_root or maps_root in resolved.parents):
                return resolved
            candidate = Path(safe_name)

        candidates = [
            FLEET_MAPS_OUT_ROOT / candidate,
            FLEET_MAP_DATA_ROOT / candidate,
            FLEET_ROOT / candidate,
            FLEET_MAPS_OUT_ROOT / safe_name,
        ]
        if not safe_name.endswith(".smap"):
            candidates.append(FLEET_MAPS_OUT_ROOT / f"{safe_name}.smap")
        for item in candidates:
            resolved = item.resolve()
            if resolved.exists() and (resolved == maps_root or maps_root in resolved.parents):
                return resolved
        return DEFAULT_FLEET_MAP_DIR.resolve()

    def _resolve_map_dir_by_name(self, map_name: str) -> Path:
        if not map_name:
            return self.map_dir
        safe_name = Path(str(map_name)).name
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        target = (self.maps_root / safe_name).resolve()
        root = self.maps_root.resolve()
        if root not in target.parents and target != root:
            raise ValueError("map must stay inside fleet maps_out")
        if not target.is_dir():
            raise ValueError(f"map not found: {safe_name}")
        return target

    def _load_context(self, map_dir: Path) -> None:
        loaded_map = WarehouseMapLoader(map_dir).load()
        params = load_route_params(self.params_path, create=True)
        self.loaded_map = loaded_map
        self.map_dir = loaded_map.map_dir.resolve()
        self.maps_root = self.map_dir.parent
        self._scene3d_cache = None
        self.manager = FleetManager(
            loaded_map.landmarks,
            loaded_map.edges,
            params=params,
            map_dir=loaded_map.map_dir,
            map_metadata=loaded_map.map_metadata,
            remote_adapter=self.remote_adapter,
        )
        self._sync_manager_mode()

    def _sync_manager_mode(self) -> None:
        if not hasattr(self, "manager"):
            return
        self.manager.set_active_robot_modes(self._active_robot_modes())

    def _active_robot_modes(self) -> set[str]:
        if self.mode == "robots":
            return {"remote"}
        return {"simulated"}

    def _static_scene3d_payload(self) -> dict[str, Any]:
        if isinstance(self._scene3d_cache, dict):
            return self._scene3d_cache

        metadata = self.loaded_map.map_metadata
        wall_height = 1.8
        walls = self._wall_rectangles_from_pgm(wall_height=wall_height)
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
            "walls": walls,
            "wallHeight": wall_height,
            "lms": [self.loaded_map.landmarks[name].to_dict() for name in sorted(self.loaded_map.landmarks)],
            "edges": [edge.to_dict() for edge in self.loaded_map.edges],
        }
        self._scene3d_cache = payload
        return payload

    def _wall_rectangles_from_pgm(self, *, wall_height: float) -> list[dict[str, Any]]:
        ros_map_yaml = self._find_ros_map_yaml()
        ros_map = yaml.safe_load(ros_map_yaml.read_text(encoding="utf-8"))
        if not isinstance(ros_map, dict):
            raise ValueError(f"Unexpected ROS map file format: {ros_map_yaml}")
        image_path = (self.map_dir / str(ros_map["image"])).resolve()
        width, height, pixels = WarehouseMapLoader(self.map_dir)._load_pgm(image_path)
        occupied_thresh = float(ros_map.get("occupied_thresh", 0.65) or 0.65)
        negate = int(ros_map.get("negate", 0) or 0)
        max_rectangles = 6000
        for stride in (1, 2, 4, 8):
            rectangles = self._build_wall_rectangles(
                width,
                height,
                pixels,
                occupied_thresh=occupied_thresh,
                negate=negate,
                stride=stride,
                wall_height=wall_height,
            )
            if len(rectangles) <= max_rectangles or stride == 8:
                return rectangles
        return []

    def _build_wall_rectangles(
        self,
        width: int,
        height: int,
        pixels: bytes,
        *,
        occupied_thresh: float,
        negate: int,
        stride: int,
        wall_height: float,
    ) -> list[dict[str, Any]]:
        resolution = float(self.loaded_map.map_metadata.resolution)
        grid_width = math.ceil(width / stride)
        grid_height = math.ceil(height / stride)
        active: dict[tuple[int, int], dict[str, int]] = {}
        rectangles_px: list[dict[str, int]] = []

        def block_occupied(cell_x: int, cell_y: int) -> bool:
            start_x = cell_x * stride
            start_y = cell_y * stride
            end_x = min(width, start_x + stride)
            end_y = min(height, start_y + stride)
            for py in range(start_y, end_y):
                row_offset = py * width
                for px in range(start_x, end_x):
                    value = pixels[row_offset + px]
                    occupancy = (value / 255.0) if negate else ((255 - value) / 255.0)
                    if occupancy > occupied_thresh:
                        return True
            return False

        for cell_y in range(grid_height):
            runs: list[tuple[int, int]] = []
            run_start: int | None = None
            for cell_x in range(grid_width):
                if block_occupied(cell_x, cell_y):
                    if run_start is None:
                        run_start = cell_x
                elif run_start is not None:
                    runs.append((run_start, cell_x - run_start))
                    run_start = None
            if run_start is not None:
                runs.append((run_start, grid_width - run_start))

            next_active: dict[tuple[int, int], dict[str, int]] = {}
            for x, w in runs:
                key = (x, w)
                rect = active.pop(key, None)
                if rect is None:
                    rect = {"x": x, "y": cell_y, "w": w, "h": 1}
                else:
                    rect["h"] += 1
                next_active[key] = rect
            rectangles_px.extend(active.values())
            active = next_active

        rectangles_px.extend(active.values())
        rectangles: list[dict[str, Any]] = []
        for rect in rectangles_px:
            px = rect["x"] * stride
            py = rect["y"] * stride
            pw = min(width - px, rect["w"] * stride)
            ph = min(height - py, rect["h"] * stride)
            rectangles.append(
                {
                    "x": round((px + (pw / 2.0)) * resolution, 4),
                    "z": round((py + (ph / 2.0)) * resolution, 4),
                    "width": round(pw * resolution, 4),
                    "depth": round(ph * resolution, 4),
                    "height": wall_height,
                    "stride": stride,
                }
            )
        return rectangles

    def _find_ros_map_yaml(self) -> Path:
        candidates = sorted(
            path
            for path in self.map_dir.glob("*.yaml")
            if path.name not in {"LMs.yaml", "graphs.yaml", "graph_edges_lengths.yaml"}
        )
        if not candidates:
            raise FileNotFoundError(f"No ROS map yaml found in {self.map_dir}")
        return candidates[0]

    def _state_with_context(self, state: Any) -> dict[str, Any]:
        if not isinstance(state, dict):
            state = self.manager.state()
        state["mode"] = self.mode
        state["mapName"] = self.map_dir.stem.replace(".smap", "")
        state["managerId"] = self.manager_id
        state["managerName"] = self.display_name
        return state

    def _result_with_context(self, result: dict[str, Any]) -> dict[str, Any]:
        state = result.get("state")
        if isinstance(state, dict):
            result["state"] = self._state_with_context(state)
        result["mode"] = self.mode
        result["mapName"] = self.map_dir.stem.replace(".smap", "")
        result["managerId"] = self.manager_id
        result["managerName"] = self.display_name
        return result
