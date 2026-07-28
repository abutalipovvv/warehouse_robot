#!/usr/bin/env python3
"""Run a bounded dynamic-order benchmark against a local RDS Core.

The benchmark mirrors Fleet Manager Sim's continuous mode: every selected
robot owns at most one order and receives a new, distant graph goal as soon as
the previous order becomes terminal.  RDS remains the only traffic controller;
this client merely supplies work and records observable behaviour.
"""

from __future__ import annotations

import argparse
import base64
import http.client
import json
import math
import random
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import defaultdict, deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TERMINAL_STATES = {"FINISHED", "FAILED", "STOPPED", "ERROR"}


def _property_value(item: dict[str, Any]) -> Any:
    for key in ("boolValue", "int32Value", "doubleValue", "stringValue"):
        if key in item:
            return item[key]
    raw = item.get("value")
    if not isinstance(raw, str):
        return raw
    try:
        return base64.b64decode(raw).decode("utf-8")
    except (ValueError, UnicodeDecodeError):
        return raw


def _properties(items: Any) -> dict[str, Any]:
    if not isinstance(items, list):
        return {}
    return {
        str(item["key"]): _property_value(item)
        for item in items
        if isinstance(item, dict) and item.get("key") is not None
    }


@dataclass
class ActiveOrder:
    order_id: str
    block_id: str
    robot: str
    origin: str
    target: str
    created_wall: float
    created_mono: float
    hops: int
    distance_m: float
    seen_running: bool = False
    missing_polls: int = 0


class RdsClient:
    def __init__(self, base_url: str, *, timeout_sec: float = 5.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec

    def request(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        body = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(
            f"{self.base_url}/{path.lstrip('/')}",
            data=body,
            headers=headers,
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                result = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"{method} {path}: HTTP {exc.code}: {detail}") from exc
        except (
            urllib.error.URLError,
            TimeoutError,
            OSError,
            http.client.HTTPException,
            json.JSONDecodeError,
        ) as exc:
            raise RuntimeError(f"{method} {path}: {exc}") from exc
        if not isinstance(result, dict):
            raise RuntimeError(f"{method} {path}: expected JSON object")
        return result

    def robots_status(self) -> dict[str, Any]:
        return self.request("GET", "/robotsStatus")

    def set_order(self, payload: dict[str, Any]) -> dict[str, Any]:
        result = self.request("POST", "/setOrder", payload)
        if int(result.get("code", 0) or 0) != 0:
            raise RuntimeError(f"RDS rejected order: {result}")
        return result

    def order_details(self, order_id: str) -> dict[str, Any]:
        encoded = urllib.parse.quote(order_id, safe="")
        return self.request("GET", f"/orderDetails/{encoded}")

    def update_sim_robot_state(
        self,
        vehicle_id: str,
        **state: Any,
    ) -> dict[str, Any]:
        return self.request(
            "POST",
            "/updateSimRobotState",
            {"vehicle_id": vehicle_id, **state},
        )


class MapGraph:
    def __init__(self, path: Path, *, landmark_limit: int = 0) -> None:
        payload = json.loads(path.read_text(encoding="utf-8"))
        self.points: dict[str, tuple[float, float]] = {}
        self.point_properties: dict[str, dict[str, Any]] = {}
        for item in payload.get("advancedPointList", []):
            if not isinstance(item, dict) or item.get("className") != "LocationMark":
                continue
            if landmark_limit > 0 and len(self.points) >= landmark_limit:
                break
            name = str(item.get("instanceName") or "")
            pos = item.get("pos") or {}
            if name and isinstance(pos, dict):
                self.points[name] = (float(pos["x"]), float(pos["y"]))
                self.point_properties[name] = _properties(item.get("property"))

        self.adjacency: dict[str, set[str]] = defaultdict(set)
        for item in (
            list(payload.get("advancedLineList") or [])
            + list(payload.get("advancedCurveList") or [])
        ):
            if not isinstance(item, dict):
                continue
            start = item.get("startPos") or (item.get("line") or {}).get("startPos") or {}
            end = item.get("endPos") or (item.get("line") or {}).get("endPos") or {}
            source = str(start.get("instanceName") or "")
            target = str(end.get("instanceName") or "")
            if source in self.points and target in self.points and source != target:
                self.adjacency[source].add(target)

        self.corridors: list[dict[str, Any]] = []
        for item in payload.get("advancedAreaList", []):
            if not isinstance(item, dict):
                continue
            props = _properties(item.get("property"))
            if str(props.get("kind") or "").lower() != "controlled_corridor":
                continue
            polygon = [
                (float(point["x"]), float(point["y"]))
                for point in item.get("posGroup", [])
                if isinstance(point, dict) and "x" in point and "y" in point
            ]
            if len(polygon) < 3:
                continue
            xs = [point[0] for point in polygon]
            ys = [point[1] for point in polygon]
            self.corridors.append(
                {
                    "id": str(item.get("instanceName") or f"corridor-{len(self.corridors) + 1}"),
                    "min_x": min(xs),
                    "max_x": max(xs),
                    "min_y": min(ys),
                    "max_y": max(ys),
                    "axis": "x" if max(xs) - min(xs) >= max(ys) - min(ys) else "y",
                }
            )

    def safe_spawn_points(self, count: int, *, margin_m: float) -> list[str]:
        def outside_corridors(name: str) -> bool:
            x, y = self.points[name]
            return not any(
                corridor["min_x"] - margin_m <= x <= corridor["max_x"] + margin_m
                and corridor["min_y"] - margin_m <= y <= corridor["max_y"] + margin_m
                for corridor in self.corridors
            )

        candidates = [
            name
            for name in self.points
            if self.point_properties.get(name, {}).get("waitAllowed", True) is not False
            and outside_corridors(name)
        ]
        if len(candidates) < count:
            raise RuntimeError(
                f"only {len(candidates)} safe spawn LMs are available for {count} robots"
            )
        # Deterministic farthest-point sampling gives the initial fleet enough
        # room to rotate and makes repeated benchmark runs reproducible.
        first = min(
            candidates,
            key=lambda name: (
                self.points[name][0] + self.points[name][1],
                name,
            ),
        )
        selected = [first]
        while len(selected) < count:
            selected.append(
                max(
                    (name for name in candidates if name not in selected),
                    key=lambda name: (
                        min(
                            math.dist(self.points[name], self.points[chosen])
                            for chosen in selected
                        ),
                        name,
                    ),
                )
            )
        return selected

    def nearest(self, x: float, y: float) -> str:
        return min(
            self.points,
            key=lambda name: math.hypot(
                self.points[name][0] - x,
                self.points[name][1] - y,
            ),
        )

    def distances(self, origin: str, *, maximum: int) -> dict[str, int]:
        distances = {origin: 0}
        queue = deque([origin])
        while queue:
            source = queue.popleft()
            next_distance = distances[source] + 1
            if next_distance > maximum:
                continue
            for target in self.adjacency.get(source, ()):
                if target in distances:
                    continue
                distances[target] = next_distance
                queue.append(target)
        return distances


def _robot_name(index: int) -> str:
    return f"sim_{index:03d}" if index < 10 else f"sim_{index:04d}"


def _reports_by_name(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    reports: dict[str, dict[str, Any]] = {}
    for item in payload.get("report", []):
        if not isinstance(item, dict):
            continue
        name = str(item.get("vehicle_id") or item.get("uuid") or "")
        if name:
            reports[name] = item
    return reports


def _current_station(report: dict[str, Any], graph: MapGraph) -> str:
    rbk = report.get("rbk_report") or {}
    station = str(rbk.get("current_station") or "")
    if station in graph.points:
        return station
    return graph.nearest(float(rbk.get("x") or 0.0), float(rbk.get("y") or 0.0))


def _path_tokens(value: Any) -> tuple[str, ...]:
    tokens: list[str] = []

    def visit(item: Any) -> None:
        if isinstance(item, str):
            tokens.append(item)
        elif isinstance(item, dict):
            for key in ("id", "source_id", "end_id", "start_id", "name"):
                if item.get(key) not in {None, ""}:
                    tokens.append(str(item[key]))
            if not tokens:
                for child in item.values():
                    visit(child)
        elif isinstance(item, list):
            for child in item:
                visit(child)

    visit(value)
    return tuple(tokens)


def _is_consumed_suffix(previous: tuple[str, ...], current: tuple[str, ...]) -> bool:
    if not previous or not current:
        return True
    if current == previous:
        return True
    if len(current) <= len(previous):
        return any(previous[index:] == current for index in range(len(previous)))
    return any(current[index:] == previous for index in range(len(current)))


class Benchmark:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.client = RdsClient(args.base_url, timeout_sec=args.http_timeout_sec)
        self.graph = MapGraph(args.map, landmark_limit=args.landmark_limit)
        self.rng = random.Random(args.seed)
        self.robots = [_robot_name(index) for index in range(1, args.robot_count + 1)]
        self.active: dict[str, ActiveOrder] = {}
        self.sequence = 0
        self.started_wall = time.time()
        self.started_mono = time.monotonic()
        self.generation_deadline = self.started_mono + args.duration_sec
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        self.session_id = f"rds-dynamic-{stamp}-{args.seed}"
        self.spawn_assignments: dict[str, str] = {}
        self.samples_path = args.output_dir / f"{self.session_id}.jsonl"
        self.summary_path = args.output_dir / f"{self.session_id}-summary.json"
        self.samples_file = self.samples_path.open("w", encoding="utf-8")
        self.generated = 0
        self.completed = 0
        self.failed = 0
        self.submit_errors = 0
        self.http_errors = 0
        self.order_durations: list[float] = []
        self.blocked_seconds: dict[str, float] = defaultdict(float)
        self.blocked_events: dict[str, int] = defaultdict(int)
        self.stationary_seconds: dict[str, float] = defaultdict(float)
        self.stationary_streak: dict[str, float] = defaultdict(float)
        self.max_stationary_streak: dict[str, float] = defaultdict(float)
        self.route_changes: dict[str, int] = defaultdict(int)
        self.rollbacks: dict[str, int] = defaultdict(int)
        self.previous_blocked: dict[str, bool] = defaultdict(bool)
        self.previous_path: dict[str, tuple[str, ...]] = {}
        self.station_history: dict[str, deque[tuple[str, float]]] = defaultdict(
            lambda: deque(maxlen=8)
        )
        self.previous_positions: dict[str, tuple[float, float]] = {}
        self.corridor_max_occupancy: dict[str, int] = defaultdict(int)
        self.corridor_robot_seconds: dict[str, float] = defaultdict(float)
        self.min_pair_distance = math.inf
        self.last_sample_mono = self.started_mono

    def rds_lm_name(self, graph_lm_name: str) -> str:
        """Translate an exported graph LM into the name RDS exposes.

        RDS normalizes numeric benchmark names such as ``B0419`` to
        ``LM419`` while importing the map. Other maps keep their names, so the
        conversion is opt-in.
        """
        prefix = str(self.args.rds_numeric_lm_prefix or "").strip()
        if not prefix:
            return graph_lm_name
        match = re.search(r"(\d+)$", graph_lm_name)
        if match is None:
            raise RuntimeError(
                f"cannot convert non-numeric LM {graph_lm_name!r} "
                f"with prefix {prefix!r}"
            )
        return f"{prefix}{int(match.group(1))}"

    def close(self) -> None:
        self.samples_file.close()

    def validate(self, reports: dict[str, dict[str, Any]]) -> None:
        missing = [name for name in self.robots if name not in reports]
        if missing:
            raise RuntimeError(f"RDS is missing benchmark robots: {missing}")
        wrong_map = {
            name: str((reports[name].get("rbk_report") or {}).get("current_map") or "")
            for name in self.robots
            if str((reports[name].get("rbk_report") or {}).get("current_map") or "")
            != self.args.expected_map
        }
        if wrong_map:
            raise RuntimeError(f"robots are on unexpected maps: {wrong_map}")
        busy = [
            name
            for name in self.robots
            if reports[name].get("procBusiness")
            or (
                reports[name].get("current_order")
                and str(
                    (reports[name].get("current_order") or {}).get("state") or ""
                ).upper()
                not in TERMINAL_STATES
            )
        ]
        if busy:
            raise RuntimeError(f"robots already have active orders: {busy}")
        unavailable = [
            name
            for name in self.robots
            if int(reports[name].get("connection_status", 0) or 0) != 1
            or not bool(reports[name].get("dispatchable"))
        ]
        if unavailable:
            raise RuntimeError(f"robots are not dispatchable: {unavailable}")

    def relocate(self) -> dict[str, dict[str, Any]]:
        if self.args.spawn_state is not None:
            payload = json.loads(self.args.spawn_state.read_text(encoding="utf-8"))
            source_robots = payload.get("robots")
            if not isinstance(source_robots, list):
                raise RuntimeError(
                    f"{self.args.spawn_state} does not contain a robots list"
                )

            def robot_index(item: dict[str, Any]) -> tuple[int, str]:
                name = str(item.get("name") or "")
                try:
                    return int(name.rsplit("_", 1)[1]), name
                except (IndexError, ValueError):
                    return 10**9, name

            source_lms = [
                str(item.get("currentLm") or "")
                for item in sorted(
                    (item for item in source_robots if isinstance(item, dict)),
                    key=robot_index,
                )
            ]
            if len(source_lms) < len(self.robots):
                raise RuntimeError(
                    f"{self.args.spawn_state} has only {len(source_lms)} robot poses; "
                    f"{len(self.robots)} are required"
                )
            spawn_lms = source_lms[: len(self.robots)]
            unknown = [name for name in spawn_lms if name not in self.graph.points]
            if unknown:
                raise RuntimeError(
                    f"spawn state contains LMs absent from the RDS map: {unknown}"
                )
            if len(set(spawn_lms)) != len(spawn_lms):
                raise RuntimeError("spawn state assigns more than one robot to an LM")
        else:
            spawn_lms = self.graph.safe_spawn_points(
                len(self.robots),
                margin_m=self.args.spawn_corridor_margin_m,
            )
        self.spawn_assignments = dict(zip(self.robots, spawn_lms, strict=True))
        for name, lm_name in self.spawn_assignments.items():
            x, y = self.graph.points[lm_name]
            motion_state: dict[str, Any] = {}
            # A failed RDS order can leave the simulated vehicle excluded from
            # dispatch even after /terminate. Reset simulator-side faults as
            # part of every reproducible relocation.
            motion_state.update(
                {
                    "error": json.dumps([], separators=(",", ":")),
                    "fail_current_task": False,
                    "emergency": False,
                }
            )
            if self.args.robot_speed > 0.0:
                motion_state["speed"] = self.args.robot_speed
            if self.args.robot_rotate_speed > 0.0:
                motion_state.update(
                    {
                        "rotate_speed": self.args.robot_rotate_speed,
                        "disable_rotate": False,
                    }
                )
            result = self.client.update_sim_robot_state(
                name,
                # Current RDS expects JSON-valued simulator properties as a
                # JSON string, not as a nested request object.
                position=json.dumps({"x": x, "y": y}, separators=(",", ":")),
                angle=0.0,
                blocked=False,
                connection_status=1,
                **motion_state,
            )
            if int(result.get("code", 0) or 0) != 0:
                raise RuntimeError(f"RDS rejected relocation of {name}: {result}")

        deadline = time.monotonic() + self.args.relocate_timeout_sec
        last_reports: dict[str, dict[str, Any]] = {}
        while time.monotonic() < deadline:
            last_reports = _reports_by_name(self.client.robots_status())
            pending = [
                name
                for name, lm_name in self.spawn_assignments.items()
                if (
                    math.hypot(
                        float(
                            (
                                last_reports.get(name, {}).get("rbk_report")
                                or {}
                            ).get("x")
                            or 0.0
                        )
                        - self.graph.points[lm_name][0],
                        float(
                            (
                                last_reports.get(name, {}).get("rbk_report")
                                or {}
                            ).get("y")
                            or 0.0
                        )
                        - self.graph.points[lm_name][1],
                    )
                    > 0.05
                    or not bool(last_reports.get(name, {}).get("dispatchable"))
                )
            ]
            if not pending:
                print(
                    json.dumps(
                        {
                            "relocated": len(self.spawn_assignments),
                            "spawnAssignments": self.spawn_assignments,
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                return last_reports
            time.sleep(0.2)
        actual = {
            name: str(
                (last_reports.get(name, {}).get("rbk_report") or {}).get(
                    "current_station"
                )
                or ""
            )
            for name in self.robots
        }
        raise RuntimeError(
            f"RDS relocation did not settle before timeout: "
            f"expected={self.spawn_assignments}, actual={actual}"
        )

    def choose_goal(
        self,
        robot: str,
        report: dict[str, Any],
        reports: dict[str, dict[str, Any]],
    ) -> tuple[str, str, int, float]:
        origin = _current_station(report, self.graph)
        occupied = {
            _current_station(item, self.graph)
            for name, item in reports.items()
            if name in self.robots and name != robot
        }
        reserved = {order.target for name, order in self.active.items() if name != robot}
        excluded = occupied | reserved | {origin}
        distances = self.graph.distances(origin, maximum=self.args.max_hops)
        candidates = [
            name
            for name, hops in distances.items()
            if self.args.min_hops <= hops <= self.args.max_hops
            and name not in excluded
        ]
        if not candidates:
            candidates = [
                name
                for name, hops in distances.items()
                if hops >= max(2, self.args.min_hops // 3) and name not in excluded
            ]
        if not candidates:
            raise RuntimeError(f"no reachable free goal for {robot} from {origin}")
        ox, oy = self.graph.points[origin]
        candidates.sort(
            key=lambda name: math.hypot(
                self.graph.points[name][0] - ox,
                self.graph.points[name][1] - oy,
            )
        )
        pool_size = max(1, math.ceil(len(candidates) * self.args.far_fraction))
        target = self.rng.choice(candidates[-pool_size:])
        tx, ty = self.graph.points[target]
        return target, origin, distances[target], math.hypot(tx - ox, ty - oy)

    def submit(self, robot: str, reports: dict[str, dict[str, Any]]) -> None:
        target, origin, hops, distance_m = self.choose_goal(
            robot,
            reports[robot],
            reports,
        )
        self.sequence += 1
        order_id = f"{self.session_id}-{self.sequence:06d}-{robot}"
        block_id = f"{order_id}-navigate"
        payload = {
            "id": order_id,
            "externalId": self.session_id,
            "vehicle": robot,
            "keyRoute": [self.rds_lm_name(target)],
            "priority": self.rng.choice((0, 0, 1, 1, 2, 3)),
            "blocks": [
                {"blockId": block_id, "location": self.rds_lm_name(target)}
            ],
            "complete": True,
        }
        self.client.set_order(payload)
        now_wall = time.time()
        now_mono = time.monotonic()
        self.active[robot] = ActiveOrder(
            order_id=order_id,
            block_id=block_id,
            robot=robot,
            origin=origin,
            target=target,
            created_wall=now_wall,
            created_mono=now_mono,
            hops=hops,
            distance_m=distance_m,
        )
        self.generated += 1

    def order_state(self, order: ActiveOrder, report: dict[str, Any]) -> str:
        current = report.get("current_order") or {}
        current_id = str(current.get("id") or "")
        state = str(current.get("state") or "").upper()
        if current_id == order.order_id:
            order.seen_running = order.seen_running or state in {"RUNNING", "WAITING"}
            order.missing_polls = 0
            return state or "RUNNING"
        if report.get("procBusiness"):
            order.missing_polls = 0
            return "TOBEDISPATCHED"
        order.missing_polls += 1
        if order.missing_polls < 2 and not order.seen_running:
            return "CREATED"
        details = self.client.order_details(order.order_id)
        return str(details.get("state") or "").upper()

    def finish_order(self, robot: str, state: str) -> None:
        order = self.active.pop(robot)
        duration = max(0.0, time.monotonic() - order.created_mono)
        if state == "FINISHED":
            self.completed += 1
            self.order_durations.append(duration)
        else:
            self.failed += 1
        self.previous_path.pop(robot, None)

    def observe(self, reports: dict[str, dict[str, Any]], now_mono: float) -> None:
        dt = max(0.0, min(self.args.poll_sec * 3.0, now_mono - self.last_sample_mono))
        self.last_sample_mono = now_mono
        sample_robots: list[dict[str, Any]] = []
        positions: list[tuple[str, float, float]] = []
        corridor_occupants: dict[str, list[str]] = defaultdict(list)
        for name in self.robots:
            report = reports[name]
            rbk = report.get("rbk_report") or {}
            x = float(rbk.get("x") or 0.0)
            y = float(rbk.get("y") or 0.0)
            vx = float(rbk.get("vx") or 0.0)
            vy = float(rbk.get("vy") or 0.0)
            angular = float(rbk.get("w") or 0.0)
            blocked = bool(rbk.get("blocked"))
            active = self.active.get(name)
            moving = math.hypot(vx, vy) >= self.args.moving_epsilon or abs(angular) >= 0.01
            if blocked:
                self.blocked_seconds[name] += dt
            if blocked and not self.previous_blocked[name]:
                self.blocked_events[name] += 1
            self.previous_blocked[name] = blocked
            if active is not None and not moving:
                self.stationary_seconds[name] += dt
                self.stationary_streak[name] += dt
                self.max_stationary_streak[name] = max(
                    self.max_stationary_streak[name],
                    self.stationary_streak[name],
                )
            else:
                self.stationary_streak[name] = 0.0

            path = _path_tokens(report.get("unfinished_path"))
            previous_path = self.previous_path.get(name)
            if (
                active is not None
                and previous_path is not None
                and path
                and not _is_consumed_suffix(previous_path, path)
            ):
                self.route_changes[name] += 1
            if path:
                self.previous_path[name] = path

            station = str(rbk.get("current_station") or "")
            history = self.station_history[name]
            if station and (not history or history[-1][0] != station):
                if (
                    len(history) >= 2
                    and history[-2][0] == station
                    and now_mono - history[-2][1] <= self.args.rollback_window_sec
                ):
                    self.rollbacks[name] += 1
                history.append((station, now_mono))

            for corridor in self.graph.corridors:
                if (
                    corridor["min_x"] <= x <= corridor["max_x"]
                    and corridor["min_y"] <= y <= corridor["max_y"]
                ):
                    corridor_occupants[corridor["id"]].append(name)
                    self.corridor_robot_seconds[corridor["id"]] += dt

            resources = 0
            for area in report.get("area_resources_occupied") or []:
                resources += len(area.get("path_occupied") or [])
            current = report.get("current_order") or {}
            sample_robots.append(
                {
                    "name": name,
                    "x": round(x, 4),
                    "y": round(y, 4),
                    "station": station,
                    "vx": round(vx, 4),
                    "vy": round(vy, 4),
                    "w": round(angular, 4),
                    "blocked": blocked,
                    "taskStatus": rbk.get("task_status"),
                    "procBusiness": bool(report.get("procBusiness")),
                    "orderId": current.get("id"),
                    "orderState": current.get("state"),
                    "goal": active.target if active else "",
                    "pathTokens": path,
                    "reservedPaths": resources,
                }
            )
            positions.append((name, x, y))

        for index, (_name_a, ax, ay) in enumerate(positions):
            for _name_b, bx, by in positions[index + 1 :]:
                self.min_pair_distance = min(
                    self.min_pair_distance,
                    math.hypot(ax - bx, ay - by),
                )
        for corridor_id, names in corridor_occupants.items():
            self.corridor_max_occupancy[corridor_id] = max(
                self.corridor_max_occupancy[corridor_id],
                len(names),
            )
        self.samples_file.write(
            json.dumps(
                {
                    "elapsedSec": round(now_mono - self.started_mono, 3),
                    "generated": self.generated,
                    "completed": self.completed,
                    "failed": self.failed,
                    "corridors": corridor_occupants,
                    "robots": sample_robots,
                },
                ensure_ascii=False,
                separators=(",", ":"),
            )
            + "\n"
        )
        self.samples_file.flush()

    def progress(self, now_mono: float) -> None:
        elapsed = now_mono - self.started_mono
        throughput = self.completed * 60.0 / elapsed if elapsed > 0 else 0.0
        blocked = sum(1 for value in self.previous_blocked.values() if value)
        waiting = sum(
            1
            for name in self.robots
            if name in self.active and self.stationary_streak[name] >= 2.0
        )
        print(
            json.dumps(
                {
                    "elapsedSec": round(elapsed, 1),
                    "generated": self.generated,
                    "completed": self.completed,
                    "active": len(self.active),
                    "failed": self.failed,
                    "throughputPerMin": round(throughput, 2),
                    "blockedNow": blocked,
                    "stationaryOver2s": waiting,
                    "routeChanges": sum(self.route_changes.values()),
                    "rollbacks": sum(self.rollbacks.values()),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )

    def summary(self, *, drained: bool) -> dict[str, Any]:
        elapsed = max(0.000001, time.monotonic() - self.started_mono)
        durations = self.order_durations
        summary = {
            "sessionId": self.session_id,
            "baseUrl": self.args.base_url,
            "map": self.args.expected_map,
            "robots": self.robots,
            "seed": self.args.seed,
            "spawnAssignments": self.spawn_assignments,
            "generationDurationSec": self.args.duration_sec,
            "elapsedSec": round(elapsed, 3),
            "drained": drained,
            "activeAtEnd": {
                name: {
                    "id": order.order_id,
                    "origin": order.origin,
                    "target": order.target,
                    "ageSec": round(time.monotonic() - order.created_mono, 3),
                }
                for name, order in self.active.items()
            },
            "orders": {
                "generated": self.generated,
                "completed": self.completed,
                "failed": self.failed,
                "submitErrors": self.submit_errors,
                "throughputPerMin": round(self.completed * 60.0 / elapsed, 3),
                "averageDurationSec": (
                    round(sum(durations) / len(durations), 3) if durations else 0.0
                ),
                "maxDurationSec": round(max(durations), 3) if durations else 0.0,
            },
            "traffic": {
                "blockedRobotSec": round(sum(self.blocked_seconds.values()), 3),
                "blockedEvents": sum(self.blocked_events.values()),
                "stationaryRobotSec": round(sum(self.stationary_seconds.values()), 3),
                "maxStationarySec": round(max(self.max_stationary_streak.values(), default=0.0), 3),
                "routeChanges": sum(self.route_changes.values()),
                "rollbacks": sum(self.rollbacks.values()),
                "minPairDistanceM": (
                    round(self.min_pair_distance, 4)
                    if math.isfinite(self.min_pair_distance)
                    else None
                ),
                "perRobot": {
                    name: {
                        "blockedSec": round(self.blocked_seconds[name], 3),
                        "blockedEvents": self.blocked_events[name],
                        "stationarySec": round(self.stationary_seconds[name], 3),
                        "maxStationarySec": round(self.max_stationary_streak[name], 3),
                        "routeChanges": self.route_changes[name],
                        "rollbacks": self.rollbacks[name],
                    }
                    for name in self.robots
                },
            },
            "corridors": {
                corridor["id"]: {
                    "maxOccupancy": self.corridor_max_occupancy[corridor["id"]],
                    "robotSeconds": round(
                        self.corridor_robot_seconds[corridor["id"]],
                        3,
                    ),
                }
                for corridor in self.graph.corridors
            },
            "samples": str(self.samples_path),
        }
        self.summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return summary

    def run(self) -> dict[str, Any]:
        status = self.client.robots_status()
        reports = _reports_by_name(status)
        self.validate(reports)
        if self.args.relocate:
            reports = self.relocate()
            self.validate(reports)
        if self.args.relocate_only:
            return self.summary(drained=True)
        print(
            json.dumps(
                {
                    "sessionId": self.session_id,
                    "robots": len(self.robots),
                    "landmarks": len(self.graph.points),
                    "corridors": len(self.graph.corridors),
                    "samples": str(self.samples_path),
                }
            ),
            flush=True,
        )
        for name in self.robots:
            try:
                self.submit(name, reports)
            except RuntimeError as exc:
                self.submit_errors += 1
                print(f"initial submit warning for {name}: {exc}", flush=True)
            if self.args.submit_spacing_sec > 0.0:
                time.sleep(self.args.submit_spacing_sec)

        next_progress = time.monotonic()
        drain_deadline = self.generation_deadline + self.args.drain_sec
        while True:
            cycle_started = time.monotonic()
            try:
                status = self.client.robots_status()
                reports = _reports_by_name(status)
                missing = [name for name in self.robots if name not in reports]
                if missing:
                    raise RuntimeError(f"robots disappeared: {missing}")
                for name, order in list(self.active.items()):
                    state = self.order_state(order, reports[name])
                    if state in TERMINAL_STATES:
                        self.finish_order(name, state)
                now_mono = time.monotonic()
                if now_mono < self.generation_deadline:
                    for name in self.robots:
                        if name not in self.active:
                            try:
                                self.submit(name, reports)
                            except RuntimeError as exc:
                                self.submit_errors += 1
                                print(f"submit warning for {name}: {exc}", flush=True)
                            if self.args.submit_spacing_sec > 0.0:
                                time.sleep(self.args.submit_spacing_sec)
                self.observe(reports, now_mono)
            except RuntimeError as exc:
                self.http_errors += 1
                print(f"poll warning: {exc}", flush=True)
                if self.http_errors >= self.args.max_http_errors:
                    raise
                now_mono = time.monotonic()

            if now_mono >= next_progress:
                self.progress(now_mono)
                next_progress = now_mono + self.args.progress_sec
            if now_mono >= self.generation_deadline and not self.active:
                return self.summary(drained=True)
            if now_mono >= drain_deadline:
                return self.summary(drained=False)
            delay = self.args.poll_sec - (time.monotonic() - cycle_started)
            if delay > 0:
                time.sleep(delay)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8088")
    parser.add_argument(
        "--map",
        type=Path,
        default=Path(
            "operator_app/operator_data/fleet_manager_sim/maps/"
            "smart_kiva_large_w_mode_export.smap"
        ),
    )
    parser.add_argument("--expected-map", default="smart_kiva_large_w_mode_export")
    parser.add_argument("--robot-count", type=int, default=20)
    parser.add_argument(
        "--landmark-limit",
        type=int,
        default=0,
        help="use only the first N ordered map landmarks (0 uses the full map)",
    )
    parser.add_argument("--duration-sec", type=float, default=300.0)
    parser.add_argument("--drain-sec", type=float, default=180.0)
    parser.add_argument("--poll-sec", type=float, default=0.5)
    parser.add_argument("--progress-sec", type=float, default=10.0)
    parser.add_argument(
        "--submit-spacing-sec",
        type=float,
        default=0.0,
        help="minimum wall-time spacing between RDS setOrder calls",
    )
    parser.add_argument("--http-timeout-sec", type=float, default=5.0)
    parser.add_argument("--max-http-errors", type=int, default=10)
    parser.add_argument("--seed", type=int, default=20260727)
    parser.add_argument(
        "--relocate",
        action="store_true",
        help="place robots on deterministic safe LMs before generating orders",
    )
    parser.add_argument(
        "--spawn-state",
        type=Path,
        help=(
            "Fleet Manager Sim state JSON whose robots/currentLm list supplies "
            "the exact relocation LMs"
        ),
    )
    parser.add_argument(
        "--relocate-only",
        action="store_true",
        help="relocate and validate the robots without submitting orders",
    )
    parser.add_argument("--spawn-corridor-margin-m", type=float, default=0.65)
    parser.add_argument("--relocate-timeout-sec", type=float, default=10.0)
    parser.add_argument(
        "--robot-speed",
        type=float,
        default=0.0,
        help="optional RDS simulator linear speed in m/s applied while relocating",
    )
    parser.add_argument(
        "--robot-rotate-speed",
        type=float,
        default=0.0,
        help="optional RDS simulator angular speed in rad/s applied while relocating",
    )
    parser.add_argument(
        "--rds-numeric-lm-prefix",
        default="",
        help=(
            "translate a graph LM's trailing digits to an RDS name, e.g. "
            "B0419 with prefix LM becomes LM419"
        ),
    )
    parser.add_argument("--min-hops", type=int, default=30)
    parser.add_argument("--max-hops", type=int, default=160)
    parser.add_argument("--far-fraction", type=float, default=0.08)
    parser.add_argument("--moving-epsilon", type=float, default=0.01)
    parser.add_argument("--rollback-window-sec", type=float, default=30.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("var/rds_benchmarks"),
    )
    args = parser.parse_args()
    if args.robot_count < 1 or args.duration_sec <= 0 or args.poll_sec <= 0:
        parser.error("robot-count, duration-sec and poll-sec must be positive")
    if args.spawn_state is not None:
        args.relocate = True
    if args.relocate_only and not args.relocate:
        parser.error("--relocate-only requires --relocate or --spawn-state")
    args.far_fraction = max(0.01, min(1.0, args.far_fraction))
    return args


def main() -> int:
    args = parse_args()
    benchmark = Benchmark(args)
    try:
        summary = benchmark.run()
    finally:
        benchmark.close()
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
