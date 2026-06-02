#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT_STR = str(PROJECT_ROOT)
if PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, PROJECT_ROOT_STR)

from route_core import LmRoutePlanner, WarehouseMapLoader


DEFAULT_BASE_URL = "http://127.0.0.1:8090"
DEFAULT_MAP_DIR = Path("map_data/maps_out/22.05.26_smap.smap")
READY_STATES = {"ARRIVED", "IDLE", "STOPPED"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stress-test serve_web.py fleet API with queued goals.")
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL, help="Fleet web backend base URL.")
    parser.add_argument("--map-dir", type=Path, default=DEFAULT_MAP_DIR, help="Path to *.smap directory.")
    parser.add_argument("--robots", type=int, default=4, help="How many robots to create.")
    parser.add_argument("--queue-length", type=int, default=6, help="How many goals to queue per robot.")
    parser.add_argument("--speed", type=float, default=0.75, help="Planner speed sent to /api/fleet/orders.")
    parser.add_argument("--tick-period", type=float, default=0.10, help="Seconds between /api/fleet/tick calls.")
    parser.add_argument("--status-period", type=float, default=1.0, help="Seconds between console summaries.")
    parser.add_argument("--dispatch-cooldown", type=float, default=0.35, help="Minimum seconds between dispatch batches.")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for deterministic queues.")
    parser.add_argument("--name-prefix", default="stress", help="Robot name prefix.")
    parser.add_argument("--candidate-pool", type=int, default=32, help="Candidate LM sample size when building queues.")
    parser.add_argument("--min-route-length", type=float, default=2.5, help="Prefer routes at least this long.")
    parser.add_argument("--max-runtime", type=float, default=180.0, help="Stop after this many seconds.")
    parser.add_argument("--cleanup-on-exit", action="store_true", help="Remove created robots on exit.")
    return parser.parse_args()


def resolve_map_dir(map_dir: Path) -> Path:
    if map_dir.exists():
        return map_dir.resolve()
    if map_dir.is_absolute():
        return map_dir
    project_root = Path(__file__).resolve().parents[1]
    relocated = project_root / map_dir
    if relocated.exists():
        return relocated.resolve()
    relocated = project_root / "map_data" / map_dir
    if relocated.exists():
        return relocated.resolve()
    return map_dir


def http_json(base_url: str, method: str, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
    url = f"{base_url.rstrip('/')}{path}"
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = Request(url, data=data, headers=headers, method=method)
    try:
        with urlopen(request, timeout=10.0) as response:
            raw = response.read().decode("utf-8")
    except HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"{method} {path} -> HTTP {exc.code}: {body}") from exc
    except URLError as exc:
        raise RuntimeError(f"{method} {path} -> network error: {exc.reason}") from exc
    try:
        decoded = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"{method} {path} -> invalid JSON response: {raw[:300]}") from exc
    if not isinstance(decoded, dict):
        raise RuntimeError(f"{method} {path} -> expected JSON object, got {type(decoded).__name__}")
    return decoded


def get_state(base_url: str) -> dict[str, Any]:
    return http_json(base_url, "GET", "/api/fleet/state")


def tick(base_url: str, started_at: float) -> dict[str, Any]:
    return http_json(
        base_url,
        "POST",
        "/api/fleet/tick",
        {"clientElapsed": max(0.0, time.monotonic() - started_at)},
    )


def choose_spawn_landmarks(landmarks: dict[str, Any], count: int) -> list[str]:
    if count > len(landmarks):
        raise ValueError(f"Requested {count} robots, but map only has {len(landmarks)} landmarks.")

    candidates = list(landmarks.values())
    center_x = sum(item.x for item in candidates) / len(candidates)
    center_y = sum(item.y for item in candidates) / len(candidates)
    first = max(candidates, key=lambda item: math.hypot(item.x - center_x, item.y - center_y))
    chosen = [first]

    while len(chosen) < count:
        chosen_names = {landmark.name for landmark in chosen}
        remaining = [item for item in candidates if item.name not in chosen_names]
        next_item = max(
            remaining,
            key=lambda item: min(math.hypot(item.x - other.x, item.y - other.y) for other in chosen),
        )
        chosen.append(next_item)
    return [item.name for item in chosen]


def build_goal_queue(
    planner: LmRoutePlanner,
    current_lm: str,
    queue_length: int,
    rng: random.Random,
    candidate_pool: int,
    min_route_length: float,
) -> list[str]:
    names = sorted(planner.landmarks)
    queue: list[str] = []
    start_lm = current_lm

    for _ in range(queue_length):
        shuffled = names[:]
        rng.shuffle(shuffled)
        sampled = shuffled[: min(candidate_pool, len(shuffled))]
        best_goal = ""
        best_score = -1.0
        fallback_goal = ""
        fallback_score = -1.0

        for goal_lm in sampled:
            if goal_lm == start_lm:
                continue
            try:
                route = planner.find_route(start_lm, goal_lm)
            except ValueError:
                continue
            score = float(route.length)
            jitter = rng.random() * 0.15
            weighted = score * (1.0 + jitter)
            if score > fallback_score:
                fallback_goal = goal_lm
                fallback_score = score
            if score >= min_route_length and weighted > best_score:
                best_goal = goal_lm
                best_score = weighted

        chosen = best_goal or fallback_goal
        if not chosen:
            raise ValueError(f"Could not find a reachable goal from {start_lm}")
        queue.append(chosen)
        start_lm = chosen

    return queue


def robot_names(prefix: str, count: int) -> list[str]:
    return [f"{prefix}-{index + 1:02d}" for index in range(count)]


def remove_existing_test_robots(base_url: str, names: list[str]) -> None:
    state = get_state(base_url)
    known = {str(robot.get("name", "")) for robot in state.get("robots", []) if isinstance(robot, dict)}
    for name in names:
        if name in known:
            http_json(base_url, "POST", "/api/fleet/robots/remove", {"name": name})


def add_robot(base_url: str, name: str, current_lm: str) -> None:
    http_json(
        base_url,
        "POST",
        "/api/fleet/robots",
        {
            "name": name,
            "currentLm": current_lm,
            "status": "IDLE",
        },
    )


def dispatch_ready_robots(
    base_url: str,
    planner: LmRoutePlanner,
    queues: dict[str, deque[str]],
    state: dict[str, Any],
    speed: float,
) -> tuple[list[str], dict[str, Any] | None]:
    robots = {
        str(robot.get("name", "")): robot
        for robot in state.get("robots", [])
        if isinstance(robot, dict)
    }
    requests: list[dict[str, Any]] = []
    requested_goals: dict[str, str] = {}

    for name, queue in queues.items():
        if not queue:
            continue
        robot = robots.get(name)
        if not robot:
            continue
        status = str(robot.get("status", "") or "")
        target = str(robot.get("targetName") or robot.get("targetLm") or "").strip()
        current_lm = str(robot.get("currentLm") or "").strip()
        if status not in READY_STATES or target or not current_lm:
            continue

        goal_lm = queue[0]
        try:
            planner.find_route(current_lm, goal_lm)
        except ValueError:
            continue

        request: dict[str, Any] = {
            "name": name,
            "startLm": current_lm,
            "goalLm": goal_lm,
        }
        pose = robot.get("pose")
        if isinstance(pose, dict):
            request["startPose"] = {
                "x": float(pose.get("x", 0.0) or 0.0),
                "y": float(pose.get("y", 0.0) or 0.0),
                "yaw": float(pose.get("yaw", 0.0) or 0.0),
            }
        requests.append(request)
        requested_goals[name] = goal_lm

    if not requests:
        return [], None

    result = http_json(
        base_url,
        "POST",
        "/api/fleet/orders",
        {
            "speed": speed,
            "blocked_lms": [],
            "robots": requests,
        },
    )
    planned_names = [
        str(plan.get("robot", ""))
        for plan in result.get("plans", [])
        if isinstance(plan, dict) and str(plan.get("robot", ""))
    ]
    planned_set = set(planned_names)
    for name, goal_lm in requested_goals.items():
        if name in planned_set and queues[name] and queues[name][0] == goal_lm:
            queues[name].popleft()
    return planned_names, result


def print_plan(names: list[str], spawns: list[str], queues: dict[str, deque[str]]) -> None:
    print("Stress fleet scenario:")
    for name, spawn in zip(names, spawns, strict=True):
        chain = " -> ".join([spawn, *list(queues[name])])
        print(f"  {name}: {chain}")


def compact_status(state: dict[str, Any], queues: dict[str, deque[str]]) -> str:
    parts: list[str] = []
    for robot in state.get("robots", []):
        if not isinstance(robot, dict):
            continue
        name = str(robot.get("name", ""))
        status = str(robot.get("status", ""))
        current_lm = str(robot.get("currentLm") or "-")
        target_lm = str(robot.get("targetName") or robot.get("targetLm") or "-")
        remaining = len(queues.get(name, ()))
        parts.append(f"{name}:{status}:{current_lm}->{target_lm}:q{remaining}")
    return " | ".join(sorted(parts))


def all_done(state: dict[str, Any], queues: dict[str, deque[str]], names: list[str]) -> bool:
    robots = {
        str(robot.get("name", "")): robot
        for robot in state.get("robots", [])
        if isinstance(robot, dict)
    }
    for name in names:
        if queues[name]:
            return False
        robot = robots.get(name)
        if robot is None:
            return False
        status = str(robot.get("status", "") or "")
        target = str(robot.get("targetName") or robot.get("targetLm") or "").strip()
        if target:
            return False
        if status not in READY_STATES | {"BLOCKED"}:
            return False
    return True


def cleanup(base_url: str, names: list[str]) -> None:
    for name in names:
        try:
            http_json(base_url, "POST", "/api/fleet/robots/remove", {"name": name})
        except Exception:
            pass


def main() -> int:
    args = parse_args()
    rng = random.Random(args.seed)
    map_dir = resolve_map_dir(args.map_dir)
    loaded_map = WarehouseMapLoader(map_dir).load()
    planner = LmRoutePlanner(loaded_map.landmarks, loaded_map.edges)

    if args.robots <= 0:
        raise SystemExit("--robots must be > 0")
    if args.queue_length <= 0:
        raise SystemExit("--queue-length must be > 0")

    names = robot_names(args.name_prefix, args.robots)
    spawns = choose_spawn_landmarks(loaded_map.landmarks, args.robots)
    queues = {
        name: deque(
            build_goal_queue(
                planner,
                current_lm=spawn,
                queue_length=args.queue_length,
                rng=rng,
                candidate_pool=args.candidate_pool,
                min_route_length=args.min_route_length,
            )
        )
        for name, spawn in zip(names, spawns, strict=True)
    }

    print_plan(names, spawns, queues)
    remove_existing_test_robots(args.base_url, names)
    for name, spawn in zip(names, spawns, strict=True):
        add_robot(args.base_url, name, spawn)

    started_at = time.monotonic()
    last_summary_at = 0.0
    last_dispatch_at = -999.0

    try:
        while True:
            state = tick(args.base_url, started_at)
            now = time.monotonic()

            if now - last_dispatch_at >= args.dispatch_cooldown:
                planned, result = dispatch_ready_robots(
                    args.base_url,
                    planner=planner,
                    queues=queues,
                    state=state,
                    speed=args.speed,
                )
                if planned:
                    debug = result.get("debug", {}) if isinstance(result, dict) else {}
                    reason = debug.get("reason", "ok") if isinstance(debug, dict) else "ok"
                    print(f"[dispatch] {', '.join(planned)} reason={reason}")
                    last_dispatch_at = now
                    state = result.get("fleetState", state) if isinstance(result, dict) else state

            if now - last_summary_at >= args.status_period:
                print(f"[state] {compact_status(state, queues)}")
                last_summary_at = now

            if all_done(state, queues, names):
                print("Stress scenario finished: all queues completed.")
                return 0

            if now - started_at >= args.max_runtime:
                print("Stress scenario timed out.")
                return 2

            time.sleep(max(0.02, args.tick_period))
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    finally:
        if args.cleanup_on_exit:
            cleanup(args.base_url, names)


if __name__ == "__main__":
    sys.exit(main())
