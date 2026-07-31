#!/usr/bin/env python3
"""Run a bounded continuous-order benchmark against Fleet Manager Sim."""

from __future__ import annotations

import argparse
import json
import math
import random
import time
import urllib.error
import urllib.request
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


class Client:
    def __init__(self, base_url: str, timeout_sec: float) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec

    def request(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        body = (
            json.dumps(payload, separators=(",", ":")).encode("utf-8")
            if payload is not None
            else None
        )
        request = urllib.request.Request(
            f"{self.base_url}/{path.lstrip('/')}",
            data=body,
            headers={
                "Accept": "application/json",
                **({"Content-Type": "application/json"} if body is not None else {}),
            },
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                result = json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"{method} {path}: {exc}") from exc
        if not isinstance(result, dict):
            raise RuntimeError(f"{method} {path}: expected a JSON object")
        return result

    def get_state(self) -> dict[str, Any]:
        return self.request("GET", "/api/fleet-manager-sim/state")

    def benchmark(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self.request("POST", "/api/fleet-manager-sim/benchmark", payload)

    def add_robot(self, name: str, lm_name: str) -> dict[str, Any]:
        return self.request(
            "POST",
            "/api/fleet-manager-sim/robots",
            {"name": name, "spawnLm": lm_name, "mode": "simulated"},
        )


def load_landmarks(path: Path) -> dict[str, tuple[float, float]]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    lms = payload.get("LMs", []) if isinstance(payload, dict) else []
    return {
        str(item["name"]): (float(item["x"]), float(item["y"]))
        for item in lms
        if isinstance(item, dict) and item.get("name")
    }


def spaced_random_lms(
    landmarks: dict[str, tuple[float, float]],
    *,
    count: int,
    seed: int,
    minimum_distance_m: float,
) -> list[str]:
    candidates = sorted(landmarks)
    random.Random(seed + 7919).shuffle(candidates)
    selected: list[str] = []
    for name in candidates:
        point = landmarks[name]
        if all(
            math.dist(point, landmarks[other]) + 1e-9 >= minimum_distance_m
            for other in selected
        ):
            selected.append(name)
            if len(selected) == count:
                return selected
    raise RuntimeError(
        f"only {len(selected)} of {count} LMs can be placed "
        f"{minimum_distance_m:.2f} m apart"
    )


def setup_fleet(
    client: Client,
    *,
    landmarks: dict[str, tuple[float, float]],
    robot_count: int,
    seed: int,
    minimum_distance_m: float,
) -> dict[str, Any]:
    client.benchmark({"count": 0, "reset": True, "seed": seed})
    spawn_lms = spaced_random_lms(
        landmarks,
        count=robot_count,
        seed=seed,
        minimum_distance_m=minimum_distance_m,
    )
    for index, lm_name in enumerate(spawn_lms, start=1):
        client.add_robot(f"bench_{index:03d}", lm_name)
    state = client.get_state()
    robots = state.get("robots", [])
    if len(robots) != robot_count:
        raise RuntimeError(f"expected {robot_count} robots, got {len(robots)}")
    return state


class Benchmark:
    def __init__(self, args: argparse.Namespace, client: Client) -> None:
        self.args = args
        self.client = client
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.session_id = f"fleet-sim-dynamic-{stamp}-{args.seed}"
        args.output_dir.mkdir(parents=True, exist_ok=True)
        self.samples_path = args.output_dir / f"{self.session_id}.jsonl"
        self.summary_path = args.output_dir / f"{self.session_id}-summary.json"
        self.samples_file = self.samples_path.open("w", encoding="utf-8")
        self.started_mono = time.monotonic()
        self.previous_sample_mono = self.started_mono
        self.previous_positions: dict[str, tuple[float, float]] = {}
        self.stationary_streak: dict[str, float] = defaultdict(float)
        self.max_stationary_streak: dict[str, float] = defaultdict(float)
        self.waiting_robot_sec = 0.0
        self.stationary_robot_sec = 0.0
        self.min_pair_distance = math.inf
        self.max_waiting = 0
        self.generation_sample: dict[str, Any] = {}

    def close(self) -> None:
        self.samples_file.close()

    def observe(self, state: dict[str, Any], now: float) -> dict[str, Any]:
        dt = max(0.0, min(self.args.poll_sec * 3.0, now - self.previous_sample_mono))
        self.previous_sample_mono = now
        robots = state.get("robots", [])
        positions: list[tuple[str, float, float]] = []
        statuses: Counter[str] = Counter()
        stationary = 0
        waiting = 0
        for robot in robots:
            name = str(robot.get("name") or "")
            pose = robot.get("pose") or {}
            x = float(pose.get("x") or 0.0)
            y = float(pose.get("y") or 0.0)
            status = str(robot.get("status") or "")
            statuses[status] += 1
            positions.append((name, x, y))
            previous = self.previous_positions.get(name)
            moved = previous is not None and math.hypot(x - previous[0], y - previous[1]) >= 0.01
            has_order = bool(robot.get("assignedOrderId") or robot.get("activeOrderId"))
            if has_order and not moved:
                stationary += 1
                self.stationary_streak[name] += dt
                self.max_stationary_streak[name] = max(
                    self.max_stationary_streak[name],
                    self.stationary_streak[name],
                )
            else:
                self.stationary_streak[name] = 0.0
            if has_order and (
                status in {"WAITING", "BLOCKED", "REPLANNING"}
                or bool(robot.get("waitDependency"))
            ):
                waiting += 1
            self.previous_positions[name] = (x, y)

        self.waiting_robot_sec += waiting * dt
        self.stationary_robot_sec += stationary * dt
        self.max_waiting = max(self.max_waiting, waiting)
        for index, (_name_a, ax, ay) in enumerate(positions):
            for _name_b, bx, by in positions[index + 1 :]:
                self.min_pair_distance = min(
                    self.min_pair_distance,
                    math.hypot(ax - bx, ay - by),
                )
        dynamic = dict(state.get("dynamicBenchmark") or {})
        traffic = dict(state.get("traffic") or {})
        sample = {
            "elapsedSec": round(now - self.started_mono, 3),
            "dynamic": dynamic,
            "traffic": traffic,
            "statuses": dict(statuses),
            "waiting": waiting,
            "stationary": stationary,
        }
        self.samples_file.write(
            json.dumps(sample, ensure_ascii=False, separators=(",", ":")) + "\n"
        )
        self.samples_file.flush()
        return sample

    def run(self) -> dict[str, Any]:
        start = self.client.benchmark(
            {
                "action": "plan",
                "count": self.args.robot_count,
                "reset": False,
                "seed": self.args.seed,
                "horizonSec": self.args.horizon_sec,
                "orderIntervalSec": self.args.order_interval_sec,
                "queueDepth": 1,
                "speed": self.args.robot_speed,
                "acceleration": self.args.robot_acceleration,
                "rotate": True,
                "turnSpeed": self.args.robot_rotate_speed,
                "fast": True,
            }
        )
        print(
            json.dumps(
                {
                    "sessionId": self.session_id,
                    "started": (start.get("benchmark") or {}).get("ordersGenerated"),
                    "samples": str(self.samples_path),
                }
            ),
            flush=True,
        )
        generation_deadline = self.started_mono + self.args.duration_sec
        drain_deadline = generation_deadline + self.args.drain_sec
        stopped = False
        next_progress = self.started_mono
        final_state: dict[str, Any] = {}
        while True:
            cycle_started = time.monotonic()
            now = cycle_started
            if not stopped and now >= generation_deadline:
                self.client.benchmark(
                    {
                        "action": "stop",
                        "count": self.args.robot_count,
                        "reset": False,
                        "seed": self.args.seed,
                    }
                )
                stopped = True
            final_state = self.client.get_state()
            sample = self.observe(final_state, now)
            if not stopped:
                self.generation_sample = sample
            dynamic = sample["dynamic"]
            if now >= next_progress:
                print(
                    json.dumps(
                        {
                            "elapsedSec": round(now - self.started_mono, 1),
                            "generated": dynamic.get("ordersGenerated", 0),
                            "completed": dynamic.get("ordersCompleted", 0),
                            "outstanding": dynamic.get("ordersOutstanding", 0),
                            "throughputPerMin": dynamic.get("throughputOrdersPerMin", 0),
                            "waiting": sample["waiting"],
                            "stationary": sample["stationary"],
                            "rollbacks": dynamic.get("runtimeSafetyRollbacks", 0),
                        }
                    ),
                    flush=True,
                )
                next_progress = now + self.args.progress_sec
            if stopped and int(dynamic.get("ordersOutstanding", 0) or 0) <= 0:
                break
            if now >= drain_deadline:
                break
            delay = self.args.poll_sec - (time.monotonic() - cycle_started)
            if delay > 0.0:
                time.sleep(delay)

        final_dynamic = dict(final_state.get("dynamicBenchmark") or {})
        generation_dynamic = dict(self.generation_sample.get("dynamic") or {})
        elapsed = max(1e-6, time.monotonic() - self.started_mono)
        summary = {
            "sessionId": self.session_id,
            "map": final_state.get("mapName"),
            "robots": self.args.robot_count,
            "seed": self.args.seed,
            "generationDurationSec": self.args.duration_sec,
            "elapsedSec": round(elapsed, 3),
            "drained": int(final_dynamic.get("ordersOutstanding", 0) or 0) == 0,
            "generation": generation_dynamic,
            "final": final_dynamic,
            "traffic": {
                "waitingRobotSec": round(self.waiting_robot_sec, 3),
                "stationaryRobotSec": round(self.stationary_robot_sec, 3),
                "maxWaitingRobots": self.max_waiting,
                "maxStationarySec": round(
                    max(self.max_stationary_streak.values(), default=0.0),
                    3,
                ),
                "minPairDistanceM": (
                    round(self.min_pair_distance, 4)
                    if math.isfinite(self.min_pair_distance)
                    else None
                ),
                **dict(final_state.get("traffic") or {}),
            },
            "samples": str(self.samples_path),
        }
        self.summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
        return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8780")
    parser.add_argument(
        "--lms",
        type=Path,
        default=Path(
            "fleet_manager/map_data/maps_out/"
            "benchmark_open_kiva.smap/LMs.yaml"
        ),
    )
    parser.add_argument("--robot-count", type=int, default=50)
    parser.add_argument("--seed", type=int, default=20260727)
    parser.add_argument("--minimum-start-distance-m", type=float, default=2.4)
    parser.add_argument("--setup", action="store_true")
    parser.add_argument("--setup-only", action="store_true")
    parser.add_argument(
        "--start-state-output",
        type=Path,
        default=Path("/tmp/fmsim_ab_start.json"),
    )
    parser.add_argument("--duration-sec", type=float, default=300.0)
    parser.add_argument("--drain-sec", type=float, default=180.0)
    parser.add_argument("--poll-sec", type=float, default=1.0)
    parser.add_argument("--progress-sec", type=float, default=10.0)
    parser.add_argument("--http-timeout-sec", type=float, default=30.0)
    parser.add_argument("--horizon-sec", type=float, default=10.0)
    parser.add_argument("--order-interval-sec", type=float, default=0.25)
    parser.add_argument("--robot-speed", type=float, default=1.37)
    parser.add_argument("--robot-acceleration", type=float, default=0.6)
    parser.add_argument("--robot-rotate-speed", type=float, default=0.9)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("var/fleet_sim_benchmarks"),
    )
    args = parser.parse_args()
    if args.setup_only:
        args.setup = True
    return args


def main() -> int:
    args = parse_args()
    client = Client(args.base_url, args.http_timeout_sec)
    if args.setup:
        state = setup_fleet(
            client,
            landmarks=load_landmarks(args.lms),
            robot_count=args.robot_count,
            seed=args.seed,
            minimum_distance_m=args.minimum_start_distance_m,
        )
        args.start_state_output.write_text(
            json.dumps(state, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        robots = state.get("robots", [])
        print(
            json.dumps(
                {
                    "robots": len(robots),
                    "uniqueLms": len(
                        {str(robot.get("currentLm") or "") for robot in robots}
                    ),
                    "startState": str(args.start_state_output),
                }
            ),
            flush=True,
        )
        if args.setup_only:
            return 0

    benchmark = Benchmark(args, client)
    try:
        benchmark.run()
    finally:
        benchmark.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
