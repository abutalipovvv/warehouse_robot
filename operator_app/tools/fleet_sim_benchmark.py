from __future__ import annotations

import argparse
import copy
import json
import random
import sys
import time
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from fleet_manager.core.mapf.fleet_planner import FleetMapfPlanner
from fleet_manager.core.route_core.map_loader import WarehouseMapLoader
from fleet_manager.core.route_core.models import Landmark
from fleet_manager.core.route_core.params import load_route_params


DEFAULT_MAP_NAME = "benchmark_open_kiva"
DEFAULT_MAP_ROOT = PROJECT_ROOT / "fleet_manager" / "map_data" / "maps_out"
DEFAULT_PARAMS = PROJECT_ROOT / "fleet_manager" / "config" / "params.yaml"


def _resolve_map_dir(map_root: Path, map_name: str) -> Path:
    raw = Path(map_name).expanduser()
    if raw.is_dir():
        return raw.resolve()
    safe_name = raw.name
    if not safe_name.endswith(".smap"):
        safe_name = f"{safe_name}.smap"
    return (map_root / safe_name).resolve()


def _distance_sq(a: Landmark, b: Landmark) -> float:
    return ((a.x - b.x) ** 2) + ((a.y - b.y) ** 2)


def _make_requests(
    landmarks: dict[str, Landmark],
    count: int,
    seed: int,
) -> list[dict[str, Any]]:
    if count <= 0:
        return []
    if len(landmarks) < count * 2:
        raise ValueError(f"map needs at least {count * 2} landmarks for {count} unique starts and goals")

    rng = random.Random(seed)
    names = sorted(landmarks)
    rng.shuffle(names)
    start_names = names[:count]
    used_goals: set[str] = set()
    requests: list[dict[str, Any]] = []

    for index, start_name in enumerate(start_names, start=1):
        start = landmarks[start_name]
        goal_name = max(
            (
                name
                for name in names
                if name != start_name and name not in used_goals
            ),
            key=lambda name: _distance_sq(start, landmarks[name]),
        )
        used_goals.add(goal_name)
        requests.append(
            {
                "name": f"bench_{index:03d}",
                "startLm": start_name,
                "goalLm": goal_name,
            }
        )
    return requests


def _prepare_params(params: dict[str, Any], backend: str, max_time: int | None) -> dict[str, Any]:
    prepared = copy.deepcopy(params)
    fleet = prepared.setdefault("fleet", {})
    if not isinstance(fleet, dict):
        fleet = {}
        prepared["fleet"] = fleet
    if backend:
        fleet["planner_backend"] = backend
    if max_time is not None:
        fleet["cbs_low_level_max_time"] = int(max_time)
    return prepared


def _summarize_result(count: int, elapsed_ms: float, result: dict[str, Any]) -> dict[str, Any]:
    plans = result.get("plans", [])
    if not isinstance(plans, list):
        plans = []
    arrivals = [
        float(plan.get("arrivalTime", 0.0) or 0.0)
        for plan in plans
        if isinstance(plan, dict)
    ]
    debug = result.get("debug", {})
    if not isinstance(debug, dict):
        debug = {}
    return {
        "count": count,
        "ok": bool(result.get("ok")),
        "planned": len(plans),
        "elapsedMs": round(elapsed_ms, 3),
        "plannerBackend": debug.get("plannerBackend", ""),
        "reason": debug.get("reason", ""),
        "fallback": debug.get("reservedFallbackReason", ""),
        "expandedNodes": debug.get("expandedNodes", 0),
        "highLevelNodes": debug.get("highLevelNodes", 0),
        "maxArrivalSec": round(max(arrivals), 3) if arrivals else 0.0,
        "avgArrivalSec": round(sum(arrivals) / len(arrivals), 3) if arrivals else 0.0,
        "sumArrivalSec": round(sum(arrivals), 3),
    }


def run_benchmark(
    map_dir: Path,
    params_path: Path,
    counts: list[int],
    seed: int,
    speed: float,
    backend: str,
    max_time: int | None,
    progress: bool = True,
) -> dict[str, Any]:
    loaded = WarehouseMapLoader(map_dir).load()
    params = _prepare_params(load_route_params(params_path, create=True), backend, max_time)
    planner = FleetMapfPlanner(loaded.landmarks, loaded.edges, params=params)
    cases: list[dict[str, Any]] = []
    for count in counts:
        requests = _make_requests(loaded.landmarks, count, seed + count)
        payload = {
            "robots": requests,
            "speed": speed,
        }
        started = time.perf_counter()
        result = planner.plan(payload)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        summary = _summarize_result(count, elapsed_ms, result)
        cases.append(summary)
        if progress:
            print(
                (
                    f"[bench] robots={count} ok={summary['ok']} "
                    f"planned={summary['planned']}/{summary['count']} "
                    f"elapsed_ms={summary['elapsedMs']} "
                    f"expanded={summary['expandedNodes']}"
                ),
                file=sys.stderr,
                flush=True,
            )
    return {
        "ok": all(case["ok"] and case["planned"] == case["count"] for case in cases),
        "mapDir": str(map_dir),
        "mapName": loaded.map_metadata.map_name,
        "landmarks": len(loaded.landmarks),
        "directedEdges": len(loaded.edges),
        "seed": seed,
        "speed": speed,
        "backend": backend or planner.planner_backend,
        "cases": cases,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark Fleet Manager Sim MAPF planning on an .smap graph.")
    parser.add_argument("--map", default=DEFAULT_MAP_NAME)
    parser.add_argument("--map-root", type=Path, default=DEFAULT_MAP_ROOT)
    parser.add_argument("--params", type=Path, default=DEFAULT_PARAMS)
    parser.add_argument("--counts", type=int, nargs="+", default=[20, 50, 100])
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--speed", type=float, default=1.37)
    parser.add_argument("--backend", choices=["", "hybrid", "rolling_sipp", "cbs"], default="hybrid")
    parser.add_argument("--max-time", type=int, default=320)
    parser.add_argument("--quiet", action="store_true", help="Suppress per-case progress on stderr.")
    args = parser.parse_args()

    map_dir = _resolve_map_dir(args.map_root, args.map)
    summary = run_benchmark(
        map_dir=map_dir,
        params_path=args.params,
        counts=args.counts,
        seed=args.seed,
        speed=args.speed,
        backend=args.backend,
        max_time=args.max_time,
        progress=not args.quiet,
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
