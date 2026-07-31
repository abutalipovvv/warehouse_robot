#!/usr/bin/env python3
"""Compare the refactored fleet planner with a planner stored in Git.

The check has two parts:

* deterministic, seeded differential planning on a small grid;
* an alternating-order latency benchmark on a real warehouse map.

When the refactor is still uncommitted, use ``--legacy-ref HEAD``. After it is
committed separately, the default ``HEAD^`` compares it with its parent.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
from pathlib import Path
import random
import statistics
import subprocess
import sys
from time import perf_counter
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fleet_manager.core.mapf.fleet_planner import FleetMapfPlanner
from fleet_manager.core.route_core.map_loader import WarehouseMapLoader
from fleet_manager.core.route_core.models import GraphEdge, Landmark, WorldPoint


PLANNER_SOURCE = "fleet_manager/core/mapf/fleet_planner.py"
DEFAULT_MAP = (
    ROOT
    / "fleet_manager"
    / "map_data"
    / "maps_out"
    / "smart_symbotic_33_39.smap"
)
DEFAULT_PARAMS = ROOT / "fleet_manager" / "config" / "params.yaml"


def load_legacy_planner(git_ref: str) -> type:
    source = subprocess.run(
        ["git", "show", f"{git_ref}:{PLANNER_SOURCE}"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    module_name = "fleet_manager.core.mapf._fleet_planner_benchmark_legacy"
    spec = importlib.util.spec_from_loader(module_name, loader=None)
    if spec is None:
        raise RuntimeError("could not create the legacy planner module")
    module = importlib.util.module_from_spec(spec)
    module.__package__ = "fleet_manager.core.mapf"
    sys.modules[module_name] = module
    exec(
        compile(source, f"<{git_ref}:{PLANNER_SOURCE}>", "exec"),
        module.__dict__,
    )
    return module.FleetMapfPlanner


def run_seeded_differential(
    legacy_planner: type,
    *,
    seed: int,
    cases: int,
) -> int:
    landmarks, edges = _grid_graph(size=5)
    params = {
        "navigation": {
            "route_speed": 0.8,
            "route_acceleration": 0.4,
            "turn_speed": 1.1,
        },
        "fleet": {
            "planner_backend": "hybrid",
            "reservation_time_step_sec": 0.5,
            "wait_time_sec": 0.5,
            "wait_cost": 6,
            "cbs_low_level_max_time": 100,
            "cbs_max_high_level_nodes": 1000,
            "cbs_max_planning_time_sec": 3.0,
            "reserved_edge_detour_enabled": False,
            "controlled_corridors_enabled": False,
        },
    }
    rng = random.Random(seed)
    names = sorted(landmarks)
    edge_pairs = [
        (edge.from_name, edge.to_name)
        for edge in edges
    ]
    backends = ("cbs", "rolling_sipp", "hybrid")

    for case_index in range(cases):
        robot_count = 1 if case_index < max(1, cases * 2 // 3) else 2
        chosen = rng.sample(names, robot_count * 2)
        robots = []
        for robot_index in range(robot_count):
            start = chosen[robot_index]
            goal = chosen[robot_index + robot_count]
            start_lm = landmarks[start]
            robots.append(
                {
                    "name": f"r{robot_index}",
                    "startLm": start,
                    "goalLm": goal,
                    "startPose": {
                        "x": start_lm.x,
                        "y": start_lm.y,
                        "yaw": rng.choice(
                            (
                                0.0,
                                math.pi / 2.0,
                                math.pi,
                                -math.pi / 2.0,
                            )
                        ),
                    },
                }
            )
        payload: dict[str, Any] = {
            "plannerBackend": backends[case_index % len(backends)],
            "speed": rng.choice((0.45, 0.7, 1.0)),
            "acceleration": rng.choice((0.0, 0.4, 0.8)),
            "rotate": bool(case_index % 2),
            "turnSpeed": rng.choice((0.7, 1.1)),
            "robots": robots,
        }
        if case_index % 5 == 0:
            terminals = {
                robot["startLm"]
                for robot in robots
            } | {
                robot["goalLm"]
                for robot in robots
            }
            candidates = [
                pair
                for pair in edge_pairs
                if pair[0] not in terminals
                and pair[1] not in terminals
            ]
            source, target = rng.choice(candidates)
            payload["blockedEdges"] = [f"{source}->{target}"]
        if case_index % 7 == 0:
            starts = {robot["startLm"] for robot in robots}
            node = rng.choice(
                [name for name in names if name not in starts]
            )
            payload["reservedVertexIntervals"] = [
                {
                    "node": node,
                    "start": 0.5,
                    "end": 1.5,
                    "robot": "external",
                }
            ]

        _assert_same_outcome(
            legacy_planner(landmarks, edges, params=params),
            FleetMapfPlanner(landmarks, edges, params=params),
            payload,
            label=f"seeded case {case_index}",
        )

    invalid_payloads = (
        {"robots": "not-a-list"},
        {"robots": [{}]},
        {
            "robots": [
                {
                    "name": "r",
                    "startLm": "N00",
                    "goalLm": "N44",
                    "routeNodes": ["N00", "N44"],
                }
            ]
        },
        {
            "robots": [
                {"name": "r", "startLm": "N00", "goalLm": "N01"},
                {"name": "r", "startLm": "N10", "goalLm": "N11"},
            ]
        },
    )
    for case_index, payload in enumerate(invalid_payloads):
        _assert_same_outcome(
            legacy_planner(landmarks, edges, params=params),
            FleetMapfPlanner(landmarks, edges, params=params),
            payload,
            label=f"invalid case {case_index}",
        )
    return cases + len(invalid_payloads)


def run_map_benchmark(
    legacy_planner: type,
    *,
    map_path: Path,
    params_path: Path,
    start_lm: str,
    goal_lm: str,
    warmups: int,
    iterations: int,
) -> dict[str, Any]:
    loaded = WarehouseMapLoader(map_path).load()
    params = yaml.safe_load(params_path.read_text(encoding="utf-8"))
    start = loaded.landmarks[start_lm]
    payload = {
        "plannerBackend": "rolling_sipp",
        "speed": 1.0,
        "acceleration": 0.0,
        "rotate": True,
        "turnSpeed": 0.9,
        "robots": [
            {
                "name": "benchmark",
                "startLm": start_lm,
                "goalLm": goal_lm,
                "startPose": {
                    "x": start.x,
                    "y": start.y,
                    "yaw": 0.0,
                },
            }
        ],
    }
    legacy = legacy_planner(
        loaded.landmarks,
        loaded.edges,
        params=params,
    )
    refactored = FleetMapfPlanner(
        loaded.landmarks,
        loaded.edges,
        params=params,
    )
    for _ in range(warmups):
        legacy.plan(payload)
        refactored.plan(payload)

    legacy_ms: list[float] = []
    refactored_ms: list[float] = []
    for iteration in range(iterations):
        planners = (
            ((legacy, legacy_ms), (refactored, refactored_ms))
            if iteration % 2 == 0
            else ((refactored, refactored_ms), (legacy, legacy_ms))
        )
        for planner, samples in planners:
            started = perf_counter()
            planner.plan(payload)
            samples.append((perf_counter() - started) * 1000.0)

    legacy_result = legacy.plan(payload)
    refactored_result = refactored.plan(payload)
    if legacy_result != refactored_result:
        raise AssertionError("real-map legacy/refactored results differ")

    legacy_stats = _timing_stats(legacy_ms)
    refactored_stats = _timing_stats(refactored_ms)
    plan = refactored_result["plans"][0]
    return {
        "map": str(map_path),
        "route": f"{start_lm}->{goal_lm}",
        "iterations": iterations,
        "legacy": legacy_stats,
        "refactored": refactored_stats,
        "median_ratio": (
            refactored_stats["median_ms"]
            / legacy_stats["median_ms"]
        ),
        "nodes": len(plan["nodes"]),
        "trajectory_samples": len(plan["trajectory"]),
    }


def _assert_same_outcome(
    legacy: Any,
    refactored: Any,
    payload: dict[str, Any],
    *,
    label: str,
) -> None:
    outcomes: list[tuple[str, Any]] = []
    for planner in (legacy, refactored):
        try:
            outcomes.append(("result", planner.plan(payload)))
        except Exception as error:
            outcomes.append(
                (
                    "error",
                    (type(error).__qualname__, str(error)),
                )
            )
    if outcomes[0] != outcomes[1]:
        raise AssertionError(
            f"{label} differs:\nlegacy={outcomes[0]!r}\n"
            f"refactored={outcomes[1]!r}"
        )


def _grid_graph(
    *,
    size: int,
) -> tuple[dict[str, Landmark], list[GraphEdge]]:
    landmarks = {
        f"N{x}{y}": Landmark(
            name=f"N{x}{y}",
            x=float(x),
            y=float(y),
        )
        for x in range(size)
        for y in range(size)
    }
    edges: list[GraphEdge] = []
    for x in range(size):
        for y in range(size):
            for delta_x, delta_y in ((1, 0), (0, 1)):
                other_x = x + delta_x
                other_y = y + delta_y
                if other_x >= size or other_y >= size:
                    continue
                first = f"N{x}{y}"
                second = f"N{other_x}{other_y}"
                for source, target in (
                    (first, second),
                    (second, first),
                ):
                    start = landmarks[source]
                    goal = landmarks[target]
                    edges.append(
                        GraphEdge(
                            from_name=source,
                            to_name=target,
                            length=math.hypot(
                                goal.x - start.x,
                                goal.y - start.y,
                            ),
                            kind="line",
                            edge_type="FeatureLine",
                            world_points=(
                                WorldPoint(start.x, start.y),
                                WorldPoint(goal.x, goal.y),
                            ),
                            properties={
                                "direction": 0,
                                "max_speed": 1.0,
                            },
                        )
                    )
    return landmarks, edges


def _timing_stats(samples: list[float]) -> dict[str, float]:
    ordered = sorted(samples)
    p95_index = max(
        0,
        min(len(ordered) - 1, math.ceil(len(ordered) * 0.95) - 1),
    )
    return {
        "median_ms": statistics.median(ordered),
        "p95_ms": ordered[p95_index],
        "min_ms": ordered[0],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legacy-ref", default="HEAD^")
    parser.add_argument("--seed", type=int, default=20260731)
    parser.add_argument("--differential-cases", type=int, default=45)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--params", type=Path, default=DEFAULT_PARAMS)
    parser.add_argument("--start-lm", default="S002002")
    parser.add_argument("--goal-lm", default="S032035")
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--iterations", type=int, default=40)
    args = parser.parse_args()

    legacy_planner = load_legacy_planner(args.legacy_ref)
    matches = run_seeded_differential(
        legacy_planner,
        seed=args.seed,
        cases=max(1, args.differential_cases),
    )
    benchmark = run_map_benchmark(
        legacy_planner,
        map_path=args.map,
        params_path=args.params,
        start_lm=args.start_lm,
        goal_lm=args.goal_lm,
        warmups=max(0, args.warmups),
        iterations=max(1, args.iterations),
    )
    print(
        f"differential: {matches} exact matches "
        f"(seed={args.seed}, legacy={args.legacy_ref})"
    )
    print(
        "latency: "
        f"legacy median={benchmark['legacy']['median_ms']:.3f} ms, "
        f"p95={benchmark['legacy']['p95_ms']:.3f} ms; "
        "refactored median="
        f"{benchmark['refactored']['median_ms']:.3f} ms, "
        f"p95={benchmark['refactored']['p95_ms']:.3f} ms; "
        f"ratio={benchmark['median_ratio']:.4f}"
    )
    print(
        f"route: {benchmark['route']}, nodes={benchmark['nodes']}, "
        f"samples={benchmark['trajectory_samples']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
