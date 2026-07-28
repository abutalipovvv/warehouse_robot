#!/usr/bin/env python3
"""Build a map bundle containing the first N ordered landmarks and their edges."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any

import yaml


def read_yaml(path: Path) -> Any:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def write_yaml(path: Path, payload: Any) -> None:
    path.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def build(source: Path, output: Path, *, limit: int, map_name: str) -> dict[str, Any]:
    source = source.resolve()
    output.mkdir(parents=True, exist_ok=True)
    lms_payload = read_yaml(source / "LMs.yaml")
    raw_lms = lms_payload.get("LMs", [])
    selected_lms = raw_lms[:limit]
    selected_names = {
        str(item.get("name") or "")
        for item in selected_lms
        if isinstance(item, dict)
    }
    lms_payload["mapName"] = map_name
    lms_payload["LMs"] = selected_lms

    edges = read_yaml(source / "graph_edges_lengths.yaml")
    selected_edges = [
        edge
        for edge in edges
        if isinstance(edge, dict)
        and str(edge.get("from") or "") in selected_names
        and str(edge.get("to") or "") in selected_names
    ]
    graphs = read_yaml(source / "graphs.yaml")
    graphs["mapName"] = map_name
    graphs["primitives"] = [
        primitive
        for primitive in graphs.get("primitives", [])
        if isinstance(primitive, dict)
        and str(primitive.get("start_name") or "") in selected_names
        and str(primitive.get("end_name") or "") in selected_names
    ]

    ros_candidates = [
        path
        for path in source.glob("*.yaml")
        if path.name
        not in {"LMs.yaml", "graphs.yaml", "graph_edges_lengths.yaml", "traffic_zones.yaml"}
    ]
    if not ros_candidates:
        raise RuntimeError(f"ROS map YAML is missing from {source}")
    ros_payload = read_yaml(ros_candidates[0])
    image_name = str(ros_payload["image"])
    shutil.copy2(source / image_name, output / image_name)
    write_yaml(output / f"{map_name}.yaml", ros_payload)
    write_yaml(output / "LMs.yaml", lms_payload)
    write_yaml(output / "graph_edges_lengths.yaml", selected_edges)
    write_yaml(output / "graphs.yaml", graphs)
    traffic_zones = source / "traffic_zones.yaml"
    if traffic_zones.exists():
        shutil.copy2(traffic_zones, output / traffic_zones.name)
    return {
        "mapName": map_name,
        "output": str(output),
        "landmarks": len(selected_lms),
        "directedEdges": len(selected_edges),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--limit", type=int, default=360)
    parser.add_argument("--map-name", default="benchmark_open_kiva_rds360")
    args = parser.parse_args()
    print(
        json.dumps(
            build(
                args.source,
                args.output,
                limit=args.limit,
                map_name=args.map_name,
            ),
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
