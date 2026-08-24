#!/usr/bin/env python3
"""Build persistent browser-ready 3D assets for existing smap folders."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from operator_app.core.map_scene_assets import ensure_map_scene_assets


def build_all(maps_root: Path) -> list[dict[str, object]]:
    results: list[dict[str, object]] = []
    for map_dir in sorted(maps_root.resolve().glob("*.smap")):
        if not map_dir.is_dir():
            continue
        manifest = ensure_map_scene_assets(map_dir)
        walls = manifest.get("walls")
        results.append(
            {
                "map": map_dir.stem.replace(".smap", ""),
                "walls": (
                    int(walls.get("count") or 0)
                    if isinstance(walls, dict)
                    else 0
                ),
                "digest": str(manifest.get("sourceDigest") or "")[:12],
            }
        )
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--maps-root",
        type=Path,
        default=PROJECT_ROOT / "fleet_manager" / "map_data" / "maps_out",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    for result in build_all(args.maps_root):
        print(
            f"{result['map']}: walls={result['walls']} "
            f"digest={result['digest']}"
        )


if __name__ == "__main__":
    main()
