#!/usr/bin/env python3
"""Convert a RoboShop/RDS ``.smap`` JSON document into a map bundle.

The public function stays intentionally small. Parsing, graph reconstruction,
raster math and durable output are separate classes in ``smap_bundle`` and
``smap_raster``.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

if __package__ in {None, ""}:
    # Keep direct execution from the formats directory working.
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

from fleet_manager.map_data.smap_bundle import (
    SmapBundleWriter,
    SmapDocumentParser,
)
from fleet_manager.map_data.smap_raster import FREE_CELL, OCCUPIED_CELL


# Backward-compatible constant names used by older conversion scripts.
FREE = FREE_CELL
OCC = OCCUPIED_CELL


def deserialize_smap(smap_path: Path, out_dir: Path) -> None:
    """Deserialize ``smap_path`` and atomically write each bundle artifact."""

    source = Path(smap_path)
    document: Any = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("SMAP root must be a JSON object")

    bundle = SmapDocumentParser(
        document,
        fallback_name=source.stem,
    ).parse()
    SmapBundleWriter().write(bundle, Path(out_dir))


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Deserialize .smap JSON into PGM, ROS YAML, landmarks, "
            "graph primitives, edge lengths and a summary"
        )
    )
    parser.add_argument("smap", type=Path, help="Path to the .smap file")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output directory",
    )
    arguments = parser.parse_args()

    output_dir = arguments.out or Path(
        f"smap_deserialized_{arguments.smap.stem}"
    )
    deserialize_smap(arguments.smap, output_dir)
    print(f"Done.\nOutput dir: {output_dir.resolve()}")


__all__ = ["FREE", "OCC", "deserialize_smap", "main"]


if __name__ == "__main__":
    main()
