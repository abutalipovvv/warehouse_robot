#!/usr/bin/env python3
"""
Build a route demo site for LM graph visualization and A* routing.

Example:
  python build_route_demo.py --map-dir maps_out/22.05.26_smap.smap --start LM91 --goal LM323
"""

from __future__ import annotations

import argparse
from pathlib import Path

from route_demo import RouteDemoApplication, RouteDemoOptions


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build an interactive route demo for a warehouse map."
    )
    parser.add_argument(
        "--map-dir",
        required=True,
        type=Path,
        help="Directory with map yaml/pgm, LMs.yaml and graph_edges_lengths.yaml.",
    )
    parser.add_argument(
        "--start",
        default=None,
        help="Default start LM shown when the demo opens.",
    )
    parser.add_argument(
        "--goal",
        default=None,
        help="Default target LM shown when the demo opens.",
    )
    parser.add_argument(
        "--output",
        default=None,
        type=Path,
        help="Output directory. Default: <map-dir>/route_demo_web",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="Open the generated demo in the default browser after build.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_path = RouteDemoApplication().run(
        RouteDemoOptions(
            map_dir=args.map_dir,
            start=args.start,
            goal=args.goal,
            output=args.output,
            open_browser=args.open,
        )
    )
    print(f"Built route demo: {output_path}")
    if args.open:
        print(f"Opened in browser: {output_path.resolve().as_uri()}")
    else:
        print(f"Open manually: xdg-open {output_path}")
        print("Or rerun with --open")


if __name__ == "__main__":
    main()
