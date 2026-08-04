"""Command-line interface for the standalone Fleet Manager process."""

from __future__ import annotations

import argparse
import sys
from math import isfinite
from pathlib import Path

from fleet_manager.runtime.application.runner import (
    ApplicationOptions,
    FleetManagerApplication,
)
from fleet_manager.core.mapping.navigation.params import DEFAULT_PARAMS_PATH


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="fleet-manager",
        description="Run Fleet Manager without Operator App.",
    )
    parser.add_argument(
        "--mode",
        choices=("simulation", "robots"),
        default="simulation",
        help="robot execution runtime (default: simulation)",
    )
    parser.add_argument(
        "--map",
        dest="map_value",
        required=True,
        help="map bundle directory or a bundle name from map_data/maps_out",
    )
    parser.add_argument(
        "--params",
        type=Path,
        default=DEFAULT_PARAMS_PATH,
        help=f"Fleet Manager parameters (default: {DEFAULT_PARAMS_PATH})",
    )
    parser.add_argument(
        "--robot",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help=(
            "add a robot; VALUE is an LM in simulation mode or a gRPC "
            "endpoint in robots mode (repeatable)"
        ),
    )
    parser.add_argument(
        "--order",
        action="append",
        default=[],
        metavar="NAME=LM",
        help="queue an order for a configured robot (repeatable)",
    )
    parser.add_argument(
        "--tick-interval",
        type=_positive_float,
        default=0.1,
        metavar="SECONDS",
        help="runtime step interval (default: 0.1)",
    )
    parser.add_argument(
        "--duration",
        type=_non_negative_float,
        default=None,
        metavar="SECONDS",
        help="stop automatically after this duration",
    )
    parser.add_argument(
        "--state-interval",
        type=_non_negative_float,
        default=0.0,
        metavar="SECONDS",
        help="print state JSON at this interval; 0 disables output",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    options = ApplicationOptions(
        mode=args.mode,
        map_value=args.map_value,
        params_path=args.params.expanduser().resolve(),
        robots=tuple(args.robot),
        orders=tuple(args.order),
        tick_interval=args.tick_interval,
        duration=args.duration,
        state_interval=args.state_interval,
    )
    application = FleetManagerApplication(options)
    try:
        application.run()
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"fleet-manager: error: {exc}", file=sys.stderr)
        return 2
    return 0


def _positive_float(raw_value: str) -> float:
    value = float(raw_value)
    if not isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError(
            "value must be a finite number greater than zero"
        )
    return value


def _non_negative_float(raw_value: str) -> float:
    value = float(raw_value)
    if not isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError(
            "value must be a finite non-negative number"
        )
    return value
