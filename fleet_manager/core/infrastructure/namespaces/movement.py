"""Public namespace for robot movement lifecycle code."""

from pathlib import Path


__path__ = [str(Path(__file__).parents[2] / "fleet" / "movement")]
