"""Compatibility namespace over maps and navigation implementations."""

from pathlib import Path


_core_root = Path(__file__).parents[2] / "mapping"
__path__ = [
    str(_core_root / "maps"),
    str(_core_root / "navigation"),
]
