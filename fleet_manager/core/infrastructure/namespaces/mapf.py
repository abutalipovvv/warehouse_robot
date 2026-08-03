"""Compatibility namespace backed by the physically grouped MAPF sources.

The public module names intentionally remain flat, for example
``fleet_manager.core.mapf.cbs_models``.  Each directory below contributes
modules to that one namespace; the category directories are not canonical
import paths.
"""

from pathlib import Path


_SOURCE_ROOT = Path(__file__).parents[2] / "mapf"
__path__ = [
    str(_SOURCE_ROOT / category)
    for category in (
        "common",
        "graph",
        "cbs",
        "sipp",
        "rolling",
        "fleet",
    )
]
