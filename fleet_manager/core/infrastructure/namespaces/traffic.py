"""Compatibility namespace backed by grouped traffic-control sources.

Public imports intentionally stay flat, for example
``fleet_manager.core.traffic.corridor_scheduler``.  The implementation files
are grouped by responsibility without loading a class under two module names.
"""

from pathlib import Path


_SOURCE_ROOT = Path(__file__).parents[2] / "traffic"
__path__ = [
    str(_SOURCE_ROOT / relative_path)
    for relative_path in (
        "corridors",
        "corridors/admission",
        "corridors/prefetch",
        "corridors/scheduling",
        "deadlocks/arbitration",
        "deadlocks/recovery/cycles",
        "deadlocks/recovery/evacuation",
        "planning",
        "routing",
        "runtime",
    )
]
