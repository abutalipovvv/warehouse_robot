"""Import namespace for implementations grouped under ``fleet_manager/core``.

The physical tree has seven broad areas. Historical flat module names are
resolved by small adapters in ``infrastructure/compatibility``; specialized
namespaces live in ``infrastructure/namespaces``.
"""

from pathlib import Path


_SOURCE_ROOT = Path(__file__).with_suffix("")
__path__ = [
    str(_SOURCE_ROOT / "infrastructure" / "compatibility"),
    str(_SOURCE_ROOT / "infrastructure" / "namespaces"),
]
