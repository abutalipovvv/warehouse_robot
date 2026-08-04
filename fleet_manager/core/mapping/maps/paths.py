"""Canonical filesystem locations for fleet map data."""

from pathlib import Path


FLEET_MANAGER_ROOT = Path(__file__).resolve().parents[3]
DATA_ROOT = FLEET_MANAGER_ROOT / "map_data"
MAPS_ROOT = DATA_ROOT / "maps"
MAPS_OUT_ROOT = DATA_ROOT / "maps_out"
