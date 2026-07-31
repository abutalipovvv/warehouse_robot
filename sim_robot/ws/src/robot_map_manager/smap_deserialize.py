#!/usr/bin/env python3
"""Compatibility entry point for the canonical SMAP converter."""

from __future__ import annotations

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[4]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from fleet_manager.map_data.smap_deserialize import (
    deserialize_smap,
    main,
)


__all__ = ["deserialize_smap", "main"]


if __name__ == "__main__":
    main()
