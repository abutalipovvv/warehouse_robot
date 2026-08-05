"""Standalone Fleet Manager entry point."""

from __future__ import annotations

import sys
from pathlib import Path


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fleet_manager.runtime.application.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
