"""Public namespace for low-level file I/O helpers."""

from pathlib import Path


__path__ = [str(Path(__file__).parents[1] / "io")]
