"""Compatibility namespace backed by task-orchestration sources."""

from pathlib import Path


__path__ = [str(Path(__file__).parents[2] / "tasks")]
