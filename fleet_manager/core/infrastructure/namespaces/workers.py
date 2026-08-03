"""Public namespace for background workers."""

from pathlib import Path


__path__ = [str(Path(__file__).parents[1] / "workers")]
