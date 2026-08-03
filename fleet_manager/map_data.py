"""Compatibility namespace for map-format implementations in ``core``."""

from pathlib import Path


__path__ = [
    str(Path(__file__).with_name("core") / "mapping" / "formats"),
]
