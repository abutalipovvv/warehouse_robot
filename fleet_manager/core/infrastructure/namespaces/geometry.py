"""Compatibility namespace for collision geometry in ``core.safety``."""

from pathlib import Path


_CORE_ROOT = Path(__file__).parents[2]
__path__ = [str(_CORE_ROOT / "fleet" / "safety")]
