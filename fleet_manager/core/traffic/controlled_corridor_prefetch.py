"""Composition root for controlled-corridor prefetch transactions."""

from __future__ import annotations

from .controlled_corridor_prefetch_gate import (
    ControlledCorridorPrefetchGateMixin,
)
from .controlled_corridor_prefetch_intent import (
    ControlledCorridorPrefetchIntentMixin,
)
from .controlled_corridor_prefetch_validation import (
    ControlledCorridorPrefetchValidationMixin,
)


class ControlledCorridorPrefetchMixin(
    ControlledCorridorPrefetchGateMixin,
    ControlledCorridorPrefetchIntentMixin,
    ControlledCorridorPrefetchValidationMixin,
):
    """Compose intent, gate/currentness and commit capabilities."""


__all__ = ["ControlledCorridorPrefetchMixin"]
