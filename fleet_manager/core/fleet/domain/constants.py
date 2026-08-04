"""Stable fleet-domain constants shared by every runtime."""

TERMINAL_ORDER_STATUSES = frozenset({"COMPLETED", "FAILED", "CANCELED"})

ORDER_SEQUENCE_KEYS = (
    "targets",
    "targetLms",
    "goals",
    "orders",
    "queue",
    "blocks",
)
ORDER_ID_KEYS = ("id", "orderId", "taskId")
ORDER_TARGET_KEYS = ("targetLm", "goalLm", "location", "target", "LM")

FLEET_CONTROL_OWNER_ID = "fleet-manager"
FLEET_CONTROL_OWNER_NAME = "Fleet Manager"
EXTERNAL_CONTROL_PAUSE_PREFIX = "external control active:"
