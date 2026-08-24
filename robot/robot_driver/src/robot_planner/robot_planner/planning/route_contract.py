from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


SUPPORTED_PROTOCOL = "lm_route"
SUPPORTED_PROTOCOL_VERSION = 2
SUPPORTED_REPLACE_MODES = frozenset({"immediate"})
SUPPORTED_MOTION_DIRECTIONS = frozenset(
    {"forward", "backward", "not_specified"}
)


def _string(payload: dict[str, Any], *keys: str) -> str:
    for key in keys:
        value = str(payload.get(key) or "").strip()
        if value:
            return value
    return ""


@dataclass(frozen=True, slots=True)
class MapfTimedSegment:
    kind: str
    from_lm: str
    to_lm: str
    node: str
    motion_direction: str
    not_before_sec: float
    planned_arrival_sec: float

    @property
    def edge_id(self) -> str:
        return f"{self.from_lm}->{self.to_lm}"

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> MapfTimedSegment:
        kind = str(payload.get("kind") or "move").strip().lower()
        if kind not in {"move", "wait", "rotate"}:
            raise ValueError(f"unsupported timed segment kind: {kind}")
        from_lm = _string(payload, "from", "fromLm", "from_lm")
        to_lm = _string(payload, "to", "toLm", "to_lm")
        node = _string(payload, "node", "lm")
        if kind == "move" and (not from_lm or not to_lm):
            raise ValueError("move timed segment requires from and to LM")
        if kind != "move" and not node:
            raise ValueError(f"{kind} timed segment requires node")

        direction = str(
            payload.get("motionDirection")
            or payload.get("motion_direction")
            or "not_specified"
        ).strip().lower().replace("-", "_")
        if direction not in SUPPORTED_MOTION_DIRECTIONS:
            raise ValueError(f"unsupported motion direction: {direction}")
        try:
            not_before = float(
                payload.get("notBeforeSec")
                or payload.get("not_before_sec")
                or 0.0
            )
            arrival = float(
                payload.get("plannedArrivalSec")
                or payload.get("planned_arrival_sec")
                or not_before
            )
        except (TypeError, ValueError) as exc:
            raise ValueError("timed segment timestamps must be numeric") from exc
        if not all(math.isfinite(value) for value in (not_before, arrival)):
            raise ValueError("timed segment timestamps must be finite")
        if not_before < 0.0 or arrival < not_before:
            raise ValueError("timed segment arrival must not precede its start")
        return cls(
            kind=kind,
            from_lm=from_lm,
            to_lm=to_lm,
            node=node,
            motion_direction=direction,
            not_before_sec=not_before,
            planned_arrival_sec=arrival,
        )


@dataclass(frozen=True, slots=True)
class MapfRoutePlan:
    """Validated coordinator command; geometry and control remain robot-local."""

    route_id: str
    revision: int
    order_id: str
    start_lm: str
    goal_lm: str
    final_goal_lm: str
    nodes: tuple[str, ...]
    full_nodes: tuple[str, ...]
    replace_mode: str
    dispatch_epoch_sec: float
    timed_segments: tuple[MapfTimedSegment, ...]
    source_payload: dict[str, Any]

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> MapfRoutePlan:
        protocol = str(
            payload.get("protocol") or payload.get("routeProtocol") or "lm_route"
        ).strip().lower().replace("-", "_")
        if protocol != SUPPORTED_PROTOCOL:
            raise ValueError(f"unsupported route protocol: {protocol}")
        try:
            version = int(
                payload.get("protocolVersion")
                or payload.get("protocol_version")
                or SUPPORTED_PROTOCOL_VERSION
            )
        except (TypeError, ValueError) as exc:
            raise ValueError("route protocolVersion must be an integer") from exc
        if version != SUPPORTED_PROTOCOL_VERSION:
            raise ValueError(
                f"unsupported route protocolVersion: {version}; "
                f"expected {SUPPORTED_PROTOCOL_VERSION}"
            )
        try:
            revision = int(payload.get("revision", 0) or 0)
        except (TypeError, ValueError) as exc:
            raise ValueError("route revision must be an integer") from exc
        if revision < 0:
            raise ValueError("route revision must not be negative")

        replace_mode = str(
            payload.get("replaceMode")
            or payload.get("replace_mode")
            or "immediate"
        ).strip().lower()
        if replace_mode not in SUPPORTED_REPLACE_MODES:
            raise ValueError(f"unsupported route replaceMode: {replace_mode}")

        nodes = cls._nodes(payload.get("nodes") or payload.get("routeNodes"))
        start_lm = _string(payload, "startLm", "start_lm")
        goal_lm = _string(payload, "goalLm", "goal_lm", "targetLm")
        if nodes:
            if start_lm and start_lm != nodes[0]:
                raise ValueError(
                    f"route startLm {start_lm} does not match first node {nodes[0]}"
                )
            if goal_lm and goal_lm != nodes[-1]:
                raise ValueError(
                    f"route goalLm {goal_lm} does not match last node {nodes[-1]}"
                )
            start_lm = start_lm or nodes[0]
            goal_lm = goal_lm or nodes[-1]
        if not goal_lm:
            raise ValueError("route goalLm is required")

        full_nodes = cls._nodes(
            payload.get("fullNodes") or payload.get("full_nodes")
        )
        if not full_nodes:
            full_nodes = nodes
        final_goal_lm = _string(
            payload,
            "finalGoalLm",
            "final_goal_lm",
        ) or (full_nodes[-1] if full_nodes else goal_lm)

        try:
            dispatch_epoch = float(
                payload.get("dispatchEpochSec")
                or payload.get("dispatch_epoch_sec")
                or 0.0
            )
        except (TypeError, ValueError) as exc:
            raise ValueError("route dispatchEpochSec must be numeric") from exc
        if not math.isfinite(dispatch_epoch) or dispatch_epoch < 0.0:
            raise ValueError("route dispatchEpochSec must be finite and non-negative")

        raw_segments = payload.get("timedSegments") or payload.get(
            "timed_segments"
        )
        if raw_segments is None:
            raw_segments = []
        if not isinstance(raw_segments, list):
            raise ValueError("route timedSegments must be a list")
        segments = tuple(
            MapfTimedSegment.from_payload(item)
            for item in raw_segments
            if isinstance(item, dict)
        )
        if len(segments) != len(raw_segments):
            raise ValueError("every timedSegments item must be an object")

        return cls(
            route_id=_string(payload, "routeId", "route_id"),
            revision=revision,
            order_id=_string(payload, "orderId", "order_id"),
            start_lm=start_lm,
            goal_lm=goal_lm,
            final_goal_lm=final_goal_lm,
            nodes=nodes,
            full_nodes=full_nodes,
            replace_mode=replace_mode,
            dispatch_epoch_sec=dispatch_epoch,
            timed_segments=segments,
            source_payload=dict(payload),
        )

    @staticmethod
    def _nodes(raw_nodes: Any) -> tuple[str, ...]:
        if raw_nodes is None:
            return ()
        if not isinstance(raw_nodes, list):
            raise ValueError("route nodes must be a list")
        nodes: list[str] = []
        for item in raw_nodes:
            node = str(item).strip()
            if not node or node.startswith("CURRENT_"):
                continue
            if not nodes or nodes[-1] != node:
                nodes.append(node)
        return tuple(nodes)


__all__ = [
    "MapfRoutePlan",
    "MapfTimedSegment",
    "SUPPORTED_PROTOCOL_VERSION",
]
