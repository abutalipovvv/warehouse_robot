from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from route_core.models import GraphEdge, Landmark, MapMetadata


@dataclass(frozen=True)
class DemoPayload:
    map_metadata: MapMetadata
    landmarks: Sequence[Landmark]
    edges: Sequence[GraphEdge]
    route_catalog: dict[str, dict[str, object]]
    default_start: str
    default_goal: str

    def to_dict(self) -> dict[str, object]:
        return {
            "mapName": self.map_metadata.map_name,
            "map": self.map_metadata.to_dict(),
            "lms": [landmark.to_dict() for landmark in self.landmarks],
            "edges": [edge.to_dict() for edge in self.edges],
            "routes": dict(self.route_catalog),
            "defaultStart": self.default_start,
            "defaultGoal": self.default_goal,
        }
