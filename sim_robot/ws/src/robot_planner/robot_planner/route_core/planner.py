from __future__ import annotations

from warehouse_maps.planner import LmRoutePlanner as _LmRoutePlanner

from .params import load_route_params


class LmRoutePlanner(_LmRoutePlanner):
    def __init__(
        self,
        landmarks,
        edges,
        params: dict[str, object] | None = None,
    ) -> None:
        super().__init__(landmarks, edges, params=params or load_route_params())


__all__ = ["LmRoutePlanner"]
