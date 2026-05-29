from __future__ import annotations

import heapq
import math

from .models import GraphEdge, Landmark, PlannedRoute


class AStarRouter:
    def __init__(self, landmarks: dict[str, Landmark], edges: list[GraphEdge]) -> None:
        self.landmarks = landmarks
        self.edges = edges
        self.adjacency = self._build_adjacency()

    def find_route(self, start: str, goal: str) -> PlannedRoute:
        if start not in self.landmarks:
            raise ValueError(f"Unknown start LM: {start}")
        if goal not in self.landmarks:
            raise ValueError(f"Unknown goal LM: {goal}")

        open_heap: list[tuple[float, str]] = [(0.0, start)]
        came_from: dict[str, str] = {}
        g_score: dict[str, float] = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                nodes = [current]
                while current in came_from:
                    current = came_from[current]
                    nodes.append(current)
                nodes.reverse()
                return PlannedRoute(nodes=nodes, length=g_score[goal])

            for neighbor, cost in self.adjacency.get(current, []):
                tentative = g_score[current] + cost
                if tentative >= g_score.get(neighbor, math.inf):
                    continue

                came_from[neighbor] = current
                g_score[neighbor] = tentative
                heuristic = self._world_distance(self.landmarks[neighbor], self.landmarks[goal])
                heapq.heappush(open_heap, (tentative + heuristic, neighbor))

        raise ValueError(f"No route found from {start} to {goal}")

    def _build_adjacency(self) -> dict[str, list[tuple[str, float]]]:
        adjacency: dict[str, list[tuple[str, float]]] = {}
        for edge in self.edges:
            adjacency.setdefault(edge.from_name, []).append((edge.to_name, edge.length))
        return adjacency

    def _world_distance(self, first: Landmark, second: Landmark) -> float:
        return math.hypot(second.x - first.x, second.y - first.y)
