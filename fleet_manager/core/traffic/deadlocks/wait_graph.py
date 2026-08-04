"""Typed wait-for graph primitives used by runtime deadlock arbitration."""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import dataclass


WaitCycle = tuple[str, ...]


@dataclass(frozen=True, slots=True)
class WaitChain:
    """One dependency walk and the first node outside (or repeated in) it."""

    members: tuple[str, ...]
    terminal: str


class WaitForGraph:
    """Read-only view of one runtime wait-dependency snapshot."""

    __slots__ = ("_dependencies",)

    def __init__(self, dependencies: Mapping[str, str]) -> None:
        self._dependencies = dependencies

    def cycles(self) -> Iterator[WaitCycle]:
        """Yield each directed cycle once in deterministic traversal order."""
        handled: set[str] = set()
        for start_name in sorted(self._dependencies):
            chain: list[str] = []
            positions: dict[str, int] = {}
            current = start_name
            while (
                current in self._dependencies
                and current not in handled
            ):
                if current in positions:
                    cycle = tuple(chain[positions[current]:])
                    handled.update(cycle)
                    yield cycle
                    break
                positions[current] = len(chain)
                chain.append(current)
                current = self._dependencies[current]
            handled.update(chain)

    def walk(self, start_name: str) -> WaitChain:
        """Follow dependencies until the graph ends or a node repeats."""
        chain: list[str] = []
        current = start_name
        while (
            current in self._dependencies
            and current not in chain
        ):
            chain.append(current)
            current = self._dependencies.get(current, "")
        return WaitChain(tuple(chain), current)
