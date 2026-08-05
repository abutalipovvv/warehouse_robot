"""High-level Conflict-Based Search tree expansion."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass

from .cbs_low_level import LmCBSEnvironment
from .cbs_models import Constraints, HighLevelNode
from .cbs_setup import CbsPlanningLimits


@dataclass(frozen=True, slots=True)
class CbsTreeResult:
    """Terminal state of the high-level search."""

    solution_node: HighLevelNode | None
    reason: str
    conflicts_resolved: int
    expanded_nodes: int

    @property
    def found(self) -> bool:
        return self.solution_node is not None


class CbsHighLevelSearch:
    """Resolve conflicts by branching on one constrained robot at a time."""

    def __init__(
        self,
        environment: LmCBSEnvironment,
        limits: CbsPlanningLimits,
    ) -> None:
        self._environment = environment
        self._limits = limits

    def solve(self, start: HighLevelNode) -> CbsTreeResult:
        open_nodes = {start}
        closed_nodes: set[HighLevelNode] = set()
        conflicts_resolved = 0
        expanded_nodes = 0

        while open_nodes:
            limit_reason = self._limit_reason(expanded_nodes)
            if limit_reason:
                return CbsTreeResult(
                    solution_node=None,
                    reason=limit_reason,
                    conflicts_resolved=conflicts_resolved,
                    expanded_nodes=expanded_nodes,
                )

            current = min(open_nodes)
            open_nodes.remove(current)
            closed_nodes.add(current)
            expanded_nodes += 1
            self._environment.constraint_dict = current.constraint_dict

            conflict = self._environment.get_first_conflict(
                current.solution
            )
            if conflict is None:
                return CbsTreeResult(
                    solution_node=current,
                    reason="success",
                    conflicts_resolved=conflicts_resolved,
                    expanded_nodes=expanded_nodes,
                )

            conflicts_resolved += 1
            constraints_by_agent = (
                self._environment.create_constraints_from_conflict(
                    conflict
                )
            )
            for agent_name, constraint in constraints_by_agent.items():
                child = self._replan_agent(
                    current,
                    agent_name,
                    constraint,
                )
                if child is not None and child not in closed_nodes:
                    open_nodes.add(child)

        return CbsTreeResult(
            solution_node=None,
            reason="no_solution",
            conflicts_resolved=conflicts_resolved,
            expanded_nodes=expanded_nodes,
        )

    def _limit_reason(self, expanded_nodes: int) -> str:
        if self._limits.cancel_requested():
            return "planning_cancelled"
        if self._limits.timed_out():
            return self._limits.timeout_reason
        if expanded_nodes >= self._limits.high_level_max_nodes:
            return (
                "high_level_node_limit:"
                f"{self._limits.high_level_max_nodes}"
            )
        return ""

    def _replan_agent(
        self,
        parent: HighLevelNode,
        agent_name: str,
        constraint: Constraints,
    ) -> HighLevelNode | None:
        child = deepcopy(parent)
        child.constraint_dict[agent_name].add_constraint(constraint)
        self._environment.constraint_dict = child.constraint_dict
        self._environment.constraints = child.constraint_dict.setdefault(
            agent_name,
            Constraints(),
        )
        local_solution = self._environment.low_level_search(
            agent_name,
            self._limits.low_level_max_time,
        )
        if not local_solution:
            return None
        child.solution[agent_name] = local_solution
        child.cost = self._environment.compute_solution_cost(
            child.solution
        )
        return child
