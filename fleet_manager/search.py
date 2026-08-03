"""Backward-compatible access to graph search in ``core.math.search``."""

from pathlib import Path
import sys

from fleet_manager.core.math.search import astar as _astar
from fleet_manager.core.math.search import problem as _problem
from fleet_manager.core.math.search import result as _result
from fleet_manager.core.math.search.astar import AStarSolver
from fleet_manager.core.math.search.problem import SearchProblem
from fleet_manager.core.math.search.result import SearchResult

# Keep historical deep imports working without a second implementation.
__path__ = [
    str(Path(__file__).with_name("core") / "algorithms" / "math" / "search")
]
sys.modules[f"{__name__}.astar"] = _astar
sys.modules[f"{__name__}.problem"] = _problem
sys.modules[f"{__name__}.result"] = _result

__all__ = ["AStarSolver", "SearchProblem", "SearchResult"]
