from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import shutil
import subprocess
import sys
import webbrowser

from .models import DemoPayload
from .web_builder import RouteDemoSiteBuilder
from ..route_core import Landmark, LmRoutePlanner, WarehouseMapLoader, load_route_params


@dataclass(frozen=True)
class RouteDemoOptions:
    map_dir: Path
    start: str | None = None
    goal: str | None = None
    output: Path | None = None
    params: Path | None = None
    open_browser: bool = False


class RouteDemoApplication:
    def __init__(self, site_builder: RouteDemoSiteBuilder | None = None) -> None:
        self.site_builder = site_builder or RouteDemoSiteBuilder()

    def run(self, options: RouteDemoOptions) -> Path:
        loaded_map = WarehouseMapLoader(options.map_dir).load()
        default_start, default_goal = self._pick_defaults(
            landmarks=loaded_map.landmarks,
            requested_start=options.start,
            requested_goal=options.goal,
        )
        params = load_route_params(options.params, create=True)

        route_planner = LmRoutePlanner(loaded_map.landmarks, loaded_map.edges, params=params)
        try:
            route_planner.find_route(default_start, default_goal)
        except ValueError:
            if options.goal is not None:
                raise
            default_goal = self._first_reachable_goal(route_planner, default_start) or default_start

        payload = DemoPayload(
            map_metadata=loaded_map.map_metadata,
            map_name=loaded_map.map_dir.stem.replace(".smap", ""),
            landmarks=[loaded_map.landmarks[name] for name in sorted(loaded_map.landmarks)],
            edges=loaded_map.edges,
            route_catalog=route_planner.build_route_catalog(),
            default_start=default_start,
            default_goal=default_goal,
            params=params,
        )
        index_path = self.site_builder.build(
            payload=payload,
            map_dir=loaded_map.map_dir,
            output=options.output,
        )

        if options.open_browser:
            self._open_in_browser(index_path)
        return index_path

    def _pick_defaults(
        self,
        landmarks: dict[str, Landmark],
        requested_start: str | None,
        requested_goal: str | None,
    ) -> tuple[str, str]:
        names = sorted(landmarks)
        if not names:
            raise ValueError("No LMs were found.")

        start = requested_start or names[0]
        goal = requested_goal or names[-1]

        if start not in landmarks:
            raise ValueError(f"Default start LM does not exist: {start}")
        if goal not in landmarks:
            raise ValueError(f"Default goal LM does not exist: {goal}")

        return start, goal

    def _first_reachable_goal(self, planner: LmRoutePlanner, start: str) -> str | None:
        for goal in sorted(planner.landmarks):
            if goal == start:
                continue
            try:
                planner.find_route(start, goal)
            except ValueError:
                continue
            return goal
        return None

    def _open_in_browser(self, index_path: Path) -> None:
        resolved = index_path.resolve()
        if sys.platform.startswith("linux"):
            opener = shutil.which("xdg-open")
            if opener:
                subprocess.run([opener, str(resolved)], check=False)
                return
        elif sys.platform == "darwin":
            opener = shutil.which("open")
            if opener:
                subprocess.run([opener, str(resolved)], check=False)
                return
        elif os.name == "nt":
            os.startfile(str(resolved))
            return

        webbrowser.open(resolved.as_uri())
