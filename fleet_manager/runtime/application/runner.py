"""Standalone Fleet Manager process lifecycle."""

from __future__ import annotations

import json
import signal
import sys
from dataclasses import dataclass
from pathlib import Path
from threading import Event, RLock
from time import monotonic
from types import FrameType
from typing import Any

from fleet_manager.core.mapping.maps.paths import MAPS_OUT_ROOT
from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader
from fleet_manager.core.mapping.navigation.params import load_route_params
from fleet_manager.runtime.grpc.manager import FleetManagerROS
from fleet_manager.runtime.loop import RuntimeLoop, RuntimeLoopFailure
from fleet_manager.runtime.simulation.manager import FleetManagerSim


@dataclass(frozen=True, slots=True)
class ApplicationOptions:
    """Inputs needed to construct and run one Fleet Manager process."""

    mode: str
    map_value: str
    params_path: Path
    robots: tuple[str, ...] = ()
    orders: tuple[str, ...] = ()
    tick_interval: float = 0.1
    duration: float | None = None
    state_interval: float = 0.0


class FleetManagerApplication:
    """Own one manager and its background runtime loop."""

    def __init__(self, options: ApplicationOptions) -> None:
        self.options = options
        self.manager: Any | None = None
        self.runtime_loop: RuntimeLoop | None = None
        self.map_dir: Path | None = None
        self._manager_lock = RLock()
        self._stop_requested = Event()

    def run(self) -> None:
        """Run until a signal arrives or the configured duration expires."""

        previous_handlers = self._install_signal_handlers()
        try:
            self.start()
            self._wait_until_stopped()
        finally:
            try:
                self.close()
            finally:
                self._restore_signal_handlers(previous_handlers)

    def start(self) -> None:
        """Load inputs, configure robots and start the owned runtime loop."""

        if self.manager is not None or self.runtime_loop is not None:
            raise RuntimeError("Fleet Manager application is already started")
        if self.options.mode not in {"simulation", "robots"}:
            raise ValueError("mode must be simulation or robots")
        self._stop_requested.clear()

        map_dir = resolve_map_dir(self.options.map_value)
        loaded_map = WarehouseMapLoader(map_dir).load()
        params = load_route_params(self.options.params_path, strict=True)
        manager_type = (
            FleetManagerSim
            if self.options.mode == "simulation"
            else FleetManagerROS
        )
        manager = manager_type(
            loaded_map.landmarks,
            loaded_map.edges,
            params=params,
            map_dir=loaded_map.map_dir,
            map_metadata=loaded_map.map_metadata,
        )

        runtime_loop: RuntimeLoop | None = None
        try:
            self._add_robots(manager)
            self._add_orders(manager)
            runtime_loop = RuntimeLoop(
                self._advance_runtime,
                interval_seconds=self.options.tick_interval,
                name=f"fleet-manager-{self.options.mode}",
                on_error=self._report_runtime_failure,
            )
            manager.set_runtime_command_executor(runtime_loop.execute)
            self.manager = manager
            self.map_dir = loaded_map.map_dir
            self.runtime_loop = runtime_loop
            runtime_loop.start()
        except BaseException:
            try:
                if runtime_loop is not None:
                    runtime_loop.close()
            finally:
                manager.close()
            self.manager = None
            self.map_dir = None
            self.runtime_loop = None
            raise

        print(
            f"Fleet Manager started: mode={self.options.mode} "
            f"map={loaded_map.map_dir}",
            file=sys.stderr,
            flush=True,
        )

    def request_stop(self) -> None:
        """Ask the main wait loop to finish at the next safe boundary."""

        self._stop_requested.set()

    def close(self) -> None:
        """Stop runtime mutation before closing the manager and its worker."""

        runtime_loop = self.runtime_loop
        manager = self.manager
        self.runtime_loop = None
        self.manager = None

        try:
            if runtime_loop is not None:
                runtime_loop.close()
        finally:
            if manager is not None:
                manager.close()

    def snapshot(self) -> dict[str, Any]:
        """Return state without advancing the simulation a second time."""

        with self._manager_lock:
            if self.manager is None:
                raise RuntimeError("Fleet Manager application is not started")
            return self.manager.snapshot()

    def _add_robots(self, manager: Any) -> None:
        for raw_spec in self.options.robots:
            name, value = parse_assignment(raw_spec, option="--robot")
            if self.options.mode == "simulation":
                payload = {
                    "name": name,
                    "mode": "simulated",
                    "currentLm": value,
                }
            else:
                payload = {
                    "name": name,
                    "mode": "remote",
                    "baseUrl": value,
                }
            manager.add_robot(payload)

    def _add_orders(self, manager: Any) -> None:
        for index, raw_spec in enumerate(self.options.orders, start=1):
            robot_name, target_lm = parse_assignment(
                raw_spec,
                option="--order",
            )
            manager.set_order(
                {
                    "id": f"cli-order-{index}",
                    "vehicle": robot_name,
                    "targetLm": target_lm,
                }
            )

    def _advance_runtime(self) -> None:
        with self._manager_lock:
            if self.manager is not None:
                self.manager.advance_runtime()

    def _wait_until_stopped(self) -> None:
        started_at = monotonic()
        next_state_at = started_at

        while not self._stop_requested.is_set():
            now = monotonic()
            duration = self.options.duration
            if duration is not None and now - started_at >= duration:
                return

            if self.options.state_interval > 0.0 and now >= next_state_at:
                self._print_snapshot()
                next_state_at = now + self.options.state_interval

            wait_seconds = 0.2
            if duration is not None:
                wait_seconds = min(
                    wait_seconds,
                    max(0.0, duration - (now - started_at)),
                )
            if self.options.state_interval > 0.0:
                wait_seconds = min(
                    wait_seconds,
                    max(0.0, next_state_at - now),
                )
            self._stop_requested.wait(wait_seconds)

    def _print_snapshot(self) -> None:
        print(
            json.dumps(self.snapshot(), ensure_ascii=False, separators=(",", ":")),
            flush=True,
        )

    @staticmethod
    def _report_runtime_failure(failure: RuntimeLoopFailure) -> None:
        print(
            "Fleet Manager runtime step failed: "
            f"{failure.exception_type}: {failure.message}",
            file=sys.stderr,
            flush=True,
        )

    def _install_signal_handlers(
        self,
    ) -> dict[signal.Signals, Any]:
        previous: dict[signal.Signals, Any] = {}
        for signal_number in (signal.SIGINT, signal.SIGTERM):
            try:
                previous[signal_number] = signal.getsignal(signal_number)
                signal.signal(signal_number, self._handle_signal)
            except ValueError:
                # Only the main Python thread may own process signal handlers.
                continue
        return previous

    @staticmethod
    def _restore_signal_handlers(
        previous_handlers: dict[signal.Signals, Any],
    ) -> None:
        for signal_number, handler in previous_handlers.items():
            signal.signal(signal_number, handler)

    def _handle_signal(
        self,
        _signal_number: int,
        _frame: FrameType | None,
    ) -> None:
        self.request_stop()


def parse_assignment(raw_value: str, *, option: str) -> tuple[str, str]:
    """Parse one ``NAME=VALUE`` command-line assignment."""

    name, separator, value = str(raw_value).partition("=")
    name = name.strip()
    value = value.strip()
    if not separator or not name or not value:
        raise ValueError(f"{option} must use NAME=VALUE")
    return name, value


def resolve_map_dir(raw_value: str) -> Path:
    """Resolve an explicit directory or a bundle name under maps_out."""

    clean_value = str(raw_value).strip()
    if not clean_value:
        raise ValueError("--map must not be empty")
    requested = Path(clean_value).expanduser()
    candidates: list[Path] = []
    if requested.is_absolute():
        candidates.append(requested)
    else:
        candidates.extend((Path.cwd() / requested, MAPS_OUT_ROOT / requested))

    if not requested.name.endswith(".smap"):
        candidates.append(MAPS_OUT_ROOT / f"{requested.name}.smap")

    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.is_dir():
            return resolved

    raise FileNotFoundError(f"fleet map directory not found: {raw_value}")
