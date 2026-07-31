"""Ports through which fleet policy talks to robot execution runtimes."""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class RobotGateway(Protocol):
    transport: str


@runtime_checkable
class RemoteRobotGateway(RobotGateway, Protocol):
    def identity(self, endpoint: str) -> dict[str, Any]: ...

    def status(self, endpoint: str) -> dict[str, Any]: ...

    def execute_route(
        self,
        endpoint: str,
        payload: dict[str, Any],
    ) -> dict[str, Any]: ...

    def cancel_route(
        self,
        endpoint: str,
        *,
        owner_id: str = "",
    ) -> dict[str, Any]: ...

    def stop(
        self,
        endpoint: str,
        *,
        owner_id: str = "",
    ) -> dict[str, Any]: ...

    def acquire_control(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str = "",
        force: bool = False,
        lease_ms: int = 0,
    ) -> dict[str, Any]: ...

    def release_control(
        self,
        endpoint: str,
        *,
        owner_id: str,
        force: bool = False,
    ) -> dict[str, Any]: ...

    def teleop(
        self,
        endpoint: str,
        *,
        linear: float,
        angular: float,
        timeout_ms: int = 350,
        owner_id: str = "",
    ) -> dict[str, Any]: ...

    def teleop_stop(
        self,
        endpoint: str,
        *,
        owner_id: str = "",
    ) -> dict[str, Any]: ...


class UnavailableRobotGateway:
    """Safe default until a concrete runtime selects its transport."""

    transport = "unavailable"

    def __getattr__(self, operation: str):
        def unavailable(*_args: Any, **_kwargs: Any):
            raise RuntimeError(
                f"robot gateway operation {operation!r} is unavailable for "
                "the transport-neutral FleetManagerCore"
            )

        return unavailable
