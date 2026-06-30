from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(slots=True)
class KnownRobot:
    id: str
    name: str
    host: str
    port: int = 50051
    type: str = "grpc"
    domain_id: int = 0
    namespace: str = ""
    status_topic: str = "/robot_status"
    cmd_vel_topic: str = "/cmd_vel"
    go_to_lm_topic: str = "/go_to_lm"
    last_seen: str = ""
    last_identity: dict[str, Any] | None = None

    @property
    def base_url(self) -> str:
        if self.is_grpc:
            return f"grpc://{self.host}:{self.port}"
        return ""

    @property
    def is_grpc(self) -> bool:
        return self.type.lower() in {"grpc", "aivison_grpc", "real_grpc"}

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "host": self.host,
            "port": self.port,
            "type": self.type,
            "mode": self.type,
            "domainId": self.domain_id,
            "namespace": self.namespace,
            "statusTopic": self.status_topic,
            "cmdVelTopic": self.cmd_vel_topic,
            "goToLmTopic": self.go_to_lm_topic,
            "baseUrl": self.base_url,
            "lastSeen": self.last_seen,
            "lastIdentity": self.last_identity or None,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "KnownRobot":
        robot_type = str(payload.get("type") or payload.get("mode") or "grpc").strip().lower() or "grpc"
        domain_raw = payload.get("domainId", payload.get("domain_id", 0))
        try:
            domain_id = int(domain_raw)
        except (TypeError, ValueError):
            domain_id = 0
        port_raw = payload.get("port", 50051)
        try:
            port = int(port_raw)
        except (TypeError, ValueError):
            port = 50051
        return cls(
            id=str(payload.get("id") or "").strip(),
            name=str(payload.get("name") or "").strip(),
            host=str(payload.get("host") or "").strip(),
            port=port,
            type=robot_type,
            domain_id=domain_id,
            namespace=str(payload.get("namespace") or "").strip().strip("/"),
            status_topic=str(payload.get("statusTopic") or payload.get("status_topic") or "/robot_status").strip() or "/robot_status",
            cmd_vel_topic=str(payload.get("cmdVelTopic") or payload.get("cmd_vel_topic") or "/cmd_vel").strip() or "/cmd_vel",
            go_to_lm_topic=str(payload.get("goToLmTopic") or payload.get("go_to_lm_topic") or "/go_to_lm").strip() or "/go_to_lm",
            last_seen=str(payload.get("lastSeen") or payload.get("last_seen") or "").strip(),
            last_identity=payload.get("lastIdentity") or payload.get("last_identity") or None,
        )
