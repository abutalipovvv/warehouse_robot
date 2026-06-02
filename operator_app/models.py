from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(slots=True)
class KnownRobot:
    id: str
    name: str
    host: str
    port: int
    last_seen: str = ""
    last_identity: dict[str, Any] | None = None

    @property
    def base_url(self) -> str:
        return f"http://{self.host}:{self.port}"

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "host": self.host,
            "port": self.port,
            "baseUrl": self.base_url,
            "lastSeen": self.last_seen,
            "lastIdentity": self.last_identity or None,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "KnownRobot":
        return cls(
            id=str(payload.get("id") or "").strip(),
            name=str(payload.get("name") or "").strip(),
            host=str(payload.get("host") or "").strip(),
            port=int(payload.get("port") or 8790),
            last_seen=str(payload.get("lastSeen") or payload.get("last_seen") or "").strip(),
            last_identity=payload.get("lastIdentity") or payload.get("last_identity") or None,
        )
