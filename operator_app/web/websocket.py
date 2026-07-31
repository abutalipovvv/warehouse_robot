"""Small RFC 6455 helpers for the operator's dependency-free HTTP server."""

from __future__ import annotations

import base64
import hashlib
import json
import select
import struct
from typing import Any, Mapping, Protocol

from ..core.config import WEBSOCKET_GUID


TEXT_OPCODE = 0x1
CLOSE_OPCODE = 0x8
PING_OPCODE = 0x9
PONG_OPCODE = 0xA
DEFAULT_MAX_FRAME_BYTES = 4 * 1024 * 1024


class SocketLike(Protocol):
    def gettimeout(self) -> float | None: ...

    def settimeout(self, value: float | None) -> None: ...

    def recv(self, size: int) -> bytes: ...

    def sendall(self, data: bytes) -> None: ...

    def fileno(self) -> int: ...


def websocket_accept_value(client_key: str) -> str:
    """Return the server handshake value for a browser websocket key."""

    digest = hashlib.sha1(
        f"{client_key}{WEBSOCKET_GUID}".encode("ascii")
    ).digest()
    return base64.b64encode(digest).decode("ascii")


def is_websocket_upgrade(headers: Mapping[str, str]) -> bool:
    upgrade = str(headers.get("Upgrade", "")).strip().lower()
    connection = str(headers.get("Connection", "")).strip().lower()
    return upgrade == "websocket" and "upgrade" in connection


def encode_frame(payload: bytes, opcode: int) -> bytes:
    """Encode one final, unmasked server-to-client websocket frame."""

    length = len(payload)
    first_byte = 0x80 | int(opcode)
    if length < 126:
        return bytes([first_byte, length]) + payload
    if length <= 0xFFFF:
        return bytes([first_byte, 126]) + struct.pack("!H", length) + payload
    return bytes([first_byte, 127]) + struct.pack("!Q", length) + payload


class WebSocketConnection:
    """Read and write websocket frames over an accepted socket."""

    def __init__(
        self,
        socket: SocketLike,
        *,
        max_frame_bytes: int = DEFAULT_MAX_FRAME_BYTES,
    ) -> None:
        self.socket = socket
        self.max_frame_bytes = max(1, int(max_frame_bytes))

    def send_json(self, payload: object) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.socket.sendall(encode_frame(encoded, TEXT_OPCODE))

    def client_closed(self) -> bool:
        frame = self.read_frame(timeout_sec=0.0)
        return frame is not None and frame[0] == CLOSE_OPCODE

    def read_json(
        self,
        *,
        timeout_sec: float = 0.0,
    ) -> dict[str, object] | None:
        frame = self.read_frame(timeout_sec=timeout_sec)
        if frame is None:
            return None
        opcode, payload = frame
        if opcode == CLOSE_OPCODE:
            return {"__closed": True}
        if opcode != TEXT_OPCODE:
            return None
        try:
            decoded = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return None
        return decoded if isinstance(decoded, dict) else None

    def read_frame(
        self,
        *,
        timeout_sec: float = 0.0,
    ) -> tuple[int, bytes] | None:
        readable, _, _ = select.select(
            [self.socket],
            [],
            [],
            max(0.0, float(timeout_sec)),
        )
        if not readable:
            return None

        previous_timeout = self.socket.gettimeout()
        self.socket.settimeout(0.05)
        try:
            header = self.recv_exact(2)
            if not header:
                return CLOSE_OPCODE, b""
            opcode = header[0] & 0x0F
            masked = bool(header[1] & 0x80)
            length = header[1] & 0x7F
            if length == 126:
                extended = self.recv_exact(2)
                if not extended:
                    return CLOSE_OPCODE, b""
                length = struct.unpack("!H", extended)[0]
            elif length == 127:
                extended = self.recv_exact(8)
                if not extended:
                    return CLOSE_OPCODE, b""
                length = struct.unpack("!Q", extended)[0]
            if length > self.max_frame_bytes:
                return CLOSE_OPCODE, b""

            mask = self.recv_exact(4) if masked else b""
            payload = self.recv_exact(length) if length else b""
            if length and not payload:
                return CLOSE_OPCODE, b""
            if masked and mask:
                payload = bytes(
                    value ^ mask[index % 4]
                    for index, value in enumerate(payload)
                )
            if opcode == PING_OPCODE:
                self.socket.sendall(encode_frame(payload, PONG_OPCODE))
                return None
            return opcode, payload
        except TimeoutError:
            return None
        except (ConnectionError, OSError):
            return CLOSE_OPCODE, b""
        finally:
            self.socket.settimeout(previous_timeout)

    def recv_exact(self, size: int) -> bytes:
        chunks: list[bytes] = []
        remaining = max(0, int(size))
        while remaining:
            chunk = self.socket.recv(remaining)
            if not chunk:
                return b""
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)


def teleop_command(message: Mapping[str, Any]) -> dict[str, object] | None:
    """Normalize one browser teleoperation message for the robot stream."""

    if str(message.get("type") or "teleop") == "stop":
        return {"linear": 0.0, "angular": 0.0, "timeoutMs": 80}
    try:
        linear = float(message.get("linear", 0.0) or 0.0)
        angular = float(message.get("angular", 0.0) or 0.0)
        timeout_ms = int(
            message.get(
                "timeoutMs",
                message.get("timeout_ms", 350),
            )
            or 350
        )
    except (TypeError, ValueError, OverflowError):
        return None
    return {
        "linear": linear,
        "angular": angular,
        "timeoutMs": max(80, timeout_ms),
    }


__all__ = [
    "CLOSE_OPCODE",
    "DEFAULT_MAX_FRAME_BYTES",
    "PING_OPCODE",
    "PONG_OPCODE",
    "TEXT_OPCODE",
    "WebSocketConnection",
    "encode_frame",
    "is_websocket_upgrade",
    "teleop_command",
    "websocket_accept_value",
]
