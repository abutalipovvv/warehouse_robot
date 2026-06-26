from __future__ import annotations

import json
import socket
import struct
from dataclasses import dataclass
from threading import Lock
from typing import Any


SYNC_BYTE = 0x5A
PROTOCOL_VERSION = 0x01
HEADER_FORMAT = "!BBHLH6s"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
RESERVED = b"\x00\x00\x00\x00\x00\x00"

STATUS_PORT = 19204
CONTROL_PORT = 19205
NAVIGATION_PORT = 19206
CONFIG_PORT = 19207

API_STATUS_ALL1 = 1100
API_CONTROL_MOTION = 2010
API_TASK_CANCEL = 3003
API_TASK_GOTO_TARGET = 3051
API_CONFIG_LOCK = 4005
API_CONFIG_UNLOCK = 4006


class RobokitError(RuntimeError):
    pass


class RobokitApiError(RobokitError):
    def __init__(self, message: str, *, api_number: int, payload: dict[str, Any]) -> None:
        super().__init__(message)
        self.api_number = api_number
        self.payload = payload


@dataclass(frozen=True)
class RobokitResponse:
    sequence: int
    api_number: int
    payload: dict[str, Any]


def expected_response_api(api_number: int) -> int:
    return int(api_number) + 10000


def pack_message(sequence: int, api_number: int, payload: dict[str, Any] | None = None) -> bytes:
    body = b""
    if payload:
        body = json.dumps(payload, separators=(",", ":"), ensure_ascii=True).encode("ascii")
    header = struct.pack(
        HEADER_FORMAT,
        SYNC_BYTE,
        PROTOCOL_VERSION,
        int(sequence) & 0xFFFF,
        len(body),
        int(api_number) & 0xFFFF,
        RESERVED,
    )
    return header + body


def unpack_header(data: bytes) -> tuple[int, int, int, int, int, bytes]:
    if len(data) != HEADER_SIZE:
        raise RobokitError(f"invalid Robokit header size: {len(data)}")
    sync, version, sequence, length, api_number, reserved = struct.unpack(HEADER_FORMAT, data)
    if sync != SYNC_BYTE:
        raise RobokitError(f"invalid Robokit sync byte: 0x{sync:02X}")
    if version != PROTOCOL_VERSION:
        raise RobokitError(f"unsupported Robokit protocol version: {version}")
    return sync, version, sequence, length, api_number, reserved


class RobokitConnection:
    def __init__(self, host: str, port: int, *, timeout_sec: float = 0.8, label: str | None = None) -> None:
        self.host = str(host or "").strip()
        self.port = int(port)
        self.timeout_sec = max(0.1, float(timeout_sec))
        self.label = label or f"{self.host}:{self.port}"
        self._lock = Lock()
        self._socket: socket.socket | None = None
        self._sequence = 0

    def close(self) -> None:
        with self._lock:
            self._close_unlocked()

    def request(
        self,
        api_number: int,
        payload: dict[str, Any] | None = None,
        *,
        timeout_sec: float | None = None,
        expected_api_number: int | None = None,
    ) -> dict[str, Any]:
        expected = expected_response_api(api_number) if expected_api_number is None else int(expected_api_number)
        last_error: Exception | None = None
        for attempt in range(2):
            with self._lock:
                try:
                    return self._request_unlocked(
                        int(api_number),
                        payload,
                        timeout_sec=self.timeout_sec if timeout_sec is None else max(0.1, float(timeout_sec)),
                        expected_api_number=expected,
                    )
                except RobokitApiError:
                    raise
                except (OSError, TimeoutError, RobokitError) as exc:
                    last_error = exc
                    self._close_unlocked()
                    if attempt == 1:
                        break
        raise RobokitError(f"{self.label} request {api_number} failed: {last_error}") from last_error

    def _request_unlocked(
        self,
        api_number: int,
        payload: dict[str, Any] | None,
        *,
        timeout_sec: float,
        expected_api_number: int,
    ) -> dict[str, Any]:
        sock = self._connect_unlocked(timeout_sec)
        sequence = self._next_sequence_unlocked()
        sock.sendall(pack_message(sequence, api_number, payload))
        header = self._recv_exact_unlocked(HEADER_SIZE)
        _sync, _version, response_sequence, length, response_api, _reserved = unpack_header(header)
        if response_sequence != sequence:
            raise RobokitError(f"sequence mismatch: request={sequence}, response={response_sequence}")
        if response_api != expected_api_number:
            raise RobokitError(f"unexpected response API: {response_api}, expected {expected_api_number}")
        body = self._recv_exact_unlocked(length) if length > 0 else b""
        response_payload = self._decode_payload(body)
        self._raise_for_api_error(api_number, response_payload)
        return response_payload

    def _connect_unlocked(self, timeout_sec: float) -> socket.socket:
        if not self.host:
            raise RobokitError("robot host is empty")
        if self._socket is not None:
            self._socket.settimeout(timeout_sec)
            return self._socket
        sock = socket.create_connection((self.host, self.port), timeout=timeout_sec)
        sock.settimeout(timeout_sec)
        self._socket = sock
        return sock

    def _close_unlocked(self) -> None:
        sock = self._socket
        self._socket = None
        if sock is None:
            return
        try:
            sock.close()
        except OSError:
            pass

    def _next_sequence_unlocked(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFFFF
        if self._sequence == 0:
            self._sequence = 1
        return self._sequence

    def _recv_exact_unlocked(self, size: int) -> bytes:
        chunks: list[bytes] = []
        remaining = int(size)
        while remaining > 0:
            sock = self._socket
            if sock is None:
                raise RobokitError("socket is not connected")
            chunk = sock.recv(remaining)
            if not chunk:
                raise RobokitError("connection closed by robot")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _decode_payload(self, body: bytes) -> dict[str, Any]:
        if not body:
            return {}
        try:
            decoded = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RobokitError(f"invalid JSON response from {self.label}") from exc
        if not isinstance(decoded, dict):
            raise RobokitError(f"unexpected non-object JSON response from {self.label}")
        return decoded

    def _raise_for_api_error(self, api_number: int, payload: dict[str, Any]) -> None:
        ret_code = payload.get("ret_code")
        if ret_code in (None, 0, "0"):
            return
        err_msg = str(payload.get("err_msg") or payload.get("message") or "Robokit API error")
        raise RobokitApiError(
            f"API {api_number} returned ret_code={ret_code}: {err_msg}",
            api_number=api_number,
            payload=payload,
        )


class RobokitClient:
    def __init__(
        self,
        host: str,
        *,
        status_port: int = STATUS_PORT,
        control_port: int = CONTROL_PORT,
        navigation_port: int = NAVIGATION_PORT,
        config_port: int = CONFIG_PORT,
        timeout_sec: float = 0.8,
    ) -> None:
        self.status = RobokitConnection(host, status_port, timeout_sec=timeout_sec, label="status")
        self.control = RobokitConnection(host, control_port, timeout_sec=timeout_sec, label="control")
        self.navigation = RobokitConnection(host, navigation_port, timeout_sec=timeout_sec, label="navigation")
        self.config = RobokitConnection(host, config_port, timeout_sec=timeout_sec, label="config")

    def close(self) -> None:
        self.status.close()
        self.control.close()
        self.navigation.close()
        self.config.close()

    def read_all_status(self, keys: list[str]) -> dict[str, Any]:
        return self.status.request(
            API_STATUS_ALL1,
            {
                "keys": keys,
                "return_laser": False,
                "return_beams3D": False,
            },
        )

    def send_motion(self, *, vx: float, vy: float, w: float, duration_ms: int) -> dict[str, Any]:
        return self.control.request(
            API_CONTROL_MOTION,
            {
                "vx": float(vx),
                "vy": float(vy),
                "w": float(w),
                "duration": max(1, int(duration_ms)),
            },
        )

    def goto_target(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self.navigation.request(API_TASK_GOTO_TARGET, payload)

    def cancel_navigation(self) -> dict[str, Any]:
        return self.navigation.request(API_TASK_CANCEL)

    def acquire_control(self, nick_name: str) -> dict[str, Any]:
        return self.config.request(API_CONFIG_LOCK, {"nick_name": str(nick_name or "ros2_robot_driver")})

    def release_control(self) -> dict[str, Any]:
        return self.config.request(API_CONFIG_UNLOCK)
