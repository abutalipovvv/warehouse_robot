from __future__ import annotations

import json
import struct

import pytest

from operator_app.web import websocket


class FakeSocket:
    def __init__(self, incoming: bytes = b"") -> None:
        self.incoming = bytearray(incoming)
        self.sent = bytearray()
        self.timeout: float | None = None

    def gettimeout(self) -> float | None:
        return self.timeout

    def settimeout(self, value: float | None) -> None:
        self.timeout = value

    def recv(self, size: int) -> bytes:
        chunk = bytes(self.incoming[:size])
        del self.incoming[:size]
        return chunk

    def sendall(self, data: bytes) -> None:
        self.sent.extend(data)

    def fileno(self) -> int:
        return 1


@pytest.fixture(autouse=True)
def readable_socket(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        websocket.select,
        "select",
        lambda readers, _writers, _errors, _timeout: (readers, [], []),
    )


def _masked_frame(payload: bytes, opcode: int = websocket.TEXT_OPCODE) -> bytes:
    mask = b"\x01\x02\x03\x04"
    masked = bytes(
        value ^ mask[index % 4]
        for index, value in enumerate(payload)
    )
    return bytes([0x80 | opcode, 0x80 | len(payload)]) + mask + masked


def test_websocket_handshake_matches_rfc_example() -> None:
    assert websocket.websocket_accept_value(
        "dGhlIHNhbXBsZSBub25jZQ=="
    ) == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="


@pytest.mark.parametrize("size", [0, 125, 126, 65_535, 65_536])
def test_server_frame_encodes_all_length_forms(size: int) -> None:
    payload = b"x" * size

    frame = websocket.encode_frame(payload, websocket.TEXT_OPCODE)

    assert frame[0] == 0x81
    if size < 126:
        assert frame[1] == size
        assert frame[2:] == payload
    elif size <= 65_535:
        assert frame[1] == 126
        assert struct.unpack("!H", frame[2:4])[0] == size
        assert frame[4:] == payload
    else:
        assert frame[1] == 127
        assert struct.unpack("!Q", frame[2:10])[0] == size
        assert frame[10:] == payload


def test_connection_decodes_masked_json_and_restores_timeout() -> None:
    payload = json.dumps({"linear": 0.2}).encode()
    socket = FakeSocket(_masked_frame(payload))
    socket.timeout = 2.0

    decoded = websocket.WebSocketConnection(socket).read_json()

    assert decoded == {"linear": 0.2}
    assert socket.timeout == 2.0


def test_connection_replies_to_ping() -> None:
    socket = FakeSocket(_masked_frame(b"hi", opcode=websocket.PING_OPCODE))

    result = websocket.WebSocketConnection(socket).read_frame()

    assert result is None
    assert bytes(socket.sent) == websocket.encode_frame(
        b"hi",
        websocket.PONG_OPCODE,
    )


def test_oversized_frame_is_treated_as_closed_without_allocating_payload() -> None:
    declared_size = 1024
    header = bytes([0x81, 126]) + struct.pack("!H", declared_size)
    socket = FakeSocket(header)

    result = websocket.WebSocketConnection(
        socket,
        max_frame_bytes=100,
    ).read_frame()

    assert result == (websocket.CLOSE_OPCODE, b"")


def test_teleop_command_normalizes_stop_timeout_and_bad_numbers() -> None:
    assert websocket.teleop_command({"type": "stop"}) == {
        "linear": 0.0,
        "angular": 0.0,
        "timeoutMs": 80,
    }
    assert websocket.teleop_command(
        {"linear": "0.4", "angular": -0.2, "timeoutMs": 10}
    ) == {
        "linear": 0.4,
        "angular": -0.2,
        "timeoutMs": 80,
    }
    assert websocket.teleop_command({"linear": "invalid"}) is None
