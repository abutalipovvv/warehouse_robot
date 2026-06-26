import json
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from real_robot.robokit_protocol import HEADER_FORMAT, HEADER_SIZE, pack_message, unpack_header


def test_pack_message_without_payload():
    raw = pack_message(1, 1100)
    assert len(raw) == HEADER_SIZE
    sync, version, sequence, length, api_number, reserved = unpack_header(raw)
    assert sync == 0x5A
    assert version == 0x01
    assert sequence == 1
    assert length == 0
    assert api_number == 1100
    assert reserved == b"\x00\x00\x00\x00\x00\x00"


def test_pack_message_with_payload():
    raw = pack_message(7, 3051, {"source_id": "LM1", "id": "LM2", "task_id": "abc"})
    header = raw[:HEADER_SIZE]
    body = raw[HEADER_SIZE:]
    sync, version, sequence, length, api_number, _reserved = struct.unpack(HEADER_FORMAT, header)
    assert (sync, version, sequence, api_number) == (0x5A, 0x01, 7, 3051)
    assert length == len(body)
    assert json.loads(body.decode("ascii")) == {"source_id": "LM1", "id": "LM2", "task_id": "abc"}
