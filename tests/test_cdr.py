"""CDR encoding, checked field by field against the layout the decoder expects."""

from __future__ import annotations

import struct

from fm_robot_agent.cdr import ENCAPSULATION_HEADER, CDRWriter, joint_state


def body(payload: bytes) -> bytes:
    assert payload[:4] == ENCAPSULATION_HEADER
    return payload[4:]


def test_the_encapsulation_header_opens_every_payload():
    assert joint_state(0.0, [], [], [], [])[:4] == b"\x00\x01\x00\x00"


def test_a_string_carries_its_nul_terminator():
    writer = CDRWriter()
    writer.write_string("base")
    assert body(writer.data) == struct.pack("<I", 5) + b"base\x00"


def test_an_empty_string_is_one_nul():
    writer = CDRWriter()
    writer.write_string("")
    assert body(writer.data) == struct.pack("<I", 1) + b"\x00"


def test_a_float64_aligns_to_eight_from_the_body_start():
    writer = CDRWriter()
    writer.write_uint32(1)
    writer.write_float64(2.0)
    encoded = body(writer.data)
    assert len(encoded) == 16
    assert encoded[4:8] == b"\x00\x00\x00\x00"
    assert struct.unpack("<d", encoded[8:16])[0] == 2.0


def test_an_empty_sequence_emits_no_padding_after_its_count():
    """The one case a "pad, then loop" writer gets wrong."""
    writer = CDRWriter()
    writer.write_float64_sequence([])
    assert body(writer.data) == struct.pack("<I", 0)


def test_a_joint_state_decodes_field_by_field():
    payload = joint_state(
        stamp_s=12.5,
        names=["left:SHOULDER_1", "right:ELBOW"],
        positions=[0.1, 0.2],
        velocities=[0.3, 0.4],
        efforts=[0.5, 0.6],
        frame_id="base",
    )
    encoded = body(payload)
    offset = 0

    sec, nanosec = struct.unpack_from("<iI", encoded, offset)
    offset += 8
    assert (sec, nanosec) == (12, 500_000_000)

    length = struct.unpack_from("<I", encoded, offset)[0]
    offset += 4
    assert encoded[offset : offset + length] == b"base\x00"
    offset += length

    def read_string_sequence(offset):
        offset += -offset % 4
        count = struct.unpack_from("<I", encoded, offset)[0]
        offset += 4
        values = []
        for _ in range(count):
            offset += -offset % 4
            size = struct.unpack_from("<I", encoded, offset)[0]
            offset += 4
            values.append(encoded[offset : offset + size - 1].decode())
            offset += size
        return values, offset

    def read_float_sequence(offset):
        offset += -offset % 4
        count = struct.unpack_from("<I", encoded, offset)[0]
        offset += 4
        values = []
        for _ in range(count):
            offset += -offset % 8
            values.append(struct.unpack_from("<d", encoded, offset)[0])
            offset += 8
        return values, offset

    names, offset = read_string_sequence(offset)
    positions, offset = read_float_sequence(offset)
    velocities, offset = read_float_sequence(offset)
    efforts, offset = read_float_sequence(offset)

    assert names == ["left:SHOULDER_1", "right:ELBOW"]
    assert positions == [0.1, 0.2]
    assert velocities == [0.3, 0.4]
    assert efforts == [0.5, 0.6]
    assert offset == len(encoded)
