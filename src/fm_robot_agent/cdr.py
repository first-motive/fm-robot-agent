"""Encode one ``sensor_msgs/JointState`` as CDR, so an Axol looks like a ROS rig.

The Anvil reaches the fleet through ``zenoh-bridge-ros2dds``, which puts real
DDS-serialized messages on the fabric. The Axol has no ROS graph at all — its own
stack owns the CAN bus — so the agent serializes its telemetry into the same wire
format instead. The desktop then decodes both robots with one decoder, and a
future ROS consumer subscribes to the Axol without knowing it is not a rig.

The rules are FastCDR's, the serializer behind ROS 2's default ``rmw_fastrtps``:

- The buffer opens with the 4-byte plain-CDR little-endian encapsulation header
  (``00 01 00 00``); the body starts at offset 4.
- A primitive of size ``n`` is padded to the next multiple of ``n``, counted from
  the body start.
- A string is a ``uint32`` length *including* its NUL terminator, then the bytes,
  then the NUL.
- A sequence is a ``uint32`` count then the elements, each aligned as its own
  type. An empty sequence emits no padding after the count — there is no element
  to align, and that is the one case a "pad, then loop" writer gets wrong.

This mirrors fm-desktop's ``CDRWriter``, which is the reader on the other end.
"""

from __future__ import annotations

import struct

ENCAPSULATION_HEADER = b"\x00\x01\x00\x00"


class CDRWriter:
    """A little-endian CDR buffer. Only the pieces a JointState needs."""

    def __init__(self) -> None:
        self._body = bytearray()

    @property
    def data(self) -> bytes:
        return ENCAPSULATION_HEADER + bytes(self._body)

    def align(self, size: int) -> None:
        """Pad to the next multiple of ``size``, counted from the body start."""
        remainder = len(self._body) % size
        if remainder:
            self._body.extend(b"\x00" * (size - remainder))

    def write_uint32(self, value: int) -> None:
        self.align(4)
        self._body.extend(struct.pack("<I", value))

    def write_int32(self, value: int) -> None:
        self.align(4)
        self._body.extend(struct.pack("<i", value))

    def write_float64(self, value: float) -> None:
        self.align(8)
        self._body.extend(struct.pack("<d", value))

    def write_string(self, value: str) -> None:
        encoded = value.encode("utf-8")
        self.write_uint32(len(encoded) + 1)
        self._body.extend(encoded)
        self._body.append(0)

    def write_float64_sequence(self, values: list[float]) -> None:
        self.write_uint32(len(values))
        for value in values:
            self.write_float64(value)

    def write_string_sequence(self, values: list[str]) -> None:
        self.write_uint32(len(values))
        for value in values:
            self.write_string(value)


def joint_state(
    stamp_s: float,
    names: list[str],
    positions: list[float],
    velocities: list[float],
    efforts: list[float],
    frame_id: str = "",
) -> bytes:
    """One ``sensor_msgs/JointState``, encapsulation header included.

    Field order is the message definition's: header (stamp, frame_id), then the
    name, position, velocity and effort sequences.
    """
    writer = CDRWriter()
    writer.write_int32(int(stamp_s))
    writer.write_uint32(int((stamp_s - int(stamp_s)) * 1_000_000_000))
    writer.write_string(frame_id)
    writer.write_string_sequence(names)
    writer.write_float64_sequence(positions)
    writer.write_float64_sequence(velocities)
    writer.write_float64_sequence(efforts)
    return writer.data
