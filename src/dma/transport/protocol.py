"""Wire protocol, v2 — framing, CRC, sequence numbers, device telemetry.

v1 was a 10 byte frame with a XOR checksum and no length field: a single dropped
byte desynchronised the link forever, and the device could not tell the host that
it was dropping reports.  That is unshippable.

v2 frame::

    0      magic 0xA5
    1      version (high nibble) | type (low nibble)   -> 0x21 for MOVE
    2      seq (wraps)
    3      payload length
    4..    payload
    last   CRC-8/ATM over bytes[0:4+len]

``StreamParser`` resynchronises on garbage, so a hot-plug or a firmware reset
cannot wedge the host: it keeps scanning for the magic byte and validates CRC.

Device → host ``STATUS`` frames turn the ESP32 into a monitored node: uptime,
receive count, CRC errors, queue drops.  The dashboard shows them, and a rising
CRC error count is an early warning for a bad USB cable long before aim degrades.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple

MAGIC = 0xA5
VERSION = 2

T_MOVE = 1
T_BUTTON = 2
T_PING = 3
T_STATUS = 4
T_CONFIG = 5

# legacy v1 (0xAA prefix) constants, kept so old firmware still runs
LEGACY_MAGIC = 0xAA
LEGACY_LEN = 10
L_CMD_MOVE = 0x01
L_CMD_BUTTON = 0x02
L_CMD_STOP = 0x03
L_CMD_HEARTBEAT = 0x04

def _build_crc_table() -> List[int]:
    table = [0] * 256
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table[i] = crc
    return table


_CRC8 = _build_crc_table()


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC8[crc ^ b]
    return crc & 0xFF


# ---------------------------------------------------------------------------
# framing
# ---------------------------------------------------------------------------
def encode(kind: int, payload: bytes = b"", seq: int = 0) -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too long")
    header = bytes([MAGIC, ((VERSION & 0x0F) << 4) | (kind & 0x0F), seq & 0xFF, len(payload)])
    body = header + payload
    return body + bytes([crc8(body)])


def encode_move(dx: int, dy: int, seq: int) -> bytes:
    dx = max(-32768, min(32767, int(dx)))
    dy = max(-32768, min(32767, int(dy)))
    return encode(T_MOVE, struct.pack("<hh", dx, dy), seq)


def encode_button(mask: int, seq: int) -> bytes:
    return encode(T_BUTTON, bytes([mask & 0xFF]), seq)


def encode_ping(counter: int, seq: int = 0) -> bytes:
    return encode(T_PING, struct.pack("<I", counter & 0xFFFFFFFF), seq)


@dataclass(frozen=True, slots=True)
class DeviceStatus:
    uptime_ms: int
    rx_frames: int
    crc_errors: int
    queue_drops: int
    report_hz: int
    flags: int = 0


STATUS_LEN = 13  # u32 uptime + 4 x u16 counters + u8 flags


def decode_status(payload: bytes) -> Optional[DeviceStatus]:
    if len(payload) < STATUS_LEN:
        return None
    uptime, rx, crc_err, drops, hz, flags = struct.unpack("<IHHHHB", payload[:STATUS_LEN])
    return DeviceStatus(uptime, rx, crc_err, drops, hz, flags)


def decode(frame: bytes) -> Tuple[int, bytes, int]:
    """Validate and unpack a complete frame; raises ValueError when invalid."""
    if len(frame) < 5:
        raise ValueError("frame too short")
    if frame[0] != MAGIC:
        raise ValueError(f"bad magic {frame[0]:#x}")
    ver = frame[1] >> 4
    if ver != VERSION:
        raise ValueError(f"bad version {ver}")
    length = frame[3]
    if len(frame) != 4 + length + 1:
        raise ValueError("length mismatch")
    if crc8(frame[:-1]) != frame[-1]:
        raise ValueError("crc mismatch")
    return frame[1] & 0x0F, frame[4:-1], frame[2]


class StreamParser:
    """Incremental parser with resynchronisation, for device → host traffic."""

    __slots__ = ("_buf", "crc_errors", "frames")

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors = 0
        self.frames = 0

    def feed(self, chunk: bytes) -> Iterator[Tuple[int, bytes, int]]:
        if chunk:
            self._buf.extend(chunk)
        while True:
            if not self._buf:
                return
            start = self._buf.find(MAGIC)
            if start < 0:
                self._buf.clear()
                return
            if start:
                del self._buf[:start]
            if len(self._buf) < 5:
                return
            length = self._buf[3]
            total = 4 + length + 1
            if length > 128:
                del self._buf[0]
                self.crc_errors += 1
                continue
            if len(self._buf) < total:
                return
            frame = bytes(self._buf[:total])
            try:
                kind, payload, seq = decode(frame)
            except ValueError:
                self.crc_errors += 1
                del self._buf[0]
                continue
            del self._buf[:total]
            self.frames += 1
            yield kind, payload, seq


# ---------------------------------------------------------------------------
# legacy v1 encoders (used when output.protocol == "v1")
# ---------------------------------------------------------------------------
def _xor(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def legacy_frame(cmd: int, dx: int = 0, dy: int = 0, buttons: int = 0, seq: int = 0) -> bytes:
    dx = max(-32768, min(32767, int(dx)))
    dy = max(-32768, min(32767, int(dy)))
    body = bytes(
        [
            LEGACY_MAGIC,
            cmd & 0xFF,
            dx & 0xFF,
            (dx >> 8) & 0xFF,
            dy & 0xFF,
            (dy >> 8) & 0xFF,
            buttons & 0xFF,
            seq & 0xFF,
            0x00,
        ]
    )
    return body + bytes([_xor(body)])


def legacy_move(dx: int, dy: int, seq: int) -> bytes:
    return legacy_frame(L_CMD_MOVE, dx, dy, 0, seq)


def legacy_button(mask: int, seq: int) -> bytes:
    return legacy_frame(L_CMD_BUTTON, 0, 0, mask, seq)


def legacy_heartbeat(seq: int) -> bytes:
    return legacy_frame(L_CMD_HEARTBEAT, 0, 0, 0, seq)
