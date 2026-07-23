"""Schema-backed Upper Protocol v3 codec used by tests and HIL tools."""

from __future__ import annotations

import json
import re
import struct
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCHEMA = ROOT / "docs" / "protocol" / "upper-v3.schema.json"


def load_schema(path: Path = DEFAULT_SCHEMA) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def crc8_maxim(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc & 0xFF


def build_frame(command: int, payload: bytes) -> bytes:
    if len(payload) > 99:
        raise ValueError("payload exceeds v3 maximum")
    body = bytes((len(payload) + 1, command)) + payload
    return b"\xA5\x5A" + body + bytes((crc8_maxim(body),))


def parse_frame(frame: bytes) -> tuple[int, bytes]:
    if len(frame) < 5 or frame[:2] != b"\xA5\x5A":
        raise ValueError("invalid header or short frame")
    command_length = frame[2]
    if command_length < 1 or len(frame) != command_length + 4:
        raise ValueError("invalid frame length")
    if crc8_maxim(frame[2:-1]) != frame[-1]:
        raise ValueError("CRC mismatch")
    return frame[3], frame[4:-1]


class StreamDecoder:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buffer.extend(data)
        frames: list[tuple[int, bytes]] = []
        while True:
            start = self._buffer.find(b"\xA5\x5A")
            if start < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer[-1:] == b"\xA5" else b""
                break
            del self._buffer[:start]
            if len(self._buffer) < 3:
                break
            total = self._buffer[2] + 4
            if self._buffer[2] < 1 or total > 104:
                del self._buffer[0]
                continue
            if len(self._buffer) < total:
                break
            candidate = bytes(self._buffer[:total])
            del self._buffer[:total]
            try:
                frames.append(parse_frame(candidate))
            except ValueError:
                self._buffer[:0] = candidate[1:]
        return frames


def encode_set_velocity(linear_x: float, angular_z: float, enable: bool, session_id: int, sequence: int) -> bytes:
    payload = struct.pack("<BffBBQI", 3, linear_x, angular_z, int(enable), 2, session_id, sequence)
    return build_frame(1, payload)


def encode_version_only(command: int) -> bytes:
    return build_frame(command, b"\x03")


def _unpack_field(payload: bytes, field: dict[str, Any]) -> Any:
    field_type = field["type"]
    offset = field["offset"]
    width = field["width"]
    raw = payload[offset : offset + width]
    if field_type == "bytes":
        return raw
    match = re.fullmatch(r"(u8|i8|u16|i16|u32|i32|u64|f32)(?:\[(\d+)\])?", field_type)
    if match is None:
        raise ValueError(f"unsupported schema type {field_type}")
    base, count_text = match.groups()
    formats = {"u8":"B", "i8":"b", "u16":"H", "i16":"h", "u32":"I", "i32":"i", "u64":"Q", "f32":"f"}
    count = int(count_text or "1")
    values = struct.unpack("<" + formats[base] * count, raw)
    return values[0] if count == 1 else list(values)


def decode_payload(message_name: str, payload: bytes, schema: dict[str, Any] | None = None) -> dict[str, Any]:
    contract = (schema or load_schema())["messages"][message_name]
    if len(payload) != contract["payload_length"]:
        raise ValueError(f"{message_name} payload length mismatch")
    return {field["name"]: _unpack_field(payload, field) for field in contract["fields"]}


def read_frames(chunks: Iterable[bytes]) -> list[tuple[int, bytes]]:
    decoder = StreamDecoder()
    frames: list[tuple[int, bytes]] = []
    for chunk in chunks:
        frames.extend(decoder.feed(chunk))
    return frames
