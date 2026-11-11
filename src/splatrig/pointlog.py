# SPDX-License-Identifier: MIT
"""Binary timed point log: fixed little-endian records after a JSON header.

Layout: 8-byte magic, uint32 header length, UTF-8 JSON header, zero padding to an
8-byte boundary, then records of int64 timestamp_ns, float32 x/y/z in meters,
uint8 reflectivity, tag and line, and one reserved byte.
"""

from __future__ import annotations

import json
from pathlib import Path
import struct

MAGIC = b"SRPOINT1"
RECORD = struct.Struct("<qfffBBBx")
FIELDS = ["timestamp_ns:int64", "x_m:float32", "y_m:float32", "z_m:float32",
          "reflectivity:uint8", "tag:uint8", "line:uint8", "reserved:uint8"]


class PointLogWriter:
    def __init__(self, path: Path, header: dict):
        payload = json.dumps({**header, "record_bytes": RECORD.size, "fields": FIELDS},
                             separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.stream = path.open("xb")
        self.stream.write(MAGIC + struct.pack("<I", len(payload)) + payload)
        self.stream.write(b"\0" * (-(len(MAGIC) + 4 + len(payload)) % 8))
        self.count = 0
        self.buffer = bytearray()

    def write(self, timestamp_ns: int, x: float, y: float, z: float,
              reflectivity: int = 0, tag: int = 0, line: int = 0):
        self.buffer += RECORD.pack(timestamp_ns, x, y, z, reflectivity, tag, line)
        self.count += 1
        if len(self.buffer) >= 1 << 20:
            self.flush()

    def flush(self):
        self.stream.write(self.buffer)
        self.buffer.clear()

    def close(self):
        self.flush()
        self.stream.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def read_header(path: Path):
    with path.open("rb") as stream:
        prefix = stream.read(12)
        if len(prefix) != 12 or prefix[:8] != MAGIC:
            raise ValueError(f"{path.name}: not a SplatRig point log")
        length = struct.unpack("<I", prefix[8:])[0]
        header = json.loads(stream.read(length).decode("utf-8"))
    start = 12 + length + (-(12 + length) % 8)
    if header.get("record_bytes") != RECORD.size or header.get("fields") != FIELDS:
        raise ValueError(f"{path.name}: unsupported point record layout")
    body = path.stat().st_size - start
    if body < 0 or body % RECORD.size:
        raise ValueError(f"{path.name}: truncated point records")
    return header, start, body // RECORD.size


def iter_points(path: Path, chunk_records: int = 65536):
    """Yield (timestamp_ns, x, y, z, reflectivity, tag, line) tuples."""
    _, start, count = read_header(path)
    with path.open("rb") as stream:
        stream.seek(start)
        remaining = count
        while remaining:
            block = stream.read(RECORD.size * min(remaining, chunk_records))
            remaining -= len(block) // RECORD.size
            yield from RECORD.iter_unpack(block)
