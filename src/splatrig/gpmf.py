# SPDX-License-Identifier: MIT
"""Read GoPro GPMF IMU samples (GYRO, ACCL) with media-time stamps.

GPMF is a KLV stream: 4-character key, 1-byte type, 1-byte structure size and a
big-endian 2-byte repeat count, with payloads padded to 4 bytes. Type 0 nests.
Each MP4 metadata packet covers one interval of media time; samples inside it
are spread uniformly over that packet's duration. SCAL divides raw integers
into physical units (GYRO rad/s, ACCL m/s^2). ORIN names the camera axis of
each stored component, e.g. "ZXY".
"""

from __future__ import annotations

from fractions import Fraction
import struct

from .panorama import run_tool

FORMATS = {ord("b"): "b", ord("B"): "B", ord("s"): "h", ord("S"): "H", ord("l"): "i",
           ord("L"): "I", ord("f"): "f", ord("d"): "d", ord("j"): "q", ord("J"): "Q"}


def klv(buffer: bytes):
    """Yield (key, type, size, repeat, payload) for one nesting level."""
    offset = 0
    while offset + 8 <= len(buffer):
        key = buffer[offset:offset + 4]
        kind, size = buffer[offset + 4], buffer[offset + 5]
        repeat = struct.unpack(">H", buffer[offset + 6:offset + 8])[0]
        length = size * repeat
        if not all(32 <= c < 127 for c in key) or offset + 8 + length > len(buffer):
            return
        yield key.decode("ascii"), kind, size, repeat, buffer[offset + 8:offset + 8 + length]
        offset += 8 + (length + 3) // 4 * 4


def values(kind: int, size: int, repeat: int, payload: bytes):
    code = FORMATS.get(kind)
    if code is None:
        return payload.rstrip(b"\0").decode("ascii", "replace")
    count = len(payload) // struct.calcsize(code)
    flat = struct.unpack(">" + code * count, payload[:count * struct.calcsize(code)])
    width = max(1, count // repeat) if repeat else 1
    return [flat[i:i + width] for i in range(0, len(flat), width)]


def streams(payload: bytes, wanted=("GYRO", "ACCL")) -> dict:
    """Map fourcc -> {"samples": [...scaled tuples], "orin": str|None} for one packet."""
    found = {}

    def walk(buffer):
        for key, kind, size, repeat, data in klv(buffer):
            if kind == 0 and key in {"DEVC", "STRM"}:
                if key == "STRM":
                    stream = {}
                    for inner in klv(data):
                        stream.setdefault(inner[0], inner)
                    for name in wanted:
                        if name in stream:
                            _, k, s, r, d = stream[name]
                            raw = values(k, s, r, d)
                            scale = values(*stream["SCAL"][1:]) if "SCAL" in stream else [[1]]
                            divisors = [float(v) for row in scale for v in row]
                            samples = [tuple(v / (divisors[i] if len(divisors) > 1 else divisors[0])
                                             for i, v in enumerate(row)) for row in raw]
                            orin = values(*stream["ORIN"][1:]) if "ORIN" in stream else None
                            found[name] = {"samples": samples,
                                           "orin": orin if isinstance(orin, str) else None}
                else:
                    walk(data)
    walk(payload)
    return found


def read_imu(path, stream_index: int, ffmpeg="ffmpeg", ffprobe="ffprobe") -> dict:
    """Return {"GYRO"/"ACCL": {"times_ns": [...], "samples": [...], "orin": str}}."""
    import json
    info = json.loads(run_tool([ffprobe, "-v", "error", "-select_streams", str(stream_index),
                                "-show_entries", "stream=time_base:packet=pts,duration,size",
                                "-of", "json", str(path)]))
    base = Fraction(info["streams"][0]["time_base"])
    packets = info["packets"]
    raw = run_tool([ffmpeg, "-v", "error", "-i", str(path), "-map", f"0:{stream_index}",
                    "-c", "copy", "-f", "data", "pipe:1"])
    result, offset = {}, 0
    for packet in packets:
        size = int(packet["size"])
        chunk, offset = raw[offset:offset + size], offset + size
        start = int(packet["pts"]) * base * 1_000_000_000
        duration = int(packet["duration"]) * base * 1_000_000_000
        for name, stream in streams(chunk).items():
            entry = result.setdefault(name, {"times_ns": [], "samples": [], "orin": stream["orin"]})
            count = len(stream["samples"])
            entry["times_ns"] += [round(start + duration * i / count) for i in range(count)]
            entry["samples"] += stream["samples"]
    if offset != len(raw):
        raise ValueError("GPMF packet sizes do not match the extracted metadata stream")
    return result
