# SPDX-License-Identifier: MIT
"""Dataset preview data for the console: coloured points, cameras and thumbnails.

The console shows a finished dataset in its own view: the coloured initialization
points, where each image was taken and which way it looked, and a strip of
thumbnails. Points are packed as float32 x, y, z and the colour as one float
(r * 65536 + g * 256 + b, exact in float32), so the page can draw them directly.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import struct
import subprocess

from .geometry import Pose, rotate

IMAGE = re.compile(r"[0-9]{8}\.png")
POINT = struct.Struct("<ffff")


def points_payload(dataset: Path, max_points=400_000) -> bytes:
    """Coloured points from dataset/points.ply (ASCII x y z r g b), subsampled."""
    with (dataset / "points.ply").open(encoding="ascii") as stream:
        count = 0
        for line in stream:
            if line.startswith("element vertex"):
                count = int(line.split()[2])
            if line.strip() == "end_header":
                break
        stride = max(1, -(-count // max_points))
        data = bytearray()
        for index, line in enumerate(stream):
            if index % stride:
                continue
            x, y, z, r, g, b = line.split()
            data += POINT.pack(float(x), float(y), float(z),
                               int(r) * 65536 + int(g) * 256 + int(b))
    return struct.pack("<I", len(data) // POINT.size) + bytes(data)


def fields_of_view(dataset: Path) -> dict:
    """Vertical field of view (degrees) of each PINHOLE camera in sparse/0/cameras.txt."""
    import math
    found = {}
    path = dataset / "sparse/0/cameras.txt"
    if path.is_file():
        for line in path.read_text(encoding="ascii").splitlines():
            parts = line.split()
            if len(parts) >= 8 and parts[1] == "PINHOLE":
                height, fy = float(parts[3]), float(parts[5])
                found[int(parts[0])] = round(math.degrees(2 * math.atan(height / 2 / fy)), 3)
    return found


def cameras(dataset: Path) -> list[dict]:
    """Each image's camera center, viewing direction, image-up and field of view."""
    fov = fields_of_view(dataset)
    found = []
    with (dataset / "image-poses.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            record = json.loads(line)
            pose = Pose.from_dict(record["T_world_camera"])
            q = pose.quaternion_wxyz
            found.append({"image": Path(record["image_path"]).name,
                          "camera_id": record["camera_id"],
                          "center": [round(v, 4) for v in pose.translation],
                          "forward": [round(v, 4) for v in rotate(q, (0.0, 0.0, 1.0))],
                          "up": [round(-v, 4) for v in rotate(q, (0.0, 1.0, 0.0))],
                          "fov_deg": fov.get(record["camera_id"])})
    return found


def thumbnail(dataset: Path, image: str, cache: Path, width=192, masked=True,
              ffmpeg="ffmpeg") -> bytes:
    """A small JPEG of one dataset image; pixels its mask ignores are tinted red."""
    if not IMAGE.fullmatch(image):
        raise ValueError("Unknown image")
    source = dataset / "images" / image
    if not source.is_file():
        raise ValueError("Unknown image")
    mask = dataset / "masks" / image
    target = cache / f"{image}{'-m' if masked and mask.is_file() else ''}-{width}.jpg"
    if target.is_file() and target.stat().st_mtime >= source.stat().st_mtime:
        return target.read_bytes()
    cache.mkdir(parents=True, exist_ok=True)
    if masked and mask.is_file():
        command = [ffmpeg, "-v", "error", "-nostdin", "-y", "-i", str(source), "-i", str(mask),
                   "-filter_complex",
                   f"[0]scale={width}:-2,format=gbrp[i];"
                   f"[1]scale={width}:-2,format=gray,format=gbrp[m];"
                   "[i]split[a][b];[b]colorchannelmixer=rr=.75:gg=.18:bb=.18[t];"
                   "[t][a][m]maskedmerge", "-frames:v", "1", "-q:v", "4", str(target)]
    else:
        command = [ffmpeg, "-v", "error", "-nostdin", "-y", "-i", str(source), "-vf",
                   f"scale={width}:-2", "-frames:v", "1", "-q:v", "4", str(target)]
    result = subprocess.run(command, capture_output=True)
    if result.returncode or not target.is_file():
        raise ValueError("Could not make a thumbnail")
    return target.read_bytes()
