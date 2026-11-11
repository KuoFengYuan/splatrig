# SPDX-License-Identifier: MIT
"""Fixed PINHOLE views cut from equirectangular panoramas.

The core computes every intrinsic and extrinsic; an external ffmpeg process only
resamples pixels. Its orientation convention is measured before each conversion.
"""

from __future__ import annotations

import array
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys

from .export import new_output
from .geometry import Pose, multiply, nanoseconds, rotate, vector
from .session import local_file, png_size, positive_id, read_json, read_jsonl, write_json, \
    write_jsonl

PRESETS = {
    "cube": [("front", 0, 0), ("right", 90, 0), ("back", 180, 0), ("left", -90, 0),
             ("up", 0, 90), ("down", 0, -90)],
    "ring8": [(f"yaw{angle:03d}", angle, 0) for angle in range(0, 360, 45)],
}
INTERPOLATIONS = ("line", "cubic", "lanczos")
# Measured v360 bilinear noise is about 0.4 input pixels at worst with no mean bias;
# a half-pixel or axis convention error exceeds these limits.
PROBE_MAX_ERROR_PX = 0.6
PROBE_MAX_BIAS_PX = 0.25
NOT_MODELED = ["rolling_shutter", "stitching_parallax", "per_view_exposure_time",
               "lens_masks", "time_varying_stabilization"]


def equirect_direction(u: float, v: float, width: int, height: int):
    """Unit ray in sphere frame S for continuous pixel coordinates (centers at +0.5).

    S uses OpenCV axes: +Z is the image's center column at the horizon, +X is
    90 degrees to its right and +Y points down.
    """
    longitude = u / width * 2 * math.pi - math.pi
    latitude = math.pi / 2 - v / height * math.pi
    return (math.cos(latitude) * math.sin(longitude), -math.sin(latitude),
            math.cos(latitude) * math.cos(longitude))


@dataclass(frozen=True)
class View:
    name: str
    yaw_deg: float
    pitch_deg: float
    roll_deg: float = 0.0

    def __post_init__(self):
        if not isinstance(self.name, str) or not re.fullmatch(r"[a-z0-9][a-z0-9_-]{0,31}",
                                                              self.name):
            raise ValueError("View names use 1-32 lowercase letters, digits, - or _")
        yaw, pitch, roll = vector((self.yaw_deg, self.pitch_deg, self.roll_deg), 3)
        if not -90 <= pitch <= 90 or not -180 <= roll <= 180:
            raise ValueError(f"View {self.name}: pitch must be in [-90, 90] and "
                             "roll in [-180, 180] degrees")
        yaw = (yaw + 180) % 360 - 180
        object.__setattr__(self, "yaw_deg", 180.0 if yaw == -180 else yaw)
        object.__setattr__(self, "pitch_deg", pitch)
        object.__setattr__(self, "roll_deg", roll)

    def rotation(self):
        """Quaternion (wxyz) of T_sphere_view: Ry(yaw) * Rx(pitch) * Rz(roll)."""
        def axis(index, degrees):
            half = math.radians(degrees) / 2
            q = [math.cos(half), 0.0, 0.0, 0.0]
            q[index] = math.sin(half)
            return tuple(q)
        return multiply(multiply(axis(2, self.yaw_deg), axis(1, self.pitch_deg)),
                        axis(3, self.roll_deg))

    def ffmpeg_filter(self, size: int, fov_deg: float, interp: str) -> str:
        number = lambda value: format(value, ".12g")
        return (f"v360=input=e:output=flat:yaw={number(self.yaw_deg)}"
                f":pitch={number(self.pitch_deg)}:roll={number(self.roll_deg)}"
                f":h_fov={number(fov_deg)}:v_fov={number(fov_deg)}"
                f":w={size}:h={size}:interp={interp}")


def load_views(layout: str | None = None, views_file: Path | None = None) -> list[View]:
    if views_file is not None:
        data = read_json(views_file)
        if not isinstance(data, list):
            raise ValueError("Views file must contain a JSON list")
        views = [View(item["name"], item["yaw_deg"], item["pitch_deg"],
                      item.get("roll_deg", 0.0)) for item in data]
    else:
        if layout not in PRESETS:
            raise ValueError(f"Unknown view layout: {layout}")
        views = [View(name, yaw, pitch) for name, yaw, pitch in PRESETS[layout]]
    if not views or len({view.name for view in views}) != len(views):
        raise ValueError("Views must be nonempty with unique names")
    return views


def pinhole_params(size: int, fov_deg: float):
    if isinstance(size, bool) or not isinstance(size, int) or not 8 <= size <= 16384:
        raise ValueError("View size must be an integer from 8 to 16384 pixels")
    if not math.isfinite(fov_deg) or not 1 <= fov_deg <= 170:
        raise ValueError("Field of view must be between 1 and 170 degrees")
    focal = size / 2 / math.tan(math.radians(fov_deg) / 2)
    return [focal, focal, size / 2, size / 2]


def run_tool(command, stdin: bytes | None = None) -> bytes:
    try:
        process = subprocess.run(command, input=stdin, capture_output=True, check=False)
    except FileNotFoundError as error:
        raise ValueError(f"{command[0]} is unavailable; install ffmpeg or pass its path") \
            from error
    if process.returncode:
        message = process.stderr.decode("utf-8", "replace").strip().splitlines()
        raise ValueError(f"{Path(command[0]).name} failed: {message[-1] if message else ''}")
    return process.stdout


def probe_ffmpeg(ffmpeg: str = "ffmpeg") -> dict:
    """Measure the installed v360 orientation and pixel-center convention.

    A panorama whose 16-bit pixels encode their own ray direction is resampled
    into an oblique view; each output pixel is compared with this module's model.
    Values stay within [0.1, 0.9] to avoid 16-bit interpolation overflow.
    """
    width, height, size, fov = 512, 256, 48, 90.0
    view = View("probe", 37, -23, 11)
    samples = array.array("H")
    for v in range(height):
        for u in range(width):
            ray = equirect_direction(u + 0.5, v + 0.5, width, height)
            samples.extend(round((0.5 + 0.4 * c) * 65535) for c in ray)
    if sys.byteorder != "little":
        samples.byteswap()
    output = run_tool([ffmpeg, "-v", "error", "-f", "rawvideo", "-pix_fmt", "rgb48le",
                       "-s", f"{width}x{height}", "-i", "pipe:0",
                       "-vf", view.ffmpeg_filter(size, fov, "line"),
                       "-f", "rawvideo", "-pix_fmt", "rgb48le", "pipe:1"],
                      samples.tobytes())
    result = array.array("H")
    result.frombytes(output)
    if sys.byteorder != "little":
        result.byteswap()
    if len(result) != size * size * 3:
        raise ValueError("ffmpeg probe returned an unexpected image size")
    fx, fy, cx, cy = pinhole_params(size, fov)
    q = view.rotation()
    pixel = 2 * math.pi / width
    worst, bias, count = 0.0, [0.0, 0.0], size * size

    def unit(values):
        norm = math.dist(values, (0, 0, 0))
        return [value / norm for value in values]

    for v in range(size):
        for u in range(size):
            index = (v * size + u) * 3
            seen = unit([(result[index + k] / 65535 - 0.5) / 0.4 for k in range(3)])
            expected = unit(rotate(q, ((u + 0.5 - cx) / fx, (v + 0.5 - cy) / fy, 1.0)))
            dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(seen, expected))))
            worst = max(worst, math.acos(dot) / pixel)
            longitude = (math.atan2(seen[0], seen[2]) - math.atan2(expected[0], expected[2])
                         + math.pi) % (2 * math.pi) - math.pi
            bias[0] += longitude * math.hypot(expected[0], expected[2]) / pixel / count
            bias[1] += (math.asin(-seen[1]) - math.asin(-expected[1])) / pixel / count
    version = run_tool([ffmpeg, "-version"]).decode("utf-8", "replace").splitlines()
    report = {"ffmpeg_version": version[0] if version else "unknown",
              "probe_input_width": width,
              "probe_max_error_px": round(worst, 4),
              "probe_mean_bias_px": [round(value, 4) for value in bias],
              "probe_limits_px": {"max_error": PROBE_MAX_ERROR_PX,
                                  "mean_bias": PROBE_MAX_BIAS_PX}}
    if worst > PROBE_MAX_ERROR_PX or max(map(abs, bias)) > PROBE_MAX_BIAS_PX:
        raise ValueError(f"ffmpeg v360 convention differs from SplatRig's model "
                         f"(max error {worst:.3f}, bias {bias[0]:.3f}/{bias[1]:.3f} input "
                         "pixels); refusing to convert")
    return report


def image_size(ffprobe: str, path: Path):
    output = run_tool([ffprobe, "-v", "error", "-select_streams", "v:0", "-show_entries",
                       "stream=width,height", "-of", "json", str(path)])
    streams = json.loads(output).get("streams") or [{}]
    width, height = streams[0].get("width"), streams[0].get("height")
    if not isinstance(width, int) or not isinstance(height, int):
        raise ValueError(f"{path.name}: unreadable image dimensions")
    return width, height


def panorama_views(source: Path, destination: Path, *, layout="cube", views_file=None,
                   size=1024, fov_deg=90.0, interp="lanczos", ffmpeg="ffmpeg",
                   ffprobe="ffprobe") -> dict:
    source = source.resolve()
    views = load_views(None if views_file else layout, views_file)
    params = pinhole_params(size, fov_deg)
    if interp not in INTERPOLATIONS:
        raise ValueError(f"Interpolation must be one of {', '.join(INTERPOLATIONS)}")
    rig = read_json(source / "rig.json")
    panorama = rig.get("panorama")
    if not isinstance(panorama, dict) or panorama.get("projection") != "equirectangular":
        raise ValueError("rig.json needs panorama.projection=equirectangular")
    if rig.get("cameras"):
        raise ValueError("Panorama sources must not already define PINHOLE cameras")
    if (source / "frames.jsonl").exists():
        raise ValueError("Panorama sources must not contain frames.jsonl")
    if rig.get("status") not in {"uncalibrated", "calibrated"}:
        raise ValueError("Rig status must be uncalibrated or calibrated")
    T_rig_sphere = Pose.from_dict(panorama["T_rig_sphere"])
    records, ids = [], set()
    for record in read_jsonl(source / "panoramas.jsonl"):
        panorama_id = positive_id(record["id"])
        if panorama_id in ids:
            raise ValueError(f"Duplicate panorama ID {panorama_id}")
        if (panorama_id - 1) * len(views) + len(views) >= 2**31:
            raise ValueError("Panorama IDs are too large for per-view frame IDs")
        ids.add(panorama_id)
        records.append({"id": panorama_id, "timestamp_ns": nanoseconds(record["timestamp_ns"]),
                        "path": local_file(source, record["image_path"]),
                        "image_path": record["image_path"]})
    if not records:
        raise ValueError("No panoramas listed")
    for item in records:
        width, height = image_size(ffprobe, item["path"])
        if width != 2 * height:
            raise ValueError(f"{item['image_path']}: equirectangular input must be 2:1, "
                             f"got {width}x{height}")
    probe = probe_ffmpeg(ffmpeg)
    cameras = [{"id": index, "name": view.name, "model": "PINHOLE", "width": size,
                "height": size, "params": params,
                "T_rig_camera": T_rig_sphere.compose(
                    Pose((0, 0, 0), view.rotation())).to_dict()}
               for index, view in enumerate(views, 1)]
    with new_output(destination, source) as root:
        for name in ["capture-meta.json", "clock.json", "trajectory.jsonl"]:
            shutil.copyfile(source / name, root / name)
        points = [name for name in ["points.jsonl", "points.bin"] if (source / name).exists()]
        if len(points) != 1:
            raise ValueError("Panorama sources need exactly one of points.jsonl or points.bin")
        shutil.copyfile(source / points[0], root / points[0])
        write_json(root / "rig.json", {**rig, "cameras": cameras})
        frames = []
        for view in views:
            (root / "images" / view.name).mkdir(parents=True)
        for item in records:
            labels = [f"[v{index}]" for index in range(len(views))]
            graph = f"[0:v]format=rgb24,split={len(views)}{''.join(labels)};" + ";".join(
                f"{label}{view.ffmpeg_filter(size, fov_deg, interp)}[o{index}]"
                for index, (label, view) in enumerate(zip(labels, views)))
            command = [ffmpeg, "-v", "error", "-nostdin", "-i", str(item["path"]),
                       "-filter_complex", graph]
            outputs = []
            for index, view in enumerate(views):
                name = f"images/{view.name}/{item['id']:08d}.png"
                outputs.append(name)
                command += ["-map", f"[o{index}]", "-frames:v", "1", "-pix_fmt", "rgb24",
                            "-n", str(root / name)]
            run_tool(command)
            for index, name in enumerate(outputs):
                if png_size(root / name) != (size, size):
                    raise ValueError(f"{name}: ffmpeg wrote unexpected dimensions")
                frames.append({"id": (item["id"] - 1) * len(views) + index + 1,
                               "camera_id": index + 1, "timestamp_ns": item["timestamp_ns"],
                               "capture_id": item["id"], "image_path": name,
                               "source_panorama_path": item["image_path"]})
        write_jsonl(root / "frames.jsonl", frames)
        report = {"source_capture": str(source), "panoramas": len(records),
                  "views": [{"name": view.name, "yaw_deg": view.yaw_deg,
                             "pitch_deg": view.pitch_deg, "roll_deg": view.roll_deg}
                            for view in views],
                  "frames": len(frames), "size": size, "fov_deg": fov_deg,
                  "interp": interp, "rig_status": rig["status"],
                  "provisional": rig["status"] != "calibrated", **probe,
                  "not_modeled": NOT_MODELED}
        write_json(root / "panorama-views.json", report)
    return report
