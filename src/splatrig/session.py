# SPDX-License-Identifier: MIT
"""Load one pose revision and use it for both image association and cloud replay."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import struct

from .clock import Clock
from .geometry import Pose, Trajectory, nanoseconds, vector
from .pointlog import iter_points


def reject_nonfinite(value):
    raise ValueError(f"Non-finite JSON number: {value}")


def read_json(path: Path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream, parse_constant=reject_nonfinite)


def write_json(path: Path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def read_jsonl(path: Path):
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                value = json.loads(line, parse_constant=reject_nonfinite)
                if not isinstance(value, dict):
                    raise ValueError("Expected a JSON object")
                yield value
            except (ValueError, TypeError) as error:
                raise ValueError(f"{path.name}:{line_number}: {error}") from error


def write_jsonl(path: Path, records):
    with path.open("x", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")


def positive_id(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 < value < 2**31:
        raise ValueError("IDs must be positive 32-bit integers")
    return value


def local_file(root: Path, name: str) -> Path:
    if not isinstance(name, str) or not name or Path(name).is_absolute():
        raise ValueError("Session file paths must be nonempty and relative")
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError(f"Missing file or path outside the session: {name}")
    return path


def png_size(path: Path):
    with path.open("rb") as stream:
        header = stream.read(33)
    if len(header) != 33 or header[:16] != b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR":
        raise ValueError(f"{path.name}: expected a PNG with an IHDR header")
    return struct.unpack(">II", header[16:24])


@dataclass(frozen=True)
class Camera:
    id: int
    width: int
    height: int
    params: tuple[float, ...]
    T_rig_camera: Pose

    @classmethod
    def from_dict(cls, data):
        if data.get("model") != "PINHOLE":
            raise ValueError("v0.1 accepts rectified PINHOLE camera views")
        params = vector(data["params"], 4)
        if params[0] <= 0 or params[1] <= 0:
            raise ValueError("Camera focal lengths must be positive")
        return cls(positive_id(data["id"]), positive_id(data["width"]),
                   positive_id(data["height"]), params,
                   Pose.from_dict(data["T_rig_camera"]))


DEFAULT_RGB = (180, 190, 210)


def point_samples(root: Path):
    """Yield (timestamp_ns, xyz, rgb) from points.jsonl or the binary points.bin."""
    text, binary = root / "points.jsonl", root / "points.bin"
    if text.exists() and binary.exists():
        raise ValueError("Session must contain points.jsonl or points.bin, not both")
    if binary.exists():
        for timestamp, x, y, z, *_ in iter_points(binary):
            yield nanoseconds(timestamp), vector((x, y, z), 3), DEFAULT_RGB
        return
    for sample in read_jsonl(text):
        rgb = sample.get("rgb", list(DEFAULT_RGB))
        if (len(rgb) != 3 or any(isinstance(c, bool) or not isinstance(c, int) or
                                not 0 <= c <= 255 for c in rgb)):
            raise ValueError("Point RGB must contain three integers in [0, 255]")
        yield nanoseconds(sample["timestamp_ns"]), vector(sample["xyz"], 3), rgb


@dataclass
class Prepared:
    root: Path
    cameras: dict[int, Camera]
    frames: list[dict]
    points: list[tuple]
    report: dict


def prepare(root: Path, *, allow_provisional=False, voxel_size=0.05,
            max_voxels=1_000_000) -> Prepared:
    root = root.resolve()
    if not math.isfinite(voxel_size) or voxel_size <= 0 or max_voxels <= 0:
        raise ValueError("Voxel size and voxel budget must be positive")
    meta = read_json(root / "capture-meta.json")
    if type(meta.get("schema_version")) is not int or meta["schema_version"] != 1:
        raise ValueError("Unsupported capture schema version")
    expected = {"world_up": "+Z", "units": "meters", "camera_convention": "opencv",
                "point_frame": "lidar"}
    for key, value in expected.items():
        if meta.get(key) != value:
            raise ValueError(f"Expected {key}={value}")
    if meta.get("source") not in {"synthetic", "gopro_max2_mid360", "external"}:
        raise ValueError("Unknown session source")
    revision = meta.get("pose_revision")
    if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
        raise ValueError("pose_revision must be a nonnegative integer")
    if meta.get("trajectory_status") not in {"live", "final"}:
        raise ValueError("trajectory_status must be live or final")
    rig = read_json(root / "rig.json")
    if rig.get("status") not in {"uncalibrated", "calibrated"}:
        raise ValueError("Rig status must be uncalibrated or calibrated")
    clock = Clock.from_dict(read_json(root / "clock.json"))
    provisional = (meta["trajectory_status"] != "final" or
                   rig["status"] != "calibrated" or clock.status != "calibrated")
    if provisional and not allow_provisional:
        raise ValueError("Final export requires a final trajectory and calibrated rig/clock; "
                         "--allow-provisional is for diagnostics only")
    cameras = {}
    for data in rig["cameras"]:
        camera = Camera.from_dict(data)
        if camera.id in cameras:
            raise ValueError(f"Duplicate camera ID {camera.id}")
        cameras[camera.id] = camera
    if not cameras:
        raise ValueError("Rig has no cameras")
    T_rig_lidar = Pose.from_dict(rig["T_rig_lidar"])
    poses = list(read_jsonl(root / "trajectory.jsonl"))
    if any(type(p.get("revision")) is not int or p["revision"] != revision for p in poses):
        raise ValueError("Trajectory contains a different pose revision")
    trajectory = Trajectory(poses, meta.get("max_pose_gap_ns", 200_000_000))
    frames, seen_ids, seen_paths = [], set(), set()
    for frame in read_jsonl(root / "frames.jsonl"):
        frame_id = positive_id(frame["id"])
        camera_id = positive_id(frame["camera_id"])
        if frame_id in seen_ids or camera_id not in cameras:
            raise ValueError("Duplicate frame ID or unknown camera ID")
        image = local_file(root, frame["image_path"])
        if image in seen_paths:
            raise ValueError("Each frame must reference a distinct image")
        camera = cameras[camera_id]
        if png_size(image) != (camera.width, camera.height):
            raise ValueError(f"Frame {frame_id}: image dimensions do not match calibration")
        timestamp = clock.map(frame["timestamp_ns"])
        pose = trajectory.at(timestamp).compose(camera.T_rig_camera)
        frames.append({**frame, "reference_timestamp_ns": timestamp,
                       "pose_revision": revision, "T_world_camera": pose.to_dict()})
        seen_ids.add(frame_id)
        seen_paths.add(image)
    if not frames:
        raise ValueError("Session has no image frames")
    frames.sort(key=lambda f: (f["reference_timestamp_ns"], f["id"]))
    voxels, raw_count = {}, 0
    for timestamp, xyz, rgb in point_samples(root):
        pose = trajectory.at(timestamp)
        point = pose.compose(T_rig_lidar).transform(xyz)
        key = tuple(math.floor(v / voxel_size) for v in point)
        if key not in voxels:
            if len(voxels) >= max_voxels:
                raise ValueError("Voxel budget reached; increase voxel size or max_voxels")
            voxels[key] = [0.0, 0.0, 0.0, 0, 0, 0, 0]
        total = voxels[key]
        for i, value in enumerate((*point, *rgb)):
            total[i] += value
        total[6] += 1
        raw_count += 1
    if not raw_count:
        raise ValueError("Session has no LiDAR points")
    points = []
    for total in voxels.values():
        count = total[6]
        points.append(tuple(v/count for v in total[:3]) +
                      tuple(round(v/count) for v in total[3:6]))
    gaps = [b-a for a, b in zip(trajectory.times, trajectory.times[1:])]
    report = {"schema_version": 1, "source": meta["source"],
              "synthetic": meta["source"] == "synthetic",
              "provisional": provisional, "pose_revision": revision,
              "world_up": "+Z", "units": "meters", "camera_convention": "opencv",
              "frames": len(frames), "cameras": len(cameras),
              "trajectory_samples": len(poses), "raw_points": raw_count,
              "voxel_points": len(points), "voxel_size_m": voxel_size,
              "max_pose_gap_ns": max(gaps, default=0),
              "clock": clock.to_dict(),
              "hardware_validated": False,
              "trajectory_estimation": "supplied_ground_truth" if meta["source"] == "synthetic"
                                       else "supplied_by_external_backend",
              "slam_executed_by_core": False,
              "sparse_model": "initialization_only_no_observation_tracks"}
    return Prepared(root, cameras, frames, points, report)
