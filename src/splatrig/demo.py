# SPDX-License-Identifier: MIT
"""A deterministic synthetic room for testing data geometry, not hardware quality."""

from __future__ import annotations

import math
from pathlib import Path
import struct
import zlib

from .clock import Clock
from .geometry import Pose, Trajectory, rotate
from .panorama import equirect_direction
from .session import write_json, write_jsonl


def png_bytes(width: int, height: int, rows) -> bytes:
    def chunk(kind, payload):
        return (struct.pack(">I", len(payload)) + kind + payload +
                struct.pack(">I", zlib.crc32(kind + payload) & 0xffffffff))
    raw = b"".join(b"\0" + bytes(row) for row in rows)
    if len(raw) != height*(1+width*3):
        raise ValueError("RGB row dimensions do not match the PNG")
    return (b"\x89PNG\r\n\x1a\n" +
            chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) +
            chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def rig_pose(seconds: float) -> Pose:
    angle = seconds*2*math.pi/8
    x, y = 2.3*math.cos(angle), 1.8*math.sin(angle)
    yaw = math.atan2(-y, -x)
    return Pose((x, y, 1.4), (math.cos(yaw/2), 0, 0, math.sin(yaw/2)))


def box_hit(origin, direction, lower, upper):
    near, far = -math.inf, math.inf
    for o, d, lo, hi in zip(origin, direction, lower, upper):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        left, right = (lo-o)/d, (hi-o)/d
        near, far = max(near, min(left, right)), min(far, max(left, right))
    distance = near if near > 1e-7 else far
    if far < near or distance <= 1e-7:
        return None
    return distance


def scene_hit(origin, direction):
    room = box_hit(origin, direction, (-4, -3, 0), (4, 3, 3))
    box = box_hit(origin, direction, (-0.65, -0.65, 0), (0.65, 0.65, 1.1))
    distance = min(v for v in (room, box) if v is not None)
    point = tuple(o+distance*d for o, d in zip(origin, direction))
    x, y, z = point
    checker = (math.floor(x*3)+math.floor(y*3)+math.floor(z*3)) % 2
    if box is not None and box == distance:
        color = (210, 135, 64) if checker else (154, 87, 40)
    elif z < 0.001:
        color = (82, 99, 117) if checker else (158, 172, 184)
    elif z > 2.999:
        color = (223, 229, 235)
    elif abs(x) > 3.999:
        color = (46, 132, 153) if checker else (70, 166, 181)
    else:
        color = (169, 188, 204) if checker else (211, 221, 229)
    return point, color


def write_common(destination: Path, rig: dict, note: str):
    """Write metadata, clock, trajectory and timed points shared by both demos."""
    if destination.exists():
        raise ValueError(f"Demo destination already exists: {destination}")
    destination.mkdir(parents=True)
    (destination / "images").mkdir()
    anchor = 1_000_000_000_000_000_000
    clock = Clock(0, anchor, 1.00002, "calibrated")
    write_json(destination / "capture-meta.json", {
        "schema_version": 1, "source": "synthetic", "world_up": "+Z",
        "units": "meters", "camera_convention": "opencv", "point_frame": "lidar",
        "trajectory_status": "final", "pose_revision": 1, "max_pose_gap_ns": 100_000_000,
        "note": note})
    write_json(destination / "rig.json", {
        "status": "calibrated", "T_rig_lidar": Pose.identity().to_dict(), **rig})
    write_json(destination / "clock.json", clock.to_dict())
    records = [{"timestamp_ns": anchor+i*50_000_000, "revision": 1,
                **rig_pose(i*0.05).to_dict()} for i in range(165)]
    write_jsonl(destination / "trajectory.jsonl", records)
    trajectory = Trajectory(records, 100_000_000)

    def samples():
        for scan in range(32):
            for azimuth in range(60):
                for elevation in range(9):
                    offset = (azimuth*9+elevation)*80_000_000//540
                    timestamp = anchor+scan*250_000_000+offset
                    pose = trajectory.at(timestamp)
                    yaw = azimuth*2*math.pi/60
                    pitch = math.radians(-29+elevation*59/8)
                    ray = (math.cos(pitch)*math.cos(yaw),
                           math.cos(pitch)*math.sin(yaw), math.sin(pitch))
                    point, color = scene_hit(pose.translation,
                                             rotate(pose.quaternion_wxyz, ray))
                    yield {"timestamp_ns": timestamp,
                           "xyz": pose.inverse().transform(point), "rgb": color}
    write_jsonl(destination / "points.jsonl", samples())
    return clock, trajectory, len(records)


# Synthetic camera and sphere frames: OpenCV +Z looks along rig +X, +Y along rig -Z.
T_RIG_SENSOR = Pose((0.08, 0, 0.12), (0.5, -0.5, 0.5, -0.5))


def create_demo(destination: Path):
    camera = {"id": 1, "name": "synthetic_front", "model": "PINHOLE",
              "width": 160, "height": 120, "params": [92, 92, 80, 60],
              "T_rig_camera": T_RIG_SENSOR.to_dict()}
    clock, trajectory, samples = write_common(
        destination, {"cameras": [camera]},
        "Synthetic ground truth. No GoPro, LiDAR, SLAM or hardware benchmark.")
    frames = []
    for index in range(8):
        camera_time = 500_000_000+index*900_000_000
        pose = trajectory.at(clock.map(camera_time)).compose(T_RIG_SENSOR)
        rows = []
        for v in range(camera["height"]):
            row = bytearray()
            for u in range(camera["width"]):
                ray = ((u+0.5-80)/92, (v+0.5-60)/92, 1.0)
                _, color = scene_hit(pose.translation, rotate(pose.quaternion_wxyz, ray))
                row.extend(color)
            rows.append(row)
        name = f"images/front_{index+1:04d}.png"
        (destination / name).write_bytes(png_bytes(camera["width"], camera["height"], rows))
        frames.append({"id": index+1, "camera_id": 1, "timestamp_ns": camera_time,
                       "capture_id": index+1, "image_path": name})
    write_jsonl(destination / "frames.jsonl", frames)
    return {"source": "synthetic", "frames": len(frames), "trajectory_samples": samples,
            "raw_points": 32*60*9, "path": str(destination)}


def create_panorama_demo(destination: Path, count: int = 8, width: int = 256):
    """Synthetic equirectangular captures; input for the panorama-views command."""
    if not 1 <= count <= 8 or width < 16 or width % 2:
        raise ValueError("Panorama demo needs 1-8 panoramas and an even width >= 16")
    height = width // 2
    rig = {"cameras": [], "panorama": {"projection": "equirectangular",
                                       "T_rig_sphere": T_RIG_SENSOR.to_dict()}}
    clock, trajectory, samples = write_common(
        destination, rig,
        "Synthetic equirectangular ground truth. No GoPro stitching, rolling shutter, "
        "LiDAR, SLAM or hardware benchmark.")
    records = []
    for index in range(count):
        camera_time = 500_000_000+index*900_000_000
        pose = trajectory.at(clock.map(camera_time)).compose(T_RIG_SENSOR)
        rows = []
        for v in range(height):
            row = bytearray()
            for u in range(width):
                ray = equirect_direction(u+0.5, v+0.5, width, height)
                _, color = scene_hit(pose.translation, rotate(pose.quaternion_wxyz, ray))
                row.extend(color)
            rows.append(row)
        name = f"images/panorama_{index+1:04d}.png"
        (destination / name).write_bytes(png_bytes(width, height, rows))
        records.append({"id": index+1, "timestamp_ns": camera_time, "image_path": name})
    write_jsonl(destination / "panoramas.jsonl", records)
    return {"source": "synthetic", "panoramas": count, "panorama_size": [width, height],
            "trajectory_samples": samples, "raw_points": 32*60*9, "path": str(destination)}
