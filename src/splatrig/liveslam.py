# SPDX-License-Identifier: MIT
"""Live GLIM preview for the capture console: route, current pose and a voxel map.

The console republishes Livox frames as PointCloud2 with per-point time, starts
glim_rosnode, and accumulates GLIM's corrected per-scan clouds into a voxel map
that keeps the first point of each voxel. The live map is a preview: earlier
voxels are not moved by later loop closures, so final data comes from the
offline run of the recorded bag.
"""

from __future__ import annotations

from collections import deque
import math
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import tempfile
import threading
import time

from .geometry import Pose

MAP_POINT = struct.Struct("<ffff")
GLIM_POINTS_TOPIC = "/splatrig/glim_points"


def cloud_points(message):
    """Yield (x, y, z, intensity) from a PointCloud2 with float32 x/y/z(/intensity)."""
    offsets = {field.name: (field.offset, field.datatype) for field in message.fields}
    if any(offsets.get(name, (0, 0))[1] != 7 for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 needs float32 x, y and z")
    data, step = bytes(message.data), message.point_step
    intensity = offsets.get("intensity")
    if intensity is not None and intensity[1] != 7:
        intensity = None
    if step == 16 and [offsets[k][0] for k in "xyz"] == [0, 4, 8] and \
            intensity is not None and intensity[0] == 12:
        yield from MAP_POINT.iter_unpack(data[:16 * (len(data) // 16)])
        return
    one = struct.Struct("<f")
    for start in range(0, len(data) - step + 1, step):
        yield (*(one.unpack_from(data, start + offsets[k][0])[0] for k in "xyz"),
               one.unpack_from(data, start + intensity[0])[0] if intensity else 0.0)


class SlamState:
    """Trajectory and first-point voxel map from GLIM's live topics."""

    def __init__(self, voxel=0.05, max_voxels=3_000_000, camera: Pose | None = None):
        self.lock = threading.Lock()
        self.voxel = voxel
        self.max_voxels = max_voxels
        self.camera = camera  # T_rig_sphere (rig = GLIM's IMU frame), if calibrated
        self.voxels = set()
        self.map = bytearray()
        self.dropped = 0
        self.route = bytearray()
        self.route_last = None
        self.pose = None
        self.odom_times = deque(maxlen=50)
        self.scans = 0

    def add_odometry(self, position, quaternion_wxyz, received=None):
        with self.lock:
            self.pose = Pose(position, quaternion_wxyz)
            self.odom_times.append(time.time() if received is None else received)
            if self.route_last is None or math.dist(self.route_last, position) >= 0.02:
                self.route += struct.pack("<fff", *position)
                self.route_last = tuple(position)

    def add_scan(self, points):
        new = bytearray()
        with self.lock:
            for x, y, z, i in points:
                key = (math.floor(x / self.voxel), math.floor(y / self.voxel),
                       math.floor(z / self.voxel))
                if key in self.voxels:
                    continue
                if len(self.voxels) >= self.max_voxels:
                    self.dropped += 1
                    continue
                self.voxels.add(key)
                new += MAP_POINT.pack(x, y, z, i)
            self.map += new
            self.scans += 1

    def map_payload(self, max_points=400_000) -> bytes:
        with self.lock:
            data = bytes(self.map)
        count = len(data) // MAP_POINT.size
        stride = max(1, -(-count // max_points))
        if stride > 1:
            data = b"".join(data[i:i + MAP_POINT.size]
                            for i in range(0, len(data), MAP_POINT.size * stride))
        return struct.pack("<I", len(data) // MAP_POINT.size) + data

    def route_payload(self) -> bytes:
        with self.lock:
            data = bytes(self.route)
        return struct.pack("<I", len(data) // 12) + data

    def status(self, now=None) -> dict:
        now = time.time() if now is None else now
        with self.lock:
            times, pose = list(self.odom_times), self.pose
            voxels, dropped, scans = len(self.voxels), self.dropped, self.scans
        recent = [t for t in times if now - t <= 2]
        rate = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and \
            recent[-1] > recent[0] else 0.0
        result = {"odom_hz": round(rate, 2), "map_voxels": voxels, "map_dropped": dropped,
                  "scans": scans, "odom_age_s": round(now - times[-1], 2) if times else None,
                  "pose": pose.to_dict() if pose else None}
        if pose and self.camera:
            camera = pose.compose(self.camera)
            result["camera"] = camera.to_dict()
        return result


class GlimProcess:
    """glim_rosnode in a temporary folder, with its log kept for diagnosis."""

    def __init__(self):
        from .glim import write_glim_config
        self.folder = Path(tempfile.mkdtemp(prefix="splatrig-glim-live-"))
        write_glim_config(self.folder / "config", points_topic=GLIM_POINTS_TOPIC, publish=True)
        self.log = (self.folder / "glim.log").open("wb")
        self.process = subprocess.Popen(
            ["ros2", "run", "glim_ros", "glim_rosnode", "--ros-args",
             "-p", f"config_path:={self.folder / 'config'}",
             "-p", f"dump_path:={self.folder / 'dump'}"],
            stdout=self.log, stderr=subprocess.STDOUT, env=dict(os.environ),
            start_new_session=True)

    def running(self) -> bool:
        return self.process.poll() is None

    def stop(self):
        if self.running():
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(30)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.log.close()
        shutil.rmtree(self.folder, ignore_errors=True)
