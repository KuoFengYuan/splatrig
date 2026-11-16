# SPDX-License-Identifier: MIT
"""Image masks for the operator and the mount, from LiDAR returns that move with the rig.

While the rig moves or turns, the scene passes by, but the person holding the rig
and the mount stay in the same place relative to the sensors. A LiDAR direction
whose returns during motion are mostly closer than `near_m` is therefore treated
as self-occluded. Those returns are carried into the camera's sphere frame
(OpenCV axes), and their directions, widened by `dilate_deg`, mark the pixels to
ignore. Directions the LiDAR cannot see, such as the hands below the sensor, are
not covered; the default view layout leaves out the downward view for that reason.

Nearby scene stays in the same LiDAR direction when the rig only turns a little in
place, so nothing is estimated unless the rig travelled `min_extent_m` (the span of
its positions while moving). A narrow corridor walked straight can still look like
part of the rig; `max_fraction` in the pipeline rejects implausibly large masks.

Masks follow the COLMAP and nerfstudio convention: 0 = ignore, 255 = use.
"""

from __future__ import annotations

from collections import Counter
import math
import os
from pathlib import Path
import struct

from .demo import png_bytes
from .geometry import Pose, Trajectory, rotate
from .keyframes import turn_rate_dps
from .pointlog import RECORD, read_header

BIN_DEG = 2.0
BUCKET_NS = 100_000_000


def direction_bin(azimuth_deg: float, elevation_deg: float, size=BIN_DEG) -> tuple[int, int]:
    return (int(((azimuth_deg + 180.0) % 360.0) // size),
            min(int((elevation_deg + 90.0) // size), int(180 / size) - 1))


def lidar_direction(x, y, z):
    """Azimuth about +Z from +X and elevation from the XY plane, degrees."""
    return math.degrees(math.atan2(y, x)), math.degrees(math.atan2(z, math.hypot(x, y)))


def sphere_direction(x, y, z):
    """Azimuth from +Z toward +X and elevation toward -Y (OpenCV sphere frame), degrees."""
    norm = math.sqrt(x * x + y * y + z * z)
    return math.degrees(math.atan2(x, z)), math.degrees(math.asin(max(-1.0, min(1.0, -y / norm))))


def strided_points(path: Path, stride: int, chunk_records=65536):
    """Every stride-th (timestamp_ns, x, y, z) of a point log."""
    _, start, count = read_header(path)
    step = struct.Struct(RECORD.format + f"{RECORD.size * (stride - 1)}x")
    with path.open("rb") as stream:
        stream.seek(start)
        remaining = count
        while remaining:
            take = min(remaining, chunk_records * stride)
            block = stream.read(RECORD.size * take)
            remaining -= take
            usable = len(block) - len(block) % step.size
            for record in step.iter_unpack(block[:usable]):
                yield record[:4]
            tail = block[usable:]
            if len(tail) >= RECORD.size:  # one more sampled record before the short remainder
                yield RECORD.unpack_from(tail)[:4]


def estimate(points: Path, trajectory: Trajectory, T_lidar_sphere: Pose, *, near_m=0.8,
             min_ratio=0.6, min_returns=20, min_speed=0.1, min_turn_dps=15.0, stride=10,
             dilate_deg=4.0, min_extent_m=1.0) -> dict:
    """Self-occluded sphere directions from a LiDAR point log (LiDAR frame) and trajectory."""
    totals, near, sums, motion = Counter(), Counter(), {}, {}
    low, high = [math.inf] * 3, [-math.inf] * 3  # rig positions while moving
    first, last = trajectory.times[0], trajectory.times[-1]
    near_squared = near_m ** 2
    for timestamp, x, y, z in strided_points(points, stride):
        bucket = timestamp // BUCKET_NS
        moving = motion.get(bucket)
        if moving is None:
            middle = bucket * BUCKET_NS + BUCKET_NS // 2
            start, end = max(first, middle - BUCKET_NS // 2), min(last, middle + BUCKET_NS // 2)
            try:
                speed = (math.dist(trajectory.at(start).translation, trajectory.at(end).translation)
                         / ((end - start) / 1e9)) if end > start else 0.0
                moving = end > start and (speed >= min_speed or
                                          turn_rate_dps(trajectory, middle) >= min_turn_dps)
                if moving:
                    position = trajectory.at(start).translation
                    low = [min(a, b) for a, b in zip(low, position)]
                    high = [max(a, b) for a, b in zip(high, position)]
            except ValueError:  # outside the trajectory or across a pose gap
                moving = False
            motion[bucket] = moving
        if not moving:
            continue
        key = direction_bin(*lidar_direction(x, y, z))
        totals[key] += 1
        if x * x + y * y + z * z < near_squared:
            near[key] += 1
            total = sums.setdefault(key, [0.0, 0.0, 0.0])
            total[0] += x
            total[1] += y
            total[2] += z
    extent = math.dist(low, high) if low[0] <= high[0] else 0.0
    estimated = extent >= min_extent_m
    self_bins = [key for key, count in near.items()
                 if totals[key] >= min_returns and count / totals[key] >= min_ratio] \
        if estimated else []
    T_sphere_lidar = T_lidar_sphere.inverse()
    rows, columns, reach = int(180 / BIN_DEG), int(360 / BIN_DEG), math.ceil(dilate_deg / BIN_DEG)
    masked = set()
    for key in self_bins:
        mean = [value / near[key] for value in sums[key]]
        azimuth, elevation = sphere_direction(*T_sphere_lidar.transform(mean))
        column, row = direction_bin(azimuth, elevation)
        for r in range(max(0, row - reach), min(rows, row + reach + 1)):
            center = (r + 0.5) * BIN_DEG - 90.0
            wide = min(columns // 2, math.ceil(reach / max(math.cos(math.radians(center)), 1e-3)))
            for c in range(column - wide, column + wide + 1):
                masked.add((c % columns, r))
    moving_returns = sum(totals.values())
    return {"bin_deg": BIN_DEG, "bins": sorted(masked), "self_lidar_bins": len(self_bins),
            "moving_returns_sampled": moving_returns, "stride": stride, "near_m": near_m,
            "min_ratio": min_ratio, "dilate_deg": dilate_deg,
            "motion_extent_m": round(extent, 3), "min_extent_m": min_extent_m,
            "estimated": estimated,
            "sphere_fraction": round(sum(math.cos(math.radians((r + 0.5) * BIN_DEG - 90.0))
                                         for _, r in masked) * math.radians(BIN_DEG) ** 2
                                     / (4 * math.pi), 4)}


def view_mask(bins: set, q_sphere_camera, width: int, height: int, params, step=8,
              size=BIN_DEG) -> tuple[bytes, float]:
    """Grayscale PNG for one PINHOLE camera (0 = ignore) and its ignored fraction.

    Evaluated on a step x step pixel grid and repeated, which is ample for masks
    built from 2-degree bins.
    """
    fx, fy, cx, cy = params
    rows, ignored = [], 0
    for top in range(0, height, step):
        v = min(top + step / 2, height - 0.5)
        row = bytearray()
        for left in range(0, width, step):
            u = min(left + step / 2, width - 0.5)
            ray = rotate(q_sphere_camera, ((u - cx) / fx, (v - cy) / fy, 1.0))
            value = 0 if direction_bin(*sphere_direction(*ray), size) in bins else 255
            span = min(step, width - left)
            row += bytes([value]) * span
            if not value:
                ignored += span * min(step, height - top)
        rows.extend([bytes(row)] * min(step, height - top))
    return png_bytes(width, height, rows, channels=1), ignored / (width * height)


def write_masks(dataset: Path, records, cameras: dict, T_rig_sphere: Pose, bins) -> dict:
    """masks/<image name> for every exported image; images of one camera share a file."""
    bins = {tuple(b) for b in bins}
    folder = dataset / "masks"
    folder.mkdir()
    shared, fractions = {}, {}
    for camera_id, camera in sorted(cameras.items()):
        T_sphere_camera = T_rig_sphere.inverse().compose(camera.T_rig_camera)
        data, fraction = view_mask(bins, T_sphere_camera.quaternion_wxyz, camera.width,
                                   camera.height, camera.params)
        shared[camera_id] = folder / f".camera-{camera_id}.png"
        shared[camera_id].write_bytes(data)
        fractions[camera_id] = round(fraction, 4)
    count = 0
    for record in records:
        target = folder / Path(record["image_path"]).name
        try:
            os.link(shared[record["camera_id"]], target)
        except OSError:
            target.write_bytes(shared[record["camera_id"]].read_bytes())
        count += 1
    for path in shared.values():
        path.unlink()
    return {"masks": count, "ignored_fraction_by_camera": fractions,
            "convention": "0 = ignore, 255 = use; same file name as the image"}
