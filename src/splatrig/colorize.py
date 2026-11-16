# SPDX-License-Identifier: MIT
"""Colour LiDAR map points from the panoramas, for Gaussian-splatting initialization.

Each point takes its colour from the panorama nearest in time to the point's
closest LiDAR return. The LiDAR saw the point from the rig at that moment, so
the camera 10 cm away almost always sees it too; the nearest panorama in time
keeps the viewpoint close. When the point's direction falls in the operator mask,
or the panorama is too far in time, the next-nearest panorama is tried.

Colours come from the same panoramas as the dataset images, at a reduced width.
Exposure differences between panoramas are not equalized.
"""

from __future__ import annotations

from bisect import bisect_left
from pathlib import Path

from .geometry import Pose, Trajectory
from .panorama import run_tool
from .selfmask import direction_bin, sphere_direction

CANDIDATES = 4


def decode(path: Path, width: int, ffmpeg="ffmpeg") -> bytes:
    """An equirectangular image as packed RGB of width x width/2."""
    data = run_tool([ffmpeg, "-v", "error", "-nostdin", "-i", str(path), "-vf",
                     f"scale={width}:{width // 2}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"])
    if len(data) != width * (width // 2) * 3:
        raise ValueError(f"{path.name}: unexpected decoded size")
    return data


def colorize(points, times, panoramas, *, width=2048, masked=None, near_m=0.3,
             max_gap_ns=5_000_000_000, ffmpeg="ffmpeg") -> tuple[list, dict]:
    """New (x, y, z, r, g, b) points and a report.

    panoramas: (reference_ns, T_world_sphere Pose, image Path), any order.
    masked: sphere-direction bins (selfmask) to avoid, or None.
    """
    shots = sorted(panoramas, key=lambda item: item[0])
    stamps = [shot[0] for shot in shots]
    inverse = [shot[1].inverse() for shot in shots]
    masked = {tuple(b) for b in masked} if masked else set()
    height = width // 2
    chosen = {}  # panorama index -> [(point index, u, v)]
    gaps, skipped_mask = [], 0
    for index, (point, stamp) in enumerate(zip(points, times)):
        k = bisect_left(stamps, stamp)
        order = sorted(range(max(0, k - CANDIDATES), min(len(shots), k + CANDIDATES)),
                       key=lambda j: abs(stamps[j] - stamp))[:CANDIDATES]
        for j in order:
            if abs(stamps[j] - stamp) > max_gap_ns:
                break
            x, y, z = inverse[j].transform(point[:3])
            if x * x + y * y + z * z < near_m * near_m:
                continue
            azimuth, elevation = sphere_direction(x, y, z)
            if masked and direction_bin(azimuth, elevation) in masked:
                skipped_mask += 1
                continue
            u = min(width - 1, int((azimuth + 180.0) / 360.0 * width))
            v = min(height - 1, max(0, int((90.0 - elevation) / 180.0 * height)))
            chosen.setdefault(j, []).append((index, u, v))
            gaps.append(abs(stamps[j] - stamp))
            break
    colored = list(points)
    for j, items in sorted(chosen.items()):
        image = decode(shots[j][2], width, ffmpeg)
        for index, u, v in items:
            offset = (v * width + u) * 3
            colored[index] = (*points[index][:3], image[offset], image[offset + 1],
                              image[offset + 2])
    count = sum(len(items) for items in chosen.values())
    gaps.sort()
    return colored, {"colored_points": count, "uncolored_points": len(points) - count,
                     "panoramas_used": len(chosen), "panorama_width": width,
                     "time_gap_ms": {"median": round(gaps[len(gaps) // 2] / 1e6, 1),
                                     "p95": round(gaps[int(len(gaps) * 0.95)] / 1e6, 1)}
                     if gaps else None,
                     "mask_redirects": skipped_mask,
                     "limits": ["no occlusion test beyond choosing the panorama nearest in time",
                                "exposure differences between panoramas are not equalized"]}


def session_panoramas(root: Path, trajectory: Trajectory, clock, T_rig_sphere: Pose,
                      records) -> list:
    """(reference_ns, T_world_sphere, image path) for each panorama of a session."""
    from .session import local_file
    result = []
    for record in records:
        reference = clock.map(record["timestamp_ns"])
        result.append((reference, trajectory.at(reference).compose(T_rig_sphere),
                       local_file(root, record["image_path"])))
    return result
