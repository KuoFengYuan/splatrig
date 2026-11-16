# SPDX-License-Identifier: MIT
"""Choose which video frames become dataset images, from the SLAM trajectory.

Gaussian splatting needs views spread through space and free of motion blur, not
every Nth frame: standing still repeats one view and a quick turn smears it. A
frame becomes due once the rig has moved `spacing_m` or turned `spacing_deg` since
the previous keyframe; of the frames in the following `window_s`, the one turning
slowest is kept. Frames turning faster than `max_turn_dps` are never kept.

The turn rate is that of the trajectory around the frame time. It predicts blur
from rotation only; defocus, rolling shutter and the exposure time are unknown.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .geometry import Pose, Trajectory

HALF_SPAN_NS = 50_000_000


def rotation_deg(a, b) -> float:
    """Angle between two wxyz unit quaternions, degrees."""
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2 * math.acos(min(1.0, dot)))


def turn_rate_dps(trajectory: Trajectory, timestamp_ns: int, half_ns=HALF_SPAN_NS) -> float:
    """Mean turn rate over ±half_ns, clipped to the trajectory ends."""
    start = max(trajectory.times[0], timestamp_ns - half_ns)
    end = min(trajectory.times[-1], timestamp_ns + half_ns)
    if end <= start:
        return 0.0
    angle = rotation_deg(trajectory.at(start).quaternion_wxyz,
                         trajectory.at(end).quaternion_wxyz)
    return angle / ((end - start) / 1e9)


@dataclass
class Candidate:
    index: int  # video frame index
    timestamp_ns: int  # reference (trajectory) time
    pose: Pose
    turn_dps: float


def select_keyframes(media_times_ns, camera_start_ns: int, trajectory: Trajectory, *,
                     spacing_m=0.3, spacing_deg=20.0, max_turn_dps=30.0,
                     window_s=0.5) -> dict:
    """Frame indices to extract and a report of what was skipped and why."""
    if spacing_m <= 0 or spacing_deg <= 0 or max_turn_dps <= 0 or window_s < 0:
        raise ValueError("Keyframe spacing, turn limit and window must be positive")
    first, last = trajectory.times[0], trajectory.times[-1]
    candidates, outside, gaps = [], 0, 0
    for index, media_ns in enumerate(media_times_ns):
        reference = camera_start_ns + media_ns
        if not first <= reference <= last:
            outside += 1
            continue
        try:
            candidates.append(Candidate(index, reference, trajectory.at(reference),
                                        turn_rate_dps(trajectory, reference)))
        except ValueError:  # pose gap: no pose may be interpolated here
            gaps += 1
    window_ns = round(window_s * 1e9)
    chosen, previous, blurred_windows, i = [], None, 0, 0
    while i < len(candidates):
        due = candidates[i]
        if previous is not None and \
                math.dist(due.pose.translation, previous.pose.translation) < spacing_m and \
                rotation_deg(due.pose.quaternion_wxyz,
                             previous.pose.quaternion_wxyz) < spacing_deg:
            i += 1
            continue
        best, j = None, i
        while j < len(candidates) and candidates[j].timestamp_ns <= due.timestamp_ns + window_ns:
            if candidates[j].turn_dps <= max_turn_dps and \
                    (best is None or candidates[j].turn_dps < candidates[best].turn_dps):
                best = j
            j += 1
        if best is None:  # the whole window turns too fast; stay due and look further
            blurred_windows += 1
            i = j
            continue
        previous = candidates[best]
        chosen.append(previous)
        i = best + 1
    rates = sorted(c.turn_dps for c in chosen)
    path = sum(math.dist(a.pose.translation, b.pose.translation)
               for a, b in zip(candidates, candidates[1:]))
    report = {"method": "trajectory spacing with slowest-turn frame per window",
              "video_frames": len(media_times_ns), "outside_trajectory": outside,
              "pose_gaps": gaps, "candidates": len(candidates),
              "too_fast": sum(c.turn_dps > max_turn_dps for c in candidates),
              "blurred_windows": blurred_windows, "selected": len(chosen),
              "spacing_m": spacing_m, "spacing_deg": spacing_deg,
              "max_turn_dps": max_turn_dps, "window_s": window_s,
              "path_m": round(path, 3),
              "selected_turn_dps": {"median": round(rates[len(rates) // 2], 2),
                                    "max": round(rates[-1], 2)} if rates else None,
              "limits": ["blur predicted from trajectory rotation only; "
                         "exposure time, defocus and rolling shutter are not measured"]}
    return {"frames": [c.index for c in chosen], "report": report}
