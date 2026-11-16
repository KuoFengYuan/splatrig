# SPDX-License-Identifier: MIT
"""Live scan guidance: expected photos, motion, coverage and revisit prompts.

The console cannot know which frames the dataset will keep, so it replays the
same rule online on GLIM's odometry: a photo is due after `spacing_m` of travel
or `spacing_deg` of turn and is counted at the first pose turning no faster than
`max_turn_dps`. The offline step picks the slowest frame of the next 0.5 s, so it
keeps at least these. A photo that stays due for longer than `window_s` means the
stretch was too fast; it is counted as skipped and reported while it lasts.

Coverage follows the photos. At each photo, the newest scan's points (map frame)
mark 25 cm cells as seen from the photo's horizontal direction in 10° bins; a cell
is covered once its views span `span_deg`. After `revisit_m` of new ground the
operator is asked to return to scanned ground so SLAM can correct drift, and a
return is confirmed when the rig comes within `near_m` of an older part of the
route.
"""

from __future__ import annotations

from collections import deque
import math
import struct
import threading
import time

BINS = 36  # 10-degree azimuth bins
CELL = struct.Struct("<ffff")


def heading_deg(quaternion_wxyz) -> float:
    """Yaw of the rig's +X axis about world +Z, degrees."""
    w, x, y, z = quaternion_wxyz
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def angle_deg(a, b) -> float:
    dot = abs(sum(p * q for p, q in zip(a, b)))
    return math.degrees(2 * math.acos(min(1.0, dot)))


def span_deg(mask: int) -> float:
    """Angular extent of the set azimuth bins: the smallest arc holding all of them.

    One bin gives 0 (seen from a single direction), two neighbours give 10.
    """
    if not mask:
        return 0.0
    bits = [(mask >> i) & 1 for i in range(BINS)]
    if all(bits):
        return 360.0
    longest = run = 0
    for bit in bits + bits:  # twice around, for empty arcs crossing bin 0
        run = 0 if bit else run + 1
        longest = max(longest, run)
    return (BINS - min(longest, BINS - 1) - 1) * 360.0 / BINS


class ScanGuide:
    def __init__(self, spacing_m=0.15, spacing_deg=10.0, max_turn_dps=30.0, window_s=0.5,
                 cell_m=0.25, span_target_deg=30.0, min_hits=3, max_range_m=10.0,
                 caution_speed=0.8, revisit_m=5.0, revisit_after_s=40.0, near_m=0.8,
                 old_s=20.0, sample=4):
        self.lock = threading.Lock()
        self.spacing_m, self.spacing_deg = spacing_m, spacing_deg
        self.max_turn_dps, self.window_s = max_turn_dps, window_s
        self.cell_m, self.span_target, self.min_hits = cell_m, span_target_deg, min_hits
        self.max_range_m, self.caution_speed = max_range_m, caution_speed
        self.revisit_m, self.revisit_after_s = revisit_m, revisit_after_s
        self.near_m, self.old_s, self.sample = near_m, old_s, sample
        self.poses = deque(maxlen=40)
        self.recording = False
        self.reset()

    def reset(self):
        self.photos = self.skipped = 0
        self.last_photo = None  # (position, quaternion)
        self.due_since = None
        self.blocked = False
        self.distance = 0.0
        self.cells = {}  # cell key -> [azimuth mask, hits, x, y, z]
        self.covered = 0
        self.eligible = 0
        self.route = {}  # 1 m grid key -> [(time, x, y)]
        self.route_last = None
        self.new_ground = 0.0
        self.max_new_ground = 0.0
        self.returns = 0
        self.back_at = None
        self.last_return = None
        self.started = None

    # ---------- session ----------
    def start(self, now=None):
        with self.lock:
            self.reset()
            self.recording = True
            self.started = time.time() if now is None else now
            self.last_return = self.started

    def stop(self):
        with self.lock:
            self.recording = False
            self.due_since = None
            self.blocked = False

    # ---------- odometry ----------
    def motion(self):
        """Speed (m/s) and turn rate (deg/s) over the last ~0.3 s of poses."""
        if len(self.poses) < 2:
            return 0.0, 0.0
        t1, p1, q1 = self.poses[-1]
        old = next((item for item in self.poses if t1 - item[0] <= 0.35), self.poses[0])
        t0, p0, q0 = old
        if t1 <= t0:
            return 0.0, 0.0
        return math.dist(p0, p1) / (t1 - t0), angle_deg(q0, q1) / (t1 - t0)

    def add_pose(self, position, quaternion_wxyz, scan=None, received=None):
        """Feed one odometry pose; scan() returns the newest map-frame points if a
        photo is taken now."""
        now = time.time() if received is None else received
        position = tuple(position)
        with self.lock:
            self.poses.append((now, position, tuple(quaternion_wxyz)))
            if not self.recording:
                return
            if self.route_last is not None:
                self.distance += math.dist(self.route_last, position)
            self.route_last = position
            self._revisit(now, position)
            speed, turn = self.motion()
            if self.last_photo is not None and self.due_since is None:
                p, q = self.last_photo
                if math.dist(p, position) >= self.spacing_m or \
                        angle_deg(q, quaternion_wxyz) >= self.spacing_deg:
                    self.due_since = now
            if self.last_photo is None or self.due_since is not None:
                if turn <= self.max_turn_dps:
                    self.photos += 1
                    self.last_photo = (position, tuple(quaternion_wxyz))
                    self.due_since = None
                    self.blocked = False
                    take = True
                else:
                    take = False
                    if self.due_since is not None and now - self.due_since > self.window_s \
                            and not self.blocked:
                        self.skipped += 1
                        self.blocked = True
            else:
                take = False
        if take and scan is not None:
            self._cover(position, heading_deg(quaternion_wxyz), scan())

    def _revisit(self, now, position):
        """Track new ground and returns to older parts of the route (lock held)."""
        x, y = position[0], position[1]
        key = (math.floor(x), math.floor(y))
        near = False
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for t, px, py in self.route.get((key[0] + dx, key[1] + dy), ()):
                    if now - t >= self.old_s and math.hypot(px - x, py - y) <= self.near_m:
                        near = True
                        break
        previous = self.route.get(key, [])
        if not previous or math.hypot(previous[-1][1] - x, previous[-1][2] - y) >= 0.25:
            self.route.setdefault(key, []).append((now, x, y))
        if near:
            if self.new_ground >= self.revisit_m:
                self.returns += 1
                self.back_at = now
                self.last_return = now
            self.new_ground = 0.0
        elif self.route_last is not None and len(self.poses) >= 2:
            step = math.dist(self.poses[-2][1], position)
            self.new_ground += step
            self.max_new_ground = max(self.max_new_ground, self.new_ground)

    # ---------- coverage ----------
    def _cover(self, origin, heading, points):
        size, reach = self.cell_m, self.max_range_m ** 2
        updates = 0
        with self.lock:
            for index, (x, y, z, *_) in enumerate(points):
                if index % self.sample:
                    continue
                dx, dy = x - origin[0], y - origin[1]
                if dx * dx + dy * dy + (z - origin[2]) ** 2 > reach:
                    continue
                key = (math.floor(x / size), math.floor(y / size), math.floor(z / size))
                cell = self.cells.get(key)
                if cell is None:
                    cell = self.cells[key] = [0, 0, (key[0] + 0.5) * size,
                                              (key[1] + 0.5) * size, (key[2] + 0.5) * size]
                # Direction the cell is seen from, so walking past widens the span.
                bit = 1 << (int((math.degrees(math.atan2(-dy, -dx)) + 180.0) // 10) % BINS)
                before = cell[0]
                cell[1] += 1
                if cell[1] == self.min_hits:
                    self.eligible += 1
                    if span_deg(cell[0]) >= self.span_target:
                        self.covered += 1
                if not before & bit:
                    cell[0] |= bit
                    if cell[1] >= self.min_hits and span_deg(before) < self.span_target <= \
                            span_deg(cell[0]):
                        self.covered += 1
                updates += 1
        return updates

    def cells_payload(self, max_cells=60_000) -> bytes:
        with self.lock:
            cells = [c for c in self.cells.values() if c[1] >= self.min_hits]
        stride = max(1, -(-len(cells) // max_cells))
        data = b"".join(CELL.pack(c[2], c[3], c[4], span_deg(c[0])) for c in cells[::stride])
        return struct.pack("<I", len(data) // CELL.size) + data

    # ---------- status ----------
    def status(self, now=None) -> dict:
        now = time.time() if now is None else now
        with self.lock:
            speed, turn = self.motion()
            fresh = bool(self.poses) and now - self.poses[-1][0] < 1.0
            level = max(turn / self.max_turn_dps, speed / self.caution_speed) if fresh else 0.0
            since = now - self.last_return if self.last_return is not None else 0.0
            revisit = (self.recording and self.new_ground >= self.revisit_m and
                       since >= self.revisit_after_s)
            return {"recording": self.recording, "speed_m_s": round(speed, 2),
                    "turn_dps": round(turn, 1), "motion_level": round(level, 2),
                    "max_turn_dps": self.max_turn_dps, "photos": self.photos,
                    "skipped": self.skipped, "too_fast": self.blocked,
                    "distance_m": round(self.distance, 2),
                    "coverage": round(self.covered / self.eligible, 3) if self.eligible else None,
                    "cells": self.eligible, "span_target_deg": self.span_target,
                    "new_ground_m": round(self.new_ground, 1),
                    "max_new_ground_m": round(self.max_new_ground, 1),
                    "returns": self.returns, "revisit": revisit,
                    "back": self.back_at is not None and now - self.back_at < 4.0,
                    "spacing_m": self.spacing_m}
