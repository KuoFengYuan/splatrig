# SPDX-License-Identifier: MIT
"""Rigid transforms: T_A_B maps points in B into A; quaternions are wxyz."""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math


def vector(values, length: int) -> tuple[float, ...]:
    result = tuple(float(x) for x in values)
    if len(result) != length or not all(math.isfinite(x) for x in result):
        raise ValueError(f"Expected {length} finite numbers")
    return result


def quaternion(values) -> tuple[float, ...]:
    q = vector(values, 4)
    norm = math.sqrt(sum(x * x for x in q))
    if abs(norm - 1.0) > 0.001:
        raise ValueError("Quaternion must have unit length (wxyz order)")
    return tuple(x / norm for x in q)


def multiply(a, b) -> tuple[float, ...]:
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def rotate(q, p) -> tuple[float, ...]:
    w, x, y, z = q
    a, b, c = p
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return (a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx)


def slerp(a, b, fraction: float) -> tuple[float, ...]:
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-x for x in b), -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        q = tuple(x+fraction*(y-x) for x, y in zip(a, b))
        norm = math.sqrt(sum(x*x for x in q))
        return tuple(x/norm for x in q)
    angle = math.acos(dot)
    left = math.sin((1-fraction)*angle) / math.sin(angle)
    right = math.sin(fraction*angle) / math.sin(angle)
    return tuple(left*x+right*y for x, y in zip(a, b))


@dataclass(frozen=True)
class Pose:
    translation: tuple[float, ...]
    quaternion_wxyz: tuple[float, ...]

    def __post_init__(self):
        object.__setattr__(self, "translation", vector(self.translation, 3))
        object.__setattr__(self, "quaternion_wxyz", quaternion(self.quaternion_wxyz))

    @classmethod
    def identity(cls):
        return cls((0, 0, 0), (1, 0, 0, 0))

    @classmethod
    def from_dict(cls, data):
        return cls(data["translation"], data["quaternion_wxyz"])

    def to_dict(self):
        return {"translation": list(self.translation),
                "quaternion_wxyz": list(self.quaternion_wxyz)}

    def transform(self, point):
        return tuple(a+b for a, b in zip(rotate(self.quaternion_wxyz, point),
                                        self.translation))

    def compose(self, other: Pose) -> Pose:
        return Pose(self.transform(other.translation),
                    multiply(self.quaternion_wxyz, other.quaternion_wxyz))

    def inverse(self) -> Pose:
        w, x, y, z = self.quaternion_wxyz
        q = (w, -x, -y, -z)
        return Pose(rotate(q, tuple(-v for v in self.translation)), q)


def nanoseconds(value) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("Timestamps must be nonnegative integer nanoseconds")
    return value


class Trajectory:
    def __init__(self, records, max_gap_ns: int = 200_000_000):
        self.times = []
        self.poses = []
        self.max_gap_ns = nanoseconds(max_gap_ns)
        if not self.max_gap_ns:
            raise ValueError("Maximum trajectory gap must be positive")
        for record in records:
            timestamp = nanoseconds(record["timestamp_ns"])
            if self.times and timestamp <= self.times[-1]:
                raise ValueError("Trajectory timestamps must be strictly increasing")
            self.times.append(timestamp)
            self.poses.append(Pose.from_dict(record))
        if not self.times:
            raise ValueError("Trajectory is empty")

    def at(self, timestamp_ns: int) -> Pose:
        timestamp_ns = nanoseconds(timestamp_ns)
        index = bisect_left(self.times, timestamp_ns)
        if index < len(self.times) and self.times[index] == timestamp_ns:
            return self.poses[index]
        if index == 0 or index == len(self.times):
            raise ValueError(f"Timestamp {timestamp_ns} is outside the trajectory")
        gap = self.times[index] - self.times[index-1]
        if gap > self.max_gap_ns:
            raise ValueError(f"Cannot interpolate across a {gap / 1e6:.1f} ms pose gap")
        fraction = (timestamp_ns-self.times[index-1]) / gap
        a, b = self.poses[index-1], self.poses[index]
        return Pose(tuple(x+fraction*(y-x) for x, y in zip(a.translation, b.translation)),
                    slerp(a.quaternion_wxyz, b.quaternion_wxyz, fraction))
