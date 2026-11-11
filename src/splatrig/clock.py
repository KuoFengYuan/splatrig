# SPDX-License-Identifier: MIT
"""Fit clock rate and offset around integer anchors, avoiding epoch precision loss."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .geometry import nanoseconds


@dataclass(frozen=True)
class Clock:
    camera_anchor_ns: int
    reference_anchor_ns: int
    rate: float = 1.0
    status: str = "provisional"

    def __post_init__(self):
        nanoseconds(self.camera_anchor_ns)
        nanoseconds(self.reference_anchor_ns)
        if not math.isfinite(self.rate) or not 0.99 <= self.rate <= 1.01:
            raise ValueError("Clock rate must be within 1% of unity; check timestamp units")
        if self.status not in {"provisional", "calibrated"}:
            raise ValueError("Clock status must be provisional or calibrated")

    @classmethod
    def from_dict(cls, data):
        return cls(data["camera_anchor_ns"], data["reference_anchor_ns"],
                   data.get("rate", 1.0), data.get("status", "provisional"))

    def map(self, camera_ns: int) -> int:
        delta = nanoseconds(camera_ns) - self.camera_anchor_ns
        result = self.reference_anchor_ns + round(delta*self.rate)
        return nanoseconds(result)

    def to_dict(self):
        return {"camera_anchor_ns": self.camera_anchor_ns,
                "reference_anchor_ns": self.reference_anchor_ns,
                "rate": self.rate, "status": self.status}


def fit_clock(pairs, max_residual_ns: int = 5_000_000):
    pairs = [(nanoseconds(a), nanoseconds(b)) for a, b in pairs]
    if len(pairs) < 3:
        raise ValueError("Clock fitting needs at least three paired physical events")
    if max_residual_ns <= 0:
        raise ValueError("Maximum clock residual must be positive")
    if any(a[0] >= b[0] or a[1] >= b[1] for a, b in zip(pairs, pairs[1:])):
        raise ValueError("Paired event timestamps must be strictly increasing")
    camera_zero, reference_zero = pairs[0]
    xs = [a-camera_zero for a, _ in pairs]
    ys = [b-reference_zero for _, b in pairs]
    mean_x, mean_y = sum(xs)/len(xs), sum(ys)/len(ys)
    denominator = sum((x-mean_x)**2 for x in xs)
    rate = sum((x-mean_x)*(y-mean_y) for x, y in zip(xs, ys)) / denominator
    offset = round(mean_y-rate*mean_x)
    clock = Clock(camera_zero, reference_zero+offset, rate, "calibrated")
    residuals = [clock.map(a)-b for a, b in pairs]
    largest = max(abs(r) for r in residuals)
    if largest > max_residual_ns:
        raise ValueError(f"Clock residual {largest / 1e6:.3f} ms exceeds the limit")
    report = {"events": len(pairs), "span_ns": xs[-1],
              "drift_ppm": (rate-1)*1e6,
              "rms_residual_ns": math.sqrt(sum(r*r for r in residuals)/len(pairs)),
              "max_residual_ns": largest}
    return clock, report
