# SPDX-License-Identifier: MIT
"""Livox Mid-360/Mid-360S rosbag2 recordings to a SplatRig LiDAR recording.

ROS packages are imported only by read_bag(); conversion and clock assessment
operate on plain message-like objects so they can be tested without ROS.

livox_ros_driver2 replaces device timestamps with host packet-arrival time when
the device is unsynchronized, and CustomMsg does not carry the packet time_type.
The recording therefore infers the clock source from timing structure: synced
device stamps place packet boundaries at a fixed spacing, while arrival stamps
inherit network and scheduling jitter.
"""

from __future__ import annotations

from collections import Counter
import json
import math
from pathlib import Path
import statistics

from .export import new_output
from .pointlog import PointLogWriter
from .rosview import ros_import_error
from .session import write_json

LIDAR_TYPE = "livox_ros_driver2/msg/CustomMsg"
IMU_TYPE = "sensor_msgs/msg/Imu"
BOUNDARY_WINDOW_NS = 100
FRAME_BIN_NS = 10_000
FRAME_WINDOW_NS = 20_000
FRAME_WINDOWS = 2
GAP_FACTOR = 1.5


def stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def concentration(counts: Counter, width: int, windows: int = 1) -> float:
    """Fraction of the total mass inside the heaviest windows of +/- width.

    Windows are chosen greedily and do not share mass. Synchronized frame steps
    take a few discrete values (whole packets at a fixed spacing), so a small
    number of windows covers them; arrival-time jitter stays spread out.
    """
    total = sum(counts.values())
    remaining = Counter(counts)
    covered = 0
    for _ in range(windows):
        keys = sorted(key for key, count in remaining.items() if count)
        if not keys:
            break
        best = running = left = 0
        best_range = (keys[0], keys[0])
        for key in keys:
            running += remaining[key]
            while keys[left] < key - 2 * width:
                running -= remaining[keys[left]]
                left += 1
            if running > best:
                best, best_range = running, (keys[left], key)
        covered += best
        for key in keys:
            if best_range[0] <= key <= best_range[1]:
                remaining[key] = 0
    return covered / total if total else 0.0


def assess_clock(offset_steps: Counter, frame_steps: list[int], backwards: int) -> dict:
    """Classify per-point timestamps as synchronized device time or arrival time."""
    evidence = {"offset_backwards_frames": backwards, "frame_steps": len(frame_steps)}
    if not offset_steps or len(frame_steps) < 20:
        return {"status": "indeterminate", "reason": "too few frames", **evidence}
    interval = offset_steps.most_common(1)[0][0]
    boundary = Counter({step: count for step, count in offset_steps.items()
                        if abs(step - interval) > BOUNDARY_WINDOW_NS})
    frames = Counter(round(step / FRAME_BIN_NS) * FRAME_BIN_NS for step in frame_steps)
    evidence.update({
        "point_interval_ns": interval,
        "packet_boundaries": sum(boundary.values()),
        "boundary_concentration": round(concentration(boundary, BOUNDARY_WINDOW_NS), 4),
        "frame_step_concentration": round(
            concentration(frames, FRAME_WINDOW_NS, FRAME_WINDOWS), 4)})
    if evidence["packet_boundaries"] < 20:
        return {"status": "indeterminate", "reason": "too few packet boundaries", **evidence}
    if backwards:
        status = "host_arrival_suspected"
    elif (evidence["boundary_concentration"] >= 0.9 and
          evidence["frame_step_concentration"] >= 0.8):
        status = "ptp_inferred"
    elif evidence["boundary_concentration"] < 0.5:
        status = "host_arrival_suspected"
    else:
        status = "indeterminate"
    return {"status": status, **evidence}


def percentile(sorted_values, fraction):
    return sorted_values[min(len(sorted_values) - 1, int(fraction * len(sorted_values)))]


def gaps(steps, factor=GAP_FACTOR):
    if not steps:
        return {"count": 0}
    typical = statistics.median(steps)
    large = [step for step in steps if step > factor * typical]
    return {"count": len(large), "typical_ns": typical, "largest_ns": max(steps)}


def accel_unit(mean_magnitude: float) -> str:
    if 0.8 <= mean_magnitude <= 1.2:
        return "g"
    if 7.8 <= mean_magnitude <= 11.8:
        return "m/s^2"
    return "unknown"


def convert(messages, destination: Path, source: Path, *, lidar_topic="/livox/lidar",
            imu_topic="/livox/imu", allow_unsynced=False) -> dict:
    """messages yields (topic, receive_ns, message) in recording order."""
    source = source.resolve()
    with new_output(destination, source) as root:
        frames = points = no_return = 0
        frame_counts, frame_steps, offset_steps = [], [], Counter()
        backwards = overlapping = 0
        first_time = last_time = previous_base = previous_end = None
        imu_times, imu_latency, magnitudes = [], [], []
        imu_backwards = 0
        with PointLogWriter(root / "points.bin", {
                "frame": "lidar", "units": "meters",
                "time": "per-point acquisition timestamp in the LiDAR reference clock",
                "excluded": "returns with x=y=z=0 (no echo)"}) as writer, \
                (root / "imu.jsonl").open("x", encoding="utf-8") as imu_stream:
            for topic, receive_ns, message in messages:
                if topic == lidar_topic:
                    base = int(message.timebase)
                    offsets = [int(p.offset_time) for p in message.points]
                    if not offsets:
                        continue
                    if any(b < a for a, b in zip(offsets, offsets[1:])) or max(offsets) >= 2**31:
                        backwards += 1
                    offset_steps.update(b - a for a, b in zip(offsets, offsets[1:]))
                    if previous_base is not None:
                        frame_steps.append(base - previous_base)
                        if base < previous_end:
                            overlapping += 1
                    previous_base, previous_end = base, base + offsets[-1]
                    frames += 1
                    frame_counts.append(len(offsets))
                    for point, offset in zip(message.points, offsets):
                        if point.x == 0 and point.y == 0 and point.z == 0:
                            no_return += 1
                            continue
                        if not (math.isfinite(point.x) and math.isfinite(point.y) and
                                math.isfinite(point.z)):
                            raise ValueError("Non-finite LiDAR coordinates")
                        time = base + offset
                        writer.write(time, point.x, point.y, point.z, point.reflectivity,
                                     point.tag, point.line)
                        points += 1
                        first_time = time if first_time is None else min(first_time, time)
                        last_time = time if last_time is None else max(last_time, time)
                elif topic == imu_topic:
                    time = stamp_ns(message.header.stamp)
                    if imu_times and time <= imu_times[-1]:
                        imu_backwards += 1
                    imu_times.append(time)
                    imu_latency.append((receive_ns - time) / 1e6)
                    a, g = message.linear_acceleration, message.angular_velocity
                    magnitudes.append(math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z))
                    imu_stream.write(json.dumps({
                        "timestamp_ns": time,
                        "angular_velocity_rad_s": [g.x, g.y, g.z],
                        "linear_acceleration": [a.x, a.y, a.z]}, allow_nan=False) + "\n")
        if not frames or not points:
            raise ValueError(f"No LiDAR points found on {lidar_topic}")
        clock = assess_clock(offset_steps, frame_steps, backwards)
        provisional = clock["status"] != "ptp_inferred"
        if provisional and not allow_unsynced:
            raise ValueError(f"LiDAR clock status is {clock['status']}, not ptp_inferred; "
                             "check PTP or pass --allow-unsynced for a provisional recording")
        imu_steps = [b - a for a, b in zip(imu_times, imu_times[1:])]
        latency = sorted(imu_latency)
        mean_magnitude = statistics.fmean(magnitudes) if magnitudes else 0.0
        report = {
            "format": "splatrig-lidar-recording", "version": 1,
            "source": {"bag": str(source), "lidar_topic": lidar_topic, "imu_topic": imu_topic},
            "provisional": provisional, "clock": clock,
            "time_range_ns": [first_time, last_time],
            "lidar": {"frames": frames, "points_written": points,
                      "no_return_excluded": no_return,
                      "points_per_frame": [min(frame_counts), max(frame_counts)],
                      "frame_gaps": gaps(frame_steps), "overlapping_frames": overlapping},
            "imu": {"messages": len(imu_times), "non_increasing": imu_backwards,
                    "gaps": gaps(imu_steps),
                    "rate_hz": (len(imu_times) - 1) / ((imu_times[-1] - imu_times[0]) / 1e9)
                    if len(imu_times) > 1 and imu_times[-1] > imu_times[0] else None,
                    "linear_acceleration_unit": accel_unit(mean_magnitude),
                    "mean_acceleration_magnitude": round(mean_magnitude, 6),
                    "receive_latency_ms": {"p1": percentile(latency, 0.01),
                                           "p50": percentile(latency, 0.5),
                                           "p99": percentile(latency, 0.99)}
                    if latency else None},
            "not_validated": ["exposure-level accuracy of PTP software timestamps",
                              "IMU sample timing pattern", "LiDAR/IMU extrinsics",
                              "trajectory (SLAM is external)"]}
        write_json(root / "recording.json", report)
    return report


def read_bag(path: Path, topics: dict[str, str]):
    """Yield (topic, receive_ns, message) for topics mapped to expected type names."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise ValueError(ros_import_error(error)) from error
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    available = {item.name: item.type for item in reader.get_all_topics_and_types()}
    classes = {}
    for topic, expected in topics.items():
        if available.get(topic) != expected:
            raise ValueError(f"Bag topic {topic} must be {expected}; found "
                             f"{available.get(topic, 'nothing')}")
        try:
            classes[topic] = get_message(expected)
        except (AttributeError, ModuleNotFoundError, ValueError) as error:
            raise ValueError(f"Cannot load {expected}; source the driver workspace") from error
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        yield topic, receive_ns, deserialize_message(data, classes[topic])


def import_livox(bag: Path, destination: Path, *, lidar_topic="/livox/lidar",
                 imu_topic="/livox/imu", allow_unsynced=False) -> dict:
    if not bag.exists():
        raise ValueError(f"Bag not found: {bag}")
    messages = read_bag(bag, {lidar_topic: LIDAR_TYPE, imu_topic: IMU_TYPE})
    return convert(messages, destination, bag, lidar_topic=lidar_topic, imu_topic=imu_topic,
                   allow_unsynced=allow_unsynced)
