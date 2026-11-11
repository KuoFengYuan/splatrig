# SPDX-License-Identifier: MIT
"""Bridge between Livox recordings, the external GLIM SLAM process and sessions.

GLIM reads sensor_msgs/PointCloud2 with a per-point time field ("t", seconds
relative to the header stamp) and does not read livox_ros_driver2 CustomMsg.
to_glim_bag() writes such a copy without modifying the original recording.
GLIM writes TUM trajectories ("t x y z qx qy qz qw"); read_tum() converts them
to SplatRig records with exact integer nanoseconds and wxyz quaternions.
"""

from __future__ import annotations

from decimal import Decimal, InvalidOperation
from pathlib import Path
import struct

from .export import new_output
from .geometry import Pose
from .livox import IMU_TYPE, LIDAR_TYPE, read_bag
from .session import write_json, write_jsonl

TIMED_POINT = struct.Struct("<fffff")
TIMED_FIELDS = (("x", 0), ("y", 4), ("z", 8), ("intensity", 12), ("t", 16))
# Mid-360 IMU position in the LiDAR frame from the published MID-360 layout.
# Not verified for the Mid-360S; confirm before treating results as calibrated.
MID360_T_LIDAR_IMU = [0.011, 0.02329, -0.04412, 0.0, 0.0, 0.0, 1.0]


def pack_timed_points(points) -> tuple[bytes, int]:
    """Pack returned points as float32 x, y, z, intensity, t (s after timebase)."""
    data = bytearray()
    for point in points:
        if point.x == 0 and point.y == 0 and point.z == 0:
            continue
        data += TIMED_POINT.pack(point.x, point.y, point.z, point.reflectivity,
                                 point.offset_time * 1e-9)
    return bytes(data), len(data) // TIMED_POINT.size


def to_glim_bag(bag: Path, destination: Path, *, lidar_topic="/livox/lidar",
                imu_topic="/livox/imu", points_topic="/livox/points") -> dict:
    try:
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import PointCloud2, PointField
    except ImportError as error:
        raise ValueError("ROS 2 Python packages are unavailable. Source ROS 2 and the "
                         "livox_ros_driver2 workspace, then use the system Python.") from error
    if not bag.exists():
        raise ValueError(f"Bag not found: {bag}")
    fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
              for name, offset in TIMED_FIELDS]
    frames = imu = points = 0
    with new_output(destination, bag) as root:
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(root / "bag"), storage_id="mcap"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        for index, (name, kind) in enumerate([(points_topic, "sensor_msgs/msg/PointCloud2"),
                                              (imu_topic, IMU_TYPE)]):
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=index, name=name, type=kind, serialization_format="cdr"))
        for topic, receive_ns, message in read_bag(bag, {lidar_topic: LIDAR_TYPE,
                                                        imu_topic: IMU_TYPE}):
            if topic == lidar_topic:
                data, count = pack_timed_points(message.points)
                cloud = PointCloud2(header=message.header, height=1, width=count,
                                    fields=fields, is_bigendian=False,
                                    point_step=TIMED_POINT.size,
                                    row_step=TIMED_POINT.size * count, is_dense=True)
                cloud.data = data
                writer.write(points_topic, serialize_message(cloud), receive_ns)
                frames += 1
                points += count
            else:
                writer.write(imu_topic, serialize_message(message), receive_ns)
                imu += 1
        del writer
        if not frames:
            raise ValueError(f"No frames on {lidar_topic}")
        report = {"source_bag": str(bag.resolve()), "points_topic": points_topic,
                  "imu_topic": imu_topic, "frames": frames, "points": points,
                  "imu_messages": imu,
                  "point_fields": "x y z intensity (float32), t float32 seconds after "
                                  "header stamp; no-return points excluded",
                  "glim_settings": {"points_topic": points_topic, "imu_topic": imu_topic,
                                    "acc_scale": 9.80665,
                                    "T_lidar_imu_mid360_unverified": MID360_T_LIDAR_IMU}}
        write_json(root / "conversion.json", report)
    return report


def seconds_to_ns(text: str) -> int:
    try:
        value = Decimal(text)
    except InvalidOperation as error:
        raise ValueError(f"Invalid timestamp: {text}") from error
    if not value.is_finite() or value < 0:
        raise ValueError(f"Invalid timestamp: {text}")
    return int((value * 1_000_000_000).to_integral_value())


def read_tum(path: Path, revision: int):
    """Yield trajectory records from a TUM file of T_world_frame poses."""
    previous = None
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        values = line.split()
        if len(values) != 8:
            raise ValueError(f"{path.name}:{line_number}: expected 8 TUM columns")
        timestamp = seconds_to_ns(values[0])
        if previous is not None and timestamp <= previous:
            raise ValueError(f"{path.name}:{line_number}: timestamps must increase")
        previous = timestamp
        x, y, z, qx, qy, qz, qw = map(float, values[1:])
        pose = Pose((x, y, z), (qw, qx, qy, qz))
        yield {"timestamp_ns": timestamp, "revision": revision, **pose.to_dict()}


def import_trajectory(tum: Path, destination: Path, *, revision: int, status: str,
                      frame: str) -> dict:
    """Write trajectory.jsonl plus provenance into a new directory."""
    if status not in {"live", "final"}:
        raise ValueError("Trajectory status must be live or final")
    records = list(read_tum(tum, revision))
    if len(records) < 2:
        raise ValueError("Trajectory needs at least two poses")
    with new_output(destination, tum.parent) as root:
        write_jsonl(root / "trajectory.jsonl", records)
        report = {"source": str(tum.resolve()), "poses": len(records),
                  "revision": revision, "trajectory_status": status, "tracked_frame": frame,
                  "time_range_ns": [records[0]["timestamp_ns"], records[-1]["timestamp_ns"]],
                  "note": "T_world_<tracked_frame> from an external SLAM backend; "
                          "final only when taken from loop-closed history"}
        write_json(root / "trajectory-source.json", report)
    return report
