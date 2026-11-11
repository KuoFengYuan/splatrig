# SPDX-License-Identifier: MIT
"""Optional ROS 2 odometry recorder; importing the core never requires ROS."""

from __future__ import annotations

import json
from pathlib import Path

from .geometry import Pose, nanoseconds


def odometry_record(message, world_frame: str, rig_frame: str, revision: int = 0):
    if message.header.frame_id != world_frame or message.child_frame_id != rig_frame:
        raise ValueError("Odometry frames differ from the configured world/rig frames; "
                         "apply an explicit transform before recording")
    stamp = message.header.stamp
    if (isinstance(stamp.sec, bool) or not isinstance(stamp.sec, int) or stamp.sec < 0 or
            isinstance(stamp.nanosec, bool) or not isinstance(stamp.nanosec, int) or
            not 0 <= stamp.nanosec < 1_000_000_000):
        raise ValueError("Invalid ROS timestamp")
    value = message.pose.pose
    p, q = value.position, value.orientation
    pose = Pose((p.x, p.y, p.z), (q.w, q.x, q.y, q.z))
    return {"timestamp_ns": nanoseconds(stamp.sec*1_000_000_000+stamp.nanosec),
            "revision": revision, **pose.to_dict()}


def record_poses(destination: Path, topic: str, world_frame: str, rig_frame: str):
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
    except ImportError as error:
        raise ValueError("ROS 2 Python packages are unavailable. Source the ROS 2 "
                         "environment and run with its Python interpreter.") from error
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8") as stream:
        last_timestamp = None

        def receive(message):
            nonlocal last_timestamp
            record = odometry_record(message, world_frame, rig_frame)
            if last_timestamp is not None and record["timestamp_ns"] <= last_timestamp:
                raise ValueError("Odometry clock went backwards or repeated; start a new session")
            stream.write(json.dumps(record, allow_nan=False) + "\n")
            stream.flush()
            last_timestamp = record["timestamp_ns"]

        rclpy.init()
        node = Node("splatrig_pose_recorder")
        node.create_subscription(Odometry, topic, receive, qos_profile_sensor_data)
        try:
            print(f"Recording live {world_frame} <- {rig_frame} odometry to {destination}",
                  flush=True)
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
