# SPDX-License-Identifier: MIT
"""Republish Livox CustomMsg as sensor_msgs/PointCloud2 for standard viewers.

This is a live viewing aid, not a recording path: per-point times are omitted and
no-return points (x = y = z = 0) are skipped. ROS is imported only by run().
"""

from __future__ import annotations

import os
from pathlib import Path
import struct

POINT = struct.Struct("<ffff")
FIELDS = (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))


DDS_PROFILE = Path(__file__).parent / "assets/fastdds.xml"


def use_dds_profile():
    """Use SplatRig's Fast DDS buffers unless the user configured a profile."""
    if not (os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE") or
            os.environ.get("FASTDDS_DEFAULT_PROFILES_FILE")):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(DDS_PROFILE)
    return os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE") or \
        os.environ["FASTDDS_DEFAULT_PROFILES_FILE"]


def pack_points(points) -> tuple[bytes, int]:
    """Pack returned points as little-endian float32 x, y, z, intensity."""
    data = bytearray()
    for point in points:
        if point.x == 0 and point.y == 0 and point.z == 0:
            continue
        data += POINT.pack(point.x, point.y, point.z, point.reflectivity)
    return bytes(data), len(data) // POINT.size


def run(input_topic: str, output_topic: str):
    use_dds_profile()
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2, PointField
        from livox_ros_driver2.msg import CustomMsg
    except ImportError as error:
        raise ValueError("ROS 2 and livox_ros_driver2 Python packages are unavailable. "
                         "Source both environments and use the system Python.") from error
    fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
              for name, offset in FIELDS]
    rclpy.init()
    node = Node("splatrig_pointcloud_view")
    publisher = node.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)

    def receive(message):
        if not rclpy.ok():
            return
        data, count = pack_points(message.points)
        cloud = PointCloud2(header=message.header, height=1, width=count, fields=fields,
                            is_bigendian=False, point_step=POINT.size,
                            row_step=POINT.size * count, is_dense=True)
        cloud.data = data
        publisher.publish(cloud)

    node.create_subscription(CustomMsg, input_topic, receive, qos_profile_sensor_data)
    print(f"Republishing {input_topic} as PointCloud2 on {output_topic}", flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
