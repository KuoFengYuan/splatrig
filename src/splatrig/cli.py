# SPDX-License-Identifier: MIT
"""Small command-line entry point shared by local use and continuous checks."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

from . import __version__
from .clock import fit_clock
from .demo import create_demo, create_panorama_demo
from .export import export_colmap, replay
from .glim import import_trajectory, to_glim_bag, write_glim_config
from .livox import import_livox
from .max2 import extract as extract_max2
from .rosview import run as view_pointcloud
from .panorama import INTERPOLATIONS, PRESETS, panorama_views
from .ros import record_poses
from .session import prepare, write_json


def parser():
    root = argparse.ArgumentParser(
        prog="splatrig", description="Pose-aware capture datasets for Gaussian splatting.")
    root.add_argument("--version", action="version", version=__version__)
    commands = root.add_subparsers(dest="command", required=True)
    demo = commands.add_parser("demo", help="Create a synthetic room capture; no hardware needed")
    demo.add_argument("--output", type=Path, required=True)
    demo.add_argument("--panorama", action="store_true",
                      help="Write equirectangular panoramas instead of PINHOLE frames")
    for name, help_text in [
            ("validate", "Validate session geometry, images, calibration and timing"),
            ("replay", "Replay timestamped points and image poses into a local preview"),
            ("export", "Write a COLMAP initialization dataset")]:
        command = commands.add_parser(name, help=help_text)
        command.add_argument("session", type=Path)
        command.add_argument("--voxel-size", type=float, default=0.05, metavar="METERS")
        command.add_argument("--max-voxels", type=int, default=1_000_000)
        command.add_argument("--allow-provisional", action="store_true",
                             help="Diagnostic output with unfinalized poses or calibration")
        if name != "validate":
            command.add_argument("--output", type=Path, required=True)
    views = commands.add_parser(
        "panorama-views", help="Cut fixed PINHOLE views from equirectangular panoramas "
                               "(ffmpeg required)")
    views.add_argument("session", type=Path)
    views.add_argument("--output", type=Path, required=True)
    layout = views.add_mutually_exclusive_group()
    layout.add_argument("--layout", choices=sorted(PRESETS), default="cube")
    layout.add_argument("--views", type=Path, metavar="JSON",
                        help="List of {name, yaw_deg, pitch_deg, roll_deg}")
    views.add_argument("--size", type=int, default=1024, metavar="PIXELS")
    views.add_argument("--fov-deg", type=float, default=90.0)
    views.add_argument("--interp", choices=INTERPOLATIONS, default="lanczos")
    views.add_argument("--ffmpeg", default="ffmpeg")
    views.add_argument("--ffprobe", default="ffprobe")
    livox = commands.add_parser(
        "import-livox", help="Convert a Livox rosbag2 recording (ROS 2 required)")
    livox.add_argument("bag", type=Path)
    livox.add_argument("--output", type=Path, required=True)
    livox.add_argument("--lidar-topic", default="/livox/lidar")
    livox.add_argument("--imu-topic", default="/livox/imu")
    livox.add_argument("--allow-unsynced", action="store_true",
                       help="Write a provisional recording when PTP sync is not evident")
    view = commands.add_parser(
        "ros-pointcloud", help="Republish Livox CustomMsg as PointCloud2 for viewers (ROS 2)")
    view.add_argument("--input", default="/livox/lidar")
    view.add_argument("--output", default="/splatrig/points")
    live = commands.add_parser(
        "live", help="Local live console: LiDAR cloud, MAX2 preview and recording (ROS 2)")
    live.add_argument("--port", type=int, default=8090)
    live.add_argument("--captures", type=Path, default=Path.home() / "captures")
    live.add_argument("--gopro", default="auto", help="Camera IP, 'auto' or 'none'")
    live.add_argument("--no-preview", action="store_true",
                      help="Do not start the MAX2 preview automatically")
    live.add_argument("--lidar-topic", default="/livox/lidar")
    live.add_argument("--imu-topic", default="/livox/imu")
    glim = commands.add_parser(
        "glim-bag", help="Copy a Livox bag as PointCloud2 with per-point time for GLIM (ROS 2)")
    glim.add_argument("bag", type=Path)
    glim.add_argument("--output", type=Path, required=True)
    glim.add_argument("--lidar-topic", default="/livox/lidar")
    glim.add_argument("--imu-topic", default="/livox/imu")
    glim.add_argument("--points-topic", default="/livox/points")
    glim_config = commands.add_parser(
        "glim-config", help="Write a GLIM configuration for a Livox Mid-360(S)")
    glim_config.add_argument("--output", type=Path, required=True)
    glim_config.add_argument("--gpu", action="store_true", help="Use GLIM's GPU modules")
    max2 = commands.add_parser(
        "max2-frames", help="Decode GoPro MAX2 .360 frames to equirectangular panoramas")
    max2.add_argument("video", type=Path)
    max2.add_argument("--output", type=Path, required=True)
    max2.add_argument("--every", type=int, default=15, help="Keep every Nth original frame")
    max2.add_argument("--width", type=int, default=3840)
    max2.add_argument("--format", choices=["jpg", "png"], default="jpg")
    process = commands.add_parser(
        "process-capture", help="Capture folder from `live` -> provisional COLMAP dataset (ROS 2, GLIM)")
    process.add_argument("capture", type=Path)
    process.add_argument("--output", type=Path, required=True)
    process.add_argument("--camera-offset", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                         metavar=("X", "Y", "Z"), help="MAX2 center in the LiDAR frame, meters")
    rotation = process.add_mutually_exclusive_group(required=True)
    rotation.add_argument("--camera-ypr", type=float, nargs=3, metavar=("YAW", "PITCH", "ROLL"),
                          help="Hand-entered MAX2 front relative to LiDAR +X, degrees")
    rotation.add_argument("--calibration", type=Path, help="calibration.json from calibrate-gyro")
    process.add_argument("--video", type=Path, help="Local .360 file instead of downloading")
    process.add_argument("--gopro", default="auto", help="Camera IP, 'auto' or 'none'")
    process.add_argument("--every", type=int, default=15)
    process.add_argument("--width", type=int, default=3840)
    process.add_argument("--size", type=int, default=1024)
    process.add_argument("--layout", choices=sorted(PRESETS), default="cube")
    gyro = commands.add_parser(
        "calibrate-gyro", help="MAX2-to-LiDAR rotation and clock offset from a rotation capture")
    gyro.add_argument("capture", type=Path)
    gyro.add_argument("--output", type=Path, required=True, help="New calibration directory")
    gyro.add_argument("--video", type=Path, help="Local .360 file instead of downloading")
    gyro.add_argument("--gopro", default="auto", help="Camera IP, 'auto' or 'none'")
    trajectory = commands.add_parser(
        "import-trajectory", help="Convert a TUM trajectory (e.g. GLIM traj_imu.txt)")
    trajectory.add_argument("tum", type=Path)
    trajectory.add_argument("--output", type=Path, required=True)
    trajectory.add_argument("--revision", type=int, required=True)
    trajectory.add_argument("--status", choices=["live", "final"], required=True)
    trajectory.add_argument("--frame", default="imu", help="Tracked frame, e.g. imu")
    clock = commands.add_parser("fit-clock", help="Fit paired physical-event timestamps")
    clock.add_argument("events", type=Path, help="CSV columns: camera_ns,reference_ns")
    clock.add_argument("--output", type=Path, required=True)
    clock.add_argument("--max-residual-ms", type=float, default=5.0)
    poses = commands.add_parser("record-poses", help="Record live ROS 2 Odometry (ROS required)")
    poses.add_argument("--output", type=Path, required=True)
    poses.add_argument("--topic", required=True)
    poses.add_argument("--world-frame", required=True)
    poses.add_argument("--rig-frame", required=True)
    return root


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == "demo":
            report = (create_panorama_demo if args.panorama else create_demo)(args.output)
        elif args.command == "panorama-views":
            report = panorama_views(args.session, args.output, layout=args.layout,
                                    views_file=args.views, size=args.size,
                                    fov_deg=args.fov_deg, interp=args.interp,
                                    ffmpeg=args.ffmpeg, ffprobe=args.ffprobe)
        elif args.command == "import-livox":
            report = import_livox(args.bag, args.output, lidar_topic=args.lidar_topic,
                                  imu_topic=args.imu_topic,
                                  allow_unsynced=args.allow_unsynced)
        elif args.command == "ros-pointcloud":
            view_pointcloud(args.input, args.output)
            report = {"input": args.input, "output": args.output}
        elif args.command == "live":
            from .gopro import detect_camera
            from .live import serve
            host = {"none": None, "auto": None}.get(args.gopro, args.gopro)
            if args.gopro == "auto":
                host = detect_camera()
            serve(args.port, args.captures, host, args.lidar_topic, args.imu_topic,
                  preview=not args.no_preview)
            report = {"live": "stopped"}
        elif args.command == "glim-bag":
            report = to_glim_bag(args.bag, args.output, lidar_topic=args.lidar_topic,
                                 imu_topic=args.imu_topic, points_topic=args.points_topic)
        elif args.command == "glim-config":
            report = write_glim_config(args.output, gpu=args.gpu)
        elif args.command == "process-capture":
            from .gopro import detect_camera
            from .pipeline import process_capture
            host = detect_camera() if args.gopro == "auto" else (
                None if args.gopro == "none" else args.gopro)
            report = process_capture(args.capture, args.output,
                                     camera_offset=tuple(args.camera_offset),
                                     camera_ypr=tuple(args.camera_ypr) if args.camera_ypr else None,
                                     calibration=args.calibration, video=args.video,
                                     gopro_host=host, every=args.every, width=args.width,
                                     view_size=args.size, layout=args.layout)
        elif args.command == "calibrate-gyro":
            from .gopro import detect_camera
            from .pipeline import calibrate_capture
            host = detect_camera() if args.gopro == "auto" else (
                None if args.gopro == "none" else args.gopro)
            report = calibrate_capture(args.capture, args.output, video=args.video,
                                       gopro_host=host)
        elif args.command == "max2-frames":
            report = extract_max2(args.video, args.output, every=args.every, width=args.width,
                                  image_format=args.format)
        elif args.command == "import-trajectory":
            report = import_trajectory(args.tum, args.output, revision=args.revision,
                                       status=args.status, frame=args.frame)
        elif args.command == "fit-clock":
            if args.output.exists():
                raise ValueError(f"Clock output already exists: {args.output}")
            with args.events.open(newline="", encoding="utf-8") as stream:
                pairs = [(int(row["camera_ns"]), int(row["reference_ns"]))
                         for row in csv.DictReader(stream)]
            clock, residuals = fit_clock(pairs, round(args.max_residual_ms*1_000_000))
            args.output.parent.mkdir(parents=True, exist_ok=True)
            report = {**clock.to_dict(), "fit": residuals}
            write_json(args.output, report)
        elif args.command == "record-poses":
            record_poses(args.output, args.topic, args.world_frame, args.rig_frame)
            report = {"output": str(args.output), "trajectory_status": "live"}
        else:
            data = prepare(args.session, allow_provisional=args.allow_provisional,
                           voxel_size=args.voxel_size, max_voxels=args.max_voxels)
            report = data.report
            if args.command == "replay":
                replay(data, args.output)
            elif args.command == "export":
                export_colmap(data, args.output)
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        print(f"splatrig: {error}", file=sys.stderr)
        return 2
