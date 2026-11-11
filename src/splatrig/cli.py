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
