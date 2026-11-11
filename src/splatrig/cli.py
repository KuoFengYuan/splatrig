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
from .demo import create_demo
from .export import export_colmap, replay
from .ros import record_poses
from .session import prepare, write_json


def parser():
    root = argparse.ArgumentParser(
        prog="splatrig", description="Pose-aware capture datasets for Gaussian splatting.")
    root.add_argument("--version", action="version", version=__version__)
    commands = root.add_subparsers(dest="command", required=True)
    demo = commands.add_parser("demo", help="Create a synthetic room capture; no hardware needed")
    demo.add_argument("--output", type=Path, required=True)
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
            report = create_demo(args.output)
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
