# SPDX-License-Identifier: MIT
"""Assemble a panorama session from a LiDAR recording, a SLAM trajectory and MAX2 frames.

Version 1 uses provisional inputs that stay visibly marked: a hand-measured
camera mount (rig status "uncalibrated") and a camera clock anchored to the host
time at which the camera acknowledged the record command (clock "provisional").
Final export therefore refuses this session unless --allow-provisional is given.
"""

from __future__ import annotations

import math
from pathlib import Path
import shutil

from .clock import Clock
from .export import new_output
from .geometry import Pose, multiply
from .glim import MID360_T_LIDAR_IMU
from .pointlog import PointLogWriter, iter_points, read_header
from .session import local_file, read_json, read_jsonl, write_json, write_jsonl

# OpenCV sphere frame S looking along LiDAR +X with image-up along LiDAR +Z:
# S +Z -> LiDAR +X, S +X -> LiDAR -Y, S +Y -> LiDAR -Z.
SPHERE_FORWARD = (0.5, -0.5, 0.5, -0.5)


def axis_angle(axis: int, degrees: float):
    half = math.radians(degrees) / 2
    q = [math.cos(half), 0.0, 0.0, 0.0]
    q[axis] = math.sin(half)
    return tuple(q)


def lidar_sphere(offset, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0) -> Pose:
    """T_lidar_sphere for a camera whose front is turned from LiDAR +X.

    Positive yaw turns the front toward LiDAR +Y, positive pitch tilts it up and
    positive roll turns it about its own forward axis (right-handed about +X).
    """
    q = multiply(axis_angle(3, yaw_deg), multiply(axis_angle(2, -pitch_deg),
                                                  multiply(axis_angle(1, roll_deg),
                                                           SPHERE_FORWARD)))
    return Pose(offset, q)


def build_session(destination: Path, *, recording: Path, trajectory: Path, panoramas: Path,
                  camera_start_ns: int, camera_offset=(0.0, 0.0, 0.0), camera_ypr=(0.0, 0.0, 0.0),
                  t_lidar_imu=MID360_T_LIDAR_IMU, max_pose_gap_ns=200_000_000) -> dict:
    source = read_json(trajectory / "trajectory-source.json")
    poses = list(read_jsonl(trajectory / "trajectory.jsonl"))
    first, last = poses[0]["timestamp_ns"], poses[-1]["timestamp_ns"]
    revision = source["revision"]
    T_lidar_imu = Pose(t_lidar_imu[:3], (t_lidar_imu[6], *t_lidar_imu[3:6]))
    T_rig_lidar = T_lidar_imu.inverse()  # the rig frame is the IMU tracked by SLAM
    T_rig_sphere = T_rig_lidar.compose(lidar_sphere(camera_offset, *camera_ypr))
    clock = Clock(0, camera_start_ns, 1.0, "provisional")
    lidar_report = read_json(recording / "recording.json")
    for source_dir in (trajectory, panoramas):
        resolved = destination.resolve()
        if resolved == source_dir.resolve() or resolved.is_relative_to(source_dir.resolve()):
            raise ValueError("Output must be outside every input directory")
    with new_output(destination, recording) as root:
        before = after = kept = 0
        header, _, _ = read_header(recording / "points.bin")
        with PointLogWriter(root / "points.bin", {**{k: v for k, v in header.items()
                                                     if k not in {"record_bytes", "fields"}},
                                                  "trimmed_to_trajectory": [first, last]}) as out:
            for point in iter_points(recording / "points.bin"):
                if point[0] < first:
                    before += 1
                elif point[0] > last:
                    after += 1
                else:
                    out.write(*point)
                    kept += 1
        (root / "images").mkdir()
        frames, outside = [], []
        for item in read_jsonl(panoramas / "panoramas.jsonl"):
            reference = clock.map(item["timestamp_ns"])
            if not first <= reference <= last:
                outside.append(item["id"])
                continue
            image = local_file(panoramas, item["image_path"])
            name = f"images/{image.name}"
            shutil.copyfile(image, root / name)
            frames.append({**item, "image_path": name})
        if not frames:
            raise ValueError("No panorama falls inside the trajectory; check the camera clock")
        write_jsonl(root / "panoramas.jsonl", frames)
        write_jsonl(root / "trajectory.jsonl", poses)
        write_json(root / "clock.json", clock.to_dict())
        write_json(root / "capture-meta.json", {
            "schema_version": 1, "source": "gopro_max2_mid360", "world_up": "+Z",
            "units": "meters", "camera_convention": "opencv", "point_frame": "lidar",
            "trajectory_status": source["trajectory_status"], "pose_revision": revision,
            "max_pose_gap_ns": max_pose_gap_ns,
            "note": "Version-1 assembly: hand-measured camera mount and command-time clock."})
        write_json(root / "rig.json", {
            "status": "uncalibrated", "T_rig_lidar": T_rig_lidar.to_dict(), "cameras": [],
            "panorama": {"projection": "equirectangular", "T_rig_sphere": T_rig_sphere.to_dict()}})
        report = {"rig_frame": source.get("tracked_frame", "imu"),
                  "T_lidar_imu": list(t_lidar_imu), "camera_offset_m": list(camera_offset),
                  "camera_ypr_deg": list(camera_ypr), "rig_status": "uncalibrated",
                  "clock_status": "provisional", "camera_start_ns": camera_start_ns,
                  "trajectory_ns": [first, last], "lidar_clock": lidar_report["clock"]["status"],
                  "points_kept": kept, "points_before_trajectory": before,
                  "points_after_trajectory": after, "panoramas_kept": len(frames),
                  "panoramas_outside_trajectory": outside}
        write_json(root / "assembly.json", report)
    return report


def run_glim(bag: Path, config: Path, dump: Path, log: Path, domain_id: int):
    """Run GLIM offline on an otherwise unused ROS domain so live topics stay out."""
    import os
    import subprocess
    env = {**os.environ, "ROS_DOMAIN_ID": str(domain_id)}
    command = ["ros2", "run", "glim_ros", "glim_rosbag", str(bag), "--ros-args",
               "-p", f"config_path:={config}", "-p", f"dump_path:={dump}",
               "-p", "auto_quit:=true"]
    with log.open("xb") as stream:
        try:
            code = subprocess.run(command, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                  timeout=7200).returncode
        except FileNotFoundError as error:
            raise ValueError("ros2 is unavailable; source ROS 2 and install GLIM") from error
    if code or not (dump / "traj_imu.txt").is_file():
        raise ValueError(f"GLIM failed (exit {code}); see {log.name}")


def process_capture(capture: Path, output: Path, *, camera_offset, camera_ypr,
                    video: Path | None = None, gopro_host: str | None = None, every=15,
                    width=3840, view_size=1024, layout="cube", glim_domain=73) -> dict:
    """Capture folder from `splatrig live` -> provisional COLMAP dataset."""
    from .export import export_colmap
    from .glim import import_trajectory, to_glim_bag, write_glim_config
    from .gopro import GoPro
    from .livox import import_livox
    from .max2 import extract
    from .panorama import panorama_views
    from .session import prepare

    capture, output = capture.resolve(), output.resolve()
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if output.is_relative_to(capture):
        raise ValueError("Output must be outside the capture")
    log = read_json(capture / "capture-log.json")
    gopro = log.get("gopro", {})
    work = output.with_name(f".{output.name}-work")
    if work.exists():
        raise ValueError(f"Working directory already exists: {work}")
    work.mkdir(parents=True)
    steps = {}
    try:
        steps["lidar"] = import_livox(capture / "lidar", work / "lidar")
        steps["glim_bag"] = to_glim_bag(capture / "lidar", work / "glim-input")
        write_glim_config(work / "glim-config")
        run_glim(work / "glim-input/bag", work / "glim-config", work / "glim-dump",
                 work / "glim.log", glim_domain)
        steps["trajectory"] = import_trajectory(work / "glim-dump/traj_imu.txt",
                                                work / "trajectory", revision=1,
                                                status="final", frame="imu")
        if video is None:
            media = [name for name in gopro.get("new_media", []) if name.endswith(".360")]
            if len(media) != 1:
                raise ValueError(f"Expected one new MAX2 .360 file, found {media}; "
                                 "pass --video for other cases")
            if not gopro_host:
                raise ValueError("No GoPro connected to download the recording; pass --video")
            (work / "media").mkdir()
            video = work / "media" / Path(media[0]).name
            GoPro(gopro_host, timeout=60).download(media[0], video)
        if "start_ack_ns" not in gopro:
            raise ValueError("capture-log.json has no MAX2 start time for the provisional clock")
        steps["panoramas"] = extract(video, work / "panoramas", every=every, width=width)
        steps["assembly"] = build_session(
            work / "session-panorama", recording=work / "lidar", trajectory=work / "trajectory",
            panoramas=work / "panoramas", camera_start_ns=gopro["start_ack_ns"],
            camera_offset=camera_offset, camera_ypr=camera_ypr)
        steps["views"] = panorama_views(work / "session-panorama", work / "session",
                                        layout=layout, size=view_size)
        data = prepare(work / "session", allow_provisional=True)
        export_colmap(data, work / "dataset")
        steps["dataset"] = data.report
        summary = {"capture": str(capture), "provisional": True,
                   "dataset": str(output / "dataset"),
                   "reasons": ["camera mount hand-measured (rig uncalibrated)",
                               "camera clock anchored to record-command acknowledgement"],
                   "steps": steps}
        write_json(work / "process.json", summary)
    except BaseException:
        work.rename(output.with_name(f".{output.name}-failed"))
        raise
    work.rename(output)
    return summary
