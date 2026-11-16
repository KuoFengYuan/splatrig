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
                  camera_rotation_wxyz=None, clock_method="record-command acknowledgement",
                  t_lidar_imu=MID360_T_LIDAR_IMU, max_pose_gap_ns=200_000_000,
                  min_range_m=0.0) -> dict:
    """Session from a recording, trajectory and panoramas; points closer than
    min_range_m to the LiDAR (the operator, the mount) are left out and counted."""
    source = read_json(trajectory / "trajectory-source.json")
    poses = list(read_jsonl(trajectory / "trajectory.jsonl"))
    first, last = poses[0]["timestamp_ns"], poses[-1]["timestamp_ns"]
    revision = source["revision"]
    T_lidar_imu = Pose(t_lidar_imu[:3], (t_lidar_imu[6], *t_lidar_imu[3:6]))
    T_rig_lidar = T_lidar_imu.inverse()  # the rig frame is the IMU tracked by SLAM
    T_lidar_sphere = (Pose(camera_offset, camera_rotation_wxyz) if camera_rotation_wxyz
                      else lidar_sphere(camera_offset, *camera_ypr))
    T_rig_sphere = T_rig_lidar.compose(T_lidar_sphere)
    clock = Clock(0, camera_start_ns, 1.0, "provisional")
    lidar_report = read_json(recording / "recording.json")
    for source_dir in (trajectory, panoramas):
        resolved = destination.resolve()
        if resolved == source_dir.resolve() or resolved.is_relative_to(source_dir.resolve()):
            raise ValueError("Output must be outside every input directory")
    with new_output(destination, recording) as root:
        before = after = kept = near = 0
        near_squared = min_range_m ** 2
        header, _, _ = read_header(recording / "points.bin")
        with PointLogWriter(root / "points.bin", {**{k: v for k, v in header.items()
                                                     if k not in {"record_bytes", "fields"}},
                                                  "trimmed_to_trajectory": [first, last],
                                                  "min_range_m": min_range_m}) as out:
            for point in iter_points(recording / "points.bin"):
                if point[0] < first:
                    before += 1
                elif point[0] > last:
                    after += 1
                elif point[1] * point[1] + point[2] * point[2] + point[3] * point[3] < near_squared:
                    near += 1
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
                  "camera_rotation": ("gyro calibration" if camera_rotation_wxyz
                                      else {"hand_entered_ypr_deg": list(camera_ypr)}),
                  "T_lidar_sphere": T_lidar_sphere.to_dict(), "rig_status": "uncalibrated",
                  "clock_status": "provisional", "clock_method": clock_method,
                  "camera_start_ns": camera_start_ns,
                  "trajectory_ns": [first, last], "lidar_clock": lidar_report["clock"]["status"],
                  "points_kept": kept, "points_before_trajectory": before,
                  "min_range_m": min_range_m, "points_near_sensor": near,
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


STANDARD_GRAVITY = 9.80665


def read_imus(capture: Path, video: Path) -> tuple[dict, dict]:
    """MAX2 GPMF GYRO/ACCL and the Mid-360 IMU (gyro rad/s, accelerometer m/s^2)."""
    from .gpmf import read_imu
    from .livox import IMU_TYPE, read_bag
    from .max2 import probe
    metadata = probe(video)["metadata"]
    if not metadata:
        raise ValueError(f"{video.name}: no GPMF metadata track")
    camera = read_imu(video, metadata[0])
    lidar = {"times_ns": [], "gyro": [], "accel": []}
    for _, _, message in read_bag(capture / "lidar", {"/livox/imu": IMU_TYPE}):
        lidar["times_ns"].append(message.header.stamp.sec * 1_000_000_000 +
                                 message.header.stamp.nanosec)
        g, a = message.angular_velocity, message.linear_acceleration
        lidar["gyro"].append((g.x, g.y, g.z))
        # The Mid-360(S) driver reports linear acceleration in g.
        lidar["accel"].append((a.x * STANDARD_GRAVITY, a.y * STANDARD_GRAVITY,
                               a.z * STANDARD_GRAVITY))
    return camera, lidar


def gyro_clock(capture: Path, video: Path, guess_ns: int, rotation=None, imus=None) -> dict:
    """Per-capture camera clock (and optionally rotation) from both gyroscopes."""
    from .gyrocal import calibrate
    camera, lidar = imus or read_imus(capture, video)
    return calibrate(camera["GYRO"]["times_ns"], camera["GYRO"]["samples"], lidar["times_ns"],
                     lidar["gyro"], guess_ns, rotation)


def camera_offset_from_imus(camera: dict, lidar: dict, result: dict,
                            t_lidar_imu=MID360_T_LIDAR_IMU, max_half_difference_m=0.02,
                            max_residual_ratio=0.5, min_strength=0.1) -> dict:
    """MAX2 position in the LiDAR frame from both accelerometers (see solve_lever_arm).

    The MAX2's IMU stands in for its lens center; their separation inside the
    camera is unknown (about 1-3 cm). Returns the lever-arm diagnostics with
    camera_offset_m, or with a reason when the fit is not trusted.
    """
    from .gyrocal import solve_lever_arm
    if "ACCL" not in camera:
        return {"camera_offset_m": None, "reason": "no MAX2 accelerometer stream"}
    if camera["ACCL"].get("orin") != camera["GYRO"].get("orin"):
        return {"camera_offset_m": None, "reason": "MAX2 gyro and accelerometer axes differ"}
    try:
        lever = solve_lever_arm(camera["ACCL"]["times_ns"], camera["ACCL"]["samples"],
                                lidar["times_ns"], lidar["gyro"], lidar["accel"],
                                result["offset_ns"], result["R_lidar_gpmf"])
    except ValueError as error:
        return {"camera_offset_m": None, "reason": str(error)}
    # The IMU axes are the LiDAR axes (identity rotation in T_lidar_imu).
    offset = [round(a + b, 4) for a, b in zip(t_lidar_imu[:3], lever["r_m"])]
    problems = []
    if lever["half_difference_m"] > max_half_difference_m:
        problems.append(f"the two halves of the capture disagree by "
                        f"{lever['half_difference_m'] * 100:.1f} cm")
    if lever["residual_rms_m_s2"] > max_residual_ratio * lever["signal_rms_m_s2"]:
        problems.append("the accelerometers do not fit a rigid mount")
    if lever["weakest_direction_ratio"] < min_strength:
        problems.append("one direction was barely excited; rotate about all three axes")
    return {**lever, "camera_offset_m": None if problems else offset,
            "estimate_m": offset, "reason": "; ".join(problems) or None,
            "note": "MAX2 IMU position used for the lens center; their separation inside "
                    "the camera (about 1-3 cm) is not modeled"}


def process_capture(capture: Path, output: Path, *, camera_offset=None,
                    camera_ypr=None, calibration: Path | None = None,
                    video: Path | None = None, gopro_host: str | None = None, every=None,
                    spacing_m=0.3, spacing_deg=20.0, max_turn_dps=30.0, width=3840,
                    view_size=1024, layout="cube-no-down", min_range_m=0.6, masks=True,
                    max_mask_fraction=0.25, colors=True, voxel_m=0.03, max_voxels=2_000_000,
                    glim_domain=73, min_clock_correlation=0.9, progress=None) -> dict:
    """Capture folder from `splatrig live` -> provisional COLMAP dataset.

    Frames are chosen along the trajectory (every=None) or every Nth frame; points
    within min_range_m of the LiDAR are dropped; masks/ marks the operator and mount.
    progress, if given, is called with the name of each step as it starts.
    """
    from .colorize import colorize, session_panoramas
    from .export import export_colmap
    from .geometry import Trajectory
    from .glim import import_trajectory, to_glim_bag, write_glim_config
    from .gopro import GoPro
    from .keyframes import select_keyframes
    from .livox import import_livox
    from .max2 import extract, frame_times_ns, probe
    from .panorama import panorama_views
    from .selfmask import estimate, write_masks
    from .session import VoxelBudget, prepare

    def step(name):
        if progress:
            progress(name)

    capture, output = capture.resolve(), output.resolve()
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    if output.is_relative_to(capture):
        raise ValueError("Output must be outside the capture")
    if (camera_ypr is None) == (calibration is None):
        raise ValueError("Give exactly one of a hand-entered camera rotation or a calibration")
    rotation = read_json(calibration) if calibration else None
    if camera_offset is None:
        camera_offset = (rotation or {}).get("camera_offset_m")
        if camera_offset is None:
            raise ValueError("No camera offset: give --camera-offset, or a calibration whose "
                             "accelerometer fit estimated it")
        offset_source = "estimated from both accelerometers during calibration"
    else:
        offset_source = "entered by hand"
    camera_offset = tuple(camera_offset)
    log = read_json(capture / "capture-log.json")
    gopro = log.get("gopro", {})
    work = output.with_name(f".{output.name}-work")
    if work.exists():
        raise ValueError(f"Working directory already exists: {work}")
    work.mkdir(parents=True)
    steps = {}
    try:
        step("lidar")
        steps["lidar"] = import_livox(capture / "lidar", work / "lidar")
        step("slam")
        steps["glim_bag"] = to_glim_bag(capture / "lidar", work / "glim-input")
        write_glim_config(work / "glim-config")
        run_glim(work / "glim-input/bag", work / "glim-config", work / "glim-dump",
                 work / "glim.log", glim_domain)
        steps["trajectory"] = import_trajectory(work / "glim-dump/traj_imu.txt",
                                                work / "trajectory", revision=1,
                                                status="final", frame="imu")
        trajectory = Trajectory(read_jsonl(work / "trajectory/trajectory.jsonl"))
        step("video")
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
        camera_start, method = gopro["start_ack_ns"], "record-command acknowledgement"
        try:
            clock = gyro_clock(capture, video, gopro["start_ack_ns"],
                               rotation["R_lidar_gpmf"] if rotation else None)
            steps["gyro_clock"] = {k: clock[k] for k in ("offset_ns", "magnitude_correlation")}
            if clock["magnitude_correlation"] >= min_clock_correlation:
                camera_start, method = clock["offset_ns"], "gyro |w| correlation"
            else:
                steps["gyro_clock"]["unused"] = "correlation too low (too little rotation?)"
        except ValueError as error:
            steps["gyro_clock"] = {"unused": str(error)}
        step("frames")
        frames = times = None
        if every is None:
            track = probe(video)["video"][0]
            times = frame_times_ns(video, track["index"], track["time_base"])
            chosen = select_keyframes(times, camera_start, trajectory, spacing_m=spacing_m,
                                      spacing_deg=spacing_deg, max_turn_dps=max_turn_dps)
            steps["keyframes"] = chosen["report"]
            if not chosen["frames"]:
                raise ValueError("No video frame is inside the trajectory and turning slowly "
                                 "enough; check the camera clock or pass --every")
            frames = chosen["frames"]
        steps["panoramas"] = extract(video, work / "panoramas", every=every or 1, frames=frames,
                                     width=width, times=times)
        step("assembly")
        steps["assembly"] = build_session(
            work / "session-panorama", recording=work / "lidar", trajectory=work / "trajectory",
            panoramas=work / "panoramas", camera_start_ns=camera_start,
            camera_offset=camera_offset, camera_ypr=camera_ypr or (0.0, 0.0, 0.0),
            camera_rotation_wxyz=rotation["q_lidar_sphere_wxyz"] if rotation else None,
            clock_method=method, min_range_m=min_range_m)
        step("views")
        steps["views"] = panorama_views(work / "session-panorama", work / "session",
                                        layout=layout, size=view_size)
        rig = read_json(work / "session-panorama/rig.json")
        T_rig_sphere = Pose.from_dict(rig["panorama"]["T_rig_sphere"])
        bins = None
        if masks:
            step("masks")
            found = estimate(work / "lidar/points.bin", trajectory,
                             Pose.from_dict(steps["assembly"]["T_lidar_sphere"]))
            steps["self_mask"] = {k: v for k, v in found.items() if k != "bins"}
            plausible = found["sphere_fraction"] <= max_mask_fraction
            if found["bins"] and plausible:
                bins = found["bins"]
            else:
                steps["self_mask"]["masks"] = 0
                steps["self_mask"]["reason"] = (
                    f"rig travelled {found['motion_extent_m']} m while moving, under "
                    f"{found['min_extent_m']} m" if not found["estimated"] else
                    f"mask would cover {found['sphere_fraction']:.0%} of the sphere, more than "
                    f"{max_mask_fraction:.0%}; nearby scene was probably taken for the rig"
                    if not plausible else "no LiDAR direction returned mostly near points")
        step("colors")
        try:
            data = prepare(work / "session", allow_provisional=True, voxel_size=voxel_m,
                           max_voxels=max_voxels)
        except VoxelBudget:
            data = prepare(work / "session", allow_provisional=True, voxel_size=0.05,
                           max_voxels=max_voxels)
            steps["voxel_fallback"] = (f"{max_voxels} voxels reached at {voxel_m} m; "
                                       "the map uses 0.05 m voxels")
        if colors:
            shots = session_panoramas(work / "session-panorama", trajectory,
                                      Clock.from_dict(read_json(work / "session-panorama/clock.json")),
                                      T_rig_sphere,
                                      read_jsonl(work / "session-panorama/panoramas.jsonl"))
            data.points, steps["colors"] = colorize(data.points, data.point_times, shots,
                                                    masked=bins)
            data.report["point_colors"] = "from panoramas nearest in time to each closest return"
        step("export")
        export_colmap(data, work / "dataset")
        steps["dataset"] = data.report
        if bins:
            steps["self_mask"].update(write_masks(
                work / "dataset", read_jsonl(work / "dataset/image-poses.jsonl"),
                data.cameras, T_rig_sphere, bins))
        summary = {"capture": str(capture), "provisional": True,
                   "dataset": str(output / "dataset"),
                   "camera_offset_m": list(camera_offset), "camera_offset_source": offset_source,
                   "reasons": [f"camera translation {offset_source}, not verified against "
                               "the images (rig uncalibrated)",
                               f"camera clock from {method}; exposure timing not verified"],
                   "steps": steps}
        write_json(work / "process.json", summary)
    except BaseException:
        work.rename(output.with_name(f".{output.name}-failed"))
        raise
    work.rename(output)
    return summary


def calibrate_capture(capture: Path, output: Path, *, video: Path | None = None,
                      gopro_host: str | None = None, min_correlation=0.95,
                      max_residual_ratio=0.1, progress=None) -> dict:
    """Write calibration.json: rotation, camera offset (from the accelerometers when
    the fit is trusted), this capture's clock offset and diagnostics."""
    from .export import new_output
    from .gopro import GoPro
    capture = capture.resolve()
    log = read_json(capture / "capture-log.json")
    gopro = log.get("gopro", {})
    if "start_ack_ns" not in gopro:
        raise ValueError("capture-log.json has no MAX2 recording")
    with new_output(output, capture) as root:
        if progress:
            progress("video")
        if video is None:
            media = [name for name in gopro.get("new_media", []) if name.endswith(".360")]
            if len(media) != 1 or not gopro_host:
                raise ValueError("Pass --video, or connect the MAX2 with exactly one new .360 file")
            video = root / Path(media[0]).name
            GoPro(gopro_host, timeout=60).download(media[0], video)
        if progress:
            progress("fit")
        imus = read_imus(capture, video)
        result = gyro_clock(capture, video, gopro["start_ack_ns"], imus=imus)
        fit = result["fit"]
        ratio = fit["residual_rms_rad_s"] / fit["signal_rms_rad_s"]
        if result["magnitude_correlation"] < min_correlation or ratio > max_residual_ratio:
            raise ValueError(f"Gyro fit too weak (|w| correlation "
                             f"{result['magnitude_correlation']:.3f}, residual ratio {ratio:.3f}); "
                             "rotate the rig about all three axes")
        lever = camera_offset_from_imus(*imus, result)
        if video.parent == root:
            video.unlink()
        report = {"format": "splatrig-gyro-calibration", "version": 1,
                  "capture": str(capture), "video": video.name, **result,
                  "offset_vs_command_ack_ms": (result["offset_ns"] - gopro["start_ack_ns"]) / 1e6,
                  "residual_ratio": ratio,
                  "weak_axis": fit["singular_value_ratios"][2] < 0.05,
                  "camera_offset_m": lever["camera_offset_m"], "lever_arm": lever,
                  "scope": "rotation and camera offset valid while the mount is unchanged; "
                           "the clock offset applies to this capture only",
                  "not_determined": ["MAX2 IMU-to-lens separation inside the camera",
                                     "exposure/rolling-shutter timing"]}
        write_json(root / "calibration.json", report)
    return report
