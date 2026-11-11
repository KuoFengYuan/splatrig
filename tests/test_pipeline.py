# SPDX-License-Identifier: MIT
from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from splatrig.cli import main
from splatrig.demo import png_bytes
from splatrig.export import export_colmap, replay
from splatrig.geometry import Pose
from splatrig.ros import odometry_record, record_poses
from splatrig.session import prepare, read_json, read_jsonl, write_json, write_jsonl


def fixture(root):
    root.mkdir()
    (root / "images").mkdir()
    write_json(root / "capture-meta.json", {
        "schema_version": 1, "source": "synthetic", "world_up": "+Z",
        "units": "meters", "camera_convention": "opencv", "point_frame": "lidar",
        "trajectory_status": "final", "pose_revision": 1, "max_pose_gap_ns": 2_000_000_000})
    write_json(root / "rig.json", {
        "status": "calibrated", "T_rig_lidar": Pose.identity().to_dict(),
        "cameras": [{"id": 1, "model": "PINHOLE", "width": 2, "height": 2,
                     "params": [2, 2, 1, 1],
                     "T_rig_camera": Pose((0.1, 0, 0), (1, 0, 0, 0)).to_dict()}]})
    write_json(root / "clock.json", {
        "camera_anchor_ns": 0, "reference_anchor_ns": 100_000_000_000,
        "rate": 1, "status": "calibrated"})
    write_jsonl(root / "trajectory.jsonl", [
        {"timestamp_ns": 100_000_000_000, "revision": 1, **Pose.identity().to_dict()},
        {"timestamp_ns": 101_000_000_000, "revision": 1,
         **Pose((1, 0, 0), (1, 0, 0, 0)).to_dict()}])
    for index in [1, 2]:
        (root / f"images/{index}.png").write_bytes(
            png_bytes(2, 2, [bytes([255, 0, 0]*2)]*2))
    write_jsonl(root / "frames.jsonl", [
        {"id": 1, "camera_id": 1, "timestamp_ns": 500_000_000, "image_path": "images/1.png"},
        {"id": 2, "camera_id": 1, "timestamp_ns": 1_000_000_000, "image_path": "images/2.png"}])
    write_jsonl(root / "points.jsonl", [
        {"timestamp_ns": 100_500_000_000, "xyz": [0, 0, 2], "rgb": [255, 0, 0]},
        {"timestamp_ns": 101_000_000_000, "xyz": [-0.5, 0, 2], "rgb": [0, 255, 0]}])


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.capture = self.base / "capture"
        fixture(self.capture)

    def tearDown(self):
        self.temporary.cleanup()

    def update(self, filename, key, value):
        path = self.capture / filename
        data = read_json(path)
        data[key] = value
        write_json(path, data)

    def test_moving_sensor_points_and_image_poses_share_the_world_frame(self):
        data = prepare(self.capture)
        self.assertEqual(len(data.points), 1)
        self.assertEqual(data.points[0], (0.5, 0, 2, 128, 128, 0))
        pose = Pose.from_dict(data.frames[0]["T_world_camera"])
        self.assertAlmostEqual(pose.translation[0], 0.6)
        x, y, z = pose.inverse().transform(data.points[0][:3])
        self.assertAlmostEqual(2*x/z+1, 0.9)
        self.assertAlmostEqual(2*y/z+1, 1)

    def test_colmap_binary_encodes_world_to_camera_and_empty_tracks(self):
        output = self.base / "colmap"
        export_colmap(prepare(self.capture), output)
        raw = (output / "sparse/0/images.bin").read_bytes()
        self.assertEqual(struct.unpack_from("<Q", raw)[0], 2)
        image = struct.unpack_from("<i7di", raw, 8)
        self.assertEqual(image[0], 1)
        self.assertEqual(image[1:5], (1, 0, 0, 0))
        self.assertAlmostEqual(image[5], -0.6)
        self.assertEqual(image[8], 1)
        offset = 8+struct.calcsize("<i7di")
        self.assertTrue(raw[offset:].startswith(b"00000001.png\0"))
        self.assertEqual(struct.unpack_from("<Q", raw, offset+13)[0], 0)
        cameras = (output / "sparse/0/cameras.bin").read_bytes()
        self.assertEqual(struct.unpack_from("<QiiQQ4d", cameras), (1, 1, 1, 2, 2, 2, 2, 1, 1))
        points = (output / "sparse/0/points3D.bin").read_bytes()
        self.assertEqual(struct.unpack_from("<Q", points)[0], 1)
        self.assertEqual(struct.unpack_from("<Q3d3BdQ", points, 8),
                         (1, 0.5, 0, 2, 128, 128, 0, 0, 0))
        for frame in read_jsonl(output / "image-poses.jsonl"):
            self.assertTrue((output / frame["image_path"]).is_file())
            self.assertTrue((self.capture / frame["source_image_path"]).is_file())

    def test_export_preserves_captured_bytes(self):
        before = {p.relative_to(self.capture): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in self.capture.rglob("*") if p.is_file()}
        export_colmap(prepare(self.capture), self.base / "export")
        after = {p.relative_to(self.capture): hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in self.capture.rglob("*") if p.is_file()}
        self.assertEqual(before, after)

    def test_rotated_camera_exports_analytic_world_to_camera(self):
        poses = list(read_jsonl(self.capture / "trajectory.jsonl"))
        for pose in poses:
            pose["quaternion_wxyz"] = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
        (self.capture / "trajectory.jsonl").unlink()
        write_jsonl(self.capture / "trajectory.jsonl", poses)
        output = self.base / "rotated"
        export_colmap(prepare(self.capture), output)
        raw = (output / "sparse/0/images.bin").read_bytes()
        values = struct.unpack_from("<i7di", raw, 8)
        for actual, expected in zip(values[1:8],
                                    [math.sqrt(0.5), 0, 0, -math.sqrt(0.5), -0.1, 0.5, 0]):
            self.assertAlmostEqual(actual, expected)

    def test_boolean_revision_is_not_an_integer_revision(self):
        poses = list(read_jsonl(self.capture / "trajectory.jsonl"))
        poses[0]["revision"] = True
        (self.capture / "trajectory.jsonl").unlink()
        write_jsonl(self.capture / "trajectory.jsonl", poses)
        with self.assertRaisesRegex(ValueError, "revision"):
            prepare(self.capture)

    def test_cloud_and_photos_are_recomputed_for_a_new_revision(self):
        first = prepare(self.capture)
        poses = list(read_jsonl(self.capture / "trajectory.jsonl"))
        for pose in poses:
            pose["translation"][1] += 3
            pose["revision"] = 2
        (self.capture / "trajectory.jsonl").unlink()
        write_jsonl(self.capture / "trajectory.jsonl", poses)
        self.update("capture-meta.json", "pose_revision", 2)
        second = prepare(self.capture)
        self.assertAlmostEqual(second.points[0][1]-first.points[0][1], 3)
        for a, b in zip(first.frames, second.frames):
            self.assertAlmostEqual(b["T_world_camera"]["translation"][1] -
                                   a["T_world_camera"]["translation"][1], 3)
        self.assertEqual(second.report["pose_revision"], 2)

    def test_final_export_blocks_unfinalized_calibration_and_trajectory(self):
        for filename, key, value in [
                ("capture-meta.json", "trajectory_status", "live"),
                ("rig.json", "status", "uncalibrated"),
                ("clock.json", "status", "provisional")]:
            original = read_json(self.capture / filename)
            self.update(filename, key, value)
            with self.assertRaisesRegex(ValueError, "Final export"):
                prepare(self.capture)
            self.assertTrue(prepare(self.capture, allow_provisional=True).report["provisional"])
            write_json(self.capture / filename, original)

    def test_mixed_pose_revisions_are_rejected(self):
        self.update("capture-meta.json", "pose_revision", 2)
        with self.assertRaisesRegex(ValueError, "revision"):
            prepare(self.capture)

    def test_image_path_escape_and_dimension_mismatch_are_rejected(self):
        frames = list(read_jsonl(self.capture / "frames.jsonl"))
        (self.base / "outside.png").write_bytes((self.capture / "images/1.png").read_bytes())
        frames[0]["image_path"] = "../outside.png"
        (self.capture / "frames.jsonl").unlink()
        write_jsonl(self.capture / "frames.jsonl", frames)
        with self.assertRaisesRegex(ValueError, "outside"):
            prepare(self.capture)
        frames[0]["image_path"] = "images/1.png"
        (self.capture / "frames.jsonl").unlink()
        write_jsonl(self.capture / "frames.jsonl", frames)
        rig = read_json(self.capture / "rig.json")
        rig["cameras"][0]["width"] = 3
        write_json(self.capture / "rig.json", rig)
        with self.assertRaisesRegex(ValueError, "dimensions"):
            prepare(self.capture)

    def test_point_and_photo_extrapolation_are_rejected(self):
        self.update("clock.json", "reference_anchor_ns", 200_000_000_000)
        with self.assertRaisesRegex(ValueError, "outside"):
            prepare(self.capture)

    def test_voxel_budget_does_not_silently_discard_points(self):
        with (self.capture / "points.jsonl").open("a") as stream:
            stream.write(json.dumps({"timestamp_ns": 100_000_000_000,
                                     "xyz": [8, 0, 2]}) + "\n")
        with self.assertRaisesRegex(ValueError, "budget"):
            prepare(self.capture, max_voxels=1)

    def test_existing_outputs_and_nested_output_are_rejected(self):
        data = prepare(self.capture)
        output = self.base / "output"
        output.mkdir()
        (output / "sentinel").write_text("keep")
        with self.assertRaises(ValueError):
            export_colmap(data, output)
        self.assertEqual((output / "sentinel").read_text(), "keep")
        with self.assertRaises(ValueError):
            export_colmap(data, self.capture / "export")

    def test_failed_export_does_not_publish_a_partial_dataset(self):
        output = self.base / "broken"
        with patch("splatrig.export.shutil.copyfile", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                export_colmap(prepare(self.capture), output)
        self.assertFalse(output.exists())
        self.assertEqual(list(self.base.glob(".broken-*")), [])

    def test_replay_is_self_contained_and_labeled(self):
        output = self.base / "preview"
        replay(prepare(self.capture), output)
        text = (output / "preview.html").read_text()
        self.assertNotIn("__SPLATRIG_DATA__", text)
        self.assertNotIn("<script src=", text)
        self.assertTrue(read_json(output / "report.json")["synthetic"])

    def test_cli_has_a_nonzero_exit_for_invalid_input(self):
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(main(["validate", str(self.base / "missing")]), 2)
        self.assertIn("splatrig:", errors.getvalue())


class ROSConversionTests(unittest.TestCase):
    def message(self):
        return NS(header=NS(frame_id="odom", stamp=NS(sec=10, nanosec=3)),
                  child_frame_id="imu",
                  pose=NS(pose=NS(position=NS(x=1, y=2, z=3),
                                  orientation=NS(x=0, y=0, z=0, w=1))))

    def test_message_timestamp_and_quaternion_order(self):
        record = odometry_record(self.message(), "odom", "imu")
        self.assertEqual(record["timestamp_ns"], 10_000_000_003)
        self.assertEqual(record["quaternion_wxyz"], [1, 0, 0, 0])

    def test_wrong_frames_and_invalid_stamp_are_rejected(self):
        with self.assertRaises(ValueError):
            odometry_record(self.message(), "map", "imu")
        message = self.message()
        message.header.stamp.nanosec = 1_000_000_000
        with self.assertRaises(ValueError):
            odometry_record(message, "odom", "imu")

    def test_ros_dependency_is_optional_and_failure_does_not_create_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            destination = Path(temporary) / "poses.jsonl"
            with patch.dict("sys.modules", {"rclpy": None}):
                with self.assertRaisesRegex(ValueError, "ROS 2"):
                    record_poses(destination, "/odometry", "odom", "imu")
            self.assertFalse(destination.exists())


if __name__ == "__main__":
    unittest.main()
