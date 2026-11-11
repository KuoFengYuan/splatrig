# SPDX-License-Identifier: MIT
import math
from pathlib import Path
import shutil
import tempfile
import unittest

from splatrig.demo import png_bytes
from splatrig.geometry import Pose, rotate
from splatrig.pipeline import build_session, lidar_sphere
from splatrig.pointlog import PointLogWriter, iter_points
from splatrig.session import prepare, read_json, read_jsonl, write_json, write_jsonl

T0 = 1_794_382_200_000_000_000
HAVE_FFMPEG = bool(shutil.which("ffmpeg") and shutil.which("ffprobe"))


def close(a, b, tolerance=1e-9):
    return all(abs(x - y) < tolerance for x, y in zip(a, b))


def inputs(base: Path):
    recording, trajectory, panoramas = base / "lidar", base / "trajectory", base / "panoramas"
    for folder in (recording, trajectory, panoramas / "images"):
        folder.mkdir(parents=True)
    with PointLogWriter(recording / "points.bin", {"frame": "lidar"}) as writer:
        for i in range(30):  # 0.1 s apart, from T0 - 1 s to T0 + 1.9 s
            writer.write(T0 - 1_000_000_000 + i * 100_000_000, 2.0, 0.1 * i, 0.5, 10, 0, 0)
    write_json(recording / "recording.json", {"clock": {"status": "ptp_inferred"}})
    write_jsonl(trajectory / "trajectory.jsonl", [
        {"timestamp_ns": T0 + i * 100_000_000, "revision": 1,
         **Pose((0.1 * i, 0, 0), (1, 0, 0, 0)).to_dict()} for i in range(10)])
    write_json(trajectory / "trajectory-source.json",
               {"revision": 1, "trajectory_status": "final", "tracked_frame": "imu"})
    records = []
    for index, media_ns in enumerate([0, 200_000_000, 800_000_000, 1_500_000_000], 1):
        name = f"images/panorama_{index:06d}.png"
        (panoramas / name).write_bytes(png_bytes(32, 16, [bytes([90, 120, 150] * 32)] * 16))
        records.append({"id": index, "timestamp_ns": media_ns, "image_path": name})
    write_jsonl(panoramas / "panoramas.jsonl", records)
    return recording, trajectory, panoramas


class AssemblyTests(unittest.TestCase):
    def test_camera_mount_angles(self):
        forward, down = (0, 0, 1), (0, 1, 0)
        ahead = lidar_sphere((0, 0, 0))
        self.assertTrue(close(rotate(ahead.quaternion_wxyz, forward), (1, 0, 0)))
        self.assertTrue(close(rotate(ahead.quaternion_wxyz, down), (0, 0, -1)))
        self.assertTrue(close(rotate(lidar_sphere((0, 0, 0), 90).quaternion_wxyz, forward),
                              (0, 1, 0)))
        up = rotate(lidar_sphere((0, 0, 0), 0, 30).quaternion_wxyz, forward)
        self.assertTrue(close(up, (math.cos(math.radians(30)), 0, 0.5)))
        rolled = rotate(lidar_sphere((0, 0, 0), 0, 0, 90).quaternion_wxyz, down)
        self.assertTrue(close(rolled, (0, 1, 0)))  # image-down turns toward LiDAR +Y
        self.assertEqual(lidar_sphere((0.1, 0.2, 0.3)).translation, (0.1, 0.2, 0.3))

    def test_build_session_trims_and_marks_provisional(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            recording, trajectory, panoramas = inputs(base)
            report = build_session(base / "session", recording=recording, trajectory=trajectory,
                                   panoramas=panoramas, camera_start_ns=T0 - 100_000_000,
                                   camera_offset=(0.05, 0.0, 0.1), camera_ypr=(90, 0, 0),
                                   t_lidar_imu=[0.011, 0.02329, -0.04412, 0, 0, 0, 1])
            self.assertEqual((report["points_before_trajectory"], report["points_kept"],
                              report["points_after_trajectory"]), (10, 10, 10))
            self.assertEqual(report["panoramas_outside_trajectory"], [1, 4])
            self.assertEqual(report["panoramas_kept"], 2)
            session = base / "session"
            times = [p[0] for p in iter_points(session / "points.bin")]
            self.assertEqual((times[0], times[-1]), (T0, T0 + 900_000_000))
            rig = read_json(session / "rig.json")
            self.assertEqual(rig["status"], "uncalibrated")
            self.assertEqual(read_json(session / "clock.json")["status"], "provisional")
            T_rig_lidar = Pose.from_dict(rig["T_rig_lidar"])
            self.assertTrue(close(T_rig_lidar.translation, (-0.011, -0.02329, 0.04412)))
            T_rig_sphere = Pose.from_dict(rig["panorama"]["T_rig_sphere"])
            expected = T_rig_lidar.compose(lidar_sphere((0.05, 0, 0.1), 90))
            self.assertTrue(close(T_rig_sphere.translation, expected.translation))
            self.assertTrue(close(rotate(T_rig_sphere.quaternion_wxyz, (0, 0, 1)), (0, 1, 0)))
            self.assertEqual([p["id"] for p in read_jsonl(session / "panoramas.jsonl")], [2, 3])
            with self.assertRaisesRegex(ValueError, "already exists"):
                build_session(base / "session", recording=recording, trajectory=trajectory,
                              panoramas=panoramas, camera_start_ns=T0)
            with self.assertRaisesRegex(ValueError, "outside every input"):
                build_session(panoramas / "nested", recording=recording, trajectory=trajectory,
                              panoramas=panoramas, camera_start_ns=T0)
            with self.assertRaisesRegex(ValueError, "No panorama"):
                build_session(base / "late", recording=recording, trajectory=trajectory,
                              panoramas=panoramas, camera_start_ns=T0 + 10**10)
            self.assertFalse((base / "late").exists())

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg and ffprobe are required")
    def test_assembled_session_converts_and_needs_provisional_flag(self):
        from splatrig.panorama import panorama_views
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            recording, trajectory, panoramas = inputs(base)
            build_session(base / "session", recording=recording, trajectory=trajectory,
                          panoramas=panoramas, camera_start_ns=T0 - 100_000_000)
            panorama_views(base / "session", base / "views", size=16)
            with self.assertRaisesRegex(ValueError, "allow-provisional"):
                prepare(base / "views")
            data = prepare(base / "views", allow_provisional=True)
            self.assertEqual(len(data.frames), 12)
            self.assertTrue(data.report["provisional"])
            self.assertEqual(data.report["raw_points"], 10)


if __name__ == "__main__":
    unittest.main()
