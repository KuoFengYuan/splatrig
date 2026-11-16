# SPDX-License-Identifier: MIT
"""Dataset quality steps: keyframe choice, near-point removal and operator masks."""
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from splatrig.demo import png_bytes
from splatrig.geometry import Pose, Trajectory
from splatrig.keyframes import rotation_deg, select_keyframes, turn_rate_dps
from splatrig.max2 import extract, select_expression
from splatrig.panorama import View, load_views, pinhole_params
from splatrig.pipeline import build_session, lidar_sphere
from splatrig.pointlog import PointLogWriter, iter_points
from splatrig.selfmask import estimate, strided_points, view_mask

from test_assembly import T0, inputs

HAVE_FFMPEG = bool(shutil.which("ffmpeg") and shutil.which("ffprobe"))
MS = 1_000_000


def yaw(degrees):
    half = math.radians(degrees) / 2
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def trajectory(samples):
    """samples: (seconds, x, yaw degrees) -> Trajectory at 10 ms spacing via the callable."""
    return Trajectory([{"timestamp_ns": T0 + round(t * 1e9), "revision": 1,
                        **Pose((x, 0.0, 0.0), yaw(a)).to_dict()} for t, x, a in samples])


class KeyframeTests(unittest.TestCase):
    def test_rotation_and_turn_rate(self):
        self.assertAlmostEqual(rotation_deg(yaw(10), yaw(-20)), 30, places=9)
        self.assertAlmostEqual(rotation_deg(yaw(0), tuple(-q for q in yaw(0))), 0, places=6)
        path = trajectory([(i * 0.1, 0.0, 9.0 * i) for i in range(11)])  # 90 deg/s
        self.assertAlmostEqual(turn_rate_dps(path, T0 + 500 * MS), 90, places=6)
        self.assertAlmostEqual(turn_rate_dps(path, T0), 90, places=6)  # clipped at the start

    def test_spacing_blur_and_outside_frames(self):
        # 0-2 s: walk 0.5 m/s straight; 2-3 s: stand and spin 120 deg/s; 3-5 s: walk on.
        samples = []
        for i in range(501):
            t = i / 100
            x = 0.5 * min(t, 2) + 0.5 * max(0, t - 3)
            a = 120 * min(max(t - 2, 0), 1)
            samples.append((t, x, a))
        path = trajectory(samples)
        times = [round(i * 1e9 / 30) for i in range(180)]  # 6 s of 30 fps video
        result = select_keyframes(times, T0 - 500 * MS, path, spacing_m=0.3, spacing_deg=60,
                                  max_turn_dps=30, window_s=0.2)
        report = result["report"]
        # 15 frames before the trajectory, 14 after (the frame at exactly 5 s is inside).
        self.assertEqual(report["outside_trajectory"], 29)
        self.assertGreater(report["too_fast"], 20)
        chosen = [T0 - 500 * MS + times[i] for i in result["frames"]]
        poses = [path.at(t) for t in chosen]
        for t in chosen:
            self.assertLessEqual(turn_rate_dps(path, t), 30)
        # Walking legs: one frame about every 0.3 m (at most 0.3 m + 0.2 s * 0.5 m/s apart).
        gaps = [b.translation[0] - a.translation[0] for a, b in zip(poses, poses[1:])
                if rotation_deg(a.quaternion_wxyz, b.quaternion_wxyz) < 1]
        self.assertTrue(all(0.3 - 1e-9 <= g <= 0.4 + 1e-9 for g in gaps), gaps)
        # The spin itself is too fast; the first slow frame after it is due by rotation.
        turned = [p for p in poses if rotation_deg(p.quaternion_wxyz, yaw(120)) < 1e-6]
        self.assertTrue(turned and abs(turned[0].translation[0] - 1.0) < 0.05)
        self.assertAlmostEqual(report["path_m"], 2.0, places=2)
        self.assertEqual(len(result["frames"]), report["selected"])
        self.assertEqual(result["frames"], sorted(result["frames"]))

    def test_standing_still_gives_one_frame(self):
        path = trajectory([(i * 0.1, 1.0, 0.0) for i in range(31)])
        result = select_keyframes([round(i * 1e9 / 30) for i in range(90)], T0, path)
        self.assertEqual(result["report"]["selected"], 1)
        with self.assertRaises(ValueError):
            select_keyframes([0], T0, path, spacing_m=0)


class NearPointTests(unittest.TestCase):
    def test_points_near_the_lidar_are_counted_and_dropped(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            recording, path, panoramas = inputs(base)
            with PointLogWriter(recording / "near.bin", {"frame": "lidar"}) as writer:
                for point in iter_points(recording / "points.bin"):
                    writer.write(*point)
                writer.write(T0 + 50 * MS, 0.3, 0.2, 0.1, 5, 0, 0)  # 0.37 m: the operator
            (recording / "near.bin").replace(recording / "points.bin")
            report = build_session(base / "session", recording=recording, trajectory=path,
                                   panoramas=panoramas, camera_start_ns=T0 - 100 * MS,
                                   min_range_m=0.6)
            self.assertEqual((report["points_kept"], report["points_near_sensor"]), (10, 1))


class SelfMaskTests(unittest.TestCase):
    def test_strided_points_match_every_nth_record(self):
        with tempfile.TemporaryDirectory() as temporary:
            log = Path(temporary) / "points.bin"
            with PointLogWriter(log, {}) as writer:
                for i in range(23):
                    writer.write(i, float(i), 0.0, 0.0)
            every = [p[:4] for p in iter_points(log)][::5]
            self.assertEqual(list(strided_points(log, 5, chunk_records=2)), every)
            self.assertEqual(list(strided_points(log, 1)), [p[:4] for p in iter_points(log)])

    def test_operator_behind_the_rig_is_masked_and_the_scene_is_not(self):
        # Rig walks along +X at 0.5 m/s with LiDAR = rig frame. A wall 5 m ahead and a
        # floor are scene; the operator stands 0.4 m behind the sensor (-X), always.
        path = trajectory([(i * 0.1, 0.05 * i, 0.0) for i in range(41)])
        with tempfile.TemporaryDirectory() as temporary:
            log = Path(temporary) / "points.bin"
            with PointLogWriter(log, {}) as writer:
                for i in range(4000):
                    t = T0 + i * MS
                    x = 0.0005 * i  # rig position at t
                    angle = (i * 0.37) % 1.0 - 0.5
                    writer.write(t, 5.0 - x, 2 * angle, 0.3 * angle)  # wall ahead
                    writer.write(t, -0.4, 0.3 * angle, 0.2 + 0.2 * angle)  # operator
                    writer.write(t, -3.0 + 0.5 * angle, 2.0 + angle, -0.5)  # passing scene
                writer.write(T0 + 9 * 10**9, -0.4, 0.0, 0.2)  # outside the trajectory
            mount = lidar_sphere((0.0, 0.0, 0.083))  # camera front = LiDAR +X, 8 cm above
            found = estimate(log, path, mount, stride=1)
        self.assertTrue(found["estimated"])
        self.assertGreater(found["self_lidar_bins"], 0)
        self.assertLess(found["sphere_fraction"], 0.1)
        bins = {tuple(b) for b in found["bins"]}
        size, params = 64, pinhole_params(64, 90)
        _, back = view_mask(bins, View("back", 180, 0).rotation(), size, size, params, step=4)
        _, front = view_mask(bins, View("front", 0, 0).rotation(), size, size, params, step=4)
        _, left = view_mask(bins, View("left", -90, 0).rotation(), size, size, params, step=4)
        self.assertGreater(back, 0.02)  # the operator covers part of the backward view
        self.assertEqual(front, 0.0)
        self.assertEqual(left, 0.0)

    def test_still_rig_estimates_nothing(self):
        path = trajectory([(i * 0.1, 0.0, 0.0) for i in range(11)])
        with tempfile.TemporaryDirectory() as temporary:
            log = Path(temporary) / "points.bin"
            with PointLogWriter(log, {}) as writer:
                for i in range(500):
                    writer.write(T0 + i * MS, -0.4, 0.0, 0.2)
            found = estimate(log, path, lidar_sphere((0, 0, 0)), stride=1)
        self.assertEqual((found["estimated"], found["bins"]), (False, []))
        # Turning in place moves the rig, but nearby scene would keep its LiDAR direction.
        turning = trajectory([(i * 0.1, 0.0, 6.0 * i) for i in range(11)])
        with tempfile.TemporaryDirectory() as temporary:
            log = Path(temporary) / "points.bin"
            with PointLogWriter(log, {}) as writer:
                for i in range(500):
                    writer.write(T0 + i * MS, -0.4, 0.0, 0.2)
            found = estimate(log, turning, lidar_sphere((0, 0, 0)), stride=1)
        self.assertGreater(found["moving_returns_sampled"], 0)
        self.assertEqual((found["estimated"], found["bins"], found["motion_extent_m"]),
                         (False, [], 0.0))

    def test_mask_png_is_grayscale_with_ignore_value_zero(self):
        everything = {(c, r) for c in range(180) for r in range(90)}
        data, fraction = view_mask(everything, (1, 0, 0, 0), 10, 6, pinhole_params(10, 90), step=4)
        self.assertEqual(fraction, 1.0)
        self.assertEqual(data[:8], b"\x89PNG\r\n\x1a\n")
        self.assertEqual(data[24:26], bytes([8, 0]))  # bit depth 8, grayscale
        self.assertEqual(png_bytes(2, 1, [b"\x00\xff"], channels=1)[25], 0)


class LayoutAndExtractTests(unittest.TestCase):
    def test_default_layout_drops_the_downward_view(self):
        names = [view.name for view in load_views("cube-no-down")]
        self.assertEqual(names, ["front", "right", "back", "left", "up"])

    def test_lens_views_stay_clear_of_the_seam(self):
        from splatrig.panorama import PRESET_FOV, seam_margin_deg
        fov = PRESET_FOV["lens"]
        views = load_views("lens")
        self.assertEqual(len(views), 8)
        margins = {view.name: seam_margin_deg(view, fov) for view in views}
        self.assertTrue(all(m >= 8 for m in margins.values()), margins)
        # Each lens keeps its own views: front views look forward (+Z), back views back.
        for view in views:
            from splatrig.geometry import rotate
            forward = rotate(view.rotation(), (0, 0, 1))[2]
            self.assertEqual(forward > 0, view.name.startswith("front"), view.name)
        cube = {view.name: seam_margin_deg(view, 90) for view in load_views("cube")}
        self.assertTrue(all(cube[n] < 0 for n in ("right", "left", "up", "down")), cube)
        self.assertGreater(cube["front"], 30)
        self.assertEqual(select_expression([3, 10]), "eq(n\\,3)+eq(n\\,10)")

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg and ffprobe are required")
    def test_extract_listed_frames_from_a_two_track_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            video = base / "media" / "clip.360"
            video.parent.mkdir()
            source = "testsrc2=size=5888x1920:rate=30:duration=0.2"
            subprocess.run(["ffmpeg", "-v", "error", "-f", "lavfi", "-i", source,
                            "-f", "lavfi", "-i", source, "-map", "0:v", "-map", "1:v",
                            "-c:v", "mpeg4", "-q:v", "31", "-f", "mp4", str(video)], check=True)
            report = extract(video, base / "out", frames=[4, 1], width=256)
            self.assertEqual(report["panoramas"], 2)
            self.assertIsNone(report["every_nth_frame"])
            records = (base / "out/panoramas.jsonl").read_text().splitlines()
            self.assertIn('"source_frame":1', records[0])
            self.assertIn('"source_frame":4', records[1])
            with self.assertRaisesRegex(ValueError, "Frame indices"):
                extract(video, base / "bad", frames=[6], width=256)


if __name__ == "__main__":
    unittest.main()


class LeverArmTests(unittest.TestCase):
    def test_recovers_a_known_offset_between_two_accelerometers(self):
        from splatrig.gyrocal import cross, solve_lever_arm
        r = (-0.08, 0.02, 0.12)  # camera IMU from LiDAR IMU, LiDAR axes
        rotation = ((0, -1, 0), (0, 0, -1), (1, 0, 0))  # camera stored axes -> LiDAR axes

        def omega(t):
            return (2.0 * math.sin(2.1 * t), 1.6 * math.sin(3.3 * t + 1), 1.8 * math.sin(2.7 * t + 2))

        def alpha(t):
            return (4.2 * math.cos(2.1 * t), 5.28 * math.cos(3.3 * t + 1), 4.86 * math.cos(2.7 * t + 2))
        lidar_times, gyro, accel = [], [], []
        for i in range(4000):  # 20 s at 200 Hz
            t = i * 0.005
            lidar_times.append(T0 + i * 5 * MS)
            gyro.append(omega(t))
            accel.append((0.3 * math.sin(t), 0.2, 9.81))
        camera_times, camera_accel, offset = [], [], 1_000_000_000
        for i in range(3900):  # 197 Hz on the camera clock
            t_ns = round(i * 5.08 * MS)
            t = (T0 + t_ns + offset - T0) / 1e9 - 1.0 + 1.0 - offset / 1e9
            w, a = omega(t), alpha(t)
            extra = [p + q for p, q in zip(cross(a, r), cross(w, cross(w, r)))]
            f_lidar = (0.3 * math.sin(t) + extra[0], 0.2 + extra[1], 9.81 + extra[2])
            f_lidar = (f_lidar[0] + 0.05, f_lidar[1] - 0.03, f_lidar[2])  # bias difference
            camera_times.append(t_ns)
            camera_accel.append(tuple(sum(rotation[k][i2] * f_lidar[k] for k in range(3))
                                      for i2 in range(3)))
        result = solve_lever_arm(camera_times, camera_accel, lidar_times, gyro, accel,
                                 T0, rotation)
        self.assertLess(math.dist(result["r_m"], r), 0.003, result)
        self.assertLess(result["half_difference_m"], 0.003)
        self.assertGreater(result["weakest_direction_ratio"], 0.3)
        self.assertAlmostEqual(result["bias_m_s2"][0], 0.05, delta=0.01)


@unittest.skipUnless(HAVE_FFMPEG, "ffmpeg and ffprobe are required")
class ColorizeTests(unittest.TestCase):
    def test_points_take_the_colour_they_project_to_and_avoid_the_mask(self):
        from splatrig.colorize import colorize
        from splatrig.selfmask import direction_bin
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            width, height = 64, 32
            red_front = [bytes([200, 30, 30] if 16 <= u < 48 else [30, 30, 200])
                         for u in range(width)]
            rows = [b"".join(red_front)] * height
            (base / "a.png").write_bytes(png_bytes(width, height, rows))
            green = [bytes([30, 200, 30]) * width] * height
            (base / "b.png").write_bytes(png_bytes(width, height, green))
            identity = Pose((0, 0, 0), (1, 0, 0, 0))  # sphere = world, OpenCV axes
            turned = Pose((0, 0, 0), (math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0))
            shots = [(T0, identity, base / "a.png"), (T0 + 2 * 10**9, turned, base / "b.png")]
            points = [(0, 0, 3, 0, 0, 0), (0, 0, -3, 0, 0, 0), (0, 0, 0.1, 0, 0, 0)]
            times = [T0, T0, T0]
            colored, report = colorize(points, times, shots, width=width)
            self.assertEqual(colored[0][3:], (200, 30, 30))  # ahead: red half
            self.assertEqual(colored[1][3:], (30, 30, 200))  # behind: blue half
            self.assertEqual(colored[2][3:], (0, 0, 0))  # too close to colour
            self.assertEqual((report["colored_points"], report["uncolored_points"]), (2, 1))
            # Masking the forward direction sends that point to the other (turned) panorama.
            masked = {direction_bin(0.0, 0.0)}
            colored, report = colorize(points[:1], times[:1], shots, width=width, masked=masked)
            self.assertEqual(colored[0][3:], (30, 200, 30))
            self.assertEqual(report["mask_redirects"], 1)
            # Nothing within max_gap_ns: the point keeps its colour.
            colored, _ = colorize(points[:1], [T0 + 60 * 10**9], shots, width=width)
            self.assertEqual(colored[0][3:], (0, 0, 0))


class PointTimeTests(unittest.TestCase):
    def test_each_map_point_keeps_its_closest_return_time(self):
        from splatrig.demo import create_demo
        from splatrig.session import prepare
        with tempfile.TemporaryDirectory() as temporary:
            create_demo(Path(temporary) / "demo")
            data = prepare(Path(temporary) / "demo", allow_provisional=True)
        self.assertEqual(len(data.point_times), len(data.points))
        self.assertTrue(all(isinstance(t, int) for t in data.point_times))
        self.assertGreater(len(set(data.point_times)), 1)


class RigMaskTests(unittest.TestCase):
    def test_rig_cylinder_under_the_camera(self):
        from splatrig.selfmask import any_rule, rig_rule
        # Camera 8 cm above the LiDAR origin, looking along LiDAR +X (sphere +Z).
        rule = rig_rule(lidar_sphere((0.0, 0.0, 0.08)))

        def ray(azimuth, elevation):  # sphere frame: +Z front, +X right, +Y down
            a, e = math.radians(azimuth), math.radians(elevation)
            return (math.cos(e) * math.sin(a), -math.sin(e), math.cos(e) * math.cos(a))
        self.assertTrue(rule(ray(0, -90)))  # straight down: the LiDAR
        self.assertTrue(rule(ray(123, -60)))  # the plate, any direction
        self.assertFalse(rule(ray(0, -20)))  # shallow: passes over the plate
        self.assertFalse(rule(ray(0, 10)))  # upward
        # Steep enough to pass the plate's edge low, but too far out for the handle.
        self.assertFalse(rule(ray(0, -35)))
        # Offset camera: the rig appears toward the LiDAR, not straight down.
        offset = rig_rule(lidar_sphere((-0.15, 0.0, 0.08)))
        self.assertFalse(offset(ray(0, -89.9)))
        self.assertTrue(offset(ray(0, -30)))
        self.assertFalse(offset(ray(180, -30)))
        both = any_rule(offset, None, lambda r: r[1] < -0.99)  # plus "straight up"
        self.assertTrue(both(ray(0, 90)) and both(ray(0, -30)) and not both(ray(90, 0)))


class PersonMaskRunnerTests(unittest.TestCase):
    def test_helper_contract_and_failures(self):
        import sys
        from splatrig.personmask import run
        self.assertIn("SPLATRIG_TORCH_PYTHON", run(Path("x"), python=None, command=None)["skipped"]
                      if not __import__("os").environ.get("SPLATRIG_TORCH_PYTHON") else
                      "SPLATRIG_TORCH_PYTHON")
        script = ("import json, sys; args = sys.argv[1:]; "
                  "print('loading'); print(json.dumps({'masks': 3, 'args': args}))")
        report = run(Path("/data/set"), command=[sys.executable, "-c", script],
                     rotate_cameras=[5], score=0.6, dilate=4)
        self.assertEqual(report["masks"], 3)
        self.assertEqual(report["args"], ["/data/set", "--score", "0.6", "--dilate", "4",
                                          "--rotate-cameras", "5"])
        failing = [sys.executable, "-c", "import sys; sys.stderr.write('No module named torch\\n'); sys.exit(1)"]
        with self.assertRaisesRegex(ValueError, "No module named torch"):
            run(Path("/data/set"), command=failing)
