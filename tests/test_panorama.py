# SPDX-License-Identifier: MIT
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from splatrig.demo import create_panorama_demo, scene_hit
from splatrig.geometry import Pose, rotate
from splatrig.panorama import (View, equirect_direction, load_views, panorama_views,
                               pinhole_params, probe_ffmpeg)
from splatrig.session import prepare, read_json, read_jsonl, write_json

HAVE_FFMPEG = bool(shutil.which("ffmpeg") and shutil.which("ffprobe"))


def close(a, b, places=9):
    return all(abs(x - y) < 10**-places for x, y in zip(a, b))


class PanoramaGeometryTests(unittest.TestCase):
    def test_sphere_axes_follow_opencv(self):
        width, height = 360, 180
        self.assertTrue(close(equirect_direction(180, 90, width, height), (0, 0, 1)))
        self.assertTrue(close(equirect_direction(270, 90, width, height), (1, 0, 0)))
        self.assertTrue(close(equirect_direction(90, 90, width, height), (-1, 0, 0)))
        self.assertTrue(close(equirect_direction(180, 0, width, height), (0, -1, 0)))
        self.assertTrue(close(equirect_direction(0, 90, width, height), (0, 0, -1)))

    def test_view_rotation_order(self):
        forward, right, down = (0, 0, 1), (1, 0, 0), (0, 1, 0)
        self.assertTrue(close(rotate(View("r", 90, 0).rotation(), forward), right))
        self.assertTrue(close(rotate(View("u", 0, 90).rotation(), forward), (0, -1, 0)))
        self.assertTrue(close(rotate(View("z", 0, 0, 90).rotation(), right), down))
        # Yaw is applied last: a pitched view still turns about the sphere's vertical.
        view = rotate(View("yp", 90, 30).rotation(), forward)
        self.assertTrue(close(view, (math.cos(math.radians(30)), -0.5, 0)))

    def test_view_validation_and_presets(self):
        self.assertEqual(View("w", 270, 0).yaw_deg, -90)
        self.assertEqual(View("b", -180, 0).yaw_deg, 180)
        for bad in [("Bad", 0, 0), ("p", 0, 91), ("n", math.nan, 0), ("r", 0, 0, 181)]:
            with self.assertRaises(ValueError):
                View(*bad)
        cube = load_views("cube")
        directions = [rotate(view.rotation(), (0, 0, 1)) for view in cube]
        for i, a in enumerate(directions):
            for b in directions[i+1:]:
                dot = sum(x*y for x, y in zip(a, b))
                self.assertTrue(abs(dot) < 1e-9 or abs(dot + 1) < 1e-9)
        self.assertEqual(len({tuple(round(c, 9) + 0.0 for c in d) for d in directions}), 6)
        self.assertEqual(len(load_views("ring8")), 8)
        with self.assertRaises(ValueError):
            load_views("unknown")

    def test_pinhole_params(self):
        self.assertTrue(close(pinhole_params(1024, 90), (512, 512, 512, 512)))
        for size, fov in [(4, 90), (1024, 0), (1024, 180), (True, 90)]:
            with self.assertRaises(ValueError):
                pinhole_params(size, fov)


def decode(path: Path, size: int) -> bytes:
    pixels = subprocess.run(["ffmpeg", "-v", "error", "-i", str(path), "-f", "rawvideo",
                             "-pix_fmt", "rgb24", "pipe:1"],
                            capture_output=True, check=True).stdout
    assert len(pixels) == size * size * 3
    return pixels


def agreement(pixels: bytes, pose: Pose, params, size: int) -> float:
    fx, fy, cx, cy = params
    matches = 0
    for v in range(size):
        for u in range(size):
            ray = ((u+0.5-cx)/fx, (v+0.5-cy)/fy, 1.0)
            _, color = scene_hit(pose.translation, rotate(pose.quaternion_wxyz, ray))
            index = (v*size + u) * 3
            if max(abs(a - b) for a, b in zip(pixels[index:index+3], color)) <= 24:
                matches += 1
    return matches / (size * size)


@unittest.skipUnless(HAVE_FFMPEG, "ffmpeg and ffprobe are required")
class PanoramaConversionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.capture = self.base / "capture"
        create_panorama_demo(self.capture, count=2, width=512)

    def tearDown(self):
        self.temporary.cleanup()

    def test_probe_accepts_installed_ffmpeg(self):
        report = probe_ffmpeg()
        self.assertLess(report["probe_max_error_px"], 0.6)
        self.assertTrue(report["ffmpeg_version"].startswith("ffmpeg"))

    def test_views_match_direct_renders_of_the_same_scene(self):
        size = 64
        before = sorted(p.relative_to(self.capture) for p in self.capture.rglob("*"))
        report = panorama_views(self.capture, self.base / "views", size=size,
                                interp="cubic")
        self.assertEqual(report["frames"], 12)
        self.assertFalse(report["provisional"])
        self.assertEqual(sorted(p.relative_to(self.capture)
                                for p in self.capture.rglob("*")), before)
        data = prepare(self.base / "views")
        self.assertEqual(len(data.frames), 12)
        self.assertEqual({f["capture_id"] for f in data.frames}, {1, 2})
        for frame in data.frames:
            camera = data.cameras[frame["camera_id"]]
            pixels = decode(self.base / "views" / frame["image_path"], size)
            pose = Pose.from_dict(frame["T_world_camera"])
            score = agreement(pixels, pose, camera.params, size)
            # Checker edges alias when resampled, so require a high score and that
            # the supplied pose is a local optimum rather than demanding identity.
            self.assertGreater(score, 0.8, frame["image_path"])
            if camera.id == 5:
                continue  # The uniform ceiling cannot constrain orientation.
            for angles in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                           (0, 0, 1), (0, 0, -1)]:
                nudged = pose.compose(Pose((0, 0, 0), View("n", *angles).rotation()))
                self.assertLessEqual(agreement(pixels, nudged, camera.params, size),
                                     score + 0.002, (frame["image_path"], angles))
            if camera.id == 1:
                wrong = pose.compose(Pose((0, 0, 0), View("x", 90, 0).rotation()))
                self.assertLess(agreement(pixels, wrong, camera.params, size), 0.6)

    def test_rejects_unsuitable_inputs_and_keeps_calibration_status(self):
        rig = read_json(self.capture / "rig.json")
        write_json(self.capture / "rig.json", {**rig, "status": "uncalibrated"})
        report = panorama_views(self.capture, self.base / "provisional", size=16)
        self.assertTrue(report["provisional"])
        self.assertEqual(read_json(self.base / "provisional/rig.json")["status"],
                         "uncalibrated")
        with self.assertRaises(ValueError):
            prepare(self.base / "provisional")
        with self.assertRaisesRegex(ValueError, "already exists"):
            panorama_views(self.capture, self.base / "provisional", size=16)
        with self.assertRaisesRegex(ValueError, "outside"):
            panorama_views(self.capture, self.capture / "nested", size=16)
        image = self.capture / "images/panorama_0001.png"
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-f", "lavfi", "-i",
                        "color=c=red:s=300x100", "-frames:v", "1", str(image)], check=True)
        with self.assertRaisesRegex(ValueError, "2:1"):
            panorama_views(self.capture, self.base / "wide", size=16)
        self.assertFalse((self.base / "wide").exists())
        self.assertEqual([p for p in self.base.iterdir() if p.name.startswith(".")], [])

    def test_custom_views_and_frame_ids(self):
        views = self.base / "views.json"
        write_json(views, [{"name": "ahead", "yaw_deg": 0, "pitch_deg": 0},
                           {"name": "tilted", "yaw_deg": 45, "pitch_deg": -20,
                            "roll_deg": 5}])
        panorama_views(self.capture, self.base / "custom", views_file=views, size=16,
                       fov_deg=70)
        frames = list(read_jsonl(self.base / "custom/frames.jsonl"))
        self.assertEqual([f["id"] for f in frames], [1, 2, 3, 4])
        self.assertEqual([f["camera_id"] for f in frames], [1, 2, 1, 2])
        rig = read_json(self.base / "custom/rig.json")
        self.assertEqual([c["name"] for c in rig["cameras"]], ["ahead", "tilted"])
        T_rig_sphere = Pose.from_dict(rig["panorama"]["T_rig_sphere"])
        expected = T_rig_sphere.compose(Pose((0, 0, 0), View("t", 45, -20, 5).rotation()))
        actual = Pose.from_dict(rig["cameras"][1]["T_rig_camera"])
        self.assertTrue(close(actual.translation, expected.translation))
        self.assertTrue(close(rotate(actual.quaternion_wxyz, (0.3, -0.2, 1)),
                              rotate(expected.quaternion_wxyz, (0.3, -0.2, 1))))

    def test_rejects_existing_pinhole_frames(self):
        (self.capture / "frames.jsonl").write_text("")
        with self.assertRaisesRegex(ValueError, "frames.jsonl"):
            panorama_views(self.capture, self.base / "mixed", size=16)
        with self.assertRaisesRegex(ValueError, "unavailable"):
            (self.capture / "frames.jsonl").unlink()
            panorama_views(self.capture, self.base / "missing", size=16,
                           ffmpeg="splatrig-no-such-ffmpeg",
                           ffprobe="splatrig-no-such-ffprobe")


if __name__ == "__main__":
    unittest.main()
