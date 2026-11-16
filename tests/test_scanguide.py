# SPDX-License-Identifier: MIT
"""Live scan guidance: expected photos, motion, coverage and revisits."""
import math
import struct
import unittest

from splatrig.scanguide import ScanGuide, heading_deg, span_deg


def yaw(degrees):
    half = math.radians(degrees) / 2
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def wall(x=3.0):
    """A small wall patch in front of the origin (map frame)."""
    return [(x, y / 10, z / 10, 1.0) for y in range(-5, 6) for z in range(0, 10)]


class SpanTests(unittest.TestCase):
    def test_span_and_heading(self):
        self.assertEqual(span_deg(0), 0)
        self.assertEqual(span_deg(1), 0)  # one direction only
        self.assertEqual(span_deg(0b1001), 30)
        self.assertEqual(span_deg(1 | 1 << 35), 10)  # neighbours across 0 degrees
        self.assertEqual(span_deg((1 << 36) - 1), 360)
        self.assertAlmostEqual(heading_deg(yaw(30)), 30)
        self.assertAlmostEqual(heading_deg(yaw(-170)), -170)


class GuideTests(unittest.TestCase):
    def test_photos_follow_the_dataset_spacing_rule(self):
        guide = ScanGuide(sample=1)
        guide.add_pose((0, 0, 0), yaw(0), received=0)  # not recording: ignored
        guide.start(now=0)
        for i in range(61):  # walk 3 m along +X at 0.5 m/s, 10 Hz
            guide.add_pose((0.05 * i, 0, 0), yaw(0), scan=wall, received=0.1 * i)
        status = guide.status(now=6.0)
        # One photo per 0.3 m (the first poses at or past each 0.3 m step).
        self.assertIn(status["photos"], (9, 10, 11))
        self.assertEqual((status["skipped"], status["too_fast"]), (0, False))
        self.assertAlmostEqual(status["distance_m"], 3.0)
        self.assertAlmostEqual(status["speed_m_s"], 0.5, places=6)

    def test_fast_turns_skip_photos_until_the_rig_slows(self):
        guide = ScanGuide()
        guide.start(now=0)
        t = 0.0
        guide.add_pose((0, 0, 0), yaw(0), received=t)  # first photo
        for i in range(1, 11):  # spin at 90 deg/s for 1 s
            guide.add_pose((0, 0, 0), yaw(9 * i), received=0.1 * i)
        status = guide.status(now=1.0)
        self.assertEqual(status["photos"], 1)
        self.assertEqual((status["skipped"], status["too_fast"]), (1, True))
        self.assertGreater(status["motion_level"], 2.5)
        for i in range(1, 6):  # hold still
            guide.add_pose((0, 0, 0), yaw(90), received=1.0 + 0.1 * i)
        status = guide.status(now=1.5)
        self.assertEqual((status["photos"], status["too_fast"]), (2, False))
        guide.stop()
        self.assertFalse(guide.status(now=1.6)["recording"])

    def test_coverage_grows_when_cells_are_seen_from_different_directions(self):
        guide = ScanGuide(sample=1, cell_m=0.5)
        guide.start(now=0)
        guide.add_pose((0, 0, 0), yaw(0), scan=wall, received=0)
        guide.add_pose((0, 0, 0), yaw(0), scan=wall, received=0.1)  # not due: no photo
        first = guide.status(now=0.1)
        self.assertEqual(first["photos"], 1)
        self.assertEqual(first["coverage"], 0.0)  # seen from one direction only
        for i in range(1, 31):  # walk sideways 3 m; the wall is seen from new angles
            guide.add_pose((0, 0.1 * i, 0), yaw(0), scan=wall, received=0.1 + 0.2 * i)
        status = guide.status(now=6.1)
        self.assertGreater(status["coverage"], 0.5)
        count = struct.unpack_from("<I", guide.cells_payload())[0]
        self.assertEqual(count, status["cells"])
        spans = [struct.unpack_from("<f", guide.cells_payload(), 4 + 16 * i + 12)[0]
                 for i in range(count)]
        self.assertTrue(any(s >= 30 for s in spans))

    def test_revisit_prompt_and_return(self):
        guide = ScanGuide(revisit_m=5.0, revisit_after_s=40.0, old_s=20.0, near_m=0.8)
        guide.start(now=0)
        t = 0.0
        for i in range(61):  # 6 m out along +X over 60 s
            t = i * 1.0
            guide.add_pose((0.1 * i, 0, 0), yaw(0), received=t)
        status = guide.status(now=t)
        self.assertTrue(status["revisit"])
        self.assertGreaterEqual(status["new_ground_m"], 5.0)
        for i in range(1, 60):  # walk back toward the start
            t = 60 + i * 1.0
            guide.add_pose((6.0 - 0.1 * i, 0.5, 0), yaw(180), received=t)
            if guide.status(now=t)["back"]:
                break
        status = guide.status(now=t)
        self.assertTrue(status["back"])
        self.assertEqual(status["returns"], 1)
        self.assertFalse(status["revisit"])
        self.assertGreaterEqual(status["max_new_ground_m"], 6.0)
        self.assertFalse(guide.status(now=t + 5)["back"])  # the confirmation fades


if __name__ == "__main__":
    unittest.main()


class EndpointTests(unittest.TestCase):
    def test_status_cells_and_recording_drive_the_guide(self):
        from http.server import ThreadingHTTPServer
        import json
        from pathlib import Path
        import sys
        import tempfile
        import threading
        import urllib.request
        from splatrig.live import Camera, LidarState, Recorder, make_handler
        fake_bag = [sys.executable, "-c", "import signal, sys, time\n"
                    "signal.signal(signal.SIGINT, lambda *a: sys.exit(0))\ntime.sleep(60)"]
        with tempfile.TemporaryDirectory() as temporary:
            guide = ScanGuide()
            recorder = Recorder(Path(temporary), Camera(None), command=fake_bag)
            server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(
                LidarState(), Camera(None), recorder, guide=guide))
            threading.Thread(target=server.serve_forever, daemon=True).start()
            base = f"http://127.0.0.1:{server.server_address[1]}"

            def call(path, method="GET"):
                request = urllib.request.Request(base + path, method=method, headers={"X-SplatRig": "1"})
                with urllib.request.urlopen(request, timeout=10) as response:
                    return response.read()
            try:
                self.assertFalse(json.loads(call("/api/status"))["guide"]["recording"])
                self.assertEqual(struct.unpack_from("<I", call("/api/guide/cells"))[0], 0)
                call("/api/record/start", "POST")
                self.assertTrue(json.loads(call("/api/status"))["guide"]["recording"])
                call("/api/record/stop", "POST")
                self.assertFalse(guide.status()["recording"])
            finally:
                if recorder.process:
                    recorder.stop()
                server.shutdown()
                server.server_close()
