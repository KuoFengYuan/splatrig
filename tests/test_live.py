# SPDX-License-Identifier: MIT
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

from splatrig.gopro import find_camera, split_jpegs
from splatrig.live import Camera, LidarState, Recorder, make_handler
from test_livox import driver_frames

FAKE_BAG = [sys.executable, "-c",
            "import signal, sys, time\n"
            "signal.signal(signal.SIGINT, lambda *a: sys.exit(0))\n"
            "print('recording', sys.argv[1:], flush=True)\n"
            "time.sleep(60)"]


class HelperTests(unittest.TestCase):
    def test_split_jpegs_keeps_partial_tail(self):
        buffer = bytearray(b"junk\xff\xd8one\xff\xd9\xff\xd8two\xff\xd9\xff\xd8thr")
        self.assertEqual(list(split_jpegs(buffer)), [b"\xff\xd8one\xff\xd9", b"\xff\xd8two\xff\xd9"])
        self.assertEqual(bytes(buffer), b"\xff\xd8thr")
        buffer += b"ee\xff\xd9"
        self.assertEqual(list(split_jpegs(buffer)), [b"\xff\xd8three\xff\xd9"])
        self.assertEqual(buffer, bytearray())

    def test_find_camera(self):
        output = ("2: eno1    inet 203.0.113.14/24 brd x\n"
                  "5: enx0457    inet 172.20.132.54/24 brd 172.20.132.255\n")
        self.assertEqual(find_camera(output), "172.20.132.51")
        self.assertIsNone(find_camera("2: eth0    inet 192.168.1.50/24\n"))


class LidarStateTests(unittest.TestCase):
    def test_window_downsampling_and_status(self):
        state = LidarState(window=3)
        frames = list(driver_frames(30))
        for index, frame in enumerate(frames):
            state.add_frame(frame, received=100 + index * 0.1)
            for k in range(20):
                state.add_imu(received=100 + index * 0.1 + k * 0.005)
        cloud = state.cloud()
        count = struct.unpack_from("<I", cloud)[0]
        expected = sum(1 for f in frames[-3:] for p in f.points if (p.x, p.y, p.z) != (0, 0, 0))
        self.assertEqual(count, expected)
        self.assertEqual(len(cloud), 4 + 16 * count)
        small = struct.unpack_from("<I", state.cloud(max_points=100))[0]
        self.assertLessEqual(small, 100)
        status = state.status(now=100 + 29 * 0.1 + 0.05)
        self.assertEqual(status["clock"], "ptp_inferred")
        self.assertAlmostEqual(status["lidar_hz"], 10.0, places=1)
        self.assertAlmostEqual(status["imu_hz"], 200.0, delta=1)
        self.assertEqual(status["lidar_age_s"], 0.05)

    def test_unsynced_stream_is_flagged(self):
        state = LidarState()
        for index, frame in enumerate(driver_frames(30, host_arrival=True)):
            state.add_frame(frame, received=index * 0.1)
        self.assertEqual(state.status(now=3.0)["clock"], "host_arrival_suspected")


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.captures = Path(self.temporary.name) / "captures"
        self.lidar = LidarState()
        for frame in driver_frames(25):
            self.lidar.add_frame(frame)
        self.camera = Camera(None)
        self.recorder = Recorder(self.captures, self.camera, command=FAKE_BAG)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0),
                                          make_handler(self.lidar, self.camera, self.recorder))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        if self.recorder.process:
            self.recorder.stop()
        self.server.shutdown()
        self.server.server_close()
        self.temporary.cleanup()

    def request(self, path, method="GET", header=True):
        request = urllib.request.Request(self.base + path, method=method,
                                         headers={"X-SplatRig": "1"} if header else {})
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            with error:
                return error.code, error.read()

    def test_page_status_and_points(self):
        code, page = self.request("/")
        self.assertEqual(code, 200)
        self.assertIn(b"SplatRig Live", page)
        code, body = self.request("/api/status")
        status = json.loads(body)
        self.assertEqual(status["lidar"]["clock"], "ptp_inferred")
        self.assertEqual(status["camera"], {"configured": False})
        self.assertFalse(status["record"]["recording"])
        code, cloud = self.request("/api/points")
        self.assertEqual(len(cloud), 4 + 16 * struct.unpack_from("<I", cloud)[0])
        self.assertEqual(self.request("/missing")[0], 404)
        self.assertEqual(self.request("/api/status?nocache=1")[0], 200)
        code, cloud = self.request("/api/points?t=2")
        self.assertEqual(code, 200)

    def test_post_requires_header_and_camera(self):
        self.assertEqual(self.request("/api/record/start", "POST", header=False)[0], 403)
        self.assertFalse(self.captures.exists())
        code, body = self.request("/api/preview/start", "POST")
        self.assertEqual(code, 409)
        self.assertIn("No GoPro", json.loads(body)["error"])

    def test_record_start_stop_without_camera(self):
        code, body = self.request("/api/record/start", "POST")
        self.assertEqual(code, 200, body)
        self.assertTrue(json.loads(body)["recording"])
        self.assertEqual(self.request("/api/record/start", "POST")[0], 409)
        folders = list(self.captures.iterdir())
        self.assertEqual(len(folders), 1)
        deadline = time.monotonic() + 10
        while (b"recording" not in (folders[0] / "lidar-record.log").read_bytes() and
               time.monotonic() < deadline):
            time.sleep(0.05)
        code, body = self.request("/api/record/stop", "POST")
        self.assertEqual(code, 200, body)
        log = json.loads((folders[0] / "capture-log.json").read_text())
        self.assertEqual(log["lidar"]["exit_code"], 0)
        self.assertNotIn("gopro", log)
        self.assertIn("approximate", log["note"])
        recorded = (folders[0] / "lidar-record.log").read_text()
        self.assertIn(str(folders[0] / "lidar"), recorded)
        self.assertIn("/livox/imu", recorded)
        self.assertEqual(self.request("/api/record/stop", "POST")[0], 409)


if __name__ == "__main__":
    unittest.main()
