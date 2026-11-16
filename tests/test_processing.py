# SPDX-License-Identifier: MIT
"""Calibration and dataset jobs started from the live console."""
from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

from splatrig.live import Camera, LidarState, Recorder, make_handler
from splatrig.processing import Processor, camera_offset, unique_folder

FAKE_BAG = [sys.executable, "-c", "import signal, sys, time\n"
            "signal.signal(signal.SIGINT, lambda *a: sys.exit(0))\ntime.sleep(60)"]


def capture(root: Path, name: str, seconds=30.0, video=True, mtime=None):
    folder = root / name
    folder.mkdir(parents=True)
    log = {"lidar": {"started_ns": 10**18, "stopped_ns": 10**18 + int(seconds * 1e9)},
           "gopro": {"new_media": ["100GOPRO/GS010040.360"] if video else []}}
    (folder / "capture-log.json").write_text(json.dumps(log))
    if mtime:
        os.utime(folder, (mtime, mtime))
    return folder


class FakeJobs:
    """Stand-ins for calibrate_capture/process_capture that report progress."""

    def __init__(self):
        self.release = threading.Event()
        self.release.set()
        self.calls = []

    def calibrate(self, capture, output, *, gopro_host=None, progress=None):
        self.calls.append(("calibrate", capture.name, output.name, gopro_host))
        progress("video")
        self.release.wait(5)
        progress("fit")
        output.mkdir()
        report = {"magnitude_correlation": 0.99831, "residual_ratio": 0.0445,
                  "weak_axis": False, "offset_vs_command_ack_ms": 95.83}
        (output / "calibration.json").write_text(json.dumps(report))
        return report

    def process(self, capture, output, *, calibration, camera_offset, gopro_host=None,
                progress=None):
        self.calls.append(("dataset", capture.name, output.name, calibration.name,
                           camera_offset))
        for stage in ("lidar", "slam", "video"):
            progress(stage)
        self.release.wait(5)
        if capture.name.endswith("broken"):
            raise ValueError("Expected one new MAX2 .360 file, found []")
        output.mkdir()
        return {"dataset": str(output / "dataset"), "provisional": True,
                "steps": {"dataset": {"frames": 45, "voxel_points": 22973},
                          "keyframes": {"selected": 9, "too_fast": 128},
                          "self_mask": {"masks": 0, "reason": "rig travelled 0.3 m"}}}


def wait(processor):
    deadline = time.monotonic() + 5
    while processor.busy() and time.monotonic() < deadline:
        time.sleep(0.01)
    return processor.status()["job"]


class ProcessorTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        base = Path(self.temporary.name)
        self.captures, self.root = base / "captures", base / "runs"
        self.jobs = FakeJobs()
        self.mounts = []
        self.processor = Processor(self.captures, self.root, "172.20.0.51",
                                   calibrate=self.jobs.calibrate, process=self.jobs.process,
                                   on_mount=lambda path, offset: self.mounts.append((path, offset)))

    def tearDown(self):
        self.jobs.release.set()
        wait(self.processor)
        self.temporary.cleanup()

    def test_offsets_names_and_unique_outputs(self):
        self.assertEqual(camera_offset([0, 0, 0.083]), (0.0, 0.0, 0.083))
        for bad in ([0, 0], [0, 0, 3], [0, 0, float("nan")], [True, 0, 0], "0 0 0", None):
            with self.assertRaises(ValueError):
                camera_offset(bad)
        capture(self.captures, "capture-a")
        (self.captures / "not-a-capture").mkdir()
        for bad in ("../runs", "", "capture-missing", "not-a-capture", 5, ".hidden"):
            with self.assertRaises(ValueError):
                self.processor.capture(bad)
        self.root.mkdir()
        (self.root / "scan").mkdir()
        (self.root / ".scan-2-work").mkdir()
        self.assertEqual(unique_folder(self.root, "scan").name, "scan-3")
        self.assertEqual(unique_folder(self.root, "other").name, "other")

    def test_captures_newest_first(self):
        capture(self.captures, "capture-old", 12.04, video=False, mtime=1000)
        capture(self.captures, "capture-new", 34.9, mtime=2000)
        (self.captures / "stray").mkdir()
        self.assertEqual(self.processor.list_captures(), [
            {"name": "capture-new", "duration_s": 34.9, "video": ["100GOPRO/GS010040.360"],
             "finished": True},
            {"name": "capture-old", "duration_s": 12.0, "video": [], "finished": True}])

    def test_calibrate_then_dataset_and_settings_survive_a_restart(self):
        capture(self.captures, "capture-rot")
        capture(self.captures, "capture-scan")
        with self.assertRaisesRegex(ValueError, "No calibration yet"):
            self.processor.start("dataset", "capture-scan", [0, 0, 0.083])
        self.jobs.release.clear()
        status = self.processor.start("calibrate", "capture-rot")
        self.assertEqual(status["job"]["stages"], ["video", "fit"])
        with self.assertRaisesRegex(ValueError, "already running"):
            self.processor.start("calibrate", "capture-rot")
        self.assertEqual(self.processor.status()["job"]["stage"], "video")
        self.jobs.release.set()
        job = wait(self.processor)
        self.assertIsNone(job["error"])
        self.assertEqual(job["result"]["correlation"], 0.9983)
        calibration = self.root / "calibration-capture-rot" / "calibration.json"
        self.assertEqual(job["result"]["calibration"], str(calibration))
        self.assertEqual(self.mounts[-1], (calibration, (0.0, 0.0, 0.0)))
        status = self.processor.status()
        self.assertEqual(status["calibration"]["name"], "calibration-capture-rot")
        self.assertEqual(status["calibration"]["correlation"], 0.9983)

        self.processor.start("dataset", "capture-scan", [0, 0, 0.083])
        job = wait(self.processor)
        self.assertEqual(job["result"], {
            "dataset": str(self.root / "capture-scan" / "dataset"), "images": 45,
            "points": 22973, "keyframes": 9, "too_fast": 128, "masks": 0,
            "mask_note": "rig travelled 0.3 m", "provisional": True})
        self.assertEqual(self.jobs.calls[-1], ("dataset", "capture-scan", "capture-scan",
                                               "calibration.json", (0.0, 0.0, 0.083)))
        self.assertEqual(self.mounts[-1], (calibration, (0.0, 0.0, 0.083)))
        # A second run of the same capture never overwrites the first.
        self.processor.start("dataset", "capture-scan")
        self.assertEqual(wait(self.processor)["output"], str(self.root / "capture-scan-2"))
        restarted = Processor(self.captures, self.root)
        self.assertEqual(restarted.calibration(), calibration)
        self.assertEqual(restarted.settings["camera_offset"], [0.0, 0.0, 0.083])

    def test_failure_is_reported_and_settings_unchanged(self):
        capture(self.captures, "capture-rot")
        capture(self.captures, "capture-broken")
        self.processor.start("calibrate", "capture-rot")
        wait(self.processor)
        mounts = len(self.mounts)
        self.processor.start("dataset", "capture-broken", [0.1, 0, 0])
        job = wait(self.processor)
        self.assertEqual(job["error"], "Expected one new MAX2 .360 file, found []")
        self.assertEqual(job["stage"], "video")
        self.assertIsNone(job["result"])
        self.assertEqual(self.processor.settings["camera_offset"], [0.0, 0.0, 0.0])
        self.assertEqual(len(self.mounts), mounts)
        self.assertIsNone(self.processor.interrupted())

    def test_interrupted_message_names_the_work_folder(self):
        capture(self.captures, "capture-rot")
        self.jobs.release.clear()
        self.processor.start("calibrate", "capture-rot")
        note = self.processor.interrupted()
        self.assertIn("nothing was published", note)
        self.assertIn(".calibration-capture-rot-work", note)

    def test_unreadable_settings_start_empty(self):
        self.root.mkdir()
        (self.root / "console.json").write_text("{not json")
        self.assertEqual(Processor(self.captures, self.root).settings,
                         {"calibration": None, "camera_offset": [0.0, 0.0, 0.0]})
        (self.root / "console.json").write_text(json.dumps(
            {"calibration": str(self.root / "gone.json"), "camera_offset": [0, 0, 0.1]}))
        self.assertEqual(Processor(self.captures, self.root).settings,
                         {"calibration": None, "camera_offset": [0.0, 0.0, 0.1]})


class EndpointTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        base = Path(self.temporary.name)
        self.captures = base / "captures"
        capture(self.captures, "capture-rot")
        self.jobs = FakeJobs()
        self.processor = Processor(self.captures, base / "runs", calibrate=self.jobs.calibrate,
                                   process=self.jobs.process)
        self.recorder = Recorder(self.captures, Camera(None), command=FAKE_BAG)
        handler = make_handler(LidarState(), Camera(None), self.recorder,
                               processor=self.processor)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        self.jobs.release.set()
        wait(self.processor)
        if self.recorder.process:
            self.recorder.stop()
        self.server.shutdown()
        self.server.server_close()
        self.temporary.cleanup()

    def call(self, path, data=None, method=None, raw=None):
        body = raw if raw is not None else (json.dumps(data).encode() if data is not None else None)
        request = urllib.request.Request(self.base + path, data=body,
                                         method=method or ("POST" if body is not None else "GET"),
                                         headers={"X-SplatRig": "1",
                                                  "Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as error:
            with error:
                return error.code, json.loads(error.read())

    def test_status_captures_and_jobs(self):
        code, status = self.call("/api/status")
        self.assertEqual(status["process"]["job"], None)
        self.assertEqual(self.call("/api/captures")[1][0]["name"], "capture-rot")
        self.assertEqual(self.call("/api/process/dataset", {"capture": "capture-rot"})[0], 409)
        self.assertEqual(self.call("/api/process/calibrate", {"capture": "../etc"})[0], 409)
        self.assertEqual(self.call("/api/process/calibrate", raw=b"[1]")[0], 409)
        self.assertEqual(self.call("/api/process/calibrate", raw=b"x" * 5000)[0], 409)
        self.jobs.release.clear()
        code, body = self.call("/api/process/calibrate", {"capture": "capture-rot"})
        self.assertEqual((code, body["job"]["kind"]), (200, "calibrate"))
        code, body = self.call("/api/record/start", method="POST")
        self.assertEqual((code, body["error"]), (409, "Processing is running; record after it finishes"))
        self.jobs.release.set()
        wait(self.processor)
        self.assertEqual(self.call("/api/status")[1]["process"]["calibration"]["correlation"],
                         0.9983)
        self.assertEqual(self.call("/api/record/start", method="POST")[0], 200)
        code, body = self.call("/api/process/calibrate", {"capture": "capture-rot"})
        self.assertEqual((code, body["error"]), (409, "Stop recording before processing"))
        self.assertEqual(self.call("/api/record/stop", method="POST")[0], 200)

    def test_without_a_processor_the_endpoints_do_not_exist(self):
        handler = make_handler(LidarState(), Camera(None), self.recorder)
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        base, self.base = self.base, f"http://127.0.0.1:{server.server_address[1]}"
        try:
            self.assertIsNone(self.call("/api/status")[1]["process"])
            self.assertEqual(self.call("/api/captures")[0], 404)
            self.assertEqual(self.call("/api/process/calibrate", {"capture": "capture-rot"})[0], 404)
        finally:
            self.base = base
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
