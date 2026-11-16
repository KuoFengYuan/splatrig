# SPDX-License-Identifier: MIT
"""The scan library: names, status, Recently Deleted, storage and dataset previews."""
from datetime import datetime
from http.server import ThreadingHTTPServer
import base64
import json
import os
from pathlib import Path
import shutil
import struct
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

from splatrig.demo import png_bytes
from splatrig.geometry import Pose
from splatrig.library import Library, default_name
from splatrig.live import Camera, LidarState, Recorder, make_handler
from splatrig.processing import Processor

DAY = 86400
HAVE_FFMPEG = bool(shutil.which("ffmpeg"))


def record(captures: Path, name: str, started: datetime, seconds=30.0, video=True):
    folder = captures / name
    (folder / "lidar").mkdir(parents=True)
    (folder / "lidar" / "data.mcap").write_bytes(b"x" * 1000)
    ns = int(started.timestamp() * 1e9)
    log = {"lidar": {"started_ns": ns, "stopped_ns": ns + int(seconds * 1e9)},
           "gopro": {"new_media": ["100GOPRO/GS010040.360"] if video else []}}
    (folder / "capture-log.json").write_text(json.dumps(log))
    return folder


def dataset(root: Path, name: str):
    """A minimal dataset folder as process-capture writes it."""
    data = root / name / "dataset"
    (data / "images").mkdir(parents=True)
    (data / "points.ply").write_text("ply\nformat ascii 1.0\nelement vertex 3\n"
                                     "property double x\nproperty double y\nproperty double z\n"
                                     "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                                     "end_header\n1 2 3 255 0 0\n4 5 6 0 255 0\n7 8 9 1 2 3\n")
    pose = Pose((1.0, 2.0, 0.5), (1.0, 0.0, 0.0, 0.0)).to_dict()
    (data / "image-poses.jsonl").write_text(json.dumps(
        {"image_path": "images/00000001.png", "camera_id": 1, "T_world_camera": pose}) + "\n")
    (data / "images" / "00000001.png").write_bytes(png_bytes(8, 8, [bytes([90, 120, 150] * 8)] * 8))
    return data


class LibraryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        base = Path(self.temporary.name)
        self.captures, self.root = base / "captures", base / "runs"
        self.now = datetime(2026, 11, 16, 15, 0).timestamp()
        self.library = Library(self.captures, self.root, clock=lambda: self.now)

    def tearDown(self):
        self.temporary.cleanup()

    def test_names_sorting_filters_and_status(self):
        self.assertEqual(default_name(datetime(2026, 11, 16, 15, 0).timestamp()),
                         "Monday afternoon scan")
        record(self.captures, "capture-a", datetime(2026, 11, 16, 14, 0))
        record(self.captures, "capture-b", datetime(2026, 11, 16, 16, 0), video=False)
        record(self.captures, "capture-c", datetime(2026, 11, 16, 9, 0))
        dataset(self.root, "capture-a")
        (self.root / "calibration-capture-c").mkdir(parents=True)
        (self.root / "calibration-capture-c" / "calibration.json").write_text("{}")
        listing = self.library.list(calibration=self.root / "calibration-capture-c/calibration.json")
        names = [(e["id"], e["name"], e["status"]) for e in listing["scans"]]
        self.assertEqual(names, [("capture-b", "Monday afternoon scan 2", "recorded"),
                                 ("capture-a", "Monday afternoon scan", "dataset"),
                                 ("capture-c", "Monday morning scan", "calibration")])
        entries = {e["id"]: e for e in listing["scans"]}
        self.assertEqual(entries["capture-b"]["tags"], ["LiDAR only"])
        self.assertEqual(entries["capture-c"]["tags"], ["current calibration"])
        self.assertTrue(entries["capture-c"]["current_calibration"])
        self.assertGreater(entries["capture-a"]["size_bytes"], entries["capture-b"]["size_bytes"])
        self.assertEqual([e["id"] for e in self.library.list("oldest")["scans"]],
                         ["capture-c", "capture-a", "capture-b"])
        self.assertEqual([e["id"] for e in self.library.list("largest")["scans"]][0], "capture-a")
        self.assertEqual([e["id"] for e in self.library.list(filter="dataset")["scans"]],
                         ["capture-a"])
        found = self.library.list(query="MORNING")
        self.assertEqual((found["found"], found["total"]), (1, 3))
        job = {"capture": "capture-b", "finished_ns": None, "stage": "slam", "stages": ["slam"]}
        busy = {e["id"]: e for e in self.library.list(job=job, queued={"capture-c"},
                                                      recording="capture-a")["scans"]}
        self.assertEqual((busy["capture-a"]["status"], busy["capture-b"]["status"],
                          busy["capture-c"]["status"]), ("recording", "processing", "queued"))
        self.assertEqual(busy["capture-b"]["stage"], "slam")
        self.library.save_meta("capture-b", last_error="GLIM failed")
        self.assertEqual(self.library.entry("capture-b")["status"], "failed")
        with self.assertRaises(ValueError):
            self.library.list(sort="random")

    def test_rename_and_cover(self):
        record(self.captures, "capture-a", datetime(2026, 11, 16, 14, 0))
        self.assertEqual(self.library.rename("capture-a", "  Office   desk "), "Office desk")
        self.assertEqual(self.library.entry("capture-a")["name"], "Office desk")
        self.library.rename("capture-a", "")  # back to the automatic name
        self.assertEqual(self.library.entry("capture-a")["name"], "Monday afternoon scan")
        for bad in (("capture-a", "x" * 41), ("capture-a", 5), ("../etc", "x"), ("missing", "x")):
            with self.assertRaises(ValueError):
                self.library.rename(*bad)
        with self.assertRaises(ValueError):
            self.library.set_cover("capture-a", b"not a jpeg")
        self.library.set_cover("capture-a", b"\xff\xd8\xff\xe0jpeg")
        self.assertTrue(self.library.entry("capture-a")["cover"])

    def test_recently_deleted_restore_and_purge(self):
        record(self.captures, "capture-a", datetime(2026, 11, 16, 14, 0))
        record(self.captures, "capture-b", datetime(2026, 11, 16, 15, 0))
        dataset(self.root, "capture-a")
        (self.root / "calibration-capture-a").mkdir()
        (self.root / ".capture-a-2-failed").mkdir()
        self.library.save_meta("capture-a", name="Kept name")
        with self.assertRaisesRegex(ValueError, "in use"):
            self.library.delete(["capture-a", "capture-b"], protected={"capture-b"})
        self.assertTrue((self.captures / "capture-a").exists())  # nothing moved
        self.library.delete(["capture-a"])
        self.assertFalse((self.captures / "capture-a").exists())
        self.assertEqual(sorted(p.name for p in (self.root / ".deleted" / "capture-a").iterdir()),
                         [".capture-a-2-failed", "calibration-capture-a", "capture-a"])
        self.assertEqual([e["id"] for e in self.library.list()["scans"]], ["capture-b"])
        gone = self.library.deleted()
        self.assertEqual((gone[0]["id"], gone[0]["name"], gone[0]["days_left"]),
                         ("capture-a", "Kept name", 30))
        self.library.restore(["capture-a"])
        self.assertTrue((self.root / "capture-a" / "dataset").is_dir())
        self.assertTrue((self.root / "calibration-capture-a").is_dir())
        self.assertEqual(self.library.deleted(), [])
        with self.assertRaises(ValueError):
            self.library.restore(["capture-a"])
        self.library.delete(["capture-a", "capture-b"])
        self.assertEqual({d["id"]: d["name"] for d in self.library.deleted()},
                         {"capture-a": "Kept name", "capture-b": "Monday afternoon scan"})
        self.now += 31 * DAY
        record(self.captures, "capture-c", datetime(2026, 12, 17, 9, 0))
        self.library.delete(["capture-c"])
        self.assertEqual(sorted(self.library.purge(expired_only=True)), ["capture-a", "capture-b"])
        self.assertEqual([e["id"] for e in self.library.deleted()], ["capture-c"])
        self.assertEqual(self.library.purge(), ["capture-c"])
        self.assertFalse((self.captures / ".deleted" / "capture-c").exists())

    def test_delete_dataset_keeps_the_recording_and_calibration(self):
        record(self.captures, "capture-a", datetime(2026, 11, 16, 14, 0))
        dataset(self.root, "capture-a")
        dataset(self.root, "capture-a-2")
        (self.root / "calibration-capture-a").mkdir()
        self.assertEqual(self.library.delete_dataset("capture-a"), 2)
        self.assertTrue((self.root / "calibration-capture-a").exists())
        self.assertTrue((self.captures / "capture-a").exists())
        storage = self.library.storage()
        self.assertGreater(storage["recordings_bytes"], 0)
        self.assertIsNotNone(storage["free_bytes"])


class EndpointTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        base = Path(self.temporary.name)
        self.captures, root = base / "captures", base / "runs"
        record(self.captures, "capture-a", datetime(2026, 11, 16, 14, 0))
        dataset(root, "capture-a")
        self.library = Library(self.captures, root)
        self.root = root
        self.processor = processor = Processor(self.captures, root)
        recorder = Recorder(self.captures, Camera(None), command=[sys.executable, "-c", "pass"])
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(
            LidarState(), Camera(None), recorder, processor=processor, library=self.library))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.temporary.cleanup()

    def call(self, path, data=None):
        body = json.dumps(data).encode() if data is not None else None
        request = urllib.request.Request(self.base + path, data=body, headers={
            "X-SplatRig": "1", "Content-Type": "application/json"},
            method="POST" if body is not None else "GET")
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            with error:
                return error.code, error.read()

    def test_list_rename_cover_preview_and_delete(self):
        code, body = self.call("/api/scans?sort=name&q=monday")
        listing = json.loads(body)
        self.assertEqual((code, listing["found"], listing["scans"][0]["status"]), (200, 1, "dataset"))
        self.assertEqual(json.loads(self.call("/api/scans/rename", {"id": "capture-a",
                                                                    "name": "Desk"})[1])["name"],
                         "Desk")
        jpeg = base64.b64encode(b"\xff\xd8\xff\xe0cover").decode()
        self.assertEqual(self.call("/api/scans/cover", {"id": "capture-a", "jpeg": jpeg})[0], 200)
        self.assertEqual(self.call("/api/scans/capture-a/cover.jpg")[1], b"\xff\xd8\xff\xe0cover")
        code, points = self.call("/api/scans/capture-a/points")
        self.assertEqual(struct.unpack_from("<I", points)[0], 3)
        x, y, z, colour = struct.unpack_from("<ffff", points, 4)
        self.assertEqual(((x, y, z), colour), ((1.0, 2.0, 3.0), 255 * 65536))
        cameras = json.loads(self.call("/api/scans/capture-a/cameras")[1])
        self.assertEqual((cameras[0]["center"], cameras[0]["forward"]), ([1.0, 2.0, 0.5], [0.0, 0.0, 1.0]))
        self.assertEqual(self.call("/api/scans/capture-a/thumb/../../x")[0], 404)
        if HAVE_FFMPEG:
            code, thumb = self.call("/api/scans/capture-a/thumb/00000001.png")
            self.assertEqual((code, thumb[:2]), (200, b"\xff\xd8"))
        self.assertEqual(self.call("/api/scans/delete", {"ids": "capture-a"})[0], 409)
        # The current calibration's scan cannot be deleted.
        calibration = self.root / "calibration-capture-a" / "calibration.json"
        calibration.parent.mkdir()
        calibration.write_text("{}")
        self.processor.settings["calibration"] = str(calibration)
        code, body = self.call("/api/scans/delete", {"ids": ["capture-a"]})
        self.assertEqual((code, "in use" in json.loads(body)["error"]), (409, True))
        self.assertTrue((self.captures / "capture-a").exists())
        self.processor.settings["calibration"] = None
        self.assertEqual(self.call("/api/scans/delete", {"ids": ["capture-a"]})[0], 200)
        self.assertEqual(json.loads(self.call("/api/scans/deleted")[1])[0]["name"], "Desk")
        self.assertEqual(self.call("/api/scans/purge", {"ids": []})[0], 409)  # must say all
        self.assertEqual(json.loads(self.call("/api/scans/deleted")[1])[0]["id"], "capture-a")
        self.assertEqual(self.call("/api/scans/restore", {"ids": ["capture-a"]})[0], 200)
        self.assertEqual(json.loads(self.call("/api/scans")[1])["total"], 1)
        self.assertEqual(self.call("/api/scans/nothing/points")[0], 404)


if __name__ == "__main__":
    unittest.main()
