# SPDX-License-Identifier: MIT
"""MAX2 downloads: complete files only, and a clear error while the camera records."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import unittest

from splatrig.gopro import GoPro

BODY = b"x" * 300_000


class FakeCamera(BaseHTTPRequestHandler):
    encoding = 0
    send_bytes = len(BODY)

    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/gopro/camera/state":
            data = json.dumps({"status": {"10": self.encoding, "8": 0}}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif self.path.startswith("/videos/DCIM/"):
            self.send_response(200)
            self.send_header("Content-Length", str(len(BODY)))
            self.end_headers()
            self.wfile.write(BODY[:self.send_bytes])
            self.close_connection = True
        else:
            self.send_response(404)
            self.end_headers()


class DownloadTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), FakeCamera)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.camera = GoPro("127.0.0.1", timeout=5)
        self.camera.base = f"http://127.0.0.1:{self.server.server_address[1]}"
        FakeCamera.encoding, FakeCamera.send_bytes = 0, len(BODY)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.temporary.cleanup()

    def test_complete_download(self):
        target = Path(self.temporary.name) / "a.360"
        self.camera.download("100GOPRO/GS010038.360", target)
        self.assertEqual(target.read_bytes(), BODY)

    def test_truncated_download_is_removed_and_reported(self):
        FakeCamera.send_bytes = 100_000
        target = Path(self.temporary.name) / "b.360"
        with self.assertRaisesRegex(ValueError, "stopped at 0 of 0 MB; keep the MAX2 idle"):
            self.camera.download("100GOPRO/GS010038.360", target)
        self.assertFalse(target.exists())

    def test_recording_camera_is_refused(self):
        FakeCamera.encoding = 1
        target = Path(self.temporary.name) / "c.360"
        with self.assertRaisesRegex(ValueError, "recording"):
            self.camera.download("100GOPRO/GS010038.360", target)
        self.assertFalse(target.exists())


if __name__ == "__main__":
    unittest.main()
