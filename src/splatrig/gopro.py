# SPDX-License-Identifier: MIT
"""Minimal Open GoPro HTTP client for a USB-connected MAX2, plus preview helpers.

Only the standard library is used. Command acknowledgements are host-side events,
not exposure timestamps, and the preview stream's PTS is not wall-clock time.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess
import threading
import time
import urllib.error
import urllib.request

VIDEO_PRESET = 1835008
PREVIEW_PORT = 8554
STATUS_KEYS = {"8": "busy", "10": "encoding", "13": "video_progress_s",
               "35": "remaining_video_s", "70": "battery_percent",
               "97": "active_preset", "116": "usb_control_owner"}


def find_camera(ip_output: str) -> str | None:
    """Return the camera address from `ip -4 -o addr` output, if a GoPro link exists.

    A wired GoPro hands the host 172.2X.1YZ.5x and serves HTTP at 172.2X.1YZ.51.
    """
    for match in re.finditer(r"inet (172\.2\d\.1\d\d)\.5\d/", ip_output):
        return f"{match.group(1)}.51"
    return None


def detect_camera() -> str | None:
    try:
        output = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True, text=True,
                                timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    return find_camera(output)


class GoPro:
    def __init__(self, host: str, timeout: float = 6.0):
        self.base = f"http://{host}:8080"
        self.timeout = timeout

    def get(self, path: str):
        try:
            with urllib.request.urlopen(self.base + path, timeout=self.timeout) as response:
                body = response.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as error:
            error.close()
            raise ValueError(f"GoPro {path} returned HTTP {error.code}") from error
        except (urllib.error.URLError, OSError) as error:
            raise ValueError(f"GoPro unreachable: {error}") from error
        return json.loads(body) if body.strip() else {}

    def download(self, name: str, destination: Path):
        """Copy DCIM/<name> (e.g. 100GOPRO/GS010031.360) to a new local file."""
        if not re.fullmatch(r"\d{3}GOPRO/[A-Z0-9]+\.[0-9A-Za-z]+", name):
            raise ValueError(f"Unexpected media name: {name}")
        try:
            with urllib.request.urlopen(f"{self.base}/videos/DCIM/{name}",
                                        timeout=self.timeout) as response, \
                    destination.open("xb") as stream:
                while chunk := response.read(1 << 20):
                    stream.write(chunk)
        except (urllib.error.URLError, OSError) as error:
            raise ValueError(f"GoPro download failed: {error}") from error

    def info(self) -> dict:
        data = self.get("/gopro/camera/info")
        return {key: data.get(key) for key in ("model_name", "firmware_version")}

    def status(self) -> dict:
        status = self.get("/gopro/camera/state").get("status", {})
        return {name: status.get(key) for key, name in STATUS_KEYS.items()}

    def wait_idle(self, limit_s: float = 10.0):
        deadline = time.monotonic() + limit_s
        while time.monotonic() < deadline:
            if not self.status().get("busy"):
                return
            time.sleep(0.25)
        raise ValueError("GoPro stayed busy")

    def media(self) -> set[str]:
        data = self.get("/gopro/media/list")
        return {f"{group['d']}/{item['n']}" for group in data.get("media", [])
                for item in group.get("fs", [])}

    def claim(self):
        self.get("/gopro/camera/control/wired_usb?p=1")
        self.wait_idle()
        self.get(f"/gopro/camera/presets/load?id={VIDEO_PRESET}")

    def release(self):
        self.get("/gopro/camera/control/wired_usb?p=0")

    def start_preview(self):
        self.get(f"/gopro/camera/stream/start?port={PREVIEW_PORT}")

    def stop_preview(self):
        self.get("/gopro/camera/stream/stop")

    def keep_alive(self):
        self.get("/gopro/camera/keep_alive")

    def shutter(self, on: bool):
        """Return host wall-clock ns before sending and after the acknowledgement."""
        sent = time.time_ns()
        self.get(f"/gopro/camera/shutter/{'start' if on else 'stop'}")
        return sent, time.time_ns()


def split_jpegs(buffer: bytearray):
    """Remove and yield complete JPEG images (SOI..EOI) from a byte buffer."""
    while True:
        start = buffer.find(b"\xff\xd8")
        if start < 0:
            buffer.clear()
            return
        end = buffer.find(b"\xff\xd9", start + 2)
        if end < 0:
            del buffer[:start]
            return
        yield bytes(buffer[start:end + 2])
        del buffer[:end + 2]


class PreviewDecoder:
    """Decode the UDP preview with ffmpeg into JPEGs; keep the newest one."""

    def __init__(self, width=1024, fps=15, ffmpeg="ffmpeg"):
        self.command = [ffmpeg, "-v", "error", "-fflags", "nobuffer", "-flags", "low_delay",
                        "-analyzeduration", "500000", "-probesize", "1000000",
                        "-i", f"udp://@:{PREVIEW_PORT}?overrun_nonfatal=1&fifo_size=50000000",
                        "-map", "0:v:0", "-an", "-vf", f"fps={fps},scale={width}:-2",
                        "-q:v", "6", "-f", "mjpeg", "pipe:1"]
        self.condition = threading.Condition()
        self.frame = None
        self.frame_time = 0.0
        self.frames = 0
        self.process = None

    def start(self):
        if self.process and self.process.poll() is None:
            return
        self.process = subprocess.Popen(self.command, stdout=subprocess.PIPE,
                                        stderr=subprocess.DEVNULL)
        threading.Thread(target=self._read, args=(self.process,), daemon=True).start()

    def _read(self, process):
        buffer = bytearray()
        while True:
            chunk = process.stdout.read1(1 << 16)
            if not chunk:
                break
            buffer += chunk
            for image in split_jpegs(buffer):
                with self.condition:
                    self.frame, self.frame_time = image, time.time()
                    self.frames += 1
                    self.condition.notify_all()

    def stop(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(5)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.process = None

    def wait_frame(self, after: int, timeout: float = 2.0):
        with self.condition:
            self.condition.wait_for(lambda: self.frames > after, timeout)
            return self.frames, self.frame
