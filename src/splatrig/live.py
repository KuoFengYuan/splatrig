# SPDX-License-Identifier: MIT
"""Local live capture console: LiDAR cloud, MAX2 preview, status and recording.

The server binds to localhost and uses only the standard library. ROS is needed
only for live LiDAR input (start_ros) and recording, never by the offline core.
Preview frames and command acknowledgements are monitoring aids, not timing data.
"""

from __future__ import annotations

from collections import Counter, deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time

from .gopro import GoPro, PreviewDecoder
from .livox import assess_clock
from .rosview import POINT, pack_points, use_dds_profile

WINDOW_FRAMES = 10
CLOCK_FRAMES = 40
MAX_POINTS = 60_000
ASSET = Path(__file__).parent / "assets/live.html"


class LidarState:
    """Rolling LiDAR/IMU window updated from driver callbacks."""

    def __init__(self, window=WINDOW_FRAMES, clock_frames=CLOCK_FRAMES):
        self.lock = threading.Lock()
        self.frames = deque(maxlen=window)
        self.clock = deque(maxlen=clock_frames)
        self.lidar_times = deque(maxlen=50)
        self.imu_times = deque(maxlen=1000)
        self.last_points = 0
        self.no_return = 0

    def add_frame(self, message, received=None):
        received = time.time() if received is None else received
        offsets = [int(p.offset_time) for p in message.points]
        steps = Counter(b - a for a, b in zip(offsets, offsets[1:]))
        backwards = bool(offsets) and (any(b < a for a, b in zip(offsets, offsets[1:])) or
                                       max(offsets) >= 2**31)
        data, count = pack_points(message.points)
        with self.lock:
            self.frames.append(data)
            self.clock.append((int(message.timebase), steps, backwards))
            self.lidar_times.append(received)
            self.last_points = count
            self.no_return = len(offsets) - count

    def add_imu(self, received=None):
        with self.lock:
            self.imu_times.append(time.time() if received is None else received)

    def cloud(self, max_points=MAX_POINTS) -> bytes:
        with self.lock:
            data = b"".join(self.frames)
        count = len(data) // POINT.size
        stride = max(1, -(-count // max_points))
        if stride > 1:
            data = b"".join(data[i:i + POINT.size]
                            for i in range(0, len(data), POINT.size * stride))
        return struct.pack("<I", len(data) // POINT.size) + data

    def status(self, now=None) -> dict:
        now = time.time() if now is None else now
        with self.lock:
            clock = list(self.clock)
            lidar, imu = list(self.lidar_times), list(self.imu_times)
            points, no_return = self.last_points, self.no_return

        def rate(times, span=2.0):
            recent = [t for t in times if now - t <= span]
            return round((len(recent) - 1) / (recent[-1] - recent[0]), 2) \
                if len(recent) > 1 and recent[-1] > recent[0] else 0.0

        steps = Counter()
        for _, counter, _ in clock:
            steps.update(counter)
        bases = [base for base, _, _ in clock]
        sync = assess_clock(steps, [b - a for a, b in zip(bases, bases[1:])],
                            sum(1 for *_, backwards in clock if backwards))
        return {"lidar_hz": rate(lidar), "imu_hz": rate(imu),
                "points_per_frame": points, "no_return_per_frame": no_return,
                "lidar_age_s": round(now - lidar[-1], 2) if lidar else None,
                "imu_age_s": round(now - imu[-1], 2) if imu else None,
                "clock": sync["status"],
                "clock_evidence": {k: sync.get(k) for k in
                                   ("boundary_concentration", "frame_step_concentration")}}


class Camera:
    """MAX2 control, preview decoding and keep-alive; all failures become status."""

    def __init__(self, gopro: GoPro | None, decoder: PreviewDecoder | None = None):
        self.gopro = gopro
        self.decoder = decoder or PreviewDecoder()
        self.lock = threading.Lock()
        self.previewing = False
        self.claimed = False
        self.error = None
        self.cached = {}
        self.cached_at = 0.0
        self.running = True
        if gopro:
            threading.Thread(target=self._keep_alive, daemon=True).start()

    def _keep_alive(self):
        while self.running:
            time.sleep(2.5)
            if self.claimed:
                try:
                    self.gopro.keep_alive()
                except ValueError as error:
                    self.error = str(error)

    def status(self) -> dict:
        if not self.gopro:
            return {"configured": False}
        if time.time() - self.cached_at > 1.0:
            try:
                self.cached = self.gopro.status()
                self.error = None
            except ValueError as error:
                self.error = str(error)
            self.cached_at = time.time()
        frame_age = round(time.time() - self.decoder.frame_time, 2) \
            if self.decoder.frame_time else None
        return {"configured": True, "previewing": self.previewing, "claimed": self.claimed,
                "preview_frame_age_s": frame_age, "error": self.error, **self.cached}

    def claim(self):
        with self.lock:
            if not self.claimed:
                self.gopro.claim()
                self.claimed = True

    def start_preview(self):
        self.claim()
        with self.lock:
            self.gopro.start_preview()
            self.decoder.start()
            self.previewing = True

    def stop_preview(self):
        with self.lock:
            self.decoder.stop()
            if self.previewing:
                self.gopro.stop_preview()
            self.previewing = False

    def close(self):
        self.running = False
        if not self.gopro:
            return
        for action in (self.stop_preview, self.gopro.release):
            try:
                action()
            except ValueError:
                pass
        self.claimed = False


class Recorder:
    """Start a LiDAR bag and, if present, a MAX2 recording into a new capture folder."""

    def __init__(self, captures: Path, camera: Camera, topics=("/livox/lidar", "/livox/imu"),
                 command=("ros2", "bag", "record")):
        self.captures = captures
        self.camera = camera
        self.topics = list(topics)
        self.command = list(command)
        self.lock = threading.Lock()
        self.process = None
        self.folder = None
        self.log = None
        self.output = None

    def status(self) -> dict:
        if not self.log:
            return {"recording": False, "captures": str(self.captures)}
        elapsed = (time.time_ns() - self.log["lidar"]["started_ns"]) / 1e9 \
            if self.process else None
        return {"recording": self.process is not None, "folder": str(self.folder),
                "elapsed_s": round(elapsed, 1) if elapsed is not None else None,
                "last": self.public_log()}

    def public_log(self) -> dict:
        record = json.loads(json.dumps(self.log))
        record.get("gopro", {}).pop("_before", None)
        return record

    def start(self) -> dict:
        with self.lock:
            if self.process:
                raise ValueError("Already recording")
            name = datetime.now().strftime("capture-%Y%m%d-%H%M%S")
            folder = self.captures / name
            folder.mkdir(parents=True)
            log = {"format": "splatrig-capture-log", "version": 1,
                   "note": "Command times are host wall-clock events, not exposure or "
                           "acquisition timestamps; LiDAR and camera starts are approximate.",
                   "lidar": {"bag": "lidar", "topics": self.topics,
                             "started_ns": time.time_ns()}}
            self.output = (folder / "lidar-record.log").open("xb")
            self.process = subprocess.Popen(self.command + ["-o", str(folder / "lidar"),
                                                            *self.topics],
                                            stdout=self.output, stderr=subprocess.STDOUT)
            camera = self.camera
            if camera.gopro:
                try:
                    camera.claim()
                    before = camera.gopro.media()
                    sent, acked = camera.gopro.shutter(True)
                    log["gopro"] = {"media_before": len(before), "start_sent_ns": sent,
                                    "start_ack_ns": acked, "_before": sorted(before)}
                except ValueError as error:
                    log["gopro"] = {"error": str(error)}
            self.folder, self.log = folder, log
            self._write()
            return self.status()

    def stop(self) -> dict:
        with self.lock:
            if not self.process:
                raise ValueError("Not recording")
            gopro = self.log.get("gopro", {})
            camera = self.camera
            if camera.gopro and "start_sent_ns" in gopro:
                try:
                    sent, acked = camera.gopro.shutter(False)
                    gopro.update(stop_sent_ns=sent, stop_ack_ns=acked)
                    camera.gopro.wait_idle(20)
                    gopro["new_media"] = sorted(camera.gopro.media() - set(gopro["_before"]))
                except ValueError as error:
                    gopro["stop_error"] = str(error)
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            self.output.close()
            self.log["lidar"].update(stopped_ns=time.time_ns(),
                                     exit_code=self.process.returncode,
                                     log="lidar-record.log")
            gopro.pop("_before", None)
            self.process = None
            self._write()
            return self.status()

    def _write(self):
        record = self.public_log()
        path = self.folder / "capture-log.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)


def make_handler(lidar: LidarState, camera: Camera, recorder: Recorder):
    page = ASSET.read_bytes()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def send(self, code, body: bytes, kind="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, code, value):
            self.send(code, json.dumps(value, allow_nan=False).encode())

        def do_GET(self):
            if self.path == "/":
                self.send(200, page, "text/html; charset=utf-8")
            elif self.path == "/api/status":
                self.send_json(200, {"time_ns": time.time_ns(), "lidar": lidar.status(),
                                     "camera": camera.status(), "record": recorder.status()})
            elif self.path == "/api/points":
                self.send(200, lidar.cloud(), "application/octet-stream")
            elif self.path == "/api/preview.mjpg":
                self.stream_preview()
            else:
                self.send_json(404, {"error": "not found"})

        def do_POST(self):
            # Cross-site forms cannot set custom headers without a CORS preflight.
            if self.headers.get("X-SplatRig") != "1":
                self.send_json(403, {"error": "missing X-SplatRig header"})
                return
            actions = {"/api/preview/start": camera.start_preview,
                       "/api/preview/stop": camera.stop_preview,
                       "/api/record/start": recorder.start,
                       "/api/record/stop": recorder.stop}
            action = actions.get(self.path)
            if action is None:
                self.send_json(404, {"error": "not found"})
                return
            if self.path.startswith("/api/preview") and not camera.gopro:
                self.send_json(409, {"error": "No GoPro configured"})
                return
            try:
                result = action()
                self.send_json(200, result if isinstance(result, dict) else {"ok": True})
            except (ValueError, OSError) as error:
                self.send_json(409, {"error": str(error)})

        def stream_preview(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            # No length or chunking: the stream ends when the connection closes.
            self.send_header("Connection", "close")
            self.end_headers()
            seen = 0
            try:
                while camera.previewing:
                    seen, frame = camera.decoder.wait_frame(seen)
                    if frame is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                     str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            self.close_connection = True

    return Handler


def start_ros(lidar: LidarState, lidar_topic: str, imu_topic: str):
    use_dds_profile()  # also inherited by the ros2 bag record subprocess
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Imu
        from livox_ros_driver2.msg import CustomMsg
    except ImportError as error:
        raise ValueError("ROS 2 and livox_ros_driver2 Python packages are unavailable. "
                         "Source both environments and use the system Python.") from error
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("splatrig_live")
    node.create_subscription(CustomMsg, lidar_topic, lidar.add_frame, qos_profile_sensor_data)
    # A LiDAR callback takes tens of ms; a deep IMU queue keeps 200 Hz samples
    # from being discarded while it runs.
    imu_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Imu, imu_topic, lambda _: lidar.add_imu(), imu_qos)

    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()

    def stop():
        # Shut the context down first so spin() returns before the node is destroyed.
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=5)
        node.destroy_node()
    return stop


def serve(port: int, captures: Path, gopro_host: str | None, lidar_topic: str,
          imu_topic: str, preview: bool = True):
    lidar = LidarState()
    camera = Camera(GoPro(gopro_host) if gopro_host else None)
    recorder = Recorder(captures, camera, (lidar_topic, imu_topic))
    stop_ros = start_ros(lidar, lidar_topic, imu_topic)
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(lidar, camera, recorder))
    server.daemon_threads = True
    if camera.gopro and preview:
        try:
            camera.start_preview()
        except ValueError as error:
            camera.error = str(error)
    print(f"SplatRig live console: http://127.0.0.1:{port}/  (GoPro: {gopro_host or 'none'})",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if recorder.process:
            try:
                recorder.stop()
            except ValueError:
                pass
        camera.close()
        stop_ros()
