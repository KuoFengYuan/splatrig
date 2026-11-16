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
import sys
import threading
import time

from .gopro import GoPro, PreviewDecoder
from .livox import assess_clock
from .liveslam import GLIM_POINTS_TOPIC, GlimProcess, SlamState, cloud_points
from .library import Library
from .processing import Processor
from .scanguide import ScanGuide
from .rosview import POINT, pack_points, ros_import_error, use_dds_profile

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
            except (ValueError, OSError):  # camera unplugged or not answering
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


def make_handler(lidar: LidarState, camera: Camera, recorder: Recorder,
                 slam: SlamState | None = None, glim: GlimProcess | None = None,
                 processor: Processor | None = None, guide: ScanGuide | None = None,
                 library: Library | None = None):
    page = ASSET.read_bytes()

    def context() -> dict:
        """What the library needs to tell each scan's state."""
        state = processor.status() if processor else {"job": None, "queue": []}
        return {"job": state["job"], "queued": {q["capture"] for q in state["queue"]},
                "recording": recorder.folder.name if recorder.process and recorder.folder
                else None, "calibration": processor.calibration() if processor else None}

    def protected() -> set:
        """Scans that must not be deleted now: recording, processing, queued, or the
        current calibration (which needs the context to be recognized)."""
        state = context()
        names = set(state["queued"])
        if state["recording"]:
            names.add(state["recording"])
        if state["job"] and not state["job"].get("finished_ns"):
            names.add(state["job"]["capture"])
        for entry in library.list(**state)["scans"] if library else []:
            if entry["current_calibration"]:
                names.add(entry["id"])
        return names

    def new_scan():
        """Each scan starts a fresh SLAM session and an empty live map.

        The old GLIM keeps publishing until it is stopped, and the new one starts
        at the origin, so the map and the guide are cleared again afterwards.
        """
        if slam is not None:
            slam.reset()
        if glim is None:
            return

        def restart():
            try:
                glim.restart()
            except Exception as error:  # reported by the SLAM status
                print(f"Could not restart GLIM: {error}", file=sys.stderr, flush=True)
            if slam is not None:
                slam.reset()
            if guide is not None and guide.recording:
                guide.start()
        threading.Thread(target=restart, daemon=True).start()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def handle(self):
            # A reload or closed tab drops the socket mid-response; that is not
            # a server fault and must not print a traceback for every request.
            try:
                super().handle()
            except (BrokenPipeError, ConnectionResetError):
                self.close_connection = True

        def send(self, code, body: bytes, kind="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, code, value):
            self.send(code, json.dumps(value, allow_nan=False).encode())

        def route(self) -> str:
            # Clients append cache-busting queries (e.g. /api/preview.mjpg?123).
            return self.path.split("?", 1)[0]

        def do_GET(self):
            path = self.route()
            if path == "/":
                self.send(200, page, "text/html; charset=utf-8")
            elif path == "/api/status":
                slam_status = {"enabled": False}
                if slam is not None:
                    slam_status = {"enabled": True, "running": bool(glim and glim.running()),
                                   **slam.status()}
                self.send_json(200, {"time_ns": time.time_ns(), "lidar": lidar.status(),
                                     "camera": camera.status(), "record": recorder.status(),
                                     "slam": slam_status,
                                     "process": processor.status() if processor else None,
                                     "guide": guide.status() if guide else None})
            elif path == "/api/guide/cells" and guide is not None:
                self.send(200, guide.cells_payload(), "application/octet-stream")
            elif path == "/api/captures" and processor is not None:
                self.send_json(200, processor.list_captures())
            elif path.startswith("/api/scans") and library is not None:
                self.scans_get(path)
            elif path == "/api/map" and slam is not None:
                self.send(200, slam.map_payload(), "application/octet-stream")
            elif path == "/api/route" and slam is not None:
                self.send(200, slam.route_payload(), "application/octet-stream")
            elif path == "/api/slam/pose" and slam is not None:
                self.send_json(200, slam.pose_status())
            elif path == "/api/slam/scan" and slam is not None:
                self.send(200, slam.scan_payload(), "application/octet-stream")
            elif path == "/api/points":
                self.send(200, lidar.cloud(), "application/octet-stream")
            elif path == "/api/preview.mjpg":
                self.stream_preview()
            else:
                self.send_json(404, {"error": "not found"})

        def scans_get(self, path):
            query = dict(part.split("=", 1) for part in self.path.partition("?")[2].split("&")
                         if "=" in part)
            from urllib.parse import unquote_plus
            query = {k: unquote_plus(v) for k, v in query.items()}
            parts = path.split("/")[3:]  # after /api/scans
            try:
                if not parts:
                    self.send_json(200, library.list(query.get("sort", "newest"),
                                                     query.get("filter", "all"),
                                                     query.get("q", ""), **context()))
                elif parts == ["deleted"]:
                    self.send_json(200, library.deleted())
                elif len(parts) == 2 and parts[1] == "cover.jpg":
                    library.capture(parts[0])
                    cover = library.cover_path(parts[0])
                    if cover.is_file():
                        self.send(200, cover.read_bytes(), "image/jpeg")
                        return
                    # No map snapshot (older scans): the dataset's first photo.
                    from . import preview
                    entry = library.entry(parts[0])
                    if not entry["dataset"]:
                        raise ValueError("No cover")
                    dataset = Path(entry["dataset"])
                    first = preview.cameras(dataset)[0]["image"]
                    cache = library.meta_dir / "thumbs" / dataset.parent.name
                    self.send(200, preview.thumbnail(dataset, first, cache, width=480,
                                                     masked=False), "image/jpeg")
                elif len(parts) >= 2 and parts[1] in ("points", "cameras", "thumb"):
                    from . import preview
                    entry = library.entry(parts[0], **context())
                    if not entry["dataset"]:
                        raise ValueError("This scan has no dataset yet")
                    dataset = Path(entry["dataset"])
                    if parts[1] == "points":
                        self.send(200, preview.points_payload(dataset), "application/octet-stream")
                    elif parts[1] == "cameras":
                        self.send_json(200, preview.cameras(dataset))
                    elif len(parts) == 3:
                        cache = library.meta_dir / "thumbs" / dataset.parent.name
                        width = min(1600, max(64, int(query.get("w", "192"))))
                        self.send(200, preview.thumbnail(dataset, parts[2], cache, width=width,
                                                         masked=query.get("masked") != "0"),
                                  "image/jpeg")
                    else:
                        raise ValueError("Unknown image")
                else:
                    self.send_json(404, {"error": "not found"})
            except (ValueError, OSError) as error:
                self.send_json(404, {"error": str(error)})

        def scans_post(self, path, data):
            ids = data.get("ids")
            if path == "/api/scans/rename":
                return {"name": library.rename(data.get("id"), data.get("name", ""))}
            if path == "/api/scans/cover":
                import base64
                library.set_cover(data.get("id"), base64.b64decode(data.get("jpeg", ""),
                                                                    validate=True))
                return {"ok": True}
            if path == "/api/scans/delete-dataset":
                if data.get("id") in protected():
                    raise ValueError("This scan is in use and its dataset cannot be deleted now")
                return {"removed": library.delete_dataset(data.get("id"))}
            if not isinstance(ids, list) or not all(isinstance(i, str) for i in ids):
                raise ValueError("Expected a list of scan ids")
            if path == "/api/scans/delete":
                return {"deleted": library.delete(ids, protected())}
            if path == "/api/scans/restore":
                return {"restored": library.restore(ids)}
            if path == "/api/scans/purge":
                # Deleting for good: an empty list means everything, and must say so.
                if not ids and data.get("all") is not True:
                    raise ValueError("Name the scans to delete for good, or send all: true")
                return {"purged": library.purge(ids or None)}
            raise ValueError("not found")

        def body(self, limit=4096) -> dict:
            length = int(self.headers.get("Content-Length") or 0)
            if length > limit:
                raise ValueError("Request body too large")
            value = json.loads(self.rfile.read(length)) if length else {}
            if not isinstance(value, dict):
                raise ValueError("Expected a JSON object")
            return value

        def do_POST(self):
            # Cross-site forms cannot set custom headers without a CORS preflight.
            if self.headers.get("X-SplatRig") != "1":
                self.send_json(403, {"error": "missing X-SplatRig header"})
                return
            if self.route().startswith("/api/scans/") and library is not None:
                try:
                    limit = 1_200_000 if self.route() == "/api/scans/cover" else 4096
                    self.send_json(200, self.scans_post(self.route(), self.body(limit)))
                except (ValueError, OSError) as error:
                    self.send_json(409, {"error": str(error)})
                return
            if self.route() == "/api/process/unqueue" and processor is not None:
                try:
                    self.send_json(200, processor.unqueue(self.body().get("capture")))
                except ValueError as error:
                    self.send_json(409, {"error": str(error)})
                return
            jobs = {"/api/process/calibrate": "calibrate", "/api/process/dataset": "dataset"}
            if self.route() in jobs and processor is not None:
                try:
                    data = self.body()
                    if recorder.process:
                        raise ValueError("Stop recording before processing")
                    self.send_json(200, processor.start(jobs[self.route()], data.get("capture"),
                                                        data.get("camera_offset")))
                except (ValueError, OSError) as error:
                    self.send_json(409, {"error": str(error)})
                return
            if self.route() == "/api/record/start" and processor is not None and processor.busy():
                self.send_json(409, {"error": "Processing is running; record after it finishes"})
                return
            actions = {"/api/preview/start": camera.start_preview,
                       "/api/preview/stop": camera.stop_preview,
                       "/api/record/start": recorder.start,
                       "/api/record/stop": recorder.stop}
            path = self.route()
            action = actions.get(path)
            if action is None:
                self.send_json(404, {"error": "not found"})
                return
            if path.startswith("/api/preview") and not camera.gopro:
                self.send_json(409, {"error": "No GoPro configured"})
                return
            try:
                result = action()
                if path == "/api/record/start":
                    new_scan()
                if guide is not None and path == "/api/record/start":
                    guide.start()
                elif path == "/api/record/stop":
                    if guide is not None:
                        guide.stop()
                    if library is not None and recorder.folder is not None:
                        summary = guide.status() if guide is not None else None
                        library.save_meta(recorder.folder.name, guide=summary)
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


def start_ros(lidar: LidarState, lidar_topic: str, imu_topic: str,
              slam: SlamState | None = None, guide: ScanGuide | None = None):
    use_dds_profile()  # also inherited by the ros2 bag record subprocess
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2, PointField
        from livox_ros_driver2.msg import CustomMsg
    except ImportError as error:
        raise ValueError(ros_import_error(error)) from error
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("splatrig_live")
    receive = lidar.add_frame
    if slam is not None:
        from .glim import TIMED_FIELDS, TIMED_POINT, pack_timed_points
        fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in TIMED_FIELDS]
        glim_points = node.create_publisher(PointCloud2, GLIM_POINTS_TOPIC,
                                            qos_profile_sensor_data)

        def receive(message):
            data, count = pack_timed_points(message.points)
            cloud = PointCloud2(header=message.header, height=1, width=count, fields=fields,
                                is_bigendian=False, point_step=TIMED_POINT.size,
                                row_step=TIMED_POINT.size * count, is_dense=True)
            cloud.data = data
            if rclpy.ok():
                glim_points.publish(cloud)
            lidar.add_frame(message)

        def odometry(message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            slam.add_odometry((p.x, p.y, p.z), (q.w, q.x, q.y, q.z))
            if guide is not None:
                guide.add_pose((p.x, p.y, p.z), (q.w, q.x, q.y, q.z), scan=slam.latest_points)
        node.create_subscription(Odometry, "/glim_ros/odom_corrected", odometry, 10)
        node.create_subscription(PointCloud2, "/glim_ros/aligned_points_corrected",
                                 lambda message: slam.add_scan(cloud_points(message)), 10)

        def count_publishers():
            # More than one means another GLIM writes into the same live map.
            slam.publishers = node.count_publishers("/glim_ros/odom_corrected")
        node.create_timer(2.0, count_publishers)
    node.create_subscription(CustomMsg, lidar_topic, receive, qos_profile_sensor_data)
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


def camera_mount(calibration: Path | None, offset=(0.0, 0.0, 0.0)):
    """T_rig_sphere (rig = GLIM's IMU frame) from a gyro calibration, if given."""
    if calibration is None:
        return None
    from .geometry import Pose
    from .glim import MID360_T_LIDAR_IMU
    rotation = json.loads(calibration.read_text(encoding="utf-8"))["q_lidar_sphere_wxyz"]
    t = MID360_T_LIDAR_IMU
    T_rig_lidar = Pose(t[:3], (t[6], *t[3:6])).inverse()
    return T_rig_lidar.compose(Pose(offset, rotation))


def serve(port: int, captures: Path, gopro_host: str | None, lidar_topic: str,
          imu_topic: str, preview: bool = True, live_slam: bool = False,
          calibration: Path | None = None, camera_offset=None, ready=None,
          output_root: Path | None = None):
    lidar = LidarState()
    camera = Camera(GoPro(gopro_host) if gopro_host else None)
    recorder = Recorder(captures, camera, (lidar_topic, imu_topic))
    # A calibration or offset given on the command line becomes the console's current
    # one; otherwise the one remembered from the last calibration or dataset is used.
    root = output_root or Path.home() / "splatrig-runs"
    library = Library(captures, root)
    library.purge(expired_only=True)

    def finished(job):
        library.save_meta(job["capture"], last_error=job["error"],
                          last_job={"kind": job["kind"], "finished_ns": job["finished_ns"],
                                    "error": job["error"]})
    processor = Processor(captures, root, gopro_host, on_finish=finished)
    if calibration is not None:
        processor.settings["calibration"] = str(calibration.resolve())
    if camera_offset is not None:
        processor.settings["camera_offset"] = [float(v) for v in camera_offset]
    if calibration is not None or camera_offset is not None:
        processor.save_settings()
    calibration = processor.calibration()
    camera_offset = tuple(processor.settings["camera_offset"])
    slam = SlamState(camera=camera_mount(calibration, camera_offset)) if live_slam else None
    if slam is not None:
        def remount(path, offset):
            slam.camera = camera_mount(path, offset)
        processor.on_mount = remount
    guide = ScanGuide() if slam is not None else None
    stop_ros = start_ros(lidar, lidar_topic, imu_topic, slam, guide)
    glim = GlimProcess() if live_slam else None
    if glim and glim.reaped:
        print(f"Stopped GLIM left by an earlier console (PID {', '.join(map(str, glim.reaped))})",
              flush=True)
    server = ThreadingHTTPServer(("127.0.0.1", port),
                                 make_handler(lidar, camera, recorder, slam, glim, processor,
                                              guide, library))
    server.daemon_threads = True
    if camera.gopro and preview:
        try:
            camera.start_preview()
        except ValueError as error:
            camera.error = str(error)
    print(f"SplatRig live console: http://127.0.0.1:{port}/  (GoPro: {gopro_host or 'none'})",
          flush=True)
    if calibration:
        print(f"Calibration: {calibration} (camera offset {camera_offset} m)", flush=True)
    print(f"Datasets and calibrations go to {processor.root}", flush=True)
    print("Press Ctrl+C here to stop.", flush=True)
    if ready:
        ready()
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    # Ctrl+C, a closed terminal (SIGHUP) and a plain kill (SIGTERM) take one cleanup
    # path; SIGINT is set explicitly because a shell may start jobs with it ignored.
    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, interrupt)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        def patience(signum, frame):
            try:
                print("Still stopping; GLIM may take a few seconds", flush=True)
            except OSError:
                pass
        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(number, patience)
        print("Stopping the console", flush=True)
        note = processor.interrupted()
        if note:
            print(note, file=sys.stderr, flush=True)
        # Each step runs even if an earlier one fails, so GLIM is never left behind.
        steps = [("web server", server.server_close),
                 ("recording", recorder.stop if recorder.process else None),
                 ("GLIM", glim.stop if glim else None),
                 ("camera", camera.close), ("ROS", stop_ros)]
        for name, step in steps:
            if step is None:
                continue
            try:
                step()
            except Exception as error:  # report and continue with the next step
                print(f"Could not stop the {name} cleanly: {error}", file=sys.stderr, flush=True)
