# SPDX-License-Identifier: MIT
"""Calibration and dataset jobs started from the live console.

One job runs at a time in a background thread. Results go to an output root
(default ~/splatrig-runs) that is separate from the captures: captures are never
modified, and every job writes a new folder that the pipeline publishes only on
success. The console remembers the newest calibration and camera offset in
<root>/console.json, so a restarted console shows the camera pose again.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
import threading
import time

NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")
SETTINGS = "console.json"
STAGES = {"calibrate": ["video", "fit"],
          "dataset": ["lidar", "slam", "video", "frames", "assembly", "views", "masks", "colors",
                      "export", "people"]}
MAX_OFFSET_M = 2.0


def unique_folder(root: Path, name: str) -> Path:
    candidate, number = root / name, 2
    while candidate.exists() or (root / f".{candidate.name}-work").exists():
        candidate, number = root / f"{name}-{number}", number + 1
    return candidate


def camera_offset(value) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            or abs(v) > MAX_OFFSET_M for v in value):
        raise ValueError(f"Camera offset must be three numbers within ±{MAX_OFFSET_M} m")
    return tuple(float(v) for v in value)


def default_calibrate(capture, output, *, gopro_host=None, progress=None):
    from .pipeline import calibrate_capture
    return calibrate_capture(capture, output, gopro_host=gopro_host, progress=progress)


def default_process(capture, output, *, calibration, camera_offset, gopro_host=None,
                    progress=None):
    from .pipeline import process_capture
    return process_capture(capture, output, calibration=calibration,
                           camera_offset=camera_offset, gopro_host=gopro_host,
                           progress=progress)


class Processor:
    def __init__(self, captures: Path, root: Path, gopro_host: str | None = None,
                 calibrate=default_calibrate, process=default_process, on_mount=None):
        self.captures, self.root = captures, root
        self.gopro_host = gopro_host
        self.calibrate, self.process = calibrate, process
        self.on_mount = on_mount  # called with (calibration path, offset) after a change
        self.lock = threading.Lock()
        self.job = None
        self.thread = None
        self.settings = self.load_settings()

    # ---------- settings ----------
    def load_settings(self) -> dict:
        settings = {"calibration": None, "camera_offset": [0.0, 0.0, 0.0]}
        try:
            stored = json.loads((self.root / SETTINGS).read_text(encoding="utf-8"))
            settings["camera_offset"] = list(camera_offset(stored.get("camera_offset")))
            if stored.get("calibration") and Path(stored["calibration"]).is_file():
                settings["calibration"] = stored["calibration"]
        except (OSError, ValueError, AttributeError):
            pass  # no or unreadable settings: start without a calibration
        return settings

    def save_settings(self):
        self.root.mkdir(parents=True, exist_ok=True)
        path = self.root / SETTINGS
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.settings, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)

    def calibration(self) -> Path | None:
        value = self.settings["calibration"]
        return Path(value) if value else None

    def measured_offset(self):
        """The current calibration's accelerometer offset estimate, if any."""
        path = self.calibration()
        try:
            value = json.loads(path.read_text(encoding="utf-8")).get("camera_offset_m") \
                if path else None
            return camera_offset(value) if value is not None else None
        except (OSError, ValueError, AttributeError):
            return None

    # ---------- captures ----------
    def capture(self, name) -> Path:
        if not isinstance(name, str) or not NAME.fullmatch(name):
            raise ValueError("Unknown capture")
        folder = self.captures / name
        if not (folder / "capture-log.json").is_file():
            raise ValueError(f"{name} is not a capture recorded by the console")
        return folder

    def list_captures(self, limit=30) -> list[dict]:
        found = []
        if self.captures.is_dir():
            for folder in self.captures.iterdir():
                if NAME.fullmatch(folder.name) and (folder / "capture-log.json").is_file():
                    found.append(folder)
        found.sort(key=lambda f: f.stat().st_mtime, reverse=True)
        result = []
        for folder in found[:limit]:
            try:
                log = json.loads((folder / "capture-log.json").read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            lidar, gopro = log.get("lidar", {}), log.get("gopro", {})
            duration = (lidar["stopped_ns"] - lidar["started_ns"]) / 1e9 \
                if "stopped_ns" in lidar and "started_ns" in lidar else None
            result.append({"name": folder.name,
                           "duration_s": round(duration, 1) if duration is not None else None,
                           "video": [m for m in gopro.get("new_media", []) if m.endswith(".360")],
                           "finished": "stopped_ns" in lidar})
        return result

    # ---------- jobs ----------
    def busy(self) -> bool:
        with self.lock:
            return self.job is not None and self.job["finished_ns"] is None

    def status(self) -> dict:
        with self.lock:
            job = json.loads(json.dumps(self.job)) if self.job else None
        calibration = None
        path = self.calibration()
        if path:
            calibration = {"path": str(path), "name": path.parent.name}
            try:
                report = json.loads(path.read_text(encoding="utf-8"))
                calibration.update(correlation=round(report["magnitude_correlation"], 4),
                                   weak_axis=bool(report.get("weak_axis")),
                                   camera_offset_m=report.get("camera_offset_m"))
            except (OSError, ValueError, KeyError, TypeError):
                pass
        return {"job": job, "calibration": calibration,
                "camera_offset": self.settings["camera_offset"], "output_root": str(self.root)}

    def start(self, kind: str, name, offset=None) -> dict:
        if kind not in STAGES:
            raise ValueError("Unknown job")
        capture = self.capture(name)
        offset = camera_offset(offset if offset is not None else self.settings["camera_offset"])
        if kind == "dataset" and self.calibration() is None:
            raise ValueError("No calibration yet: record a rotation of the fixed rig, "
                             "then press Use as calibration")
        with self.lock:
            if self.job is not None and self.job["finished_ns"] is None:
                raise ValueError("A job is already running")
            output = unique_folder(self.root, f"calibration-{name}" if kind == "calibrate"
                                   else name)
            self.job = {"kind": kind, "capture": name, "output": str(output),
                        "stages": STAGES[kind], "stage": None,
                        "started_ns": time.time_ns(), "finished_ns": None,
                        "error": None, "result": None}
            job = self.job
        self.root.mkdir(parents=True, exist_ok=True)
        self.thread = threading.Thread(target=self._run, args=(job, capture, output, offset),
                                       daemon=True)
        self.thread.start()
        return self.status()

    def _run(self, job, capture: Path, output: Path, offset):
        def progress(stage):
            with self.lock:
                job["stage"] = stage
        result = error = None
        try:
            if job["kind"] == "calibrate":
                report = self.calibrate(capture, output, gopro_host=self.gopro_host,
                                        progress=progress)
                result = {"calibration": str(output / "calibration.json"),
                          "correlation": round(report["magnitude_correlation"], 4),
                          "residual_ratio": round(report["residual_ratio"], 4),
                          "weak_axis": bool(report.get("weak_axis")),
                          "offset_vs_command_ack_ms": round(report["offset_vs_command_ack_ms"], 1),
                          "camera_offset_m": report.get("camera_offset_m"),
                          "offset_note": (report.get("lever_arm") or {}).get("reason")}
                self.settings["calibration"] = result["calibration"]
                if result["camera_offset_m"]:  # the estimate replaces a hand-entered offset
                    self.settings["camera_offset"] = list(camera_offset(result["camera_offset_m"]))
            else:
                # An offset equal to the calibration's measurement is passed as None, so
                # the dataset records it as measured rather than entered by hand.
                measured = self.measured_offset()
                same = measured is not None and all(abs(a - b) < 1e-9
                                                    for a, b in zip(measured, offset))
                report = self.process(capture, output, calibration=self.calibration(),
                                      camera_offset=None if same else offset,
                                      gopro_host=self.gopro_host, progress=progress)
                steps = report.get("steps", {})
                mask = steps.get("masks", {})
                people = mask.get("people", {})
                keyframes = steps.get("keyframes", {})
                result = {"dataset": report.get("dataset"),
                          "images": steps.get("dataset", {}).get("frames"),
                          "points": steps.get("dataset", {}).get("voxel_points"),
                          "keyframes": keyframes.get("selected"),
                          "too_fast": keyframes.get("too_fast"),
                          "masks": mask.get("masks"),
                          "people": people.get("person_fraction_by_camera") is not None,
                          "mask_note": people.get("skipped") or people.get("failed"),
                          "provisional": report.get("provisional", True)}
                self.settings["camera_offset"] = list(offset)
            self.save_settings()
            if self.on_mount:
                self.on_mount(self.calibration(), tuple(self.settings["camera_offset"]))
        except Exception as failure:  # reported in the console, never raised into the server
            error = str(failure) if isinstance(failure, ValueError) else \
                f"{type(failure).__name__}: {failure}"
        with self.lock:
            job.update(result=result, error=error, finished_ns=time.time_ns())

    def interrupted(self) -> str | None:
        """Message for a job cut off by the console stopping, if one is running."""
        with self.lock:
            job = self.job
            if job is None or job["finished_ns"] is not None:
                return None
            output = Path(job["output"])
        return (f"Stopped while processing {job['capture']}; nothing was published. "
                f"Partial work may remain in {output.with_name('.' + output.name + '-work')}")
