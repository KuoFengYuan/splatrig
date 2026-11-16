# SPDX-License-Identifier: MIT
"""The scan library: names, status, covers, storage and Recently Deleted.

Captures stay where the recorder wrote them and are not modified. What the console
adds lives beside its outputs: <root>/library/<capture>.json holds the name and the
live guide's summary at Stop, and <root>/library/covers/<capture>.jpg a snapshot of
the live map. Deleting moves a capture and its outputs into `.deleted` folders with
a dated marker; restoring moves them back, and they are purged after `keep_days`.
"""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import time

NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")
NAME_MAX = 40
COVER_MAX = 800_000
DELETED = ".deleted"
SORTS = ("newest", "oldest", "name", "largest")
FILTERS = ("all", "dataset", "unprocessed")


def default_name(started: float) -> str:
    moment = datetime.fromtimestamp(started)
    part = ("night" if moment.hour < 5 else "morning" if moment.hour < 12 else
            "afternoon" if moment.hour < 17 else "evening" if moment.hour < 21 else "night")
    return f"{moment.strftime('%A')} {part} scan"


def folder_size(path: Path) -> int:
    total = 0
    for item in path.rglob("*"):
        try:
            if item.is_file() and not item.is_symlink():
                total += item.stat().st_size
        except OSError:
            continue
    return total


class Library:
    def __init__(self, captures: Path, root: Path, keep_days=30, clock=time.time):
        self.captures, self.root = captures, root
        self.meta_dir = root / "library"
        self.keep_days, self.clock = keep_days, clock
        self._sizes = {}

    # ---------- files ----------
    def capture(self, name) -> Path:
        if not isinstance(name, str) or not NAME.fullmatch(name):
            raise ValueError("Unknown scan")
        folder = self.captures / name
        if not (folder / "capture-log.json").is_file():
            raise ValueError(f"{name} is not a scan recorded by the console")
        return folder

    def ids(self) -> list[str]:
        if not self.captures.is_dir():
            return []
        return [f.name for f in self.captures.iterdir()
                if NAME.fullmatch(f.name) and (f / "capture-log.json").is_file()]

    def outputs(self, name: str, root: Path | None = None) -> list[Path]:
        """Datasets and calibrations made from a capture, newest last."""
        root = root or self.root
        if not root.is_dir():
            return []
        pattern = re.compile(rf"(calibration-)?{re.escape(name)}(-\d+)?|"
                             rf"\.(calibration-)?{re.escape(name)}(-\d+)?-(work|failed)")
        found = [p for p in root.iterdir() if p.is_dir() and pattern.fullmatch(p.name)]
        return sorted(found, key=lambda p: p.stat().st_mtime)

    def meta(self, name: str) -> dict:
        try:
            value = json.loads((self.meta_dir / f"{name}.json").read_text(encoding="utf-8"))
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError):
            return {}

    def save_meta(self, scan: str, /, **fields):
        self.meta_dir.mkdir(parents=True, exist_ok=True)
        value = {**self.meta(scan), **fields}
        path = self.meta_dir / f"{scan}.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)

    def cover_path(self, name: str) -> Path:
        return self.meta_dir / "covers" / f"{name}.jpg"

    # ---------- entries ----------
    def size(self, path: Path) -> int:
        key = str(path)
        try:
            stamp = path.stat().st_mtime
        except OSError:
            return 0
        cached = self._sizes.get(key)
        if cached and cached[0] == stamp:
            return cached[1]
        value = folder_size(path)
        self._sizes[key] = (stamp, value)
        return value

    def entry(self, name: str, *, job=None, queued=(), recording=None,
              calibration: Path | None = None) -> dict:
        folder = self.captures / name
        try:
            log = json.loads((folder / "capture-log.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            log = {}
        lidar, gopro = log.get("lidar", {}), log.get("gopro", {})
        started = lidar.get("started_ns", int(folder.stat().st_mtime * 1e9)) / 1e9
        duration = (lidar["stopped_ns"] - lidar["started_ns"]) / 1e9 \
            if "stopped_ns" in lidar and "started_ns" in lidar else None
        meta = self.meta(name)
        outputs = self.outputs(name)
        datasets = [p for p in outputs if p.name.startswith(name) and (p / "dataset").is_dir()]
        calibrations = [p for p in outputs if p.name.startswith("calibration-") and
                        (p / "calibration.json").is_file()]
        current = calibration is not None and any(
            calibration.resolve().is_relative_to(p.resolve()) for p in calibrations)
        video = [m for m in gopro.get("new_media", []) if m.endswith(".360")]
        running = job is not None and job.get("capture") == name and not job.get("finished_ns")
        status = ("recording" if recording == name else
                  "processing" if running else
                  "queued" if name in queued else
                  "failed" if meta.get("last_error") else
                  "dataset" if datasets else
                  "calibration" if calibrations else "recorded")
        tags = []
        if not video:
            tags.append("LiDAR only")
        if current:
            tags.append("current calibration")
        elif calibrations:
            tags.append("calibration")
        return {"id": name, "name": meta.get("name") or default_name(started),
                "custom_name": bool(meta.get("name")), "started": started,
                "duration_s": round(duration, 1) if duration is not None else None,
                "finished": "stopped_ns" in lidar, "video": video,
                "size_bytes": self.size(folder) + sum(self.size(p) for p in outputs),
                "status": status, "stage": job.get("stage") if running else None,
                "stages": job.get("stages") if running else None, "tags": tags,
                "dataset": str(datasets[-1] / "dataset") if datasets else None,
                "calibration": str(calibrations[-1] / "calibration.json") if calibrations else None,
                "current_calibration": current, "guide": meta.get("guide"),
                "last_error": meta.get("last_error"),
                "cover": self.cover_path(name).is_file() or bool(datasets)}

    def list(self, sort="newest", filter="all", query="", **context) -> dict:
        if sort not in SORTS or filter not in FILTERS:
            raise ValueError("Unknown sort or filter")
        entries = [self.entry(name, **context) for name in self.ids()]
        # Automatic names that repeat get numbers, oldest first; custom names stay.
        seen = {}
        for item in sorted(entries, key=lambda e: e["started"]):
            if not item["custom_name"]:
                seen[item["name"]] = seen.get(item["name"], 0) + 1
                if seen[item["name"]] > 1:
                    item["name"] = f"{item['name']} {seen[item['name']]}"
        total = len(entries)
        if filter == "dataset":
            entries = [e for e in entries if e["dataset"]]
        elif filter == "unprocessed":
            entries = [e for e in entries if not e["dataset"]]
        needle = query.strip().casefold()
        if needle:
            entries = [e for e in entries if needle in e["name"].casefold() or needle in e["id"]]
        key = {"newest": lambda e: -e["started"], "oldest": lambda e: e["started"],
               "name": lambda e: e["name"].casefold(), "largest": lambda e: -e["size_bytes"]}[sort]
        entries.sort(key=key)
        return {"scans": entries, "total": total, "found": len(entries),
                "deleted": len(self.deleted()), "storage": self.storage()}

    # ---------- edits ----------
    def rename(self, name: str, title) -> str:
        self.capture(name)
        if not isinstance(title, str):
            raise ValueError("A name must be text")
        title = " ".join(title.split())
        if len(title) > NAME_MAX:
            raise ValueError(f"Names are at most {NAME_MAX} characters")
        self.save_meta(name, name=title or None)  # blank returns to the automatic name
        return title

    def set_cover(self, name: str, data: bytes):
        self.capture(name)
        if not data.startswith(b"\xff\xd8") or len(data) > COVER_MAX:
            raise ValueError("A cover must be a JPEG under 800 kB")
        path = self.cover_path(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_bytes(data)
        temporary.replace(path)

    def delete_dataset(self, name: str) -> int:
        """Remove the datasets made from a capture; the recording and calibrations stay."""
        self.capture(name)
        removed = 0
        for path in self.outputs(name):
            if path.name.startswith(name) or path.name.startswith("." + name):
                shutil.rmtree(path)
                removed += 1
        return removed

    # ---------- Recently Deleted ----------
    def delete(self, names, protected=()) -> list[str]:
        moved = []
        for name in names:
            self.capture(name)
            if name in protected:
                raise ValueError(f"{name} is in use (recording, processing or the current "
                                 "calibration) and cannot be deleted now")
        for name in names:
            stamp = self.clock()
            trash = self.captures / DELETED
            trash.mkdir(exist_ok=True)
            if (trash / name).exists():
                shutil.rmtree(trash / name)
            (self.captures / name).rename(trash / name)
            out_trash = self.root / DELETED / name
            for path in self.outputs(name):
                out_trash.mkdir(parents=True, exist_ok=True)
                path.rename(out_trash / path.name)
            (trash / f"{name}.deleted.json").write_text(
                json.dumps({"deleted": stamp}) + "\n", encoding="utf-8")
            moved.append(name)
        return moved

    def deleted(self) -> list[dict]:
        trash = self.captures / DELETED
        if not trash.is_dir():
            return []
        found = []
        for marker in trash.glob("*.deleted.json"):
            name = marker.name[:-len(".deleted.json")]
            try:
                stamp = json.loads(marker.read_text(encoding="utf-8"))["deleted"]
            except (OSError, ValueError, KeyError, TypeError):
                stamp = marker.stat().st_mtime
            left = self.keep_days - (self.clock() - stamp) / 86400
            meta = self.meta(name)
            found.append({"id": name, "name": meta.get("name") or name, "deleted": stamp,
                          "days_left": max(0, int(left + 0.999)),
                          "size_bytes": self.size(trash / name) +
                          self.size(self.root / DELETED / name)})
        return sorted(found, key=lambda e: -e["deleted"])

    def restore(self, names) -> list[str]:
        trash, restored = self.captures / DELETED, []
        for name in names:
            if not isinstance(name, str) or not NAME.fullmatch(name) or \
                    not (trash / name).is_dir():
                raise ValueError(f"{name} is not in Recently Deleted")
            if (self.captures / name).exists():
                raise ValueError(f"A scan named {name} already exists")
        for name in names:
            (trash / name).rename(self.captures / name)
            for path in self.outputs(name, self.root / DELETED / name):
                if not (self.root / path.name).exists():
                    path.rename(self.root / path.name)
            shutil.rmtree(self.root / DELETED / name, ignore_errors=True)
            (trash / f"{name}.deleted.json").unlink(missing_ok=True)
            restored.append(name)
        return restored

    def purge(self, names=None, expired_only=False) -> list[str]:
        """Delete for good: the given scans, all of Recently Deleted, or the expired."""
        trash, purged = self.captures / DELETED, []
        for item in self.deleted():
            if names is not None and item["id"] not in names:
                continue
            if expired_only and item["days_left"] > 0:
                continue
            shutil.rmtree(trash / item["id"], ignore_errors=True)
            shutil.rmtree(self.root / DELETED / item["id"], ignore_errors=True)
            (trash / f"{item['id']}.deleted.json").unlink(missing_ok=True)
            (self.meta_dir / f"{item['id']}.json").unlink(missing_ok=True)
            self.cover_path(item["id"]).unlink(missing_ok=True)
            purged.append(item["id"])
        return purged

    def storage(self) -> dict:
        trash = self.captures / DELETED
        scans = sum(self.size(self.captures / name) for name in self.ids())
        outputs = sum(self.size(p) for p in self.root.iterdir()
                      if p.is_dir() and p.name not in {"library", DELETED}) \
            if self.root.is_dir() else 0
        deleted = sum(item["size_bytes"] for item in self.deleted()) if trash.is_dir() else 0
        try:
            free = shutil.disk_usage(self.root if self.root.exists() else self.captures).free
        except OSError:
            free = None
        return {"recordings_bytes": scans, "outputs_bytes": outputs,
                "deleted_bytes": deleted, "free_bytes": free}
