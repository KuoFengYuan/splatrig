# SPDX-License-Identifier: MIT
"""One-command start of the capture console: checks, Livox driver and console.

`splatrig doctor` reports whether this machine is ready; `splatrig start` runs the
same checks, starts the Livox driver unless one is already running, then the live
console (with GLIM unless --no-slam) and opens it in a browser. Everything it
started is stopped again on exit. Machine-specific values (NIC name, addresses)
are discovered at run time from the installed Livox configuration and the host's
interfaces, so nothing machine-specific is stored in the repository.

This module imports no ROS package; it reads AMENT_PREFIX_PATH set by the ROS
environment (tools/splatrig-ros sources it).
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import tempfile
import threading
import time

DRIVER_PACKAGE = "livox_ros_driver2"
DRIVER_LAUNCH = "msg_MID360s_launch.py"
DRIVER_CONFIG = "MID360s_config.json"
PTP_INSTALL = "sudo tools/install-ptp-service {nic}"


@dataclass
class Check:
    name: str
    level: str  # "ok", "warn" or "fail"
    detail: str
    fix: str = ""


def prefixes(environ=None) -> list[Path]:
    value = (os.environ if environ is None else environ).get("AMENT_PREFIX_PATH", "")
    return [Path(p) for p in value.split(os.pathsep) if p]


def find_share(package: str, relative: str, roots) -> Path | None:
    for root in roots:
        candidate = root / "share" / package / relative
        if candidate.exists():
            return candidate
    return None


def livox_addresses(config: Path) -> tuple[list[str], list[str]]:
    """Host and LiDAR IPv4 addresses from a livox_ros_driver2 JSON configuration."""
    data = json.loads(config.read_text(encoding="utf-8"))
    hosts = [h["host_ip"] for value in data.values() if isinstance(value, dict)
             for h in value.get("host_net_info", []) if "host_ip" in h]
    lidars = [item["ip"] for item in data.get("lidar_configs", []) if "ip" in item]
    return sorted(set(hosts)), lidars


def interface_addresses(ip_output: str) -> dict[str, str]:
    """Map IPv4 address -> interface from `ip -4 -o addr` output."""
    return {m.group(2): m.group(1)
            for m in re.finditer(r"^\d+:\s+(\S+)\s+inet\s+([\d.]+)/", ip_output, re.M)}


def processes(proc=Path("/proc")) -> list[tuple[int, list[str]]]:
    found = []
    for entry in proc.glob("[0-9]*"):
        try:
            argv = (entry / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        argv = [a.decode("utf-8", "replace") for a in argv if a]
        if argv:
            found.append((int(entry.name), argv))
    return found


def ptp_masters(procs) -> list[dict]:
    """ptp4l processes (not their sudo wrappers) with interface and minor version."""
    result = []
    for pid, argv in procs:
        if Path(argv[0]).name != "ptp4l":
            continue
        option = lambda flag: argv[argv.index(flag) + 1] if flag in argv[:-1] else None
        result.append({"pid": pid, "nic": option("-i"),
                       "minor": option("--ptp_minor_version"), "software": "-S" in argv})
    return result


def driver_running(procs) -> bool:
    return any(any(a.endswith("livox_ros_driver2_node") for a in argv[:1]) for _, argv in procs)


def port_free(port: int) -> bool:
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def run_text(command, timeout=5) -> str:
    try:
        return subprocess.run(command, capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


def reachable(address: str) -> bool:
    try:
        return subprocess.run(["ping", "-c", "1", "-W", "1", address], capture_output=True,
                              timeout=5).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def checks(port=8090, gopro="auto", slam=True, environ=None, procs=None,
           ip_output=None, ping=reachable, ros_python=None) -> list[Check]:
    """Readiness of this machine for a live capture, most basic first."""
    from .gopro import find_camera
    roots = prefixes(environ)
    procs = processes() if procs is None else procs
    ip_output = run_text(["ip", "-4", "-o", "addr"]) if ip_output is None else ip_output
    out = []
    if ros_python is None:
        ros_python = importlib.util.find_spec("rclpy") is not None
    if not roots or not ros_python:
        out.append(Check("ROS 2", "fail", "ROS 2 environment not loaded",
                         "start through tools/splatrig-ros (it loads ROS 2 and the Livox workspace)"))
        return out
    out.append(Check("ROS 2", "ok", str(roots[-1])))
    config = find_share(DRIVER_PACKAGE, f"config/{DRIVER_CONFIG}", roots)
    launch = find_share(DRIVER_PACKAGE, f"launch_ROS2/{DRIVER_LAUNCH}", roots)
    if not config or not launch:
        out.append(Check("Livox driver", "fail", f"{DRIVER_PACKAGE} with {DRIVER_CONFIG} not found",
                         "build livox_ros_driver2 (docs/HARDWARE.md); set LIVOX_WS if it is not in ~/ws_livox"))
        return out
    try:
        hosts, lidars = livox_addresses(config)
    except (OSError, ValueError, KeyError, TypeError) as error:
        out.append(Check("Livox driver", "fail", f"cannot read {config}: {error}"))
        return out
    out.append(Check("Livox driver", "ok", f"host {', '.join(hosts)} → LiDAR {', '.join(lidars)}"))
    if slam:
        glim = find_share("glim", "config/config.json", roots)
        out.append(Check("GLIM", "ok", "installed") if glim else
                   Check("GLIM", "fail", "glim_ros not found",
                         "install GLIM (docs/HARDWARE.md), or start with --no-slam"))
    addresses = interface_addresses(ip_output)
    nics = sorted({addresses[h] for h in hosts if h in addresses})
    if not nics:
        out.append(Check("LiDAR network", "fail", f"no interface has {', '.join(hosts)}",
                         "plug in the LiDAR network adapter and bring its connection up "
                         "(nmcli connection up <name>)"))
    else:
        out.append(Check("LiDAR network", "ok", f"{', '.join(nics)} has {', '.join(hosts)}"))
        silent = [ip for ip in lidars if not ping(ip)]
        out.append(Check("LiDAR", "warn", f"{', '.join(silent)} does not answer",
                         "check LiDAR power and cable") if silent else
                   Check("LiDAR", "ok", f"{', '.join(lidars)} answers"))
        nic = nics[0]
        masters = ptp_masters(procs)
        mine = [m for m in masters if m["nic"] == nic]
        if not masters:
            out.append(Check("PTP time sync", "warn", "ptp4l is not running; LiDAR stamps fall "
                             "back to host arrival time", "one-time setup: " + PTP_INSTALL.format(nic=nic)))
        elif len(masters) > 1:
            out.append(Check("PTP time sync", "fail",
                             f"{len(masters)} ptp4l processes (PIDs "
                             f"{', '.join(str(m['pid']) for m in masters)}); run exactly one",
                             "stop the manually started one (Ctrl+C in its terminal)"))
        elif not mine:
            out.append(Check("PTP time sync", "fail", f"ptp4l runs on {masters[0]['nic']}, not {nic}",
                             PTP_INSTALL.format(nic=nic)))
        elif mine[0]["minor"] != "0":
            out.append(Check("PTP time sync", "warn", "ptp4l without --ptp_minor_version 0; "
                             "the Mid-360S ignores PTP v2.1", PTP_INSTALL.format(nic=nic)))
        else:
            out.append(Check("PTP time sync", "ok", f"ptp4l on {nic}"))
    out.append(Check("Livox driver process", "ok", "already running; it will be reused")
               if driver_running(procs) else
               Check("Livox driver process", "ok", "will be started"))
    camera = find_camera(ip_output) if gopro == "auto" else (None if gopro == "none" else gopro)
    out.append(Check("MAX2", "ok", f"camera at {camera}") if camera else
               Check("MAX2", "warn", "not connected; the console runs without the camera",
                     "connect the MAX2 by USB and switch it on") if gopro != "none" else
               Check("MAX2", "ok", "disabled (--gopro none)"))
    if camera and not (shutil.which("ffmpeg") and shutil.which("ffprobe")):
        out.append(Check("ffmpeg", "warn", "not found; no MAX2 preview or video processing",
                         "sudo apt install ffmpeg"))
    out.append(Check("Console port", "ok", f"{port} free") if port_free(port) else
               Check("Console port", "fail", f"{port} is in use; a console may already be running",
                     f"open http://127.0.0.1:{port}/ or stop that console (Ctrl+C), or use --port"))
    return out


def format_checks(results) -> str:
    mark = {"ok": " OK ", "warn": "WARN", "fail": "FAIL"}
    width = max(len(c.name) for c in results)
    lines = []
    for c in results:
        lines.append(f"  [{mark[c.level]}] {c.name.ljust(width)}  {c.detail}")
        if c.fix and c.level != "ok":
            lines.append(f"  {'':6} {'':{width}}  → {c.fix}")
    return "\n".join(lines)


class Driver:
    """livox_ros_driver2 started by `start`, in its own session, log kept for diagnosis."""

    def __init__(self, launch=DRIVER_LAUNCH, log_dir: Path | None = None, command=None):
        folder = log_dir or Path(tempfile.gettempdir())
        self.log_path = folder / f"splatrig-livox-driver-{os.getpid()}.log"
        self.log = self.log_path.open("wb")
        self.process = subprocess.Popen(command or ["ros2", "launch", DRIVER_PACKAGE, launch],
                                        stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)

    def running(self) -> bool:
        return self.process.poll() is None

    def stop(self):
        from .liveslam import stop_group
        stop_group(self.process.pid, timeout=10, leader=self.process)
        self.process.wait()
        self.log.close()
        self.log_path.unlink(missing_ok=True)


def open_browser(url: str, delay=1.5):
    def later():
        time.sleep(delay)
        import webbrowser
        try:
            if not webbrowser.open(url):
                print(f"Open {url} in a browser", flush=True)
        except Exception:  # no display or browser: the URL is printed already
            print(f"Open {url} in a browser", flush=True)
    threading.Thread(target=later, daemon=True).start()


def start(port=8090, captures: Path | None = None, gopro="auto", slam=True, browser=True,
          calibration: Path | None = None, camera_offset=(0.0, 0.0, 0.0),
          launch=DRIVER_LAUNCH, lidar_topic="/livox/lidar", imu_topic="/livox/imu"):
    from .gopro import detect_camera
    from .live import serve
    results = checks(port, gopro, slam)
    print("SplatRig start-up check\n" + format_checks(results), flush=True)
    if any(c.level == "fail" for c in results):
        raise ValueError("Fix the FAIL items above, then start again")
    driver = None if driver_running(processes()) else Driver(launch)
    if driver:
        print("Started the Livox driver", flush=True)
    view = f"http://127.0.0.1:{port}/" + ("?view=map" if slam else "")
    try:
        serve(port, captures or Path.home() / "captures",
              detect_camera() if gopro == "auto" else (None if gopro == "none" else gopro),
              lidar_topic, imu_topic, live_slam=slam, calibration=calibration,
              camera_offset=camera_offset,
              ready=(lambda: open_browser(view)) if browser else None)
    finally:
        if driver:
            if not driver.running():
                print(f"The Livox driver had exited; see {driver.log_path}", flush=True)
                driver.log.close()
            else:
                driver.stop()
                print("Stopped the Livox driver", flush=True)
