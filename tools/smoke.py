# SPDX-License-Identifier: MIT
"""Exercise the installed CLI in an isolated workspace."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def run(*args):
    process = subprocess.run([sys.executable, "-m", "splatrig", *map(str, args)],
                             capture_output=True, text=True)
    if process.returncode:
        raise RuntimeError(process.stderr or process.stdout)
    return json.loads(process.stdout)


def main():
    with tempfile.TemporaryDirectory(prefix="splatrig-smoke-") as temporary:
        base = Path(temporary)
        capture = base / "capture"
        run("demo", "--output", capture)
        report = run("validate", capture)
        run("replay", capture, "--output", base / "replay")
        run("export", capture, "--output", base / "colmap")
        events = base / "events.csv"
        events.write_text("camera_ns,reference_ns\n0,1000000000\n"
                          "1000000000,2000100000\n2000000000,3000200000\n")
        fit = run("fit-clock", events, "--output", base / "clock.json")
        panorama = "skipped: ffmpeg/ffprobe unavailable"
        if shutil.which("ffmpeg") and shutil.which("ffprobe"):
            run("demo", "--panorama", "--output", base / "panorama")
            views = run("panorama-views", base / "panorama", "--output", base / "views",
                        "--size", "64")
            checked = run("validate", base / "views")
            run("export", base / "views", "--output", base / "views-colmap")
            if views["frames"] != 48 or checked["cameras"] != 6 or checked["frames"] != 48:
                raise RuntimeError("Panorama views changed")
            panorama = {"frames": views["frames"],
                        "probe_max_error_px": views["probe_max_error_px"]}
        if report["frames"] != 8 or not report["synthetic"]:
            raise RuntimeError("Synthetic source or frame count was lost")
        if abs(fit["fit"]["drift_ppm"]-100) > 0.000001:
            raise RuntimeError("Clock fit changed")
        for item in ["cameras.bin", "images.bin", "points3D.bin"]:
            if not (base / "colmap/sparse/0" / item).is_file():
                raise RuntimeError(f"Missing export: {item}")
        print(json.dumps({"result": "passed", "frames": report["frames"],
                          "raw_points": report["raw_points"],
                          "voxel_points": report["voxel_points"],
                          "panorama_views": panorama, "synthetic": True}, indent=2))


if __name__ == "__main__":
    main()
