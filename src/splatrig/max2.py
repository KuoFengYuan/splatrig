# SPDX-License-Identifier: MIT
"""GoPro MAX2 .360 recordings to equirectangular panoramas with media timestamps.

Each .360 file carries two HEVC tracks of 5888x1920. Track 0 holds the LEFT,
FRONT and RIGHT equi-angular cube faces and track 1 holds DOWN, BACK and TOP
(rotated 90 degrees). LEFT, RIGHT, DOWN and TOP are 1984 px wide: two lens
halves of 992 px that overlap by 64 px around the face midline. Removing the
64 px at the midline gives 1920 px faces; stacking the tracks gives the YouTube
EAC layout that ffmpeg's v360 filter reads. Layout per max2sphere's MAX2
template {5888, 1920, 1984, 1920, 32, 7680}; orientation was checked visually.

The hard cut at the lens midline shows stitching parallax for near objects.
Timestamps are media time in the camera clock (first video frame = 0); they are
not wall-clock or LiDAR time until a clock model maps them.
"""

from __future__ import annotations

from fractions import Fraction
import json
from pathlib import Path
import re
import subprocess

from .export import new_output
from .panorama import run_tool
from .session import write_json, write_jsonl

TRACK = (5888, 1920)
FACE = 1920
HALF = 960
OVERLAP = 64


def eac_graph(width: int) -> str:
    """ffmpeg filter graph: two MAX2 tracks -> equirectangular [out] of width x width/2."""
    def row(stream, tag):
        left = f"[{tag}0]crop={HALF}:{FACE}:0:0[{tag}l0];[{tag}1]crop={HALF}:{FACE}:{HALF + OVERLAP}:0[{tag}l1]"
        center = f"[{tag}2]crop={FACE}:{FACE}:{FACE + OVERLAP}:0[{tag}c]"
        right_x = 2 * FACE + OVERLAP
        right = (f"[{tag}3]crop={HALF}:{FACE}:{right_x}:0[{tag}r0];"
                 f"[{tag}4]crop={HALF}:{FACE}:{right_x + HALF + OVERLAP}:0[{tag}r1]")
        return (f"[0:v:{stream}]split=5[{tag}0][{tag}1][{tag}2][{tag}3][{tag}4];{left};{center};"
                f"{right};[{tag}l0][{tag}l1]hstack[{tag}L];[{tag}r0][{tag}r1]hstack[{tag}R];"
                f"[{tag}L][{tag}c][{tag}R]hstack=3[{tag}row]")
    return (f"{row(0, 'a')};{row(1, 'b')};[arow][brow]vstack[eac];"
            f"[eac]v360=input=eac:output=e:w={width}:h={width // 2}")


def probe(path: Path, ffprobe="ffprobe") -> dict:
    data = json.loads(run_tool([ffprobe, "-v", "error", "-show_entries",
                                "stream=index,codec_type,width,height,time_base,r_frame_rate:"
                                "stream_tags=handler_name", "-of", "json", str(path)]))
    video = [s for s in data.get("streams", []) if s.get("codec_type") == "video"]
    if len(video) < 2 or any((s.get("width"), s.get("height")) != TRACK for s in video[:2]):
        raise ValueError(f"{path.name}: expected two {TRACK[0]}x{TRACK[1]} MAX2 video tracks")
    return {"video": video[:2], "metadata": [s["index"] for s in data["streams"]
                                              if s.get("tags", {}).get("handler_name") == "GoPro MET"]}


def frame_times_ns(path: Path, stream_index: int, time_base: str, ffprobe="ffprobe") -> list[int]:
    """Presentation times of every frame of one video track, ns from the first frame."""
    output = run_tool([ffprobe, "-v", "error", "-select_streams", str(stream_index),
                       "-show_entries", "packet=pts", "-of", "csv=p=0", str(path)]).decode()
    pts = sorted(int(value) for value in re.findall(r"-?\d+", output))
    if not pts:
        raise ValueError(f"{path.name}: no video packets")
    base = Fraction(time_base)
    return [round((value - pts[0]) * base * 1_000_000_000) for value in pts]


def extract(path: Path, destination: Path, *, every: int = 15, width: int = 3840,
            image_format="jpg", first_id: int = 1, ffmpeg="ffmpeg", ffprobe="ffprobe") -> dict:
    """Decode every Nth original frame; write panoramas and panoramas.jsonl."""
    if every < 1 or width < 256 or width % 2 or image_format not in {"jpg", "png"}:
        raise ValueError("Use every >= 1, an even width >= 256 and jpg or png output")
    info = probe(path, ffprobe)
    track = info["video"][0]
    times = frame_times_ns(path, track["index"], track["time_base"], ffprobe)
    selected = list(range(0, len(times), every))
    with new_output(destination, path.parent) as root:
        (root / "images").mkdir()
        pattern = root / "images" / f"panorama_%06d.{image_format}"
        quality = ["-q:v", "2"] if image_format == "jpg" else []
        run_tool([ffmpeg, "-v", "error", "-nostdin", "-i", str(path), "-filter_complex",
                  eac_graph(width) + f",select='not(mod(n\\,{every}))'[out]", "-map", "[out]",
                  "-fps_mode", "passthrough", *quality, "-start_number", "1", str(pattern)])
        written = sorted((root / "images").glob(f"panorama_*.{image_format}"))
        if len(written) != len(selected):
            raise ValueError(f"Decoded {len(written)} panoramas, expected {len(selected)}")
        records = [{"id": first_id + index, "timestamp_ns": times[frame],
                    "image_path": f"images/{image.name}", "source_frame": frame}
                   for index, (frame, image) in enumerate(zip(selected, written))]
        write_jsonl(root / "panoramas.jsonl", records)
        report = {"source": str(path.resolve()), "track_frames": len(times),
                  "every_nth_frame": every, "panoramas": len(records), "width": width,
                  "clock": "camera media time, ns from the first video frame",
                  "metadata_streams": info["metadata"],
                  "limitations": ["hard cut at lens midline: near-object stitching parallax",
                                  "rolling shutter and per-lens exposure not modeled",
                                  "GPMF orientation/stabilization metadata not applied"]}
        write_json(root / "max2-frames.json", report)
    return report
