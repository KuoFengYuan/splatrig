# SPDX-License-Identifier: MIT
"""COLMAP seed models and self-contained replay previews."""

from __future__ import annotations

from contextlib import contextmanager
import json
from pathlib import Path
import shutil
import struct
import tempfile

from .geometry import Pose
from .session import Prepared, local_file, write_json, write_jsonl


@contextmanager
def new_output(destination: Path, source: Path):
    destination = destination.resolve()
    if destination == source.resolve() or destination.is_relative_to(source.resolve()):
        raise ValueError("Output must be outside the captured session")
    if destination.exists():
        raise ValueError(f"Output already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{destination.name}-",
                                     dir=destination.parent))
    try:
        yield temporary
        if destination.exists():
            raise ValueError(f"Output appeared during export: {destination}")
        temporary.rename(destination)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def write_ply(path: Path, points):
    with path.open("x", encoding="ascii") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {len(points)}\n")
        for name in ["x", "y", "z"]:
            stream.write(f"property double {name}\n")
        for name in ["red", "green", "blue"]:
            stream.write(f"property uchar {name}\n")
        stream.write("end_header\n")
        for x, y, z, r, g, b in points:
            stream.write(f"{x:.9g} {y:.9g} {z:.9g} {r} {g} {b}\n")


def write_preview(root: Path, data: Prepared):
    cloud = data.points[::max(1, (len(data.points)+19999)//20000)]
    payload = {"points": cloud,
               "frames": [{k: frame[k] for k in ("id", "T_world_camera",
                          "reference_timestamp_ns")} for frame in data.frames],
               "report": data.report}
    template = (Path(__file__).parent / "assets/viewer.html").read_text(encoding="utf-8")
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False)
    encoded = encoded.replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")
    (root / "preview.html").write_text(template.replace("__SPLATRIG_DATA__", encoded),
                                      encoding="utf-8")


def pose_records(data: Prepared, exported_images=False):
    for frame in data.frames:
        record = {key: value for key, value in frame.items() if key != "image_path"}
        record["source_image_path"] = frame["image_path"]
        if exported_images:
            record["image_path"] = f"images/{frame['id']:08d}.png"
        yield record


def replay(data: Prepared, destination: Path):
    with new_output(destination, data.root) as root:
        write_ply(root / "points.ply", data.points)
        write_jsonl(root / "image-poses.jsonl", pose_records(data))
        write_json(root / "report.json", data.report)
        write_preview(root, data)


def export_colmap(data: Prepared, destination: Path):
    with new_output(destination, data.root) as root:
        sparse = root / "sparse/0"
        sparse.mkdir(parents=True)
        (root / "images").mkdir()
        with (sparse / "cameras.bin").open("xb") as binary, (
                sparse / "cameras.txt").open("x", encoding="ascii") as text:
            binary.write(struct.pack("<Q", len(data.cameras)))
            text.write("# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n")
            for camera in sorted(data.cameras.values(), key=lambda c: c.id):
                binary.write(struct.pack("<iiQQ4d", camera.id, 1, camera.width,
                                         camera.height, *camera.params))
                params = " ".join(format(v, ".17g") for v in camera.params)
                text.write(f"{camera.id} PINHOLE {camera.width} {camera.height} {params}\n")
        with (sparse / "images.bin").open("xb") as binary, (
                sparse / "images.txt").open("x", encoding="ascii") as text:
            binary.write(struct.pack("<Q", len(data.frames)))
            text.write("# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n")
            text.write("# Empty POINTS2D[]: this is a seed model, not triangulated SfM.\n")
            for frame in data.frames:
                frame_id = frame["id"]
                name = f"{frame_id:08d}.png"
                shutil.copyfile(local_file(data.root, frame["image_path"]),
                                root / "images" / name)
                w2c = Pose.from_dict(frame["T_world_camera"]).inverse()
                values = (*w2c.quaternion_wxyz, *w2c.translation)
                binary.write(struct.pack("<i7di", frame_id, *values, frame["camera_id"]))
                binary.write(name.encode("ascii") + b"\0" + struct.pack("<Q", 0))
                text.write(f"{frame_id} " + " ".join(format(v, ".17g") for v in values)
                           + f" {frame['camera_id']} {name}\n\n")
        with (sparse / "points3D.bin").open("xb") as binary, (
                sparse / "points3D.txt").open("x", encoding="ascii") as text:
            binary.write(struct.pack("<Q", len(data.points)))
            text.write("# POINT3D_ID X Y Z R G B ERROR; no observation tracks\n")
            for index, (x, y, z, r, g, b) in enumerate(data.points, 1):
                binary.write(struct.pack("<Q3d3BdQ", index, x, y, z, r, g, b, 0.0, 0))
                text.write(f"{index} {x:.17g} {y:.17g} {z:.17g} {r} {g} {b} 0\n")
        write_ply(root / "points.ply", data.points)
        write_jsonl(root / "image-poses.jsonl", pose_records(data, exported_images=True))
        write_json(root / "report.json", data.report)
        write_preview(root, data)
