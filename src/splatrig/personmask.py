# SPDX-License-Identifier: MIT
"""Person masks for a dataset's images, merged into masks/ (0 = ignore).

People in the photos, the operator above all, appear at a different place in
every photo and leave ghosts in Gaussian splatting. A COCO instance segmentation
model (torchvision Mask R-CNN v2, BSD licence, weights downloaded on first use)
finds them in each image; views looking steeply up or down see people rotated,
so those are also run at the other three right-angle turns. The person pixels,
widened by `dilate` pixels, are set to 0 in masks/<image name>, keeping whatever
the mask already ignored (the rig).

This module runs in a separate Python with torch and torchvision, started by the
pipeline as `python -m splatrig.personmask`; the offline core never imports torch.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import time

TORCH_PYTHON = "SPLATRIG_TORCH_PYTHON"


def run(dataset: Path, python: str | None = None, rotate_cameras=(), score=0.5, dilate=12,
        command=None) -> dict:
    """Start the helper in `python` (or $SPLATRIG_TORCH_PYTHON) and return its report."""
    python = python or os.environ.get(TORCH_PYTHON)
    if not python and command is None:
        return {"masks": 0, "skipped": f"set {TORCH_PYTHON} to a Python with torch and "
                                       "torchvision to mask people"}
    package = str(Path(__file__).resolve().parent.parent)
    environment = {**os.environ, "PYTHONPATH": package + os.pathsep +
                   os.environ.get("PYTHONPATH", "")}
    argv = (command or [python, "-m", "splatrig.personmask"]) + [
        str(dataset), "--score", str(score), "--dilate", str(dilate),
        "--rotate-cameras", ",".join(str(c) for c in rotate_cameras)]
    result = subprocess.run(argv, capture_output=True, text=True, env=environment)
    if result.returncode:
        lines = (result.stderr or result.stdout).strip().splitlines()
        raise ValueError(f"Person masks failed: {lines[-1] if lines else 'no output'}")
    return json.loads(result.stdout.strip().splitlines()[-1])


def main(argv=None) -> int:
    import argparse
    parser = argparse.ArgumentParser(prog="python -m splatrig.personmask")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--score", type=float, default=0.5)
    parser.add_argument("--dilate", type=int, default=12)
    parser.add_argument("--rotate-cameras", default="")
    args = parser.parse_args(argv)

    import torch
    import torch.nn.functional as F
    from torchvision.io import ImageReadMode, decode_png, read_image, write_png
    from torchvision.models.detection import (MaskRCNN_ResNet50_FPN_V2_Weights,
                                              maskrcnn_resnet50_fpn_v2)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = maskrcnn_resnet50_fpn_v2(weights=MaskRCNN_ResNet50_FPN_V2_Weights.DEFAULT)
    model = model.eval().to(device)
    person = MaskRCNN_ResNet50_FPN_V2_Weights.DEFAULT.meta["categories"].index("person")
    rotate = {int(c) for c in args.rotate_cameras.split(",") if c}

    def people(image):
        with torch.no_grad():
            found = model([image.float() / 255])[0]
        keep = (found["labels"] == person) & (found["scores"] >= args.score)
        if not keep.any():
            return torch.zeros(image.shape[1:], dtype=torch.bool, device=image.device)
        return (found["masks"][keep, 0] > 0.5).any(0)

    started = time.time()
    folder = args.dataset / "masks"
    folder.mkdir(exist_ok=True)
    fractions, records = {}, []
    with (args.dataset / "image-poses.jsonl").open(encoding="utf-8") as stream:
        records = [json.loads(line) for line in stream if line.strip()]
    for record in records:
        name = Path(record["image_path"]).name
        image = read_image(str(args.dataset / "images" / name), ImageReadMode.RGB).to(device)
        found = people(image)
        if record["camera_id"] in rotate:
            for turns in (1, 2, 3):
                turned = torch.rot90(image, turns, (1, 2))
                found |= torch.rot90(people(turned), -turns, (0, 1))
        if args.dilate > 0:
            size = 2 * args.dilate + 1
            found = F.max_pool2d(found[None, None].float(), size, 1, args.dilate)[0, 0] > 0
        target = folder / name
        keep = torch.full(found.shape, 255, dtype=torch.uint8, device=device)
        if target.exists():  # an existing mask (the rig) stays ignored
            keep = decode_png(torch.frombuffer(bytearray(target.read_bytes()), dtype=torch.uint8),
                              ImageReadMode.GRAY)[0].to(device)
        keep[found] = 0
        # Masks of one camera may be hard links to one file; replace, never write through.
        target.unlink(missing_ok=True)
        write_png(keep[None].cpu(), str(target))
        fractions.setdefault(record["camera_id"], []).append(float(found.float().mean()))
    report = {"masks": len(records), "model": "torchvision maskrcnn_resnet50_fpn_v2 (COCO)",
              "device": device, "score": args.score, "dilate_px": args.dilate,
              "rotated_cameras": sorted(rotate), "seconds": round(time.time() - started, 1),
              "person_fraction_by_camera": {str(k): round(sum(v) / len(v), 4)
                                            for k, v in sorted(fractions.items())}}
    print(json.dumps(report))
    return 0


if __name__ == "__main__":
    sys.exit(main())
