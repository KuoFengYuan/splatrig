---
name: splatrig-development
description: Develop or validate SplatRig capture-data processing, sensor adapters and COLMAP exports. Use for changes in the SplatRig repository, not unrelated SLAM or Gaussian splatting projects.
---

# SplatRig development

Read the repository's [working agreement](../../../AGENTS.md) first and preserve
the user's current delivery scope. Local testing does not authorize remote publication.

The offline pipeline is deliberately independent of ROS and SLAM libraries. Keep
optional device imports inside adapter entry points. The synthetic room is a
geometry fixture, not measured reconstruction or evidence of hardware performance.

For time, pose, depth or export work, read [the format](../../../docs/FORMAT.md).
Keep integer clock anchors, wxyz quaternion order, OpenCV camera axes and +Z world
up consistent. Both image poses and replayed points must come from one trajectory
revision, with the correct sensor extrinsics. Final export rejects provisional
clock/rig calibration and never extrapolates beyond recorded poses.

For sensor work, read [hardware integration](../../../docs/HARDWARE.md).
Check timestamp semantics, frame IDs and acceleration units at the adapter boundary.
Do not infer exposure time from packet arrival, or infer final global poses from
live odometry alone. Treat MAX2 preview, high-resolution files and metadata as
different data sources.

Run the checks affected by the change:

- Processing/math: python -m unittest discover -s tests -v
- CLI/export/package: python tools/smoke.py
- Documentation/links: python tools/check_project.py
- Viewer: open the generated preview in a browser; check orbit, zoom, camera
  visibility, reset, keyboard input and a narrow viewport.

Check actual reprojections or independent analytic coordinates for geometry changes.
Check a COLMAP export with COLMAP itself when available. Keep captures unchanged,
write new results separately, and report synthetic versus physical-device evidence.
Use [the roadmap](../../../docs/ROADMAP.md) to distinguish implemented features
from planned integrations; update both documentation languages when behavior changes.
