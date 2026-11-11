# Development roadmap

**English** | [繁體中文](ROADMAP.zh-TW.md)

Product goal: walk indoors or outdoors with one MAX2 and one MID-360, inspect
live geometry and camera trajectory, then export images and final poses for 3DGS.
This repository is currently developed and tested locally.

## Foundation — implemented

CPU-only synthetic room, timestamped point replay, rigid-pose interpolation,
affine clock fitting, independent sensor extrinsics, consistent pose revisions,
COLMAP binary/text initialization export, local preview and automated checks.
The optional ROS recorder's message conversion is tested; its live runtime is not.
Equirectangular panoramas convert to fixed PINHOLE views with a measured ffmpeg
convention check; this is tested on synthetic panoramas, not MAX2 media.

## M1 — real sensor acquisition

- Record original MID-360 points/IMU with timestamp types and per-point offsets.
- Run GLIM alongside capture; preserve local odometry and final corrected history.
- Control MAX2 recording and collect preview frames, original chapters and metadata.
- Identify preview projection and original-media decoding on the intended OS.
- Record packet/frame loss, clock resets and interrupted writes explicitly.

Acceptance: a stationary run, a controlled rotation run and a 30-minute walking
run can be replayed from preserved originals. Report measured rates/latency and
all loss; make no throughput claim before this test.

## M2 — calibration and image association

- Estimate rig/LiDAR/camera transforms and retain calibration provenance.
- Extract matched timing events or gyro sequences from the actual media.
- Validate clock rate/offset across the full capture, including chapter boundaries.
- Convert 360 imagery to fixed PINHOLE views, retaining exposure groups and masks.
  Equirectangular conversion exists; masks and original .360/EAC decoding remain.
- Test output orientation, near-object stitching and rolling shutter.

Acceptance: known geometry projects consistently during stationary, translating
and rotating sequences at a declared image resolution. Retain independent checks;
clock-fit residuals alone do not establish exposure synchronization.

## M3 — indoor/outdoor training dataset

- Recompute points and image poses from the final trajectory revision.
- Add frame selection using measured image sharpness and coverage.
- Project sparse camera-Z depths with visibility/validity masks.
- Test real datasets with a selected desktop trainer; record its version/settings.
- Compare held-out photographs and independent geometric measurements.

Acceptance: one room, corridor, exterior and mixed indoor/outdoor dataset produce
reproducible exports and actual trained models. Publish measured results only
after agreeing on what capture data can be released.

## M4 — capture experience and broader release

- Live point cloud, camera route, preview-frame poses and coverage feedback.
- Bounded map storage, status/recovery and reproducible installation.
- AnyReplica external-scan adapter for review/training, with explicit axes/units.
- Demonstration footage from physical hardware, calibration records and a sample.

Release readiness requires the hardware demo and a fresh-machine quick start.
Synthetic previews remain useful tests and stay labeled.

## Next task

Implement and test the MID-360 raw recording adapter against a device or a real
recorded fixture. Keep SLAM in an external process and adapt its output at the
[documented data boundary](FORMAT.md).
