# Hardware integration

**English** | [繁體中文](HARDWARE.zh-TW.md)

Target: one GoPro MAX2 and one Livox MID-360, handheld, indoors and outdoors.
Physical-device capture has not been validated in this repository yet.

## Responsibilities

| Part | Responsibility | Current status |
| --- | --- | --- |
| MID-360 + built-in IMU | Timed points and inertial measurements | External driver |
| Livox Driver 2 | Sensor transport; preserve per-point time | Integration pending |
| GLIM | Live cloud, rig trajectory and global optimization | External dependency |
| ROS odometry recorder | Save configured Odometry topic | Implemented; message conversion tested, live ROS untested |
| Panorama view converter | Equirectangular → fixed PINHOLE views via ffmpeg | Implemented; synthetic panoramas only, no MAX2 media yet |
| MAX2 adapter | Preview/control, original media and time metadata | Planned |
| SplatRig core | Calibration model, pose association, replay, COLMAP | Implemented with synthetic checks |
| Desktop trainer | Train a Gaussian model | External; no capture-quality benchmark yet |

## Rig and compute

Mount both sensors on one rigid bracket with clear sight lines and repeatable
mounting positions. Determine T_rig_lidar and T_rig_camera from calibration;
the tracked frame may be the IMU rather than the LiDAR optical origin.
Recheck calibration after remounting.

Start integration on an Ubuntu laptop/workstation. Measure processing, recording,
preview and power load together before selecting a compact computer or Jetson.
Use the power requirements in each device's manual.

The MID-360 has 360° horizontal and 59° vertical coverage. MAX2's visible sphere is
larger than the LiDAR depth coverage. Missing depth must stay invalid; plan height
and tilt changes for coverage. Initially visualize intensity/height point clouds;
precise RGB coloring depends on camera timing and visibility.

## MID-360 and GLIM

Use official Livox SDK2 / Driver 2 and the GLIM sensor setup guide. Lock working
versions only after an actual device run; no device/firmware combination is
currently certified here. Preserve per-point acquisition timestamps, point units,
IMU units and device diagnostics.

Check whether acceleration is reported in g or m/s². GLIM's sensor guide notes
the Livox scaling setting; do not apply it twice if an upstream adapter already
converted the data. Check IMU/LiDAR extrinsics and timestamp origins before SLAM.
PTP can synchronize MID-360 with the host; it does not synchronize MAX2 exposures.

The included recorder accepts nav_msgs/Odometry:

```sh
splatrig record-poses --output runs/live-poses.jsonl \
  --topic /your/odometry --world-frame odom --rig-frame imu
```

Source the ROS 2 environment and install SplatRig into an environment where its
Python packages are available. Read the actual topic and frame IDs from your
SLAM configuration; the names above are examples. The recorder requires an exact
frame match, uses message acquisition stamps, flushes each accepted pose, and
stops on repeated/backwards time. It does not perform a hidden TF conversion.

This log is live odometry, not final loop-corrected history. A complete adapter
still needs original timed point logging and extraction of the final corrected
trajectory. Relabeling the log as final is insufficient.

## MAX2 gate before capture integration

Characterize these together on the intended firmware and host:

1. Recording and preview can run simultaneously in the selected 360 mode.
2. Preview dimensions, projection, buffering, frame drops and PTS are understood.
3. Original .360 chapters and GPMF can be retained and decoded reproducibly.
4. Preview and original media can be associated with an explicit clock model.
5. Output orientation remains calibrated while the physical rig rotates.

Open GoPro supports MAX2. Its FAQ says recorded metadata becomes available after
the file is saved, and command execution is not deterministic synchronization.
Use live rig poses for the online camera trajectory, then refine high-resolution
frame association from the saved media. Online preview poses remain provisional.

Evaluate Labs NROT and stitch-depth controls in a controlled rotation sequence.
Removing rotation stabilization alone does not prove a perfectly central camera,
remove rolling shutter or eliminate stitching parallax. Test near objects and
seams before trusting projected depths.

## Indoor/outdoor acceptance captures

Use a small room, a long corridor, a building exterior and a continuous
indoor-to-outdoor walk. Include revisits, turns, slow/fast motion, blank walls,
glass and moving objects. Record dropped packets/frames, clock residuals,
latency, trajectory gaps, loop correction and resource use.

Check image reprojection and held-out rendering quality separately from LiDAR
surface agreement. Model appearance does not establish metric accuracy.
AnyReplica's pose experiments are useful precedents for this distinction.

## Primary references

- [Livox MID-360 specifications](https://www.livoxtech.com/mid-360)
- [Livox Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Livox time synchronization](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)
- [GLIM sensor setup](https://github.com/koide3/glim/wiki/Sensor-setup-guide)
- [Open GoPro](https://gopro.github.io/OpenGoPro/) and [FAQ](https://gopro.github.io/OpenGoPro/docs/faq/)
- [Labs release notes](https://gopro.github.io/labs/control/notes/)
- [LiDAR/camera calibration toolbox](https://github.com/koide3/direct_visual_lidar_calibration)
- [AnyReplica photo/pose validation](https://github.com/KuoFengYuan/anyreplica/blob/main/docs/POSE_REFINEMENT.md)
