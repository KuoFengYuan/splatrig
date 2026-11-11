# Hardware integration

**English** | [繁體中文](HARDWARE.zh-TW.md)

Target: one GoPro MAX2 and one Livox MID-360, handheld, indoors and outdoors.
Physical-device capture has not been validated in this repository yet.

## Responsibilities

| Part | Responsibility | Current status |
| --- | --- | --- |
| MID-360 + built-in IMU | Timed points and inertial measurements | External driver |
| Livox Driver 2 | Sensor transport; preserve per-point time | Mid-360S running with PTP; bag importer implemented |
| GLIM | Live cloud, rig trajectory and global optimization | External dependency |
| ROS odometry recorder | Save configured Odometry topic | Implemented; message conversion tested, live ROS untested |
| Panorama view converter | Equirectangular → fixed PINHOLE views via ffmpeg | Implemented; synthetic panoramas only, no MAX2 media yet |
| MAX2 adapter | Preview/control, original media and time metadata | Preview and recording control tested over USB; adapter planned |
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

## Local MID-360S bring-up record

One Mid-360S was brought up on Ubuntu 24.04 with ROS 2 Jazzy, Livox SDK2 and
livox_ros_driver2 1.2.8 (device firmware 35.01.01.08). This established data flow
and PTP time synchronization only; no capture or calibration has been accepted yet.

- **Identify the model.** The discovery reply reported dev_type 35 (Mid-360S), not
  9 (MID-360). Use the driver's `MID360s_config.json` and `msg_MID360s_launch.py`;
  with the MID-360 configuration the SDK answers discovery but never opens data
  ports and prints no error.
- **Network.** Give the LiDAR NIC a static host address on the device subnet
  (factory default host 192.168.1.50; device 192.168.1.1XX from the last two serial
  digits) with no default route. Put that address in every `host_ip` field and the
  device address in `lidar_configs`, then rebuild so the installed copy updates.
- **Build environment.** Deactivate conda before building or running ROS so CMake
  and ros2 use the system Python. With CMake 4.x, configure Livox SDK2 with
  `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`, then `sudo make install && sudo ldconfig`.
- **Time synchronization.** Without sync the packet `time_type` is 0, the device
  clock counts from power-on, and the driver replaces device stamps with host
  packet-arrival time. linuxptp 4.0 sends PTP v2.1 by default, which the device
  ignores (HMS warning 0x0403). Run exactly one master:

  ```sh
  sudo ptp4l -i <lidar-nic> -S -m --ptp_minor_version 0
  ```

  Do not also run a gPTP master (HMS 0x040A). The device's internal state
  (`time_sync_type`, `last_sync_time`) and the packet `time_type` confirm sync.
  This NIC offered software timestamps only.
- **Observed while stationary.** Points 10.000 Hz with about 20,000 points per
  message and per-point offsets spanning 0–100 ms; IMU 200 Hz with acceleration
  in g (magnitude about 0.99); about half of returned points were (0, 0, 0)
  no-return entries. After sync, IMU stamps alternated near 4.26 and 5.78 ms
  rather than a uniform 5 ms; this pattern still needs a longer recording.
- **First recording.** A 59.3 s stationary bag (230 MiB) had no frame or IMU gaps:
  592 frames, 199,996 points/s, IMU 199.999 Hz. IMU receive latency was
  0.72/0.99/1.25 ms (p1/p50/p99) with a −0.86 ppm trend in 5 s minima, consistent
  with sustained PTP lock. 52.1% of points were no-return entries. Import it with:

  ```sh
  conda deactivate
  source /opt/ros/jazzy/setup.bash && source ~/ws_livox/install/setup.bash
  tools/splatrig-ros import-livox <bag> --output <recording>
  ```

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

### Local MAX2 preview check

One MAX2 (firmware H24.02.01.22.00, Open GoPro 2.0) was tested over USB. It appears
as a CDC-NCM network interface; give it a DHCP connection with no default route
(`ipv4.never-default yes`). The camera serves HTTP on port 8080 at the subnet's
`.51` address. Test sequence: enable wired control, load the video preset, start
the preview stream, record about 10 s, then stop, reload the photo preset and
release control. The preset reload returned HTTP 500 while the camera was busy and
succeeded after it became idle.

- Preview: MPEG-TS over UDP port 8554, HEVC Main 1408×704 at 29.97 fps, plus audio
  streams. The picture is **two unstitched fisheye circles side by side**, not
  equirectangular. Decoding joined mid-stream reports missing references until
  the next keyframe.
- Preview during recording: frames continued at 30 fps, with gaps of about 1.9 s
  when recording started and 1.2 s when it stopped.
- Preview PTS stayed continuous across those gaps, so preview timestamps are not
  wall-clock time and must not be used for association.
- Recordings were saved as `.360` files. Their decoding, timing and stitching
  remain untested.

## Indoor/outdoor acceptance captures

Use a small room, a long corridor, a building exterior and a continuous
indoor-to-outdoor walk. Include revisits, turns, slow/fast motion, blank walls,
glass and moving objects. Record dropped packets/frames, clock residuals,
latency, trajectory gaps, loop correction and resource use.

Check image reprojection and held-out rendering quality separately from LiDAR
surface agreement. Model appearance does not establish metric accuracy.
AnyReplica's pose experiments are useful precedents for this distinction.

## Live capture console

Commands that need ROS 2 run through `tools/splatrig-ros`, which starts a clean
environment (no conda), sources ROS 2 and `~/ws_livox` (override with `LIVOX_WS`)
and appends the repository to the ROS `PYTHONPATH`. Setting `PYTHONPATH=…/src`
directly after sourcing ROS replaces the ROS entries and hides `rclpy`.

`splatrig live` serves a local page at http://127.0.0.1:8090/ (localhost only). It
shows the last second of LiDAR points in the sensor frame, the MAX2 preview, rate
and clock status, and a Record button that starts a `ros2 bag record` of the raw
Livox topics plus a MAX2 recording into a new folder under `~/captures`. Run it
with the system Python after sourcing ROS 2 and the driver workspace, with ptp4l
and the driver already running:

```sh
tools/splatrig-ros live            # --gopro none for LiDAR only
```

- The page is a monitoring aid. Preview frames are labeled not exposure-timed, and
  `capture-log.json` records host command times only; LiDAR and camera starts are
  approximate, not synchronized.
- POST requests require an `X-SplatRig: 1` header. The server never binds to other
  interfaces; phone access needs a separate, deliberate setup.
- With the MAX2 preview running, ~380 KB Livox frames overflowed the 208 KiB default
  UDP receive buffer and best-effort DDS dropped whole frames (1–3 Hz). `live`,
  `ros-pointcloud` and the bag recorder they start use the packaged Fast DDS profile
  (shared memory, 4 MiB UDP buffers) unless a profile is already configured; a
  separately started `ros2 bag record` should set `FASTRTPS_DEFAULT_PROFILES_FILE`
  to `src/splatrig/assets/fastdds.xml`.
- `--slam` also starts `glim_rosnode` (CPU modules, ROS publishing enabled) in a
  temporary folder, feeds it the Livox frames as PointCloud2 with per-point time on
  `/splatrig/glim_points`, and adds a **Map** view: GLIM's corrected scans in a 5 cm
  first-point voxel map, the route (2 cm spacing) and the current IMU pose. With
  `--calibration calibration.json --camera-offset X Y Z` the MAX2 forward axis is
  drawn too. `?view=map` opens the Map view; "Hide ceiling" drops points more than
  1 m above the current pose. The live map is not moved by later loop closures and
  is a preview; datasets still come from the offline run of the recorded bag.
  Replaying the 27 s rotation bag in an isolated domain gave 10 Hz odometry and a
  23k-voxel map with no dropped voxels.
- `splatrig ros-pointcloud` republishes CustomMsg as PointCloud2 for RViz2/Foxglove
  (no per-point time).

## GLIM bridge

GLIM reads PointCloud2 with a per-point time field and not Livox CustomMsg.
`splatrig glim-bag <bag> --output <dir>` writes `<dir>/bag` with `/livox/points`
(x, y, z, intensity, `t` seconds after the header stamp; no-return points excluded)
and the unchanged IMU messages; the original bag is not modified. Set GLIM's
`points_topic`/`imu_topic` accordingly and `acc_scale` to 9.80665. The MID-360
IMU offset in `conversion.json` comes from the published MID-360 layout and is not
verified for the Mid-360S. `splatrig import-trajectory traj_imu.txt --revision N
--status final --frame imu --output <dir>` converts a GLIM TUM trajectory into
`trajectory.jsonl` with exact integer nanoseconds; use `final` only for the
loop-closed `traj_*` files, never for `odom_*`.

Tested with the CPU packages from the GLIM PPA (`ros-jazzy-glim-ros` 1.2.2); its
GPU packages target CUDA 12.6/13.1 and were not tried on this RTX 50-series GPU.

```sh
splatrig glim-config --output runs/glim-config        # CPU modules, Livox topics, acc_scale
splatrig glim-bag <bag> --output runs/glim-input
ROS_DOMAIN_ID=73 ros2 run glim_ros glim_rosbag runs/glim-input/bag --ros-args \
  -p config_path:=$PWD/runs/glim-config -p dump_path:=$PWD/runs/glim-dump -p auto_quit:=true
splatrig import-trajectory runs/glim-dump/traj_imu.txt --revision 1 --status final \
  --frame imu --output runs/trajectory
```

Use an otherwise unused `ROS_DOMAIN_ID`: `glim_rosbag` also subscribes to live
topics, and a running driver injected current IMU messages into an offline run.
On the 59 s stationary bag, GLIM processed the data in 4.4 s and the IMU pose
stayed within 7.9 mm and 0.31° of its first pose. Stationary drift does not
establish accuracy while moving.

## Version-1 processing

`tools/splatrig-ros process-capture <capture> --output <run> --camera-offset X Y Z
--camera-ypr YAW PITCH ROLL` turns a folder recorded by `live` into a COLMAP dataset:
LiDAR import, GLIM bag and configuration, an offline GLIM run on ROS domain 73,
trajectory import, download of the new MAX2 `.360` file, panorama extraction, session
assembly, cube views and export. Work happens in a hidden directory that is renamed
only after success (`.<run>-failed` is kept for inspection otherwise).

The result is provisional by construction: the MAX2 mount is hand-entered (camera
center in LiDAR meters; yaw toward LiDAR +Y, pitch up, roll about the camera's
forward axis, from a front looking along LiDAR +X with image-up along +Z), so the
rig stays `uncalibrated`; the camera clock is the record-command acknowledgement
plus media time, so it stays `provisional`. Points and panoramas outside the GLIM
trajectory are excluded and counted in `assembly.json`.

`splatrig max2-frames <file.360>` rebuilds the equi-angular cubemap from the two
5888×1920 MAX2 tracks (LEFT/FRONT/RIGHT and DOWN/BACK/TOP; the 64 px lens overlap
at each split face midline is cut) and converts every Nth original frame to an
equirectangular image. Timestamps come from packet PTS as integer media-time ns.
The hard cut leaves visible stitching parallax for near objects.

First run (stationary desk, 21 s): no LiDAR gaps, 179 GLIM poses, 39 panoramas of
which 33 fell inside the trajectory, 198 cube views and a provisional COLMAP model,
in 72 s. A yaw search correlating LiDAR reflectivity with panorama brightness was
ambiguous (peaks 0.215 at −25° and 0.208 at −135°) and is not used.

### Planned handheld rig (v3, one-piece print)

Measured from the rig STL (mm, Z up, LiDAR mounting surface at z = 0): the four M3
holes form a 48 × 36 pattern centered at (0, 0); the two locating pins sit at
(0, −16) and (0, 23); the MAX2 cradle's 1/4" hole is at (0, 0) with its seat at
z = 95, between lips 27.5 apart across X, so the MAX2 lenses face ±X. The cable
holder and column are at −X. The camera mount is therefore directly above the
LiDAR mount center. Assuming the Mid-360 point origin ~47 mm above its base and the
MAX2 optical center ~35 mm above its base (both to be confirmed from the device
drawings), the nominal `--camera-offset` is `0 0 0.083` in meters, independent of
which lens faces forward. Rotation and per-capture clock still come from
`calibrate-gyro` after mounting. The column lies inside the LiDAR field of view
near the sensor and the operator stays in every panorama, so self-points and
camera masks need handling before training.

## Gyro calibration of the MAX2 mount

With the MAX2 and LiDAR rigidly fixed together, record ~30 s that starts and ends
still and rotates the rig about all three axes, then run
`tools/splatrig-ros calibrate-gyro <capture> --output <dir>`. The MAX2 GPMF gyro
(GYRO, ~790 Hz, ORIN `ZXY`) and the Mid-360S IMU (200 Hz) measure one rotation:
the clock offset maximizes the correlation of |ω| and is refined with the rotated
vector residual; the rotation is the orthogonal polar factor of Σ l cᵀ, with
reflections rejected and the weakest singular value reported. Each gyro is
averaged over a ±4 ms window; plain linear interpolation biased the offset by up
to 1.6 ms toward sample phases with less interpolated noise.

The image axes of the MAX2 relative to its GPMF axes were established once by
projecting LiDAR depth edges into a still panorama for the four horizontal
candidates: forward = −Y, down = −Z, right = −X (stored order Z, X, Y). IORI was
identity throughout, so the stored image orientation is fixed to the camera.

`process-capture --calibration calibration.json` uses that rotation and re-runs
the |ω| correlation on every capture for its own clock offset, falling back to
the record-command time (and saying so) when the correlation is below 0.9.
Translation is still hand-entered (`--camera-offset`), so output stays provisional.

Measured on the first mount: |ω| correlation 0.9996, residual 2.2% of the signal
RMS, camera media time 95.8 ms after the record-command acknowledgement; the
weakest axis was barely excited (singular value ratio 0.013). On synthetic data
the method recovers the offset within 0.2 ms and the rotation within 0.04°.
Reprojected LiDAR depth edges stayed on static structures across the rotation by
visual inspection; this is not a quantified accuracy.

## Primary references

- [Livox MID-360 specifications](https://www.livoxtech.com/mid-360)
- [Livox Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Livox time synchronization](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)
- [GLIM sensor setup](https://github.com/koide3/glim/wiki/Sensor-setup-guide)
- [Open GoPro](https://gopro.github.io/OpenGoPro/) and [FAQ](https://gopro.github.io/OpenGoPro/docs/faq/)
- [Labs release notes](https://gopro.github.io/labs/control/notes/)
- [LiDAR/camera calibration toolbox](https://github.com/koide3/direct_visual_lidar_calibration)
- [AnyReplica photo/pose validation](https://github.com/KuoFengYuan/anyreplica/blob/main/docs/POSE_REFINEMENT.md)
