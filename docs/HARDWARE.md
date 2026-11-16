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
and tilt changes for coverage. The live view colors points by height or
intensity; datasets take point colors from the panoramas (see dataset quality
steps), which depends on camera timing and has no occlusion test.

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

### One-command start

`tools/splatrig-ros start` runs the start-up checks, starts `livox_ros_driver2`
(`msg_MID360s_launch.py`, override with `--driver-launch`) unless a driver node is
already running, then the console with live GLIM (`--no-slam` to skip), and opens
the Map view in the default browser (`--no-browser`). Ctrl+C stops the console,
GLIM and a driver it started; a driver that was already running is left alone.
`tools/splatrig-ros doctor` prints the same checks and exits non-zero on a failure:

- ROS 2 environment, the installed driver configuration and GLIM;
- the NIC that holds the configured `host_ip`, and whether the LiDAR answers ping;
- exactly one ptp4l on that NIC with `--ptp_minor_version 0`;
- a running driver node (reused), the MAX2 USB link and a free console port.

Addresses and the NIC name are read from the installed `MID360s_config.json` and
the host interfaces, so nothing machine-specific is stored in the repository.
`sudo tools/install-ptp-service <nic>` installs `splatrig-ptp@<nic>.service`
(`ptp4l -i <nic> -S -m --ptp_minor_version 0`), bound to that interface so it
starts at boot or when the USB adapter is plugged in; stop a manually started
ptp4l first, since exactly one may run (`--remove` uninstalls).
`tools/splatrig-app --install` adds a "SplatRig" menu entry that runs `start` in a
terminal and keeps the window open afterwards so messages stay readable.
`tools/splatrig-ros` passes the desktop session variables through so a browser
can open. Tested on this machine: doctor, and `start` in an isolated ROS domain
with a replayed bag (10 Hz odometry, one GLIM, stopped in 1 s); the driver-start
path, the PTP service and the menu entry are covered by unit tests or file
validation only, not yet by a cold start.

### Console details

`splatrig live` serves a local page at http://127.0.0.1:8090/ (localhost only),
laid out like a phone scanning app.
- **Full-screen view:** the SLAM map, or the last second of LiDAR points in the
  sensor frame without SLAM.
- **Top centre:** the timer and one guidance message.
- **Right:** a tool rail.
- **Bottom:** a heading-up mini-map and the round record button.

The button starts a `ros2 bag record` of the raw Livox topics plus a MAX2 recording
in a new folder under `~/captures`. Run it
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
- **Guidance during a scan** (`scanguide.py`). The console replays the dataset's
  photo rule on GLIM's odometry: a photo is due every 0.15 m or 10°, taken when
  turning no faster than 30°/s. A photo due for more than 0.5 s counts as a
  stretch too fast for photos.
  - Coverage: at each photo, the newest scan marks 25 cm cells as seen from the
    photo's horizontal direction (10° bins). A cell with views spanning 30° or more
    is covered.
    - The record button's ring shows the covered share: red below 30%, orange
      below 60%, green from 60%.
    - **Coverage** colours the cells the same way, in the map and the mini-map.
  - Revisits: after 5 m of new ground and 40 s, the console asks for a return to a
    scanned area so SLAM can correct drift. Coming within 0.8 m of route older than
    20 s confirms it.
  - One message is shown at a time, most important first:
    1. LiDAR or SLAM failures
    2. too fast for photos
    3. no PTP
    4. MAX2 problems
    5. back in a scanned area
    6. return prompt
    7. nearly too fast
    8. first-photo hint
  - A blocking problem while recording draws a red frame around the screen.
  - A motion meter appears above the button at 60% of the turn or speed limit.
  - The guide's numbers predict the dataset; the dataset itself is decided offline.
- **Tool rail:**
  - Follow: behind the rig, heading up.
  - Top view.
  - Coverage.
  - Ceiling: hide points more than 1 m above the rig.
  - Live scan.
  - Camera: MAX2 preview, picture in picture.
  - Captures.

  **Devices** (top left) lists LiDAR, IMU, clock, MAX2, SLAM and calibration, and
  its dot shows the worst state.
- **After Stop**, a review sheet shows:
  - duration, distance, photos and coverage;
  - notes on one-sided surfaces, stretches too fast for photos, long walks without
    returning, and a missing MAX2 video;
  - one primary action: **Make 3DGS dataset** when a calibration exists, otherwise
    **Use as calibration**.

  **Captures** opens the same sheet for an earlier recording.
- The two buttons run `calibrate-gyro` (**Use as calibration**) and `process-capture`
  (**Make 3DGS dataset**), the same functions as the commands. The sheet then shows
  each step with a progress bar, and the result or the error. Jobs started while
  another runs wait in line; a waiting job can be removed.
- **Scans** (`library.py`) lists the recordings like a project library.
  - **Cards:** cover, name, date, duration, size and status (Not processed,
    Calibration, Queued, Processing n/9, Dataset ready, Failed), plus tags such as
    LiDAR only or current calibration.
  - **Cover:** the live map snapshot taken at Stop, or else the dataset's first photo.
  - **Names:** automatic names ("Monday afternoon scan") are numbered when they
    repeat; a scan can be renamed (40 characters).
  - **Search, sort and filter:** search by name; sort by newest, oldest, name or
    size; filter by with or without a dataset.
  - **Select mode** deletes several scans at once.
  - **Storage line:** recordings, results and free space.
  - **Where data lives:** captures are not modified. Names, the guide's summary at
    Stop and covers live in `<output-root>/library/`.
- **Deleting** moves the capture and its results into `.deleted` folders with a
  dated marker.
  - Recently Deleted shows the days left and can recover a scan or delete it for
    good. Deleting for good asks for DELETE to be typed, and the server refuses an
    unnamed purge without `all: true`.
  - Expired scans are removed when the console starts (30 days).
  - Scans that are recording, processing, waiting, or the current calibration
    cannot be deleted.
- **Preview dataset** draws the dataset's coloured points and a mark for each image
  (where it was taken and which way it looked), with a strip of thumbnails whose
  masked pixels are tinted red. Choosing a thumbnail puts the eye at that camera
  with its field of view and lays the photo over the points, with an opacity
  slider, to check alignment by eye.
  - Behavior:
    - One job runs at a time, in the background.
    - The panel shows the current step and then the result or the error.
    - Jobs are refused while recording, and Record is refused while a job runs.
  - Output:
    - Results go to `--output-root` (default `~/splatrig-runs`).
    - The calibration goes in `calibration-<capture>`, the dataset in `<capture>`.
    - A repeated run gets `-2`, `-3`; nothing is overwritten.
  - Saved settings:
    - The newest calibration and camera offset are kept in
      `<output-root>/console.json`.
    - They draw the camera pose in the Map view right away.
    - A restarted console uses them, unless `--calibration` or `--camera-offset` is
      given. A value given that way becomes the saved one.
  - Stopping the console mid-job publishes nothing and names the leftover
    `.<name>-work` folder.
  - Checked through the API and the page with the 35 s rotation capture of
    2026-11-16:
    - Calibration took 13 s, with gyro correlation 0.998.
    - The dataset took 77 s: 14 frames, 70 images, 270 frames skipped as too fast,
      and no masks, because the rig travelled 0.82 m.
- With the MAX2 preview running, ~380 KB Livox frames overflowed the 208 KiB default
  UDP receive buffer and best-effort DDS dropped whole frames (1–3 Hz). `live`,
  `ros-pointcloud` and the bag recorder they start use the packaged Fast DDS profile
  (shared memory, 4 MiB UDP buffers) unless a profile is already configured; a
  separately started `ros2 bag record` should set `FASTRTPS_DEFAULT_PROFILES_FILE`
  to `src/splatrig/assets/fastdds.xml`.
- `--slam` also starts `glim_rosnode` (CPU modules, ROS publishing enabled) in a
  temporary folder, feeds it the Livox frames as PointCloud2 with per-point time on
  `/splatrig/glim_points`, and adds a **Map** view: GLIM's corrected scans in a 5 cm
  voxel map, the route (2 cm spacing) and the current IMU pose. A voxel joins the
  map only when scans at least 0.5 s apart both return from it, and unconfirmed
  voxels expire after 10 s, so people walking past or a swinging arm leave no
  trail; returns within 0.6 m of the sensor (the operator, the cable, a desk right
  under the rig) stay out of the map but remain in the cyan newest scan. Without
  PTP the LiDAR and IMU stamps are host arrival times, GLIM drifts while moving and
  walls double; the SLAM chip says so. With
  `--calibration calibration.json --camera-offset X Y Z` the MAX2 forward axis is
  drawn too. `?view=map` opens the Map view; "Hide ceiling" drops points more than
  1 m above the current pose. The live map is not moved by later loop closures and
  is a preview; datasets still come from the offline run of the recorded bag.
  Replaying the 27 s rotation bag in an isolated domain gave 10 Hz odometry and a
  17k-voxel map (23k before confirmation was required) with no dropped voxels.
- Every scan starts a new SLAM session. Pressing the record button clears the live
  map, route and pose and restarts GLIM, so drift from earlier scans or from time
  between scans does not carry over. GLIM needs a few seconds to start, which is
  why scans begin by holding still. The map stays visible after Stop for review.
  Datasets were never affected: each is rebuilt by its own offline GLIM run.
  Checked by replaying a recorded bag into a console and recording twice: the map
  emptied at each start and was tracked again within 6 s, with one GLIM throughout.
- Checked with Playwright and Chrome at 1280×800 and 390×844 on copies of real
  datasets: library, search, detail, preview, a photo laid over its points (walls,
  monitors and the desk line up), and leaving the preview, with no page errors.
- The pose is polled every 0.1 s and GLIM's newest scan ("Live scan", cyan) every
  0.2 s. The map and route refresh every second, the coverage cells every 2 s.
- Checked by replaying the 35 s capture of 2026-11-16 into a console in an isolated
  ROS domain, driving Chrome with Playwright at 1280×800 and 390×844:
  - record, coverage and top view, stop, review sheet, captures list;
  - a real **Use as calibration** job: offset −7.3, 2.0, 7.4 cm;
  - no console errors.
- GLIM runs in its own session, so a console that dies without its cleanup leaves
  it running, and a second GLIM then writes another world frame into the same map
  (doubled, thick walls and a tangled route). The console stops GLIM on Ctrl+C,
  SIGTERM and SIGHUP, each shutdown step running even if an earlier one fails;
  every live folder records its owner PID, and a new console stops GLIM left by a
  console that no longer exists. More than one odometry publisher turns the SLAM
  chip red. Verified by killing a console with SIGKILL and starting another one.
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
trajectory import, download of the new MAX2 `.360` file, frame choice, panorama
extraction, session assembly, views, masks, point colours and export. Work happens in a hidden
directory that is renamed only after success (`.<run>-failed` is kept for inspection
otherwise). Each step's name is printed to stderr as it starts. A MAX2 download is
refused while the camera records, and a transfer shorter than the camera's
`Content-Length` is deleted and reported. The camera ends a transfer early when it
starts recording or sleeps, and the truncated `.360` used to fail later as
"Invalid data" in ffprobe.

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

### Dataset quality steps

- **Frame choice.** Frames are chosen along the GLIM trajectory, not every Nth frame.
  A frame is due once the rig moved 0.15 m (`--spacing`) or turned 10°
  (`--spacing-deg`) since the last one. Of the frames in the next 0.5 s, the one
  turning slowest is kept. A frame turning faster than 30°/s (`--max-turn`) is never
  kept, because it would be blurred. Standing still therefore gives one frame, not
  dozens. The turn rate comes from the trajectory: blur from rotation is predicted,
  but exposure time, defocus and rolling shutter are not measured. `--every N`
  restores fixed spacing. The spacing was halved from 0.3 m / 20° after a 31 s desk
  capture gave only 11 frames (55 images), too few for training. The frame filter now runs before the cubemap conversion,
  so skipped frames are only decoded; the output images are byte-identical to the
  previous order.
- **Near points.** LiDAR points closer than 0.6 m to the sensor (`--min-range`) are
  left out of the session and `points.ply` and counted in `assembly.json`
  (`points_near_sensor`). These are the operator, the mount and the cable.
- **Lens views at full resolution.** The MAX2's two lenses look along sphere +Z and
  −Z and are stitched along the great circle z = 0. That seam runs through the
  middle of the cube's right, left, up and down views, where brightness and
  near-object parallax jump between lenses.
  - The default layout `lens` cuts four 80° views inside each lens: ahead, 40° to
    either side and 40° up. Each stays at least 8.4° from the seam; `process.json`
    records every view's `seam_margin_deg`, which is negative for a view that
    crosses it.
  - Each view is a separate COLMAP camera, so a trainer can correct exposure per
    lens (LichtFeld `--ppisp`).
  - Panoramas are decoded at 7680 px (`--width`), the MAX2's full ~21 px/degree;
    the previous 3840 px halved it. Views are 1600 px (`--size`).
  - `--layout cube-no-down` (five 90° cube views) and `cube` remain available.
  - On the 2026-11-16 desk scan, 26 frames gave 208 images instead of 135. The run
    took 155 s and 4.5 GB of memory, and the dataset is 634 MB.
  - LichtFeld, 30k iterations, 1M Gaussians, `--mask-mode ignore`, every 8th image
    held out. Held-out front views scored PSNR 23.68 / SSIM 0.877 / LPIPS 0.293
    against 17.87 / 0.628 / 0.438 for the previous cube layout (26 against 4
    images).
  - With 8 views per frame, `--test-every 8` holds out only front views;
    `--test-every 7` rotates through all of them.
- **Masks.** `dataset/masks/<image name>` marks pixels to ignore with 0 and usable
  pixels with 255. This is the COLMAP, nerfstudio and LichtFeld convention; train
  LichtFeld with `--mask-mode ignore`. The original 3DGS trainer ignores masks.
  - **Rig.** The LiDAR, its mount and the handle are masked from geometry, placed by
    the calibrated camera offset. The model has three parts:
    - an 8 cm-radius cylinder from 1.5 cm above the LiDAR origin, 8 cm tall
      (`--rig-radius`);
    - continued by a 3 cm handle, 40 cm long.

    On the 2026-11-16 mount this covers the LiDAR dome and its plate in the
    panoramas, and 22% of the front view.
  - **People.** People are found in each image by torchvision's Mask R-CNN v2 (COCO,
    BSD licence; weights downloaded on first use). This includes the operator's head,
    body, arms and hands, and people walking past.
    - Views looking up or down are also run at three right-angle turns.
    - Person pixels are widened by 12 px.
    - The helper runs in a separate Python with torch and torchvision, set by
      `SPLATRIG_TORCH_PYTHON` (or `--person-python`). Put the variable in
      `~/.config/splatrig/env`; `tools/splatrig-ros` reads it.
    - Without it, `process.json` (`masks.people`) says it was skipped. If the helper
      fails, the error is recorded and the LiDAR estimate below is used instead.
    - On the 27 s walking capture of 2026-11-16, 85 images took 7 s on an RTX 5070.
      People covered 25% of the right view, 55% of the back view and 18% of the up
      view. The operator's head, body, arms and hands were covered, and a jacket on a
      chair was also taken for a person.
  - **LiDAR operator estimate** (used only without person masks). How it works:
  - While the rig moves, a LiDAR direction (2° bins) whose returns are mostly closer
    than 0.8 m is taken to be part of the rig or the operator.
  - Those returns are carried into the camera's sphere frame and widened by 4°.
  - Directions outside the LiDAR field of view, such as the hands below the sensor,
    are not covered.
  - Nothing is estimated unless the rig travelled at least 1 m. When the rig only
    turns a little in place, nearby scene keeps its direction too.
  - A mask covering more than 25% of the sphere is rejected as implausible. A narrow
    corridor walked straight can still be mistaken for the rig.
  - `process.json` (`self_mask`) states why no masks were written.
  - `--no-masks` turns masks off.
- **Point colours.** The map is built with 3 cm voxels, falling back to 5 cm if
  2 million voxels are reached; the fallback is reported. Each voxel remembers the
  time of its closest LiDAR return. Its colour is taken from the panorama nearest
  in time; if the point's direction is masked, or it is within 0.3 m of the camera,
  the next of up to four panoramas within 5 s is tried. Colours are sampled at
  2048 px width. `points.ply` and `points3D` carry the colours.
  - There is no occlusion test beyond choosing the panorama nearest in time.
  - Exposure differences between panoramas are not equalized.
  - `--no-colors` keeps the old gray.
  - On the 2026-11-16 rotation capture, 159,768 of 160,468 points were coloured
    (median 0.63 s from their panorama). The whole run took 70 s. Rendering the
    coloured cloud from two keyframe poses matched the photographs: lamps,
    monitors and partitions in place.

Re-run on the 26 s gyro-rotation capture with its calibration, the whole pipeline
took 41 s:
- 746 video frames: 88 outside the trajectory, 128 turning faster than 30°/s.
- 9 frames kept, at a median 2.7°/s, which gave 45 views.
- 595k points within 0.6 m dropped; this was a desk scene.
- No masks: the rig travelled 0.28 m.
- COLMAP's `model_analyzer` read 5 cameras, 45 images and 23k points.

Before the 1 m gate, masks from this capture covered monitors and partitions near
the desk as well as the operator, which is why the gate exists. Masks have not yet
been checked on a walking capture.

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
The camera offset comes from the same capture's accelerometers. On a rigid
mount, the specific forces at two points differ by α×r + ω×(ω×r); gravity and the
rig's own acceleration cancel. Both accelerometers are box-averaged over ±15 ms,
the MAX2's is rotated into LiDAR axes and shifted by the clock offset, α is a
central difference of the Mid-360 gyro, and r (MAX2 IMU minus Mid-360 IMU) is
solved by least squares with a constant bias difference. The camera offset is r
plus the Mid-360 IMU position in the LiDAR frame.

The fit is trusted only when:
- each half of the capture gives an r within 2 cm of the other,
- the residual is under half of the signal, and
- every direction was excited.

Otherwise `camera_offset_m` stays null with the reason in `lever_arm.reason`.
`process-capture` uses `camera_offset_m` when `--camera-offset` is not given. The
console fills it in after **Use as calibration**. Two things remain unmodeled and
keep output provisional:
- the MAX2 IMU stands in for the lens center, and their separation inside the camera
  (about 1–3 cm) is unknown;
- the Mid-360 IMU position is the published Mid-360 value, not verified for the
  Mid-360S.

Measured on the first mount: |ω| correlation 0.9996, residual 2.2% of the signal
RMS, camera media time 95.8 ms after the record-command acknowledgement; the
weakest axis was barely excited (singular value ratio 0.013). On synthetic data
the method recovers the offset within 0.2 ms and the rotation within 0.04°.
Reprojected LiDAR depth edges stayed on static structures across the rotation by
visual inspection; this is not a quantified accuracy.

Second mount (2026-11-16, 35 s rotation):
- Gyroscopes: |ω| correlation 0.998, residual 4.5%, weakest axis 0.081.
- Camera offset (−0.073, 0.020, 0.074) m: the MAX2 sits behind and above the LiDAR.
  - The two halves agree within 3.6 mm.
  - The accelerometer residual is 0.065 of 0.259 m/s² RMS.
  - Direction strength is 0.90.
- Independent check against the panoramas:
  - Method: LiDAR depth edges within 4 m were projected into all 14 keyframe
    panoramas, scoring image gradient at the edges.
  - Scores: 2.44 with this offset, 2.17 for "straight above" (0, 0, 0.083) and
    1.56 for zero.
  - A 2 cm grid search peaked at (−0.08, 0.02, 0.10). That agrees in X and Y; Z is
    2.6 cm higher, which is within the unmodeled IMU-to-lens separation.
- On a simulated rigid rotation the solver recovers r within 3 mm.

## Primary references

- [Livox MID-360 specifications](https://www.livoxtech.com/mid-360)
- [Livox Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Livox time synchronization](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)
- [GLIM sensor setup](https://github.com/koide3/glim/wiki/Sensor-setup-guide)
- [Open GoPro](https://gopro.github.io/OpenGoPro/) and [FAQ](https://gopro.github.io/OpenGoPro/docs/faq/)
- [Labs release notes](https://gopro.github.io/labs/control/notes/)
- [LiDAR/camera calibration toolbox](https://github.com/koide3/direct_visual_lidar_calibration)
- [AnyReplica photo/pose validation](https://github.com/KuoFengYuan/anyreplica/blob/main/docs/POSE_REFINEMENT.md)
