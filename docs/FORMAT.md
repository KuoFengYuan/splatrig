# Capture data format

**English** | [繁體中文](FORMAT.zh-TW.md)

Schema version 1 is the boundary between device adapters and offline processing.
Processing accepts rectified PINHOLE views saved as PNG; equirectangular panoramas
are first converted to such views. Raw .360 media is not decoded.

## Session files

```text
capture/
├── capture-meta.json
├── rig.json
├── clock.json
├── trajectory.jsonl
├── frames.jsonl
├── points.jsonl   (or points.bin)
└── images/
```

The demo command creates a complete example. Raw captures stay unchanged;
replay and export create separate output directories and reject existing outputs.

`capture-meta.json`:

```json
{
  "schema_version": 1,
  "source": "gopro_max2_mid360",
  "world_up": "+Z",
  "units": "meters",
  "camera_convention": "opencv",
  "point_frame": "lidar",
  "trajectory_status": "final",
  "pose_revision": 1,
  "max_pose_gap_ns": 200000000
}
```

Sources are synthetic, gopro_max2_mid360 or external. The trajectory status is live
or final. A final trajectory must come from the actual corrected history; changing
the status label alone does not finalize SLAM.

## Coordinates and calibration

`T_A_B` maps a point from B into A. World is +Z up. Camera coordinates follow
OpenCV: +X right, +Y down, +Z forward. All translations are meters. Quaternions
are unit-length wxyz, not ROS/SciPy's usual xyzw serialization.

`rig.json` contains status (uncalibrated or calibrated), T_rig_lidar and cameras:

```json
{
  "status": "uncalibrated",
  "T_rig_lidar": {"translation": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]},
  "cameras": [{
    "id": 1,
    "model": "PINHOLE",
    "width": 1536,
    "height": 1536,
    "params": [768, 768, 768, 768],
    "T_rig_camera": {"translation": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]}
  }]
}
```

These identity transforms are placeholders, not MAX2/MID-360 calibration.
The rig frame is the frame tracked by the supplied trajectory, often the IMU.
A virtual camera's transform includes its fixed view rotation and physical offset.
The current schema assumes a fixed output orientation. Time-varying stabilization
must be removed or explicitly modeled before using it.

## Equirectangular panoramas

`splatrig panorama-views` converts a panorama source into an ordinary schema 1
capture with fixed PINHOLE views. The source keeps the usual metadata, clock,
trajectory and points, has no frames.jsonl, and declares an empty camera list and
a panorama entry in `rig.json`:

```json
{
  "status": "uncalibrated",
  "T_rig_lidar": {"translation": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]},
  "cameras": [],
  "panorama": {
    "projection": "equirectangular",
    "T_rig_sphere": {"translation": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]}
  }
}
```

`panoramas.jsonl` lists positive id, timestamp_ns in the camera clock and
image_path. Images must be 2:1 equirectangular files that ffmpeg can decode.

Sphere frame S uses OpenCV axes: +Z is the image's center column on the horizon,
+X is 90° to its right (three-quarter width) and +Y points down. Pixel center
(u+0.5, v+0.5) of a W×H image has longitude (u+0.5)/W·360° − 180° and latitude
90° − (v+0.5)/H·180°.

```text
R_sphere_view = Ry(yaw) · Rx(pitch) · Rz(roll)
T_rig_camera  = T_rig_sphere · R_sphere_view
fx = fy = size / 2 / tan(fov / 2),  cx = cy = size / 2
```

Positive yaw turns right, positive pitch looks up, and positive roll turns the
view's +X axis toward +Y. Presets are `cube` (front, right, back, left, up, down)
and `ring8` (yaw every 45°); `--views` accepts a JSON list of name, yaw_deg,
pitch_deg and optional roll_deg. Each view becomes one camera. Frame IDs are
(panorama id − 1) × view count + view index, capture_id is the panorama id and
source_panorama_path records provenance. The rig status carries over, so an
uncalibrated T_rig_sphere still blocks final export.

ffmpeg's v360 filter resamples pixels after converting input to 8-bit RGB;
ffmpeg 6.1 overflows 16-bit bilinear samples at full white. Before each run,
SplatRig resamples a panorama whose pixels encode their own ray direction and
refuses to convert if the installed ffmpeg exceeds 0.6 input pixels of maximum
error or 0.25 pixels of mean bias. `panorama-views.json` records the ffmpeg version
and probe result. All views share their panorama's timestamp. Rolling shutter,
stitching parallax, per-view exposure time, lens masks and time-varying
stabilization are not modeled, and original .360/EAC media is not decoded.

## Time

All timestamps are nonnegative integer nanoseconds. Camera times and reference
times may have very different origins. `clock.json` defines:

```text
reference_ns = reference_anchor_ns
             + round((camera_ns - camera_anchor_ns) * rate)
```

Fields: camera_anchor_ns, reference_anchor_ns, rate and status (provisional or
calibrated). Subtract anchors before floating point math to retain epoch precision.
`fit-clock` estimates an affine model from at least three increasing paired
physical events and rejects excessive residuals. Its default maximum residual is
5 ms, a software acceptance setting, not measured MAX2 synchronization accuracy.
Record time span and residuals; events clustered at one end do not establish drift
over a long capture. The tool does not extract these events from GPMF yet.

Preview arrival time and a shared recording command are not exposure timestamps.
Device-specific PTS, exposure midpoint and rolling-shutter timing remain adapter
responsibilities.

## Trajectory, images and points

Each trajectory line contains timestamp_ns in the reference clock, revision,
translation and quaternion_wxyz, representing T_world_rig. Times must increase
strictly, and every revision must equal capture-meta.json's pose_revision.

Each frame line contains positive integer id and camera_id, timestamp_ns in the
camera clock and image_path relative to the capture folder. An optional capture_id
can group virtual views from one source exposure. Paths must remain inside the
capture. PNG header dimensions must match the camera calibration. The header
check is not a complete image decoder.

Each point line contains timestamp_ns in the reference clock, xyz in the LiDAR
frame and optional integer rgb in [0, 255]. Missing color defaults to gray.
An adapter must preserve each point's acquisition time rather than assigning one
packet-receipt time to an entire moving scan.

Instead of points.jsonl, a session may contain the binary `points.bin` (never both).
It starts with the 8-byte magic `SRPOINT1`, a little-endian uint32 header length,
a UTF-8 JSON header and zero padding to an 8-byte boundary. Fixed 24-byte records
follow: int64 timestamp_ns, float32 x/y/z in meters in the LiDAR frame, uint8
reflectivity, tag and line, and one reserved byte. The header lists these fields;
readers reject other layouts and truncated records. Binary points use the default
gray color.

```text
T_world_camera(t) = T_world_rig(t) · T_rig_camera
p_world(t)        = T_world_rig(t) · T_rig_lidar · p_lidar(t)
```

Translation is linearly interpolated and rotation uses shortest-path SLERP.
No extrapolation is allowed. Interpolation across a gap above max_pose_gap_ns is
rejected. Querying an exact recorded pose does not require interpolation.
Both photos and points are recomputed from the same revision after loop correction.

## LiDAR recordings

`splatrig import-livox BAG --output DIR` converts a livox_ros_driver2 rosbag2
recording (CustomMsg points and sensor_msgs/Imu) into a LiDAR recording, an input
for later session assembly rather than a complete session:

```text
recording/
├── recording.json   source, counts, gaps, clock evidence and limitations
├── points.bin       per-point time = timebase + offset_time; no-return (0, 0, 0) excluded
└── imu.jsonl        timestamp_ns, angular_velocity_rad_s, linear_acceleration
```

The IMU acceleration unit is recorded from the mean magnitude (`g` or `m/s^2`);
values are not rescaled. Excluded no-return points, frame and IMU gaps, overlapping
frames and non-increasing IMU stamps are counted, never dropped silently.

CustomMsg does not carry the device's timestamp type, and an unsynchronized driver
substitutes host packet-arrival time. The importer therefore classifies the clock
from timing structure. Synchronized stamps place packet boundaries at one fixed
point spacing and frame steps on a few discrete values; arrival stamps inherit
jitter. Status `ptp_inferred` needs a boundary concentration of at least 0.9 and
two-window frame-step concentration of at least 0.8, with no backwards offsets.
Other statuses (`host_arrival_suspected`, `indeterminate`) stop the import unless
`--allow-unsynced` writes a recording marked provisional. This is an inference
from data, not the device's own report; record the PTP setup separately.

The importer needs ROS 2 and the driver's message package at run time, so run it
with the system Python after sourcing both environments. The offline core does
not import ROS.

## Outputs and limits

Replay writes a PLY, image-poses.jsonl, report.json and an embedded browser preview.
The replay pose log uses source_image_path relative to the original capture;
it does not copy images. Export adds copied images and COLMAP binary/text models.
Its pose log has image_path relative to the exported dataset and source_image_path
for provenance. Camera poses in this log are c2w; COLMAP images store w2c.
There is no implicit world-up flip.

COLMAP points have empty observation tracks, and image POINTS2D arrays are empty.
These are valid initialization inputs, not feature correspondences for bundle
adjustment. The report records this distinction and the provisional/synthetic state.
See the [COLMAP format](https://colmap.github.io/format.html).

Voxel replay has a configurable occupied-voxel budget; it fails explicitly when
full. It is not a streaming, unlimited city-scale mapper. Trajectory and frame
records are loaded into memory. The preview embeds at most 20,000 sampled points.

Per-camera depth, masks, raw .360/EAC decoding, feature tracks, image-quality
metrics and trainer execution are future milestones. LiDAR range must be transformed
to camera Z depth with visibility masking before any depth-supervision integration.
