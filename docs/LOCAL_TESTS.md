# Local test record

**English** | [繁體中文](LOCAL_TESTS.zh-TW.md)

Date: 2026-11-11. Software checks ran locally; one Mid-360S was later connected
for a stationary recording. No MAX2 was connected.
No repository or captured data was published to GitHub.

## What actually ran

The demo generated a virtual room, rendered eight small PINHOLE images, supplied
a known camera/rig trajectory and synthesized timed LiDAR points. The processing
code then associated images with that trajectory, replayed the points and wrote
a COLMAP initialization dataset. It did not estimate the trajectory using SLAM.

| Check | Result |
| --- | --- |
| Geometry, timing, export and ROS-message conversion | 47 unit tests passed, including 9 panorama and 10 LiDAR-import tests |
| Python runtimes | Unit tests and CLI smoke passed on Python 3.12 and 3.14 |
| End-to-end CLI | Demo → validate → replay → export and clock fit passed |
| Distribution | Wheel built and installed into a clean environment; CLI smoke passed without runtime dependencies |
| Demo data | 8 images, 165 poses, 17,280 source points, 14,806 voxels at 0.05 m |
| COLMAP native reader | Read 8 registered images and 14,806 points |
| Panorama views | Synthetic 2:1 panoramas → cube views; 48 frames from 6 cameras validated and exported |
| Panorama geometry | Each view matched a direct render of the same scene; ±1° rotations never matched better |
| ffmpeg convention probe | ffmpeg 6.1.1: 0.39 input pixels maximum error, −0.09/0.11 pixels mean bias |
| COLMAP native reader, panorama views | Read 6 cameras, 48 registered images and 14,806 points |
| Physical Mid-360S, stationary 59.3 s | No frame/IMU gaps; clock `ptp_inferred` (boundary 0.9996, frame steps 0.9932) |
| Mid-360S import fidelity | All 5,672,997 returned points matched the bag in time, float32 xyz, reflectivity, tag and line |
| Observation tracks | Zero, intentionally: this is an initialization model |
| Browser | Desktop 1280×900 and narrow 390×844 viewport checked |
| Browser controls | Pointer/keyboard orbit, wheel zoom, cameras and reset passed |
| Offline preview | No external HTTP(S) requests or browser script errors |
| Documentation | Local links, language counterparts and source syntax checked |
| Repository skill | Skill format validator passed |

Native COLMAP was 4.3.0.dev0, commit 3c1a1b95. Browser checks used headless Google
Chrome 151.0.7922.137. Synthetic reprojection checks use independent analytic
coordinates; the native reader's zero-observation reprojection value is not an
accuracy measurement.

Python 3.10 is included in the prepared CI matrix but was not installed locally;
that runtime and hosted CI have not been run in this local-only phase.

## Not established by these checks

- MAX2 capture, or any moving or long-duration LiDAR capture.
- Detection of an unsynchronized physical recording (tested with modeled driver stamps only).
- Real-time SLAM, loop closure or measured live throughput.
- MAX2 exposure synchronization or actual rig calibration.
- Original .360/EAC decoding and conversion of real MAX2 stitched panoramas.
- Rolling shutter, stitching parallax or masks in panorama views.
- Real indoor/outdoor geometry accuracy.
- 3DGS training, image quality or trainer-specific compatibility.
- Physical phone interaction, sensor thermals or portable power/runtime.

The ROS recorder was tested with constructed messages, not a running ROS graph.
See the [roadmap](ROADMAP.md) for physical-device acceptance work.

## Reproduce locally

After installing the project:

    python -m unittest discover -s tests -v
    python tools/smoke.py
    python tools/check_project.py

Panorama tests and the panorama smoke step need ffmpeg and ffprobe on PATH;
without them the conversion tests are skipped and the smoke report says so.

Optional browser checks require the dev extra and a local Chrome/Chromium:

    python -m pip install -e '.[dev]'
    python tools/check_viewer.py runs/replay/preview.html

The browser checker also accepts --browser with a compatible executable path.
The tests create new temporary captures; the existing scans stay unchanged.
