# SplatRig

[![Check](https://github.com/KuoFengYuan/splatrig/actions/workflows/check.yml/badge.svg)](https://github.com/KuoFengYuan/splatrig/actions/workflows/check.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)

**English** | [繁體中文](README.zh-TW.md)

SplatRig records a handheld Livox Mid-360S + GoPro MAX2 rig and turns each
recording into a COLMAP dataset for Gaussian splatting. A browser console shows
the live SLAM map while you scan.

> **Status:** early development, tested on one desktop with the sensors on cables.
> Datasets are marked provisional; accuracy has not been measured.

## Features

- Live console: GLIM map, route, camera pose, coverage and speed warnings
- One-command start and a machine check (`doctor`)
- Camera-to-LiDAR calibration from a 30 s rotation (rotation, time offset, position)
- Dataset export: keyframe selection, cube views, coloured points, operator masks

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy
- Livox Mid-360S on Ethernet, GoPro MAX2 on USB
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2),
  [GLIM](https://github.com/koide3/glim), ffmpeg, linuxptp

## Installation

Run `conda deactivate` before building.

1. Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then:

   ```sh
   sudo apt install -y ros-jazzy-desktop ros-dev-tools linuxptp ffmpeg
   ```

2. Livox SDK2 and driver:

   ```sh
   git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
   cd ~/Livox-SDK2 && mkdir build && cd build && cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
   make -j"$(nproc)" && sudo make install && sudo ldconfig
   mkdir -p ~/ws_livox/src && git clone https://github.com/Livox-SDK/livox_ros_driver2.git ~/ws_livox/src/livox_ros_driver2
   ```

   Edit `config/MID360s_config.json`: set `host_ip` to `192.168.1.50` and `ip` to
   `192.168.1.1XX` (XX = last two serial digits). Then build:

   ```sh
   source /opt/ros/jazzy/setup.bash && cd ~/ws_livox/src/livox_ros_driver2 && ./build.sh jazzy
   ```

3. GLIM:

   ```sh
   curl -s --compressed https://koide3.github.io/ppa/ubuntu2404/KEY.gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null
   echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2404 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list
   sudo apt update && sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev libgtsam-points-dev ros-jazzy-glim-ros
   ```

4. Network (find NIC names with `ip -br link`):

   ```sh
   sudo nmcli con add type ethernet ifname <lidar-nic> con-name livox ipv4.method manual \
     ipv4.addresses 192.168.1.50/24 ipv4.never-default yes ipv6.method disabled
   sudo nmcli con add type ethernet ifname <gopro-nic> con-name gopro ipv4.method auto \
     ipv4.never-default yes ipv6.method disabled
   ```

5. This repository, the PTP service and the menu entry:

   ```sh
   git clone https://github.com/KuoFengYuan/splatrig.git && cd splatrig
   sudo tools/install-ptp-service <lidar-nic>
   tools/splatrig-app --install
   ```

6. Optional, to mask people: a Python with PyTorch and torchvision (CUDA
   recommended):

   ```sh
   mkdir -p ~/.config/splatrig
   echo 'export SPLATRIG_TORCH_PYTHON=/path/to/python' >> ~/.config/splatrig/env
   ```

## Usage

### 1. Prepare the rig

- Fix the MAX2 and the LiDAR rigidly to each other. Any later change to the mount
  needs a new calibration.
- Power the LiDAR and connect its Ethernet adapter.
- Connect the MAX2 by USB and switch it on.

### 2. Start the console

```sh
cd splatrig
tools/splatrig-ros doctor   # everything OK; the MAX2 may only warn
tools/splatrig-ros start    # starts the driver, GLIM and the console
```

Or start "SplatRig" from the application menu. The console opens at
http://127.0.0.1:8090/. Hold the rig still until the map appears and **Devices**
(top left) turns green.

### 3. Calibrate (first time and after changing the mount)

1. Press the round button.
2. Hold still for 3 s.
3. Turn the rig about all three axes for about 30 s: left and right, up and down,
   and a roll. Keep each turn larger than 45°, but not jerky.
4. Hold still for 3 s, then press the button again.
5. In the panel that opens, press **Use as calibration**.

The result should show a gyro correlation of at least 0.95, "good" for all three
axes, and a measured camera offset.

### 4. Scan

1. Press the round button and hold still for 3 s.
2. Walk at about half your normal pace. A photo is taken every 15 cm.
3. Turn slowly: a 90° turn should take 3 s or more.
4. Hold the rig high and in front of you, so that you stay behind it.
5. Follow the message at the top of the screen. The table below lists them.
6. Watch the ring around the button: it turns green once most surfaces have been
   seen from several directions. **Coverage** shows which areas are still red.
7. Finish where you started, hold still for 3 s and press the button again.

| Message | What to do |
|---|---|
| Turning too fast · photos … will be dropped | Slow down; the red frame disappears once a photo is taken |
| Nearly too fast | Slow down a little |
| Return to an area you scanned | Walk back to a place you already scanned so SLAM can correct drift |
| Back in a scanned area | Continue |
| SLAM lost tracking | Hold still, then move slowly |
| No PTP time sync | Run `sudo tools/install-ptp-service <lidar-nic>` once |
| The MAX2 is not recording | Check the camera; the scan has no images without it |

### 5. Make the dataset

In the panel shown after you stop, press **Make 3DGS dataset**. Processing takes a
few minutes and lists each step; more jobs wait in line. When it finishes,
**Preview dataset** shows the coloured points and where each photo was taken.
Pick a photo in the strip to look from its camera with the photo laid over the
points, to check that they line up.

**Scans** (right) lists every recording with its status:
- Search, sort and filter, and rename a scan.
- Delete moves a scan and its results to Recently Deleted for 30 days.
- Deleting for good asks you to type DELETE.
- The scan of the current calibration cannot be deleted.

The dataset is written to `~/splatrig-runs/<capture>/dataset/` in COLMAP layout:
`images/`, `sparse/0/`, `points.ply` with colours, and `masks/` for the rig and
people. Earlier recordings are under **Captures**.

### 6. Train

Use any trainer that reads COLMAP folders. For example, with the original
[3D Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting):

```sh
python train.py -s ~/splatrig-runs/<capture>/dataset
```

With LichtFeld Studio, use the masks so the rig and people are not trained:

```sh
LichtFeld-Studio -d ~/splatrig-runs/<capture>/dataset --mask-mode ignore
```

Press Ctrl+C in the terminal to stop the console, GLIM and the driver.

### Command line

The same calibration and processing without the console:

```sh
tools/splatrig-ros calibrate-gyro ~/captures/<rotation capture> --output runs/calibration
tools/splatrig-ros process-capture ~/captures/<capture> --output runs/<name> \
  --calibration runs/calibration/calibration.json
```

### Without hardware

```sh
python3 -m venv .venv && source .venv/bin/activate && pip install -e .
splatrig demo --output runs/demo
splatrig replay runs/demo --output runs/replay    # open runs/replay/preview.html
splatrig export runs/demo --output runs/colmap
```

The demo data is synthetic and only checks the software path.

## Documentation

- [Hardware integration](docs/HARDWARE.md)
- [Data format](docs/FORMAT.md)
- [Roadmap](docs/ROADMAP.md)
- [Local test record](docs/LOCAL_TESTS.md)

## Development

```sh
python -m unittest discover -s tests -v
python tools/smoke.py
python tools/check_project.py
```

See [CONTRIBUTING.md](CONTRIBUTING.md) and [AGENTS.md](AGENTS.md).

## License

MIT, see [LICENSE](LICENSE) and [NOTICE](NOTICE). GLIM, the Livox driver and other
external software keep their own licenses.
