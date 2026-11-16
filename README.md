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

## Usage

```sh
tools/splatrig-ros doctor   # check the machine
tools/splatrig-ros start    # start the driver, GLIM and the console
```

Or start "SplatRig" from the application menu. The console opens at
http://127.0.0.1:8090/; Ctrl+C stops everything.

1. **Calibrate** (once per mount): record about 30 s turning the fixed rig about all
   three axes, then press **Use as calibration**.
2. **Scan**: record while walking slowly, then press **Make 3DGS dataset**.

The dataset is written to `~/splatrig-runs/<capture>/dataset/` in COLMAP layout:
`images/`, `sparse/0/`, `points.ply`, and `masks/` when the operator could be
estimated. Earlier recordings are under **Captures**.

From a terminal:

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
