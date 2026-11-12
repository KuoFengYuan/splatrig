<p align="center"><img src="docs/assets/banner.svg" alt="SplatRig" width="100%"></p>

# SplatRig

**English** | [繁體中文](README.zh-TW.md)

Scan a scene with a handheld GoPro MAX2 + Livox Mid-360S and get a COLMAP dataset
for 3DGS training. The point cloud, route and camera position are shown live.

Status: the live console, recording and offline processing work. The camera
translation is still entered by hand, so output is marked provisional, and
accuracy has not been measured.

Docs: [hardware](docs/HARDWARE.md) · [data format](docs/FORMAT.md) · [roadmap](docs/ROADMAP.md) · [test record](docs/LOCAL_TESTS.md)

## Deploy

Ubuntu 24.04. Run `conda deactivate` before building.

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

5. Get this repository, set up the PTP service and the menu entry:

   ```sh
   git clone <repository-url> splatrig && cd splatrig
   sudo tools/install-ptp-service <lidar-nic>
   tools/splatrig-app --install
   ```

## Use

```sh
tools/splatrig-ros doctor   # check the machine, list what is missing
tools/splatrig-ros start    # start driver, console and GLIM, open the browser
```

Or click "SplatRig" in the application menu. Press Record in the page to record;
Ctrl+C to quit.

After recording:

```sh
# Once per mount: record ~30 s turning the rig about all three axes
tools/splatrig-ros calibrate-gyro ~/captures/<rotation capture> --output runs/calibration

tools/splatrig-ros process-capture ~/captures/<capture> --output runs/<name> \
  --calibration runs/calibration/calibration.json --camera-offset 0 0 0.083
```

The output folder is COLMAP layout (`images/`, `sparse/0/`, `points.ply`).

## Without hardware

```sh
python3 -m venv .venv && source .venv/bin/activate && pip install -e .
splatrig demo --output runs/demo
splatrig replay runs/demo --output runs/replay    # open runs/replay/preview.html
splatrig export runs/demo --output runs/colmap
```

The demo data is synthetic and only checks the software path.

## Development

```sh
python -m unittest discover -s tests -v
python tools/smoke.py
python tools/check_project.py
```

Working rules: [AGENTS.md](AGENTS.md).

## License

MIT, see [LICENSE](LICENSE) and [NOTICE](NOTICE). GLIM, the Livox driver and other
external software keep their own licenses.
