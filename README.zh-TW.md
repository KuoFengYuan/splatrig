# SplatRig

[![Check](https://github.com/KuoFengYuan/splatrig/actions/workflows/check.yml/badge.svg)](https://github.com/KuoFengYuan/splatrig/actions/workflows/check.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)

[English](README.md) | **繁體中文**

SplatRig 錄製手持的 Livox Mid-360S + GoPro MAX2 設備，並把每段錄影轉成 Gaussian
splatting 用的 COLMAP 資料集。掃描時可在瀏覽器介面看到即時的 SLAM 地圖。

> **狀態：** 開發初期，只在一台桌機上、感測器接線的情況下測試過。資料集標示為暫定，
> 精度尚未量測。

## 功能

- 即時介面：GLIM 地圖、路線、相機位姿、覆蓋率與速度警告
- 一行指令啟動，並可檢查環境（`doctor`）
- 用 30 秒的旋轉錄影校正相機與 LiDAR（旋轉、時間差、位置）
- 輸出資料集：選取關鍵幀、切成立方體視角、點雲上色、操作者遮罩

## 需求

- Ubuntu 24.04 與 ROS 2 Jazzy
- Livox Mid-360S（乙太網路）、GoPro MAX2（USB）
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)、
  [GLIM](https://github.com/koide3/glim)、ffmpeg、linuxptp

## 安裝

編譯前先 `conda deactivate`。

1. 安裝 [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)，再裝：

   ```sh
   sudo apt install -y ros-jazzy-desktop ros-dev-tools linuxptp ffmpeg
   ```

2. Livox SDK2 與 driver：

   ```sh
   git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
   cd ~/Livox-SDK2 && mkdir build && cd build && cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
   make -j"$(nproc)" && sudo make install && sudo ldconfig
   mkdir -p ~/ws_livox/src && git clone https://github.com/Livox-SDK/livox_ros_driver2.git ~/ws_livox/src/livox_ros_driver2
   ```

   修改 `config/MID360s_config.json`：`host_ip` 改成 `192.168.1.50`，`ip` 改成
   `192.168.1.1XX`（XX 為序號末兩碼）。然後編譯：

   ```sh
   source /opt/ros/jazzy/setup.bash && cd ~/ws_livox/src/livox_ros_driver2 && ./build.sh jazzy
   ```

3. GLIM：

   ```sh
   curl -s --compressed https://koide3.github.io/ppa/ubuntu2404/KEY.gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null
   echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2404 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list
   sudo apt update && sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev libgtsam-points-dev ros-jazzy-glim-ros
   ```

4. 網路（網卡名稱用 `ip -br link` 查）：

   ```sh
   sudo nmcli con add type ethernet ifname <lidar-nic> con-name livox ipv4.method manual \
     ipv4.addresses 192.168.1.50/24 ipv4.never-default yes ipv6.method disabled
   sudo nmcli con add type ethernet ifname <gopro-nic> con-name gopro ipv4.method auto \
     ipv4.never-default yes ipv6.method disabled
   ```

5. 下載本專案，設定 PTP 服務與選單捷徑：

   ```sh
   git clone https://github.com/KuoFengYuan/splatrig.git && cd splatrig
   sudo tools/install-ptp-service <lidar-nic>
   tools/splatrig-app --install
   ```

## 使用

### 1. 準備設備

- 把 MAX2 和 LiDAR 互相鎖死。之後只要固定方式有變，就要重新校正。
- LiDAR 接上電源與網路轉接器。
- MAX2 用 USB 接上並開機。

### 2. 啟動介面

```sh
cd splatrig
tools/splatrig-ros doctor   # 應全部 OK；MAX2 可以只是 WARN
tools/splatrig-ros start    # 啟動 driver、GLIM 與介面
```

也可以從應用程式選單點「SplatRig」。介面位於 http://127.0.0.1:8090/。拿穩設備不動，
直到地圖出現、左上 **Devices** 變綠色。

### 3. 校正（第一次使用，以及每次改變固定方式後）

1. 按圓形按鈕。
2. 靜止 3 秒。
3. 繞三個軸轉動設備約 30 秒：左右轉、上下點頭、左右歪。每次轉動超過 45°，但不要甩。
4. 靜止 3 秒，再按一次按鈕。
5. 在跳出的面板按 **Use as calibration**。

結果應該是：陀螺儀相關係數 0.95 以上、三個軸都是「good」，而且相機位移顯示已量測。

### 4. 掃描

1. 按圓形按鈕，靜止 3 秒。
2. 用一般走路一半的速度前進，每 30 cm 會取一張照片。
3. 轉向放慢，轉 90° 至少花 3 秒。
4. 設備舉高、放在身體前方，讓人保持在設備後面。
5. 依照畫面上方的提示操作，見下表。
6. 留意按鈕外圈：大部分表面都從多個方向看過後會變綠色。**Coverage** 會標出還是紅色的地方。
7. 回到起點，靜止 3 秒，再按一次按鈕。

| 提示 | 怎麼做 |
|---|---|
| Turning too fast · photos … will be dropped | 放慢；取到照片後紅框就會消失 |
| Nearly too fast | 稍微放慢 |
| Return to an area you scanned | 走回已掃過的地方，讓 SLAM 修正漂移 |
| Back in a scanned area | 繼續掃描 |
| SLAM lost tracking | 先靜止，再慢慢移動 |
| No PTP time sync | 執行一次 `sudo tools/install-ptp-service <lidar-nic>` |
| The MAX2 is not recording | 檢查相機；沒有錄影就沒有照片 |

### 5. 產出資料集

在停止後跳出的面板按 **Make 3DGS dataset**。處理約需幾分鐘，面板會列出每個步驟。完成後，
按 **Copy dataset path** 可以複製資料夾路徑。

資料集寫在 `~/splatrig-runs/<錄影名稱>/dataset/`，為 COLMAP 格式：`images/`、
`sparse/0/`、有顏色的 `points.ply`，能估計操作者時另有 `masks/`。之前的錄影在
**Captures** 裡。

### 6. 訓練

任何讀得了 COLMAP 資料夾的訓練器都可以。例如原版
[3D Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting)：

```sh
python train.py -s ~/splatrig-runs/<錄影名稱>/dataset
```

支援遮罩的訓練器（例如 nerfstudio）也可以使用 `masks/`。

結束時在終端機按 Ctrl+C，介面、GLIM 與 driver 會一起關閉。

### 指令列

不開介面，直接執行校正與處理：

```sh
tools/splatrig-ros calibrate-gyro ~/captures/<旋轉錄影> --output runs/calibration
tools/splatrig-ros process-capture ~/captures/<錄影> --output runs/<名稱> \
  --calibration runs/calibration/calibration.json
```

### 沒有硬體時

```sh
python3 -m venv .venv && source .venv/bin/activate && pip install -e .
splatrig demo --output runs/demo
splatrig replay runs/demo --output runs/replay    # 用瀏覽器開 runs/replay/preview.html
splatrig export runs/demo --output runs/colmap
```

示範資料是合成的，只用來驗證軟體流程。

## 文件

- [硬體整合](docs/HARDWARE.zh-TW.md)
- [資料格式](docs/FORMAT.zh-TW.md)
- [開發路線](docs/ROADMAP.zh-TW.md)
- [本機測試紀錄](docs/LOCAL_TESTS.zh-TW.md)

## 開發

```sh
python -m unittest discover -s tests -v
python tools/smoke.py
python tools/check_project.py
```

請參考 [CONTRIBUTING](CONTRIBUTING.zh-TW.md) 與 [AGENTS.md](AGENTS.zh-TW.md)。

## 授權

MIT，見 [LICENSE](LICENSE) 與 [NOTICE](NOTICE)。GLIM、Livox driver 等外部軟體依各自的授權。
