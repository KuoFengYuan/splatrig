# 硬體整合

[English](HARDWARE.md) | **繁體中文**

目標為一台 GoPro MAX2 與一顆 Livox MID-360，手持使用，涵蓋室內外。
本儲存庫目前尚未完成實機擷取驗證。

## 各元件負責的工作

| 元件 | 工作 | 目前狀態 |
| --- | --- | --- |
| MID-360 與內建 IMU | 帶時間戳的點與慣性資料 | 外部驅動 |
| Livox Driver 2 | 裝置傳輸，保留逐點時間 | Mid-360S 已以 PTP 運作；已實作 bag 匯入器 |
| GLIM | 即時點雲、rig 軌跡與全域最佳化 | 外部相依工具 |
| ROS 里程計記錄器 | 記錄指定 Odometry topic | 已實作；已測訊息轉換，尚未測即時 ROS |
| 全景視角轉換 | 透過 ffmpeg 將 equirectangular 轉為固定 PINHOLE 視角 | 已實作；僅以合成全景測試，尚未使用 MAX2 影片 |
| MAX2 轉接層 | 預覽／控制、原始錄影與時間資訊 | 已以 USB 測試預覽與錄影控制；轉接層規劃中 |
| SplatRig 核心 | 校正模型、位姿配對、重播與 COLMAP | 已實作，使用合成資料檢查 |
| 桌面訓練器 | 訓練高斯模型 | 外部工具，尚無實拍品質基準 |

## 支架與運算設備

將兩個感測器固定在剛性支架，保留視線與可重複的安裝位置。透過校正取得
T_rig_lidar 與 T_rig_camera；軌跡追蹤的座標可能是 IMU，而非 LiDAR
光學原點。拆裝後重新檢查校正。

先在 Ubuntu 筆電／工作站整合，同時量測運算、記錄、預覽與電力負載，
再選擇小型電腦或 Jetson。電源依各裝置官方手冊配置。

MID-360 水平涵蓋 360°、垂直 59°，MAX2 全景照片涵蓋範圍大於深度範圍。
缺少深度的位置維持無效，規劃不同高度及傾角補掃。起初可用強度／高度顯示
點雲；精確 RGB 著色依賴相機校時及可見性處理。

## MID-360 與 GLIM

使用官方 Livox SDK2／Driver 2 及 GLIM 感測器指南。實機成功後才鎖定可用版本；
目前未認證任何裝置／韌體組合。保留逐點擷取時間、點與 IMU 單位及裝置診斷。

確認加速度單位是 g 或 m/s²。GLIM 指南記載 Livox 的縮放設定；若上游已轉換，
不要再次縮放。SLAM 前先確認 IMU／LiDAR 外參及時間起點。
PTP 可以同步 MID-360 與主機，不會同步 MAX2 曝光。

已提供的記錄器接受 nav_msgs/Odometry：

```sh
splatrig record-poses --output runs/live-poses.jsonl \
  --topic /your/odometry --world-frame odom --rig-frame imu
```

先載入 ROS 2 環境，並將 SplatRig 安裝在能載入 ROS Python 套件的環境。
實際 topic 與 frame 名稱以 SLAM 設定為準，上述只是範例。
記錄器要求 frame 完全吻合，使用訊息擷取時間、逐筆 flush，時間重複或倒退
時停止，不會暗中執行 TF 轉換。

這份紀錄是即時里程計，不是閉環修正後的最終歷史。完整轉接層仍需記錄
原始逐點時間資料，並從後端取得最終修正軌跡；不能只把狀態改成 final。

## 本機 MID-360S 啟用紀錄

在 Ubuntu 24.04、ROS 2 Jazzy、Livox SDK2 及 livox_ros_driver2 1.2.8 上啟用了一台
Mid-360S（裝置韌體 35.01.01.08）。此紀錄只確認資料串流與 PTP 時間同步，
尚未驗收任何擷取或校正。

- **確認型號。** 探測回應回報 dev_type 35（Mid-360S），不是 9（MID-360）。
  請使用驅動的 `MID360s_config.json` 與 `msg_MID360s_launch.py`；若使用 MID-360
  設定，SDK 會回應探測卻不開啟資料埠，也不顯示錯誤。
- **網路。** LiDAR 網卡設定裝置網段的固定主機位址（出廠預設主機為 192.168.1.50；
  裝置為 192.168.1.1XX，XX 為序號末兩碼），且不設預設路由。將此位址填入所有
  `host_ip` 欄位、裝置位址填入 `lidar_configs`，再重新編譯以更新安裝目錄中的副本。
- **編譯環境。** 編譯或執行 ROS 前先停用 conda，讓 CMake 與 ros2 使用系統 Python。
  CMake 4.x 需以 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` 設定 Livox SDK2，再執行
  `sudo make install && sudo ldconfig`。
- **時間同步。** 未同步時封包 `time_type` 為 0，裝置時鐘從開機起算，驅動會改用
  主機收包時間取代裝置時間戳。linuxptp 4.0 預設送出 PTP v2.1，裝置會忽略
  （HMS 警告 0x0403）。只執行一個主時鐘：

  ```sh
  sudo ptp4l -i <lidar-nic> -S -m --ptp_minor_version 0
  ```

  不要同時執行 gPTP 主時鐘（HMS 0x040A）。以裝置內部狀態（`time_sync_type`、
  `last_sync_time`）及封包 `time_type` 確認同步。此網卡僅提供軟體時間戳。
- **靜止時觀察。** 點雲 10.000 Hz，每則約 20,000 點，逐點偏移涵蓋 0–100 ms；
  IMU 200 Hz，加速度單位為 g（大小約 0.99）；約一半的點為 (0, 0, 0) 無回波項目。
  同步後 IMU 時間戳間隔在約 4.26 與 5.78 ms 間交替，並非固定 5 ms；
  此規律仍需較長錄製確認。
- **第一段錄製。** 59.3 秒靜止 bag（230 MiB）沒有幀或 IMU 空缺：592 幀、
  每秒 199,996 點、IMU 199.999 Hz。IMU 收包延遲為 0.72／0.99／1.25 ms
  （p1／p50／p99），每 5 秒最小值的趨勢為 −0.86 ppm，符合 PTP 全程鎖定。
  52.1% 的點為無回波項目。匯入方式：

  ```sh
  conda deactivate
  source /opt/ros/jazzy/setup.bash && source ~/ws_livox/install/setup.bash
  tools/splatrig-ros import-livox <bag> --output <recording>
  ```

## MAX2 整合前的驗證

在預定韌體與主機上一起確認：

1. 選定的 360 模式可以同時錄影與預覽。
2. 已掌握預覽尺寸、投影、緩衝、掉幀與 PTS。
3. 原始 .360 分段與 GPMF 可完整保存並可重現解碼。
4. 預覽與原始錄影可透過明確時鐘模型配對。
5. 實體支架轉動時，輸出影像座標仍符合校正。

Open GoPro 支援 MAX2，FAQ 指出原始 metadata 需等檔案儲存後才能取得，
且指令執行不是確定性的同步。掃描時用 rig 位姿推算相機軌跡，
停止後從原始錄影精修高解析影像配對；線上預覽位姿維持暫定狀態。

以受控旋轉序列測試 Labs NROT 與接縫深度控制。移除旋轉防震不代表相機
已完全符合中心投影，也不會自動移除 rolling shutter 或接縫視差。
先檢查近物與接縫，再信任投影深度。

### 本機 MAX2 預覽檢查

以 USB 測試一台 MAX2（韌體 H24.02.01.22.00，Open GoPro 2.0）。它會以 CDC-NCM
網路介面出現；請為它建立 DHCP 連線並且不設預設路由（`ipv4.never-default yes`）。
相機在該網段的 `.51` 位址、8080 埠提供 HTTP。測試順序：開啟有線控制、載入錄影
預設、開始預覽串流、錄影約 10 秒，再停止、載回拍照預設並釋放控制。相機忙碌時
載入預設回傳 HTTP 500，閒置後即成功。

- 預覽：透過 UDP 8554 埠的 MPEG-TS，HEVC Main 1408×704、29.97 fps，另含音訊串流。
  畫面是**左右並排、未拼接的兩個魚眼圓**，不是 equirectangular。
  從串流中途開始解碼時，在下一個關鍵幀前會回報缺少參考幀。
- 錄影時預覽：持續 30 fps，但開始錄影時中斷約 1.9 秒、停止時中斷約 1.2 秒。
- 預覽 PTS 在中斷期間保持連續，因此預覽時間戳不是實際時間，不能用於時間配對。
- 錄影存為 `.360` 檔；其解碼、時間與拼接尚未測試。

## 室內外驗收掃描

使用小房間、長走廊、建築外圍及室內走到室外的連續路線；涵蓋重訪、
轉彎、快慢移動、素面牆、玻璃與移動物體。記錄封包／影像遺失、校時殘差、
延遲、軌跡缺口、閉環修正與資源使用。

影像重投影、保留影像的渲染品質及 LiDAR 表面對齊分別驗證。
模型外觀不代表尺寸精準，AnyReplica 的位姿實驗可作為這項區分的參考。

## 即時擷取介面

需要 ROS 2 的指令請透過 `tools/splatrig-ros` 執行：它會建立乾淨的環境（不含 conda），
source ROS 2 與 `~/ws_livox`（可用 `LIVOX_WS` 覆寫），並把專案加到 ROS 的
`PYTHONPATH` 後面。若在 source ROS 之後直接設定 `PYTHONPATH=…/src`，
會覆蓋 ROS 的路徑而找不到 `rclpy`。

`splatrig live` 在 http://127.0.0.1:8090/ 提供本機頁面（只綁定 localhost）。頁面顯示
感測器座標中最近一秒的 LiDAR 點、MAX2 預覽、頻率與時鐘狀態，以及錄製按鈕：
在 `~/captures` 下建立新資料夾，同時以 `ros2 bag record` 錄製原始 Livox topic
並啟動 MAX2 錄影。請在 ptp4l 與驅動已執行、source ROS 2 與驅動工作區後，
以系統 Python 執行：

```sh
tools/splatrig-ros live            # 只用 LiDAR 時加 --gopro none
```

- 頁面僅供監看。預覽幀標示為非曝光時間，`capture-log.json` 只記錄主機指令時間；
  LiDAR 與相機的開始時間是近似值，並未同步。
- POST 請求需帶 `X-SplatRig: 1` 標頭。伺服器不綁定其他網路介面；手機存取需另行
  審慎設定。
- MAX2 預覽執行時，約 380 KB 的 Livox 幀超過 208 KiB 的預設 UDP 接收緩衝區，
  best-effort DDS 會整幀丟棄（降到 1–3 Hz）。`live`、`ros-pointcloud` 及其啟動的
  bag 錄製會使用套件內的 Fast DDS 設定（共享記憶體、4 MiB UDP 緩衝區），
  除非已設定其他設定檔；另外手動執行 `ros2 bag record` 時，請將
  `FASTRTPS_DEFAULT_PROFILES_FILE` 設為 `src/splatrig/assets/fastdds.xml`。
- `splatrig ros-pointcloud` 將 CustomMsg 轉為 PointCloud2 供 RViz2／Foxglove 使用
  （不含逐點時間）。

## GLIM 串接

GLIM 讀取含逐點時間欄位的 PointCloud2，不讀 Livox CustomMsg。
`splatrig glim-bag <bag> --output <dir>` 寫出 `<dir>/bag`，內含 `/livox/points`
（x、y、z、intensity，以及相對標頭時間的 `t` 秒數；排除無回波點）與未改動的
IMU 訊息；原始 bag 不會被修改。請相應設定 GLIM 的 `points_topic`／`imu_topic`，
並將 `acc_scale` 設為 9.80665。`conversion.json` 中的 MID-360 IMU 偏移取自公開的
MID-360 配置，尚未針對 Mid-360S 驗證。`splatrig import-trajectory traj_imu.txt
--revision N --status final --frame imu --output <dir>` 將 GLIM 的 TUM 軌跡以精確
整數奈秒轉為 `trajectory.jsonl`；只有閉環後的 `traj_*` 檔可標為 `final`，
`odom_*` 不可。

以 GLIM PPA 的 CPU 套件（`ros-jazzy-glim-ros` 1.2.2）測試；其 GPU 套件針對
CUDA 12.6／13.1，尚未在此 RTX 50 系列 GPU 上嘗試。

```sh
splatrig glim-config --output runs/glim-config        # CPU 模組、Livox topic、acc_scale
splatrig glim-bag <bag> --output runs/glim-input
ROS_DOMAIN_ID=73 ros2 run glim_ros glim_rosbag runs/glim-input/bag --ros-args \
  -p config_path:=$PWD/runs/glim-config -p dump_path:=$PWD/runs/glim-dump -p auto_quit:=true
splatrig import-trajectory runs/glim-dump/traj_imu.txt --revision 1 --status final \
  --frame imu --output runs/trajectory
```

請使用未被佔用的 `ROS_DOMAIN_ID`：`glim_rosbag` 也會訂閱即時 topic，
執行中的驅動曾把當下的 IMU 訊息混入離線處理。59 秒靜止 bag 以 4.4 秒處理完成，
IMU 位姿與第一筆位姿的差距在 7.9 mm 與 0.31° 以內。靜止漂移不代表移動時的精度。

## 第一版處理流程

`tools/splatrig-ros process-capture <capture> --output <run> --camera-offset X Y Z
--camera-ypr YAW PITCH ROLL` 將 `live` 錄製的資料夾轉成 COLMAP 資料集：LiDAR 匯入、
GLIM bag 與設定、在 ROS domain 73 離線執行 GLIM、匯入軌跡、下載新的 MAX2 `.360` 檔、
擷取全景、組成掃描、切成 cube 視角並匯出。處理在隱藏目錄中進行，成功後才改名發布；
失敗時保留 `.<run>-failed` 供檢查。

結果在設計上就是暫定的：MAX2 安裝位置由人工輸入（相機中心在 LiDAR 座標中的公尺數；
以「前方沿 LiDAR +X、影像上方沿 +Z」為基準，yaw 轉向 LiDAR +Y、pitch 向上、
roll 繞相機前方軸），因此 rig 維持 `uncalibrated`；相機時鐘為錄影指令確認時間加上
影片時間，因此維持 `provisional`。GLIM 軌跡範圍外的點與全景會被排除，數量記在
`assembly.json`。

`splatrig max2-frames <file.360>` 由兩條 5888×1920 的 MAX2 影像軌（LEFT／FRONT／RIGHT
與 DOWN／BACK／TOP；切掉分割面中線處 64 px 的鏡頭重疊區）重建等角立方體，
並把每 N 個原始幀轉成 equirectangular 影像。時間戳取自封包 PTS，為整數奈秒的影片時間。
直接切割會在近物處留下明顯的拼接視差。

首次執行（桌上靜止、21 秒）：LiDAR 無空缺、GLIM 179 筆位姿、39 張全景中 33 張落在
軌跡內、198 張 cube 視角與暫定 COLMAP 模型，耗時 72 秒。以 LiDAR 反射率與全景亮度
相關性搜尋 yaw 的結果不明確（−25° 為 0.215、−135° 為 0.208），未採用。

### 規劃中的手持支架（v3，一體成形列印）

由支架 STL 量測（mm，Z 朝上，LiDAR 安裝面為 z = 0）：4 個 M3 孔構成 48 × 36 的孔位，
中心在 (0, 0)；兩根定位銷位於 (0, −16) 與 (0, 23)；MAX2 托座的 1/4" 孔位於 (0, 0)，
座面高 z = 95，兩道擋邊沿 X 方向內距 27.5，因此 MAX2 的鏡頭朝向 ±X。線頭托架與立柱在
−X 側。相機安裝點因此位於 LiDAR 安裝中心正上方。假設 Mid-360 點雲原點約在底面上方
47 mm、MAX2 光學中心約在底面上方 35 mm（兩者需以裝置機構圖確認），名目
`--camera-offset` 為 `0 0 0.083`（公尺），與哪顆鏡頭朝前無關。安裝後仍以
`calibrate-gyro` 求旋轉與每段錄製的時鐘。立柱位於 LiDAR 近距視野內，操作者也會出現在
每張全景中，因此訓練前需處理自身點雲與相機遮罩。

## MAX2 安裝的陀螺儀校正

將 MAX2 與 LiDAR 剛性固定在一起，錄約 30 秒：開頭與結尾保持靜止，中間繞三個軸
轉動，然後執行 `tools/splatrig-ros calibrate-gyro <capture> --output <dir>`。
MAX2 的 GPMF 陀螺儀（GYRO，約 790 Hz，ORIN `ZXY`）與 Mid-360S IMU（200 Hz）量測
同一個轉動：時鐘偏移以 |ω| 相關性最大化求得，再以旋轉後的向量殘差精修；旋轉為
Σ l cᵀ 的正交極分解因子，拒絕鏡射並回報最弱的奇異值。兩邊陀螺儀都先做 ±4 ms
視窗平均；單純線性內插會讓偏移偏向內插雜訊較小的取樣相位，誤差最多 1.6 ms。

MAX2 影像軸相對 GPMF 軸的關係已確認一次：在靜止全景中依四個水平候選方向投影
LiDAR 深度邊緣，結果為前方 = −Y、下方 = −Z、右方 = −X（儲存順序 Z、X、Y）。
IORI 全程為單位旋轉，代表儲存的影像方向固定於相機。

`process-capture --calibration calibration.json` 會使用該旋轉，並對每段錄製重新做
|ω| 相關以取得該段的時鐘偏移；相關性低於 0.9 時改用錄影指令時間並註明。
位移仍需人工輸入（`--camera-offset`），因此輸出仍為暫定。

第一個安裝的量測結果：|ω| 相關性 0.9996，殘差為訊號 RMS 的 2.2%，相機影片時間比
錄影指令確認晚 95.8 ms；最弱的軸幾乎沒有轉動（奇異值比 0.013）。合成資料下，
此方法求得的偏移誤差在 0.2 ms 內、旋轉誤差在 0.04° 內。目視檢查轉動過程中，
重投影的 LiDAR 深度邊緣都落在靜態結構上；這並非量化後的精度。

## 官方與原始來源

- [Livox MID-360 規格](https://www.livoxtech.com/mid-360)
- [Livox Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Livox 時間同步](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)
- [GLIM 感測器設定](https://github.com/koide3/glim/wiki/Sensor-setup-guide)
- [Open GoPro](https://gopro.github.io/OpenGoPro/) 與 [FAQ](https://gopro.github.io/OpenGoPro/docs/faq/)
- [Labs 發行說明](https://gopro.github.io/labs/control/notes/)
- [LiDAR／相機校正工具](https://github.com/koide3/direct_visual_lidar_calibration)
- [AnyReplica 照片／位姿驗證](https://github.com/KuoFengYuan/anyreplica/blob/main/docs/POSE_REFINEMENT.md)
