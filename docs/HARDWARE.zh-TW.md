# 硬體整合

[English](HARDWARE.md) | **繁體中文**

目標為一台 GoPro MAX2 與一顆 Livox MID-360，手持使用，涵蓋室內外。
本儲存庫目前尚未完成實機擷取驗證。

## 各元件負責的工作

| 元件 | 工作 | 目前狀態 |
| --- | --- | --- |
| MID-360 與內建 IMU | 帶時間戳的點與慣性資料 | 外部驅動 |
| Livox Driver 2 | 裝置傳輸，保留逐點時間 | 待整合 |
| GLIM | 即時點雲、rig 軌跡與全域最佳化 | 外部相依工具 |
| ROS 里程計記錄器 | 記錄指定 Odometry topic | 已實作；已測訊息轉換，尚未測即時 ROS |
| MAX2 轉接層 | 預覽／控制、原始錄影與時間資訊 | 規劃中 |
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

## 室內外驗收掃描

使用小房間、長走廊、建築外圍及室內走到室外的連續路線；涵蓋重訪、
轉彎、快慢移動、素面牆、玻璃與移動物體。記錄封包／影像遺失、校時殘差、
延遲、軌跡缺口、閉環修正與資源使用。

影像重投影、保留影像的渲染品質及 LiDAR 表面對齊分別驗證。
模型外觀不代表尺寸精準，AnyReplica 的位姿實驗可作為這項區分的參考。

## 官方與原始來源

- [Livox MID-360 規格](https://www.livoxtech.com/mid-360)
- [Livox Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Livox 時間同步](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)
- [GLIM 感測器設定](https://github.com/koide3/glim/wiki/Sensor-setup-guide)
- [Open GoPro](https://gopro.github.io/OpenGoPro/) 與 [FAQ](https://gopro.github.io/OpenGoPro/docs/faq/)
- [Labs 發行說明](https://gopro.github.io/labs/control/notes/)
- [LiDAR／相機校正工具](https://github.com/koide3/direct_visual_lidar_calibration)
- [AnyReplica 照片／位姿驗證](https://github.com/KuoFengYuan/anyreplica/blob/main/docs/POSE_REFINEMENT.md)
