# 擷取資料格式

[English](FORMAT.md) | **繁體中文**

第 1 版格式是裝置擷取與離線處理的交界。處理流程接受已校正的 PINHOLE 視角 PNG；
equirectangular 全景影像需先轉成這種視角。尚不解碼原始 .360 影片。

## 掃描檔案

```text
capture/
├── capture-meta.json
├── rig.json
├── clock.json
├── trajectory.jsonl
├── frames.jsonl
├── points.jsonl   （或 points.bin）
└── images/
```

demo 指令會建立完整範例。原始資料保持不變；重播與匯出使用不同的輸出資料夾，
並拒絕覆寫已有的結果。

`capture-meta.json`：

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

來源可為 synthetic、gopro_max2_mid360 或 external；軌跡狀態為 live 或 final。
最終軌跡必須來自真正修正過的歷史；只改標籤不會完成 SLAM 最佳化。

## 座標與校正

`T_A_B` 將 B 的點轉成 A 座標。世界 +Z 向上，相機採 OpenCV：
+X 向右、+Y 向下、+Z 向前。平移單位公尺。四元數為單位長度 wxyz，
與 ROS／SciPy 常見的 xyzw 序列化順序不同。

`rig.json` 包含狀態（uncalibrated 或 calibrated）、T_rig_lidar 與相機清單：

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

這些單位變換是格式示意，不是 MAX2／MID-360 校正值。rig 是輸入軌跡追蹤的
座標，通常是 IMU。虛擬相機外參包含固定視角旋轉與實體位置偏移。
目前格式假設輸出影像朝向固定；隨時間變化的防震必須先移除或明確建模。

## Equirectangular 全景影像

`splatrig panorama-views` 將全景來源轉成一般的第 1 版掃描，內含固定方向的
PINHOLE 視角。來源保留一般的中繼資料、時鐘、軌跡與點雲，不含 frames.jsonl，
並在 `rig.json` 中宣告空的相機清單與 panorama 項目：

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

`panoramas.jsonl` 每行包含正整數 id、相機時鐘的 timestamp_ns 及 image_path。
影像必須是 ffmpeg 可解碼的 2:1 equirectangular 檔案。

球面座標 S 使用 OpenCV 軸向：+Z 為影像中央欄的地平線方向，+X 為其右方 90°
（四分之三寬度處），+Y 朝下。W×H 影像的像素中心 (u+0.5, v+0.5) 對應經度
(u+0.5)/W·360° − 180° 與緯度 90° − (v+0.5)/H·180°。

```text
R_sphere_view = Ry(yaw) · Rx(pitch) · Rz(roll)
T_rig_camera  = T_rig_sphere · R_sphere_view
fx = fy = size / 2 / tan(fov / 2),  cx = cy = size / 2
```

yaw 為正時向右轉，pitch 為正時向上看，roll 為正時視角的 +X 軸轉向 +Y。
預設配置有 `cube`（前、右、後、左、上、下）與 `ring8`（每 45° 一個 yaw）；
`--views` 接受 JSON 清單，欄位為 name、yaw_deg、pitch_deg 及選填的 roll_deg。
每個視角成為一台相機。Frame ID 為 (全景 id − 1) × 視角數 + 視角序號，
capture_id 為全景 id，source_panorama_path 記錄來源。rig 狀態會沿用，
因此尚未校正的 T_rig_sphere 仍會阻擋正式匯出。

像素重取樣由 ffmpeg 的 v360 濾鏡執行，輸入會先轉為 8-bit RGB；ffmpeg 6.1
的 16-bit 雙線性內插在全白處會溢位。每次轉換前，SplatRig 會重取樣一張以像素
值編碼自身光線方向的全景圖；若安裝的 ffmpeg 最大誤差超過 0.6 個輸入像素，
或平均偏差超過 0.25 像素，就拒絕轉換。`panorama-views.json` 記錄 ffmpeg 版本
與探測結果。同一張全景的所有視角共用其時間戳。rolling shutter、拼接視差、
各視角曝光時間、鏡頭遮罩及隨時間變化的防震均未建模，也不解碼原始 .360／EAC 影片。

## 時間

全部時間為非負整數奈秒。相機與參考時鐘可能使用不同起點，`clock.json` 定義：

```text
reference_ns = reference_anchor_ns
             + round((camera_ns - camera_anchor_ns) * rate)
```

欄位為 camera_anchor_ns、reference_anchor_ns、rate，以及 provisional 或
calibrated 狀態。先減去錨點再做浮點運算，保留 epoch 時間的精度。
`fit-clock` 使用至少三組遞增的成對實體事件，擬合偏移與漂移並拒絕過大殘差。
預設最大殘差為 5 ms，這是軟體驗收設定，不代表已量得的 MAX2 同步精度。
需記錄事件涵蓋時間與殘差；事件只集中在開頭，不能證明長時間漂移已解決。
目前工具尚未自動從 GPMF 擷取這些事件。

預覽到達時間及共同錄影指令不是曝光時間。PTS、曝光中點及 rolling shutter
時間仍由各裝置轉接層處理。

## 軌跡、影像與點

每行軌跡包含參考時鐘的 timestamp_ns、revision、translation 與
quaternion_wxyz，表示 T_world_rig。時間嚴格遞增，所有 revision
必須等於 capture-meta.json 的 pose_revision。

每行影像包含正整數 id 與 camera_id、相機時鐘的 timestamp_ns，以及相對掃描
資料夾的 image_path。選用的 capture_id 可將同次曝光的虛擬視角分組。
路徑必須留在掃描資料夾內；PNG 標頭尺寸須符合相機校正。
標頭檢查不等於完整影像解碼驗證。

每行點資料包含參考時鐘的 timestamp_ns、LiDAR 座標的 xyz，以及選用的
整數 rgb（0～255）。沒有顏色時使用灰色。裝置轉接層必須保存逐點擷取時間，
不能將整個移動掃描都標成封包到達時間。

掃描可改用二進位 `points.bin` 取代 points.jsonl（不可兩者並存）。檔案開頭為
8 位元組識別碼 `SRPOINT1`、little-endian uint32 標頭長度、UTF-8 JSON 標頭，
並以零補齊到 8 位元組邊界。之後是固定 24 位元組的紀錄：int64 timestamp_ns、
LiDAR 座標 float32 x/y/z（公尺）、uint8 反射率、tag、line 及一個保留位元組。
標頭列出這些欄位；讀取器拒絕其他排列方式及不完整紀錄。二進位點使用預設灰色。

```text
T_world_camera(t) = T_world_rig(t) · T_rig_camera
p_world(t)        = T_world_rig(t) · T_rig_lidar · p_lidar(t)
```

平移使用線性插值，旋轉使用最短路徑 SLERP；不允許時間外插。
跨越超過 max_pose_gap_ns 的區段會被拒絕；直接查詢已有的時間點不需要插值。
閉環修正後，照片與點雲皆依同一版軌跡重算。

## LiDAR 錄製資料

`splatrig import-livox BAG --output DIR` 將 livox_ros_driver2 的 rosbag2 錄製
（CustomMsg 點雲與 sensor_msgs/Imu）轉為 LiDAR 錄製資料；這是後續組成掃描的輸入，
不是完整掃描：

```text
recording/
├── recording.json   來源、數量、空缺、時鐘證據與限制
├── points.bin       逐點時間 = timebase + offset_time；排除無回波 (0, 0, 0)
└── imu.jsonl        timestamp_ns、angular_velocity_rad_s、linear_acceleration
```

IMU 加速度單位依平均大小記錄為 `g` 或 `m/s^2`，數值不重新縮放。排除的無回波點、
幀與 IMU 空缺、重疊幀及未遞增的 IMU 時間戳都會計數，不會默默丟棄。

CustomMsg 不帶裝置的時間戳類型，而驅動在未同步時會改用主機收包時間。
因此匯入器從時間結構判斷時鐘：同步的時間戳讓封包邊界維持固定點距，
幀間隔集中在少數離散值；收包時間則帶有抖動。`ptp_inferred` 需要封包邊界集中度
至少 0.9、雙區間幀間隔集中度至少 0.8，且沒有倒退的偏移。其他狀態
（`host_arrival_suspected`、`indeterminate`）會停止匯入，除非以
`--allow-unsynced` 寫出標記為暫定的錄製資料。這是由資料推斷，並非裝置自身回報；
PTP 設定需另外記錄。

匯入器執行時需要 ROS 2 與驅動的訊息套件，請在 source 兩個環境後以系統 Python
執行。離線核心不匯入 ROS。

## 輸出與限制

重播輸出 PLY、image-poses.jsonl、report.json 與內嵌資料的瀏覽器預覽。
重播位姿紀錄的 source_image_path 相對原始掃描，不另外複製照片。
匯出則增加複製後的照片及 COLMAP 二進位／文字模型；image_path 相對匯出
資料夾，source_image_path 保存來源路徑。此紀錄為 c2w，COLMAP images
則儲存 w2c，不會隱含翻轉世界向上方向。

COLMAP 點的觀測軌跡及影像 POINTS2D 都是空陣列，是有效初始化資料，
不是可直接執行 bundle adjustment 的特徵對應。報告保存這項區別，
也保存暫定／合成資料狀態。請見 [COLMAP 格式](https://colmap.github.io/format.html)。

體素重播有可設定的佔用體素預算，超過時明確失敗，不是無上限的城市級串流
建圖器。軌跡與影像紀錄載入記憶體，預覽最多內嵌 20,000 個抽樣點。

各相機深度、遮罩、原始 .360／EAC 解碼、特徵軌跡、畫質指標及訓練器執行屬於後續
里程碑。深度監督需要先將 LiDAR 距離轉為相機 Z 深度，並處理可見性遮罩。
