# 擷取資料格式

[English](FORMAT.md) | **繁體中文**

第 1 版格式是裝置擷取與離線處理的交界。目前接受已校正的 PINHOLE 視角 PNG，
尚不直接讀取原始 .360 影片。

## 掃描檔案

```text
capture/
├── capture-meta.json
├── rig.json
├── clock.json
├── trajectory.jsonl
├── frames.jsonl
├── points.jsonl
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

```text
T_world_camera(t) = T_world_rig(t) · T_rig_camera
p_world(t)        = T_world_rig(t) · T_rig_lidar · p_lidar(t)
```

平移使用線性插值，旋轉使用最短路徑 SLERP；不允許時間外插。
跨越超過 max_pose_gap_ns 的區段會被拒絕；直接查詢已有的時間點不需要插值。
閉環修正後，照片與點雲皆依同一版軌跡重算。

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

各相機深度、遮罩、360 投影轉換、特徵軌跡、畫質指標及訓練器執行屬於後續
里程碑。深度監督需要先將 LiDAR 距離轉為相機 Z 深度，並處理可見性遮罩。
