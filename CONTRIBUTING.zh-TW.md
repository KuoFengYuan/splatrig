# 參與開發

[English](CONTRIBUTING.md) | **繁體中文**

先閱讀 [AGENTS.md](AGENTS.zh-TW.md) 與[開發路線](docs/ROADMAP.zh-TW.md)。
每次變更保持範圍明確，提供最小可重現輸入。硬體工作優先保留時間戳、
frame 名稱、單位、校正來源與原始診斷。

啟用專案環境後執行：

```sh
python -m pip install -e .
python -m unittest discover -s tests -v
python tools/smoke.py
python tools/check_project.py
```

核心只使用標準函式庫，ROS 與其他裝置套件維持選用。英文文件提供完整繁中
對應。使用 Git 時，遵循 Feature/、Bugfix/、Enhance/ 任務分支規則。
任何遠端操作前，遵守使用者指定的本機交付範圍。

處理流程變更提供獨立幾何或檔案格式檢查，不能只看內部殘差降低。
硬體變更區分模擬訊息測試與實機掃描。記錄軟體／韌體版本、
拍攝設定、場景、執行時間、資料遺失與資源使用。

檢視器變更需測桌面、窄版、鍵盤操作及修改的控制項。
原始掃描與產生結果不放進追蹤原始碼；未取得資料發布授權前不上傳掃描。

貢獻內容依儲存庫的 [MIT 授權](LICENSE) 使用。
