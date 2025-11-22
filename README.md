# Traffic Signal Control with SUMO & RL

這是一個使用 [SUMO (Simulation of Urban MObility)](https://eclipse.dev/sumo/) 和 TraCI (Traffic Control Interface) 進行交通號誌控制模擬的專案。本專案實作並比較了多種交通號誌控制演算法，包括傳統方法與強化學習 (Reinforcement Learning) 方法。

## 專案內容

本專案包含以下控制演算法的實作：

- **Fixed Time (FT)**: 固定時制控制 (`traci5.FT.py`)
- **Max Pressure**: 最大壓力控制 (`traci.maxpreesure.py`)
- **Q-Learning (QL)**: 基於 Q-Table 的強化學習 (`traci6.QL.py`)
- **Deep Q-Learning (DQL)**: 深度 Q 網路 (`traci7.DQL.py`)
- **Proximal Policy Optimization (PPO)**: 近端策略最佳化 (`traci.PPO.py`)

## 環境需求

在執行此專案之前，請確保您的系統已安裝以下軟體：

1.  **SUMO**: 請至 [SUMO 下載頁面](https://eclipse.dev/sumo/userdoc/Downloads.html) 安裝最新版本。
2.  **Python 3.x**: 建議使用 Python 3.8 或以上版本。
3.  **Python 套件**:
    - `numpy`
    - `traci` (通常包含在 SUMO 安裝中，或透過 `pip install traci` 安裝)
    - (選用) `torch` 或 `tensorflow` (如果 DQL/PPO 實作有依賴特定深度學習框架，目前 PPO 範例為 NumPy 實作)

## 安裝與設定

1.  **設定環境變數**:
    確保 `SUMO_HOME` 環境變數已正確設定為您的 SUMO 安裝路徑。
    
    *Windows 範例:*
    ```cmd
    set SUMO_HOME=C:\Program Files (x86)\Eclipse\Sumo
    ```

2.  **安裝依賴套件**:
    ```bash
    pip install numpy traci
    ```

## 執行方式

直接執行對應的 Python 腳本即可開始模擬。模擬將會啟動 SUMO GUI 並展示交通流動與號誌控制情形。

### 執行 PPO 控制
```bash
python traci.PPO.py
```

### 執行其他演算法
```bash
python traci.maxpreesure.py  # Max Pressure
python traci5.FT.py          # Fixed Time
python traci6.QL.py          # Q-Learning
python traci7.DQL.py         # Deep Q-Learning
```

## 檔案結構說明

- `Traci.sumocfg`: SUMO 主要設定檔，定義了路網、車流與附加檔案。
- `Traci.net.xml`: 路網檔案 (Network)，定義了道路、路口與號誌。
- `Traci.rou.xml`: 車流檔案 (Routes)，定義了車輛的產生與路徑。
- `Traci.add.xml`: 附加檔案 (Additional)，通常包含偵測器 (Detectors) 設定。
- `*.py`: 各種控制演算法的實作腳本。
- `e2_*.xml`: 模擬產生的偵測器輸出數據 (執行後產生)。

## 模擬設定

- **模擬時間**: 預設為 1800 秒 (可於腳本中 `SIM_TIME` 修改)。
- **步長 (Step Length)**: 0.1 秒。
- **獎勵函數 (Reward Function)**: 基於路口總排隊長度的負值 (Minimize Queue Length)。

## 結果輸出

每次模擬結束後，程式會在終端機輸出統計摘要，包含：
- 平均排隊長度 (Average Queue Length)
- 綠燈比例 (Green Light Ratio)
- 總延滯時間 (Total Queue Time)
- 累積獎勵 (Cumulative Reward)
