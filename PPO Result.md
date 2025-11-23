# PPO Simulation Results

## 11/18

### 第一次
```text
========== PPO Summary ==========
EB 平均排隊        = 42.40 veh
SB 平均排隊        = 43.34 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.25

Total queue time    = 154337.50 veh·sec
Cumulative reward   = -1543375.00
```

### 第二次
```text
========== PPO Summary ==========
EB 平均排隊        = 43.53 veh
SB 平均排隊        = 43.85 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.25

Total queue time    = 157300.30 veh·sec
Cumulative reward   = -1573003.00
```

### 第三次
```text
========== PPO Summary ==========
EB 平均排隊        = 41.93 veh
SB 平均排隊        = 44.85 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.25

Total queue time    = 156201.00 veh·sec
Cumulative reward   = -1562010.00
```

---

## 11/23更動內容 (Changes Made)

為了將優化目標從「降低排隊長度」改為「降低停等時間」，我們進行了以下修改：

1.  **狀態空間 (State Space) 擴增**：
    *   `STATE_DIM` 從 7 增加到 **13**。
    *   除了原本的車輛總數 (Queue)，新增了 **停等車輛數 (Halting)** 的資訊。
    *   State 現在包含：6 個車道的車輛數 + 6 個車道的停等數 + 1 個相位資訊。

2.  **獎勵函數 (Reward Function) 變更**：
    *   原本：`Reward = - (總排隊長度)`
    *   修改後：`Reward = - (總停等車輛數)`
    *   目標是讓 AI 更專注於減少車輛完全靜止的時間。

3.  **統計指標更新**：
    *   新增 `Total Waiting Time` (總停等時間) 作為主要成效指標。
    *   Log 輸出增加 `hEB` (東向停等數) 與 `hSB` (南向停等數) 的即時監控。

---

## 更新後的結果 (Objective: Min Waiting Time)
### Raw Output

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 43.54 veh
SB 平均排隊 (Queue) = 43.68 veh
EB 平均停等 (Halt)  = 33.72 veh
SB 平均停等 (Halt)  = 33.76 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.24

Total Waiting Time  = 121463.90 veh·sec
Cumulative reward   = -1214639.00
===============================================================
```

### 第二次測試 (Latest)

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 41.94 veh
SB 平均排隊 (Queue) = 43.60 veh
EB 平均停等 (Halt)  = 32.56 veh
SB 平均停等 (Halt)  = 33.68 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.25

Total Waiting Time  = 119230.90 veh·sec
Cumulative reward   = -1192309.00
===============================================================
```
### 第三次測試 (Latest)

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 44.30 veh
SB 平均排隊 (Queue) = 44.95 veh
EB 平均停等 (Halt)  = 34.24 veh
SB 平均停等 (Halt)  = 34.68 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.25

Total Waiting Time  = 124054.20 veh·sec
Cumulative reward   = -1240542.00
===============================================================
```
### 第四次測試 (更新輸出的格式)

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 42.57 veh
SB 平均排隊 (Queue) = 45.15 veh
Total Avg Queue     = 87.72 veh
EB 平均停等 (Halt)  = 32.99 veh
SB 平均停等 (Halt)  = 34.82 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.25
黃燈比例 (Yellow)   = 0.50

各時相比例 (Phase Ratios):
  Phase 0: 0.25
  Phase 1: 0.25
  Phase 2: 0.25
  Phase 3: 0.25

Total Arrived Veh   = 2233
Avg Delay Time      = 54.66 sec/veh
Total Waiting Time  = 122052.70 veh·sec
Cumulative reward   = -1220527.00
===============================================================
```
### 第五次測試 (修正最小綠燈時間)

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 43.97 veh
SB 平均排隊 (Queue) = 42.99 veh
Total Avg Queue     = 86.97 veh
EB 平均停等 (Halt)  = 34.33 veh
SB 平均停等 (Halt)  = 33.40 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.25
黃燈比例 (Yellow)   = 0.50

各時相比例 (Phase Ratios):
  Phase 0: 0.25
  Phase 1: 0.25
  Phase 2: 0.25
  Phase 3: 0.25

Total Arrived Veh   = 2243
Avg Delay Time      = 54.36 sec/veh
Total Waiting Time  = 121922.10 veh·sec
Cumulative reward   = -1219221.00
===============================================================
```
### 第六次測試 (修正黃燈邏輯)

**修正說明 (Fix Details):**

在前幾次測試中，我們發現「黃燈比例」異常高達 0.50，且各時相比例均為 0.25，顯示 AI 頻繁切換相位，且未能正確處理黃燈時間。為了修正此問題，我們對控制邏輯進行了重大重構：

1.  **區分黃燈與綠燈邏輯 (Separation of Logic)**：
    *   **黃燈期間 (Yellow Phase)**：程式不再讓 PPO 介入，而是強制執行固定的黃燈時間 (`YELLOW_DURATION_STEPS = 30`，即 3 秒)。一旦黃燈時間結束，程式會**自動切換**到下一個綠燈相位。
    *   **綠燈期間 (Green Phase)**：只有在綠燈期間，PPO 模型才會進行決策。PPO 必須遵守 `MIN_GREEN_STEPS` (10 秒) 的限制，只有在滿足最小綠燈時間後，才能選擇是否切換到下一個相位（變為黃燈）。

2.  **修正計時器 (Timer Fix)**：
    *   `last_switch_step` 計時器現在會在「切換至黃燈」和「切換至綠燈」時都正確更新，確保時間計算精確。

3.  **訓練優化 (Training Optimization)**：
    *   PPO 的訓練數據收集 (Buffer Storage) 現在僅在「綠燈期間」或「決策點」進行，避免將黃燈期間無法控制的狀態混入訓練數據中，讓 AI 更專注於學習綠燈時長的控制。

**結果 (Results):**

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 20.31 veh
SB 平均排隊 (Queue) = 20.75 veh
Total Avg Queue     = 41.06 veh
EB 平均停等 (Halt)  = 15.12 veh
SB 平均停等 (Halt)  = 15.45 veh

EB 綠燈比例         = 0.39
SB 綠燈比例         = 0.39
黃燈比例 (Yellow)   = 0.23

各時相比例 (Phase Ratios):
  Phase 0: 0.39
  Phase 1: 0.11
  Phase 2: 0.39
  Phase 3: 0.11

Total Arrived Veh   = 2575
Avg Delay Time      = 21.37 sec/veh
Total Waiting Time  = 55022.20 veh·sec
Cumulative reward   = -419121.00
===============================================================
```
### 第七次測試 

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 20.14 veh
SB 平均排隊 (Queue) = 20.85 veh
Total Avg Queue     = 40.99 veh
EB 平均停等 (Halt)  = 15.01 veh
SB 平均停等 (Halt)  = 15.52 veh

EB 綠燈比例         = 0.39
SB 綠燈比例         = 0.38
黃燈比例 (Yellow)   = 0.23

各時相比例 (Phase Ratios):
  Phase 0: 0.38
  Phase 1: 0.12
  Phase 2: 0.39
  Phase 3: 0.12

Total Arrived Veh   = 2585
Avg Delay Time      = 21.26 sec/veh
Total Waiting Time  = 54952.50 veh·sec
Cumulative reward   = -416311.00
===============================================================
```
### 第八次測試 

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 19.60 veh
SB 平均排隊 (Queue) = 20.41 veh
Total Avg Queue     = 40.01 veh
EB 平均停等 (Halt)  = 14.47 veh
SB 平均停等 (Halt)  = 15.20 veh

EB 綠燈比例         = 0.39
SB 綠燈比例         = 0.38
黃燈比例 (Yellow)   = 0.23

各時相比例 (Phase Ratios):
  Phase 0: 0.38
  Phase 1: 0.12
  Phase 2: 0.39
  Phase 3: 0.12

Total Arrived Veh   = 2577
```

### 第九次測試 

**修正說明 (Fix Details):**

為了進一步優化決策，我們嘗試將「黃燈期間的成本」納入 PPO 的決策考量中。
*   **黃燈成本 (Yellow Cost)**：當 AI 決定「切換 (Switch)」時，程式會模擬 3 秒鐘的黃燈過程，並將這段期間累積的所有停等車輛數 (Halting Number) 作為該次動作的負獎勵 (Negative Reward)。
*   **預期效果**：AI 應該會意識到「切換」是一個昂貴的動作（會浪費 3 秒通車時間），因此會更謹慎地選擇切換時機，避免頻繁切換。

**結果 (Results):**

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 25.73 veh
SB 平均排隊 (Queue) = 23.39 veh
Total Avg Queue     = 49.12 veh
EB 平均停等 (Halt)  = 20.35 veh
SB 平均停等 (Halt)  = 18.06 veh

EB 綠燈比例         = 0.41
SB 綠燈比例         = 0.43
黃燈比例 (Yellow)   = 0.16

各時相比例 (Phase Ratios):
  Phase 0: 0.43
  Phase 1: 0.08
  Phase 2: 0.41
  Phase 3: 0.08

Total Arrived Veh   = 2492
Avg Delay Time      = 27.89 sec/veh
Total Waiting Time  = 69508.60 veh·sec
Cumulative reward   = -692358.00
===============================================================
```

**分析 (Analysis):**
雖然黃燈比例進一步降低至 0.16 (因為 AI 更不願意切換)，但平均延滯時間 (27.89s) 卻比「固定黃燈但不計成本」的版本 (20.73s) 略高。這可能是因為 AI 過於保守，為了避免黃燈成本而過度延長綠燈時間，導致另一向車流等待過久。

### 第十次測試 (Include Yellow Cost - 2nd Run)

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 26.13 veh
SB 平均排隊 (Queue) = 24.08 veh
Total Avg Queue     = 50.21 veh
EB 平均停等 (Halt)  = 20.46 veh
SB 平均停等 (Halt)  = 18.96 veh

EB 綠燈比例         = 0.42
SB 綠燈比例         = 0.43
黃燈比例 (Yellow)   = 0.15

各時相比例 (Phase Ratios):
  Phase 0: 0.43
  Phase 1: 0.07
  Phase 2: 0.42
  Phase 3: 0.07

Total Arrived Veh   = 2441
Avg Delay Time      = 29.21 sec/veh
Total Waiting Time  = 71296.30 veh·sec
Cumulative reward   = -710414.00
===============================================================
```

### 第十一次測試 (Include Yellow Cost - 3rd Run)

```text
========== PPO Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 22.27 veh
SB 平均排隊 (Queue) = 24.71 veh
Total Avg Queue     = 46.98 veh
EB 平均停等 (Halt)  = 16.97 veh
SB 平均停等 (Halt)  = 19.86 veh

EB 綠燈比例         = 0.46
SB 綠燈比例         = 0.39
黃燈比例 (Yellow)   = 0.15

各時相比例 (Phase Ratios):
  Phase 0: 0.39
  Phase 1: 0.08
  Phase 2: 0.46
  Phase 3: 0.08

Total Arrived Veh   = 2418
Avg Delay Time      = 27.55 sec/veh
Total Waiting Time  = 66622.50 veh·sec
Cumulative reward   = -663678.00
===============================================================
```

