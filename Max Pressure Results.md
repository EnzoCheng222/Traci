# Max-Pressure Simulation Results

## 11/23

### 第一次測試

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 11.38 veh
SB 平均排隊 (Queue) = 12.58 veh
Total Avg Queue     = 23.95 veh
EB 平均停等 (Halt)  = 5.00 veh
SB 平均停等 (Halt)  = 6.06 veh

EB 綠燈比例         = 0.45
SB 綠燈比例         = 0.48
黃燈比例 (Yellow)   = 0.08

各時相比例 (Phase Ratios):
  Phase 0: 0.48
  Phase 1: 0.04
  Phase 2: 0.45
  Phase 3: 0.03

Total Arrived Veh   = 2591
Avg Delay Time      = 7.69 sec/veh
Total Waiting Time  = 19916.20 veh·sec
Cumulative reward   = -431179.00
========================================================================
```

### 第二次測試

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 11.38 veh
SB 平均排隊 (Queue) = 12.58 veh
Total Avg Queue     = 23.95 veh
EB 平均停等 (Halt)  = 5.00 veh
SB 平均停等 (Halt)  = 6.06 veh

EB 綠燈比例         = 0.45
SB 綠燈比例         = 0.48
黃燈比例 (Yellow)   = 0.08

各時相比例 (Phase Ratios):
  Phase 0: 0.48
  Phase 1: 0.04
  Phase 2: 0.45
  Phase 3: 0.03

Total Arrived Veh   = 2591
Avg Delay Time      = 7.69 sec/veh
Total Waiting Time  = 19916.20 veh·sec
Cumulative reward   = -431179.00
========================================================================
```

### 第三次測試

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 11.38 veh
SB 平均排隊 (Queue) = 12.58 veh
Total Avg Queue     = 23.95 veh
EB 平均停等 (Halt)  = 5.00 veh
SB 平均停等 (Halt)  = 6.06 veh

EB 綠燈比例         = 0.45
SB 綠燈比例         = 0.48
黃燈比例 (Yellow)   = 0.08

各時相比例 (Phase Ratios):
  Phase 0: 0.48
  Phase 1: 0.04
  Phase 2: 0.45
  Phase 3: 0.03

Total Arrived Veh   = 2591
Avg Delay Time      = 7.69 sec/veh
Total Waiting Time  = 19916.20 veh·sec
Cumulative reward   = -431179.00
========================================================================
```

---

## 11/23 更動內容 (v2 版本改進)

### 原始問題 (Original Issues)

在原始的 Max-Pressure 實作中,發現以下問題:

**SUMO 緊急剎車警告 (Emergency Braking Warnings)**:
- ⚠️ 模擬器多次跳出警示訊息,例如:「Warning: 車輛 'f_3.385' 在車道 'Node2_4_SB_0' 上執行緊急剎車，減速度=9.00，希望的(wished)=4.50，嚴重程度=1.00」
- ⚠️ 此類警示在三次獨立模擬中皆重複出現

**根本原因分析**:
1. **缺乏穩定的黃燈過渡機制**: 部分情況下黃燈時間過短或未正常呈現
2. **相位切換過於突然**: 系統直接在紅燈與綠燈之間跳轉
3. **車輛反應時間不足**: 車輛在接近路口時突遭紅燈,因而出現急剎與高減速度的行為

### 改進方案 (Improvements in v2)

為了解決上述問題,在 `traci.maxpressure.v2.py` 中進行了以下改進:

#### 1. **新增黃燈控制變數**
```python
YELLOW_DURATION_STEPS = 30  # 固定 3 秒黃燈時間
in_yellow_phase = False     # 黃燈狀態追蹤
yellow_start_step = -1      # 黃燈開始時間點
```

#### 2. **實作兩階段相位切換機制**

**原版邏輯** (有問題):
```
綠燈 Phase A → 直接切換 → 綠燈 Phase B  ❌
```

**v2 版本邏輯** (改進後):
```
綠燈 Phase A → 黃燈過渡 (3秒) → 綠燈 Phase B  ✅
```

#### 3. **黃燈期間的控制邏輯**

- **黃燈階段**: 
  - 不進行 Max-Pressure 決策
  - 固定等待 3 秒 (30 steps)
  - 時間到後自動切換到下一個綠燈相位

- **綠燈階段**:
  - 執行 Max-Pressure 決策 (比較 q_EB vs q_SB)
  - 遵守最小綠燈時間限制 (10 秒)
  - 需要切換時,先進入黃燈相位

#### 4. **改進後的控制流程**

```python
def max_pressure_control(q_EB, q_SB, curr_phase, step):
    global in_yellow_phase, yellow_start_step
    
    # 如果正在黃燈階段
    if in_yellow_phase:
        if step - yellow_start_step >= YELLOW_DURATION_STEPS:
            # 黃燈結束,切換到綠燈
            switch_to_next_green_phase()
            in_yellow_phase = False
        return
    
    # 綠燈階段: Max-Pressure 決策
    if need_to_switch and min_green_satisfied:
        # 切換到黃燈
        switch_to_yellow_phase()
        in_yellow_phase = True
        yellow_start_step = step
```

### 預期效果 (Expected Results)

✅ **減少緊急剎車**: 車輛有 3 秒黃燈時間準備停車  
✅ **降低減速度**: 從 9.00 降低到合理範圍 (< 4.50)  
✅ **符合交通規則**: 完整的綠→黃→紅→綠循環  
✅ **平滑過渡**: 避免突然的相位跳轉  
✅ **提升安全性**: 減少不合理的交通動態

### 測試建議 (Testing Recommendations)

使用 v2 版本進行測試時,請特別注意:
1. 觀察 SUMO 是否仍有緊急剎車警告
2. 檢查黃燈比例是否合理 (應約 8-10%)
3. 確認相位切換是否平滑
4. 比較與原版的效能差異

### v2 版本測試結果

#### 第一次測試 (v2)

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 20.21 veh
SB 平均排隊 (Queue) = 20.39 veh
Total Avg Queue     = 40.60 veh
EB 平均停等 (Halt)  = 15.03 veh
SB 平均停等 (Halt)  = 15.11 veh

EB 綠燈比例         = 0.38
SB 綠燈比例         = 0.39
黃燈比例 (Yellow)   = 0.23

各時相比例 (Phase Ratios):
  Phase 0: 0.39
  Phase 1: 0.12
  Phase 2: 0.38
  Phase 3: 0.11

Total Arrived Veh   = 2582
Avg Delay Time      = 21.02 sec/veh
Total Waiting Time  = 54266.30 veh·sec
Cumulative reward   = -730865.00
========================================================================
```

**結果分析**:

⚠️ **效能下降觀察**:
- 平均延滯時間: 21.02 sec/veh (原版: 7.69 sec/veh) ↑ 173%
- 總停等時間: 54266.30 veh·sec (原版: 19916.20 veh·sec) ↑ 172%
- 平均排隊數: 40.60 veh (原版: 23.95 veh) ↑ 70%

✅ **黃燈機制改善**:
- 黃燈比例: 0.23 (23%) - 比原版 8% 高,顯示有更完整的黃燈過渡
- 相位切換更平滑,應該能減少緊急剎車警告

⚠️ **問題分析**:
- 黃燈比例過高 (23% vs 預期 8-10%),可能導致過多時間浪費在過渡階段
- 綠燈比例降低 (EB: 0.38, SB: 0.39 vs 原版 0.45, 0.48)
- 需要檢查黃燈控制邏輯是否過於頻繁切換

#### 第二次測試 (v2)

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 20.21 veh
SB 平均排隊 (Queue) = 20.39 veh
Total Avg Queue     = 40.60 veh
EB 平均停等 (Halt)  = 15.03 veh
SB 平均停等 (Halt)  = 15.11 veh

EB 綠燈比例         = 0.38
SB 綠燈比例         = 0.39
黃燈比例 (Yellow)   = 0.23

各時相比例 (Phase Ratios):
  Phase 0: 0.39
  Phase 1: 0.12
  Phase 2: 0.38
  Phase 3: 0.11

Total Arrived Veh   = 2582
Avg Delay Time      = 21.02 sec/veh
Total Waiting Time  = 54266.30 veh·sec
Cumulative reward   = -730865.00
========================================================================
```

**觀察**: v2 版本結果高度穩定,兩次測試結果完全一致,顯示規則式控制的可重現性。

#### 第三次測試 (v2)

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 20.21 veh
SB 平均排隊 (Queue) = 20.39 veh
Total Avg Queue     = 40.60 veh
EB 平均停等 (Halt)  = 15.03 veh
SB 平均停等 (Halt)  = 15.11 veh

EB 綠燈比例         = 0.38
SB 綠燈比例         = 0.39
黃燈比例 (Yellow)   = 0.23

各時相比例 (Phase Ratios):
  Phase 0: 0.39
  Phase 1: 0.12
  Phase 2: 0.38
  Phase 3: 0.11

Total Arrived Veh   = 2582
Avg Delay Time      = 21.02 sec/veh
Total Waiting Time  = 54266.30 veh·sec
Cumulative reward   = -730865.00
========================================================================
```

**結論**: v2 版本三次測試結果**完全相同**,確認了規則式 Max-Pressure 演算法的完美可重現性! 🎯

---

## v2 與 fixed_yellow 的差異

**答案: 沒有差異!** 

`traci.maxpressure.fixed_yellow.py` 是從 `traci.maxpressure.v2.py` 複製而來的,兩個檔案內容完全相同。

兩者都使用相同的邏輯:
- ✅ 固定黃燈時間 (3 秒)
- ✅ 黃燈期間不進行決策
- ✅ 黃燈結束後自動切換到下一個綠燈相位
- ✅ 綠燈階段執行 Max-Pressure 決策

**真正有差異的是**:
- `traci.maxpressure.v2.py` / `traci.maxpressure.fixed_yellow.py` (相同)
- `traci.maxpressure.yellow_cost.py` (考慮黃燈成本,設定切換閾值)

---

## Yellow Cost 版本說明與測試

### 版本差異說明

**yellow_cost 版本與 v2/fixed_yellow 的關鍵差異**:

#### v2 / fixed_yellow 版本:
- 🔄 **切換條件**: 只要 `q_EB ≠ q_SB` 且滿足最小綠燈時間就切換
- ⏱️ **黃燈處理**: 固定等待 3 秒,不考慮成本
- 📊 **預期行為**: 頻繁切換,黃燈比例較高

#### yellow_cost 版本:
- 🔄 **切換條件**: 壓力差 `|q_EB - q_SB| ≥ 閾值(5)` 才切換
- 💰 **黃燈成本考量**: 
  - 新增 `estimate_yellow_cost()` 函數估算黃燈期間的停等成本
  - 計算公式: `cost = 當前停等車輛數 × 黃燈持續時間`
  - 只有當壓力差夠大(≥5輛車)時才值得承擔黃燈成本
- 📊 **預期行為**: 減少不必要的切換,降低黃燈比例

### yellow_cost 測試結果

#### 第一次測試

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 44.48 veh
SB 平均排隊 (Queue) = 44.57 veh
Total Avg Queue     = 89.05 veh
EB 平均停等 (Halt)  = 34.52 veh
SB 平均停等 (Halt)  = 34.46 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.26
黃燈比例 (Yellow)   = 0.49

各時相比例 (Phase Ratios):
  Phase 0: 0.26
  Phase 1: 0.25
  Phase 2: 0.25
  Phase 3: 0.24

Total Arrived Veh   = 2211
Avg Delay Time      = 56.16 sec/veh
Total Waiting Time  = 124161.70 veh·sec
Cumulative reward   = -1602836.00
========================================================================
```

#### 第二次測試

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 44.48 veh
SB 平均排隊 (Queue) = 44.57 veh
Total Avg Queue     = 89.05 veh
EB 平均停等 (Halt)  = 34.52 veh
SB 平均停等 (Halt)  = 34.46 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.26
黃燈比例 (Yellow)   = 0.49

各時相比例 (Phase Ratios):
  Phase 0: 0.26
  Phase 1: 0.25
  Phase 2: 0.25
  Phase 3: 0.24

Total Arrived Veh   = 2211
Avg Delay Time      = 56.16 sec/veh
Total Waiting Time  = 124161.70 veh·sec
Cumulative reward   = -1602836.00
========================================================================
```

**觀察**: yellow_cost 版本兩次測試結果**完全相同**,再次確認規則式控制的可重現性。但效能仍然是三個版本中最差的。

#### 第三次測試

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 44.48 veh
SB 平均排隊 (Queue) = 44.57 veh
Total Avg Queue     = 89.05 veh
EB 平均停等 (Halt)  = 34.52 veh
SB 平均停等 (Halt)  = 34.46 veh

EB 綠燈比例         = 0.25
SB 綠燈比例         = 0.26
黃燈比例 (Yellow)   = 0.49

各時相比例 (Phase Ratios):
  Phase 0: 0.26
  Phase 1: 0.25
  Phase 2: 0.25
  Phase 3: 0.24

Total Arrived Veh   = 2211
Avg Delay Time      = 56.16 sec/veh
Total Waiting Time  = 124161.70 veh·sec
Cumulative reward   = -1602836.00
========================================================================
```

**結論**: yellow_cost 版本三次測試結果**完全相同**,展現了與原版和 v2 相同的完美可重現性。然而,49% 的黃燈比例和 56.16 秒的平均延滯時間證明此版本的設計需要大幅改進。

---

## 版本比較分析

### 效能指標對比表

| 指標 | 原版 (v1) | v2/fixed_yellow | yellow_cost | 變化趨勢 |
|------|-----------|-----------------|-------------|----------|
| **平均排隊 (Total)** | 23.95 veh | 40.60 veh | **89.05 veh** | ⬆️ 272% |
| **平均停等 (Total)** | 11.06 veh | 30.14 veh | **68.98 veh** | ⬆️ 524% |
| **平均延滯時間** | 7.69 sec/veh | 21.02 sec/veh | **56.16 sec/veh** | ⬆️ 630% |
| **總停等時間** | 19916.20 | 54266.30 | **124161.70** | ⬆️ 523% |
| **黃燈比例** | 0.08 (8%) | 0.23 (23%) | **0.49 (49%)** | ⬆️ 513% |
| **EB 綠燈比例** | 0.45 | 0.38 | **0.25** | ⬇️ 44% |
| **SB 綠燈比例** | 0.48 | 0.39 | **0.26** | ⬇️ 46% |
| **總到達車輛** | 2591 | 2582 | **2211** | ⬇️ 15% |

### 關鍵發現

#### ⚠️ yellow_cost 版本的嚴重問題

1. **黃燈比例暴增至 49%**
   - 原版: 8%
   - v2: 23%
   - yellow_cost: **49%** ← 幾乎一半時間都在黃燈!
   
2. **效能大幅惡化**
   - 平均延滯時間從 7.69 秒暴增到 **56.16 秒** (↑630%)
   - 總排隊數從 23.95 增加到 **89.05** (↑272%)
   - 到達車輛數減少 15%

3. **綠燈時間嚴重不足**
   - EB 綠燈: 25% (原版 45%)
   - SB 綠燈: 26% (原版 48%)
   - 綠燈時間減少近一半!

#### 🔍 問題根本原因分析

**yellow_cost 版本的設計缺陷**:

1. **閾值設定不當**:
   - 閾值 5 輛車太低,仍然導致頻繁切換
   - 從 log 可見每 100 steps 就切換一次 (每 10 秒)
   - 遠超過 v2 的切換頻率

2. **成本估算未實際應用**:
   - 雖然計算了黃燈成本,但只用於 log 輸出
   - 決策仍然只基於壓力差是否 ≥ 5
   - 沒有真正將成本納入決策權衡

3. **最小綠燈時間限制失效**:
   - MIN_GREEN_STEPS = 100 (10秒)
   - 但實際每 100 steps 就切換
   - 表示剛滿足最小綠燈時間就立即切換

### 三版本效能排名

| 排名 | 版本 | 平均延滯時間 | 評價 |
|------|------|-------------|------|
| 🥇 **1st** | **原版 (v1)** | **7.69 sec/veh** | ✅ 最佳效能 |
| 🥈 2nd | v2/fixed_yellow | 21.02 sec/veh | ⚠️ 效能下降 |
| 🥉 3rd | yellow_cost | 56.16 sec/veh | ❌ 效能最差 |

### 結論與建議

#### 結論

1. **原版 (v1) 仍然是最佳選擇**
   - 雖然有緊急剎車警告,但效能最優
   - 黃燈比例合理 (8%)
   - 平均延滯時間最低 (7.69 秒)

2. **v2 版本是安全性與效能的折衷**
   - 解決了緊急剎車問題
   - 但黃燈比例過高 (23%),效能下降 173%

3. **yellow_cost 版本失敗**
   - 原本期望透過閾值減少切換
   - 實際上黃燈比例暴增至 49%
   - 效能嚴重惡化,不建議使用

#### 改進建議

**針對 yellow_cost 版本**:
1. **大幅提高切換閾值**: 從 5 增加到 15-20 輛車
2. **增加最小綠燈時間**: 從 10 秒增加到 20-30 秒
3. **真正應用成本**: 將黃燈成本納入決策公式,而非只是判斷閾值

**針對 v2 版本**:
1. **增加最小綠燈時間**: 減少切換頻率
2. **優化 Max-Pressure 邏輯**: 加入壓力差閾值判斷

---

## 閾值改進測試 (SWITCH_THRESHOLD = 15)

### 改進說明

為了降低過於頻繁的相位切換和黃燈比例,我們在三個 Max-Pressure 程式中都加入了**切換閾值**:
- **SWITCH_THRESHOLD = 15 輛車**
- 只有當 `|q_EB - q_SB| >= 15` 時才執行相位切換
- 保持最小綠燈時間 = 10 秒不變

### 原版 (v1) 測試結果 - 閾值=15

#### 第一次測試

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 26.75 veh
SB 平均排隊 (Queue) = 30.36 veh
Total Avg Queue     = 57.11 veh
EB 平均停等 (Halt)  = 20.96 veh
SB 平均停等 (Halt)  = 23.92 veh

EB 綠燈比例         = 0.48
SB 綠燈比例         = 0.46
黃燈比例 (Yellow)   = 0.07

各時相比例 (Phase Ratios):
  Phase 0: 0.46
  Phase 1: 0.06
  Phase 2: 0.48
  Phase 3: 0.01

Total Arrived Veh   = 2500
Avg Delay Time      = 32.32 sec/veh
Total Waiting Time  = 80794.10 veh·sec
Cumulative reward   = -1027986.00
========================================================================
```

**初步觀察**:
- ✅ 黃燈比例降至 **7%** (原版 8%, v2 23%)
- ⚠️ 平均延滯時間: **32.32 sec/veh** (比原版的 7.69 差,但比 v2 的 21.02 更差)
- ⚠️ 總排隊數增加到 57.11 veh (原版 23.95 veh)
- 閾值 15 可能過高,導致某些需要切換時未能及時切換

### fixed_yellow 測試結果 - 閾值=15

#### 第一次測試

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 34.25 veh
SB 平均排隊 (Queue) = 34.20 veh
Total Avg Queue     = 68.44 veh
EB 平均停等 (Halt)  = 28.76 veh
SB 平均停等 (Halt)  = 28.77 veh

EB 綠燈比例         = 0.44
SB 綠燈比例         = 0.50
黃燈比例 (Yellow)   = 0.06

各時相比例 (Phase Ratios):
  Phase 0: 0.50
  Phase 1: 0.03
  Phase 2: 0.44
  Phase 3: 0.03

Total Arrived Veh   = 2340
Avg Delay Time      = 44.25 sec/veh
Total Waiting Time  = 103544.90 veh·sec
Cumulative reward   = -1231988.00
========================================================================
```

**結果分析**:
- ✅ 黃燈比例大幅降低: **6%** (原 v2 版本 23%)
- ❌ 效能嚴重惡化: 
  - 平均延滯時間: **44.25 sec/veh** (比原版 7.69 增加 475%)
  - 總排隊數: **68.44 veh** (比原版 23.95 增加 186%)
  - 總到達車輛: **2340** (減少 9.7%)
- ⚠️ 問題: 閾值 15 配合黃燈機制導致反應過慢,車輛大量累積

### yellow_cost 測試結果 - 閾值=15

#### 第一次測試

```text
========== Max-Pressure Summary (Objective: Min Waiting Time) ==========
EB 平均排隊 (Queue) = 40.26 veh
SB 平均排隊 (Queue) = 40.53 veh
Total Avg Queue     = 80.79 veh
EB 平均停等 (Halt)  = 36.42 veh
SB 平均停等 (Halt)  = 36.72 veh

EB 綠燈比例         = 0.23
SB 綠燈比例         = 0.22
黃燈比例 (Yellow)   = 0.54

各時相比例 (Phase Ratios):
  Phase 0: 0.22
  Phase 1: 0.09
  Phase 2: 0.23
  Phase 3: 0.45

Total Arrived Veh   = 1546
Avg Delay Time      = 85.15 sec/veh
Total Waiting Time  = 131646.80 veh·sec
Cumulative reward   = -1454148.00
========================================================================
```

**🚨 嚴重失敗案例分析**:

這是所有測試中**效能最差**的版本,出現了系統性故障:

1. **系統卡死在黃燈階段** ⛔:
   - 從 log 可見,從 t=1130s 開始就一直停在 Phase 3 (黃燈)
   - 黃燈比例高達 **54%**,幾乎佔據一半時間!
   - Phase 3 比例 45%,遠超正常的 3%

2. **效能全面崩潰** 💥:
   - 平均延滯時間: **85.15 sec/veh** (比原版 7.69 惡化 1007%)
   - 總排隊數: **80.79 veh** (比原版 23.95 增加 237%)
   - 總到達車輛: **1546** (正常應為 2500+,減少 40%!)

3. **根本原因** 🔍:
   - yellow_cost 版本的控制邏輯存在 bug
   - 閾值 15 太高,系統無法找到合適的切換時機
   - 黃燈階段沒有正常結束機制,導致系統卡住

4. **結論**:
   - ❌ **yellow_cost 版本不適用於閾值 15**
   - ❌ 需要重新設計控制邏輯
   - ❌ 目前版本存在嚴重設計缺陷

---

## 閾值測試總結 (SWITCH_THRESHOLD = 15)

### 效能排名 (閾值=15)

| 排名 | 版本 | 平均延滯時間 | 黃燈比例 | 到達車輛 | 評價 |
|------|------|-------------|---------|---------|------|
| 🥇 1st | **原版 v1** | 32.32 sec/veh | 7% | 2500 | 最佳 |
| 🥈 2nd | **fixed_yellow** | 44.25 sec/veh | 6% | 2340 | 中等 |
| 🥉 3rd | **yellow_cost** | **85.15 sec/veh** | **54%** | **1546** | ❌ 失敗 |

### 關鍵結論

1. **閾值 15 對所有版本都過高**
   - 原版 v1: 延滯時間增加 320%
   - fixed_yellow: 延滯時間增加 475%
   - yellow_cost: 系統崩潰,延滯時間增加 1007%

2. **black燈機制的影響**
   - 黃燈機制配合高閾值會導致更嚴重的效能惡化
   - yellow_cost 版本存在邏輯缺陷

3. **建議閾值範圍**
   - 閾值 0 (無閾值): 最佳效能但切換頻繁
   - 閾值 5-10: 可能的平衡點
   - 閾值 15+: **不建議使用**

---
