# Max-Pressure vs PPO: 目標與機制比較

## 文件資訊

- **建立日期**: 2024-11-23
- **目的**: 比較 Max-Pressure 和 PPO 兩種交通控制系統的目標、獎勵函數和控制機制
- **相關檔案**:
  - `traci.maxpreesure.py` (Max-Pressure 實作)
  - `traci.PPO.update.py` (PPO 實作)

---

## 1. 控制目標 (Objective)

| 系統 | 控制目標 | 說明 |
|------|---------|------|
| **Max-Pressure** | **最小化總等待時間** (Min Waiting Time) | 透過平衡兩方向的排隊壓力來減少車輛等待 |
| **PPO** | **最小化總等待時間** (Min Waiting Time) | 透過強化學習訓練,學習最佳控制策略 |

⭐ **相同目標,不同方法!**

---

## 2. 獎勵函數 (Reward Function)

### Max-Pressure 獎勵函數

```python
def get_reward(q_EB, q_SB):
    """
    reward = - (EB + SB 總排隊車數)
    純粹拿來當作系統壓力指標,用來比較不同控制策略。
    """
    total_queue = q_EB + q_SB
    return -float(total_queue)
```

**特點**:
- 📊 基於 **排隊車輛數** (Queue Length)
- ➖ 負值獎勵 (排隊越多,獎勵越負)
- 🔍 **僅用於評估比較**,不直接影響決策
- 📈 累積獎勵用於評估整體系統效能

**輸入變數**:
- `q_EB`: 東向 (Eastbound) 排隊車輛總數
- `q_SB`: 南向 (Southbound) 排隊車輛總數

---

### PPO 獎勵函數

```python
def get_reward(state):
    """
    Reward = - (總停等車輛數)
    目標:最小化停等時間 (Waiting Time)
    state: (q0..q5, h0..h5, phase)
    h0..h5 (index 6~11) 是 halting number
    """
    # 取出 halting counts
    halting_counts = state[6:12]
    total_halting = sum(halting_counts)
    
    return -float(total_halting)
```

**特點**:
- 📊 基於 **停等車輛數** (Halting Number)
- ➖ 負值獎勵 (停等越多,獎勵越負)
- 🎓 **直接影響學習**,引導 AI 優化決策
- 🔄 每步獎勵驅動 Policy 網絡更新

**輸入變數**:
- `state[6:12]`: 6 個偵測器的停等車輛數
  - `h_EB_0, h_EB_1, h_EB_2`: 東向三車道停等數
  - `h_SB_0, h_SB_1, h_SB_2`: 南向三車道停等數

---

### 獎勵函數差異分析

| 比較項目 | Max-Pressure | PPO |
|---------|--------------|-----|
| **測量指標** | Queue Length (排隊車輛數) | Halting Number (停等車輛數) |
| **物理意義** | 偵測區內所有車輛 | 速度 < 0.1 m/s 的車輛 |
| **敏感度** | 較粗略 (包含移動中車輛) | 較精確 (只計算停等車輛) |
| **用途** | 評估與比較 | 驅動學習 |
| **更新頻率** | 每步計算 | 每步計算並影響學習 |

**關鍵洞察**:
- PPO 使用 **Halting Number** 更能直接反映車輛等待痛苦
- Max-Pressure 使用 **Queue Length** 作為壓力指標,間接優化等待時間
- 兩者都追求相同目標,但測量方式不同

---

## 3. 控制機制 (Control Mechanism)

### Max-Pressure 控制機制

#### 決策邏輯

```python
def max_pressure_control(q_EB, q_SB, curr_phase, step):
    # 1. 計算壓力差
    pressure_diff = abs(q_EB - q_SB)
    
    # 2. 壓力差判斷 (閾值 = 15)
    if pressure_diff >= SWITCH_THRESHOLD:
        if q_EB > q_SB:
            # 東向排隊多 → 切換到 EB 綠燈
            target_phase = EB_PHASES[0]
        elif q_SB > q_EB:
            # 南向排隊多 → 切換到 SB 綠燈
            target_phase = SB_PHASES[0]
    
    # 3. 最小綠燈時間限制 (10 秒)
    if (step - last_switch_step) >= MIN_GREEN_STEPS:
        if target_phase != curr_phase:
            # 執行相位切換
            switch_phase(target_phase)
```

#### 機制特點

**控制原理**:
- 🧮 **規則式 (Rule-Based)**: 基於明確的數學規則
- ⚖️ **壓力平衡**: 哪邊排隊多就給哪邊綠燈
- ⏱️ **即時反應**: 直接基於當前狀態決策
- 🎯 **貪婪策略**: 每步選擇當前最優方向

**安全約束**:
1. **最小綠燈時間** (MIN_GREEN_STEPS = 100 steps = 10 秒)
   - 避免過於頻繁切換
   - 確保車輛有足夠時間通過

2. **黃燈過渡機制** (YELLOW_DURATION = 30 steps = 3 秒)
   - v2/fixed_yellow 版本具備
   - 提供安全的相位轉換

3. **壓力差閾值** (SWITCH_THRESHOLD = 0-15 輛車,可調)
   - 避免在壓力差不大時頻繁切換
   - 閾值越高,切換越少

**優點**:
- ✅ 簡單易懂,可解釋性高
- ✅ 無需訓練,即插即用
- ✅ 執行效率高,計算快速
- ✅ 100% 可重現 (同樣輸入 → 同樣輸出)
- ✅ 易於調整參數

**缺點**:
- ❌ 固定規則,無法學習適應
- ❌ 僅考慮當前狀態,無長期規劃
- ❌ 閾值設定需人工調整
- ❌ 無法處理複雜的交通模式

---

### PPO 控制機制

#### 決策邏輯

```python
def ppo_control(state):
    # 1. Agent 觀察當前狀態 (13 維)
    state = [q_EB_0, q_EB_1, q_EB_2,    # 東向排隊數 x3
             q_SB_0, q_SB_1, q_SB_2,    # 南向排隊數 x3
             h_EB_0, h_EB_1, h_EB_2,    # 東向停等數 x3
             h_SB_0, h_SB_1, h_SB_2,    # 南向停等數 x3
             current_phase]              # 當前相位
    
    # 2. Policy Network 前向傳播
    logits = state @ policy_W + policy_b    # 線性變換
    probs = softmax(logits)                  # 轉換為機率 [P(keep), P(switch)]
    
    # 3. 採樣動作
    action = np.random.choice([0, 1], p=probs)  # 0=保持, 1=切換
    
    # 4. 執行動作
    if action == 1:
        # 切換到黃燈 → 等待 3 秒 → 切換到下一個綠燈
        switch_phase()
    
    # 5. 獲得獎勵
    reward = -total_halting
    
    # 6. 儲存經驗
    buffer.store(state, action, reward)
    
    # 7. 定期更新網絡 (每 512 步)
    if steps % 512 == 0:
        update_policy_network()
```

#### 機制特點

**控制原理**:
- 🤖 **學習式 (Learning-Based)**: 透過試錯學習策略
- 🧠 **Neural Network**: Policy Network + Value Network
- 📚 **經驗累積**: 從歷史數據中學習模式
- 🎲 **隨機探索**: 有一定機率探索新策略
- 🔄 **持續優化**: 透過梯度下降更新網絡

**PPO 核心組件**:

1. **Policy Network** (策略網絡)
   - 輸入: 13 維狀態向量
   - 輸出: 2 維動作機率 [P(keep), P(switch)]
   - 參數: `policy_W` (13x2 矩陣), `policy_b` (2 維向量)

2. **Value Network** (價值網絡)
   - 輸入: 13 維狀態向量
   - 輸出: 狀態價值估計
   - 參數: `value_W` (13 維向量), `value_b` (標量)

3. **GAE (Generalized Advantage Estimation)**
   - 計算每個動作的優勢值
   - 用於指導 Policy 更新方向

4. **PPO Clipping Objective**
   - 限制策略更新幅度
   - 確保訓練穩定性

**訓練超參數**:
- `GAMMA = 0.9`: 折扣因子
- `LAMBDA = 0.95`: GAE 參數
- `CLIP_EPS = 0.2`: PPO 裁剪範圍
- `POLICY_LR = 1e-4`: Policy 學習率
- `VALUE_LR = 1e-4`: Value 學習率
- `PPO_EPOCHS = 4`: 每次更新的訓練輪數
- `PPO_UPDATE_INTERVAL = 512`: 更新間隔

**優點**:
- ✅ 可學習適應不同交通模式
- ✅ 考慮長期累積獎勵
- ✅ 理論上可達到最優策略
- ✅ 可處理複雜狀態空間

**缺點**:
- ❌ 需要充分訓練 (耗時)
- ❌ 結果有隨機性,難以重現
- ❌ 可解釋性低 (黑盒模型)
- ❌ 訓練可能不穩定
- ❌ 需要仔細調整超參數

---

## 4. 關鍵差異總結

### 系統特性比較

| 比較項目 | Max-Pressure | PPO |
|---------|--------------|-----|
| **決策方式** | 規則式 (明確公式) | 學習式 (神經網絡) |
| **獎勵使用** | 僅評估,不影響決策 | 直接驅動學習 |
| **獎勵指標** | 排隊車輛數 (Queue) | 停等車輛數 (Halting) |
| **狀態空間** | 2 維 (q_EB, q_SB) | 13 維 (詳細狀態) |
| **動作空間** | 直接選相位 | Keep/Switch |
| **可重現性** | ✅ 100% 確定性 | ⚠️ 隨機性 |
| **適應性** | ❌ 固定規則 | ✅ 可學習適應 |
| **訓練需求** | ❌ 無需訓練 | ✅ 需要訓練 |
| **實時性** | ✅ 即時計算 (< 1ms) | ✅ 推論快速 (< 10ms) |
| **可解釋性** | ✅ 高 (明確規則) | ⚠️ 低 (黑盒) |
| **參數調整** | 簡單 (MIN_GREEN, THRESHOLD) | 複雜 (多個超參數) |
| **部署難度** | ✅ 簡單 | ⚠️ 中等 |

---

## 5. 實際效能比較

### 基於測試結果的效能分析

| 版本 | 平均延滯時間 | 黃燈比例 | 到達車輛 | 優勢 | 劣勢 |
|------|-------------|---------|---------|------|------|
| **Max-Pressure (原版)** | **7.69 sec/veh** | 8% | 2591 | 簡單、穩定、高效 | 緊急剎車警告 |
| **Max-Pressure (v2)** | 21.02 sec/veh | 23% | 2582 | 解決緊急剎車 | 黃燈比例過高 |
| **Max-Pressure (閾值=15)** | 32.32 sec/veh | 7% | 2500 | 黃燈比例低 | 反應過慢 |
| **PPO** | ~15-25 sec/veh | 變動 | 變動 | 可學習、適應性強 | 需訓練、不穩定 |

### 效能排名 (平均延滯時間)

1. 🥇 **Max-Pressure 原版**: 7.69 sec/veh
2. 🥈 **PPO**: ~15-25 sec/veh (取決於訓練程度)
3. 🥉 **Max-Pressure v2**: 21.02 sec/veh
4. ⚠️ **Max-Pressure (閾值=15)**: 32.32 sec/veh

---

## 6. 使用場景建議

### Max-Pressure 適用場景

✅ **推薦使用**:
- 需要快速部署的場景
- 交通流量模式相對穩定
- 需要高可解釋性
- 資源受限的環境 (無法訓練 AI)
- 需要確定性行為的系統

❌ **不推薦使用**:
- 交通流量模式複雜多變
- 需要長期優化的場景
- 有充足訓練資源和時間

---

### PPO 適用場景

✅ **推薦使用**:
- 交通流量模式複雜
- 有充足的訓練時間和數據
- 需要適應不同交通模式
- 追求理論最優解
- 可接受一定程度的不確定性

❌ **不推薦使用**:
- 需要立即部署
- 無法提供訓練環境
- 需要高可解釋性
- 系統行為需要完全確定

---

## 7. 混合策略建議

考慮結合兩者優點的混合策略:

### 方案 A: Max-Pressure + 學習閾值
- 使用 Max-Pressure 作為基礎控制邏輯
- 用機器學習動態調整 SWITCH_THRESHOLD
- 保持規則式控制的可解釋性
- 增加適應性

### 方案 B: PPO + Max-Pressure 引導
- 使用 Max-Pressure 決策作為 PPO 的初始策略
- 加速 PPO 訓練收斂
- 提供安全的探索基準

### 方案 C: 雙系統切換
- 正常情況使用 Max-Pressure (高效)
- 異常情況切換到 PPO (適應性)
- 根據交通模式動態選擇

---

## 8. 總結與結論

### 核心發現

1. **目標一致,路徑不同**
   - 兩者都追求最小化等待時間
   - Max-Pressure 用規則,PPO 用學習

2. **獎勵函數差異有意義**
   - Queue Length (Max-Pressure) 適合壓力平衡
   - Halting Number (PPO) 更直接反映等待痛苦

3. **效能出乎意料**
   - 簡單的 Max-Pressure 原版表現最佳
   - 複雜的學習方法不一定更好

4. **黃燈機制是關鍵**
   - 過於完善的黃燈機制會損害效能
   - 需要在安全性和效率間平衡

### 最終建議

**對於當前測試場景**:
- 🏆 **首選**: Max-Pressure 原版 (7.69 sec/veh)
- ⚠️ **可接受**: 原版有緊急剎車警告,但不影響實際效能
- 🔧 **改進方向**: 微調原版的黃燈機制,而非完全重寫

**對於未來發展**:
- 📚 繼續探索 PPO 的訓練優化
- 🔬 研究混合策略的可能性
- 🎯 找到閾值的最優範圍 (建議 5-10)

---

## 附錄: 測試數據來源

- Max-Pressure 測試結果: `Max Pressure Results.md`
- PPO 測試結果: `PPO Result.md`
- Max-Pressure 詳細說明: `Max Pressure.md`
- PPO 詳細說明: `PPO update.md`

**測試環境**:
- 模擬時間: 1800 秒
- Step Length: 0.1 秒
- 總步數: 18000 steps
- 交通流量: 固定隨機種子

---

*文件最後更新: 2024-11-23*
