# ================================
# Webster + PID Traffic Signal Control for Node2
# 根據論文：Networked Sensor-Based Adaptive Traffic Signal Control
# ================================

import os
import sys
import math

# 取得程式所在資料夾
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 建立 SUMO 路徑
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

import traci

# SUMO 設定檔
sumocfg_path = os.path.join(SCRIPT_DIR, 'Traci.sumocfg')

if not os.path.exists(sumocfg_path):
    sys.exit(f"找不到 SUMO 設定檔: {sumocfg_path}")

Sumo_config = [
    'sumo-gui',
    '-c', sumocfg_path,
    '--step-length', '0.10',
    '--delay', '0',
    '--lateral-resolution', '0'
]

# 啟動 SUMO
traci.start(Sumo_config)
traci.gui.setSchema("View #0", "real world")

TLS_ID = "Node2"

# ================================================
# Webster + PID 控制參數
# ================================================

# 時間參數
STEP_LENGTH = 0.1           # 秒/step
SIM_TIME = 1800.0           # 總模擬時間（秒）
TOTAL_STEPS = int(SIM_TIME / STEP_LENGTH)

# 控制參數
CONTROL_INTERVAL = 200      # 每 200 steps (20秒) 更新一次控制
T_CONTROL = CONTROL_INTERVAL * STEP_LENGTH  # 控制週期（秒）

# Webster 參數
SATURATION_FLOW_RATE = 1600.0  # veh/hr/lane（飽和流率）
NUM_LANES_EB = 3               # 東向車道數
NUM_LANES_SB = 3               # 南向車道數

# 週期與綠燈限制（與 PPO/Max Pressure 對齊）
MIN_CYCLE = 40.0               # 最小週期（秒）
MAX_CYCLE = 150.0              # 最大週期（秒）
MIN_GREEN = 10.0               # 最小綠燈時間（秒）← 與 PPO/Max Pressure 一致 (100 steps = 10秒)
YELLOW_TIME = 3.0              # 黃燈時間（秒）
LOST_PER_PHASE = 5.0           # 每相位損失時間（秒）

# PID 參數（針對 20秒 控制週期優化）
PID_Kp = 0.5                   # 比例增益
PID_Ti = 15.0                  # 積分時間常數
PID_Td = 4.0                   # 微分時間常數

# 相位定義（4 相位系統）
EB_PHASE = 0   # 東西向綠燈
SB_PHASE = 2   # 南北向綠燈

# ================================================
# 全域變數
# ================================================

# 控制週期內的統計數據
interval_stats = {
    'vehicle_count_EB': 0,
    'vehicle_count_SB': 0,
    'occupancy_sum_EB': 0.0,
    'occupancy_sum_SB': 0.0,
    'halting_sum_EB': 0,
    'halting_sum_SB': 0,
    'steps_count': 0
}
    
# PID 狀態
pid_state = {
    'e_prev': 0.0,
    'integral': 0.0
}

# 當前控制參數（東西向綠燈，南北向綠燈）
current_eb_green = 42.0    # EB 綠燈時間
current_sb_green = 42.0    # SB 綠燈時間

# 統計變數（整體）
sum_qEB = 0.0
sum_qSB = 0.0
sum_hEB = 0.0
sum_hSB = 0.0
sum_halting = 0.0
eb_green_steps = 0
sb_green_steps = 0
yellow_steps = 0
phase_counts = {}
total_arrived_vehicles = 0
total_steps = 0
cumulative_reward = 0.0

# Webster + PID 控制歷史
control_history = []


# ================================================
# 函數定義
# ================================================

def reset_interval_stats():
    """重置控制週期的統計數據"""
    return {
        'vehicle_count_EB': 0,
        'vehicle_count_SB': 0,
        'occupancy_sum_EB': 0.0,
        'occupancy_sum_SB': 0.0,
        'halting_sum_EB': 0,
        'halting_sum_SB': 0,
        'steps_count': 0
    }


def collect_statistics(stats):
    """收集當前 step 的統計數據"""
    # EB 方向（東向）
    q_EB_0 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_0")
    q_EB_1 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_1")
    q_EB_2 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_2")
    
    h_EB_0 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_0")
    h_EB_1 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_1")
    h_EB_2 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_2")
    
    # SB 方向（南向）
    q_SB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_0")
    q_SB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_1")
    q_SB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_2")
    
    h_SB_0 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_0")
    h_SB_1 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_1")
    h_SB_2 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_2")
    
    # 累積統計
    stats['vehicle_count_EB'] += (q_EB_0 + q_EB_1 + q_EB_2)
    stats['vehicle_count_SB'] += (q_SB_0 + q_SB_1 + q_SB_2)
    stats['halting_sum_EB'] += (h_EB_0 + h_EB_1 + h_EB_2)
    stats['halting_sum_SB'] += (h_SB_0 + h_SB_1 + h_SB_2)
    stats['steps_count'] += 1
    
    return (q_EB_0 + q_EB_1 + q_EB_2), (q_SB_0 + q_SB_1 + q_SB_2), \
           (h_EB_0 + h_EB_1 + h_EB_2), (h_SB_0 + h_SB_1 + h_SB_2)


def compute_averages(stats):
    """計算控制週期的平均值"""
    n = max(stats['steps_count'], 1)
    
    return {
        'avg_vehicle_EB': stats['vehicle_count_EB'] / n,
        'avg_vehicle_SB': stats['vehicle_count_SB'] / n,
        'avg_halting_EB': stats['halting_sum_EB'] / n,
        'avg_halting_SB': stats['halting_sum_SB'] / n,
    }


def compute_flow_rate(avg_vehicle_count, duration_sec):
    """
    計算流量（veh/hr）
    
    avg_vehicle_count: 平均車輛數
    duration_sec: 統計時長（秒）
    """
    if duration_sec <= 0:
        return 0.0
    
    # 轉換為每小時流量
    flow_rate = (avg_vehicle_count / duration_sec) * 3600.0
    return flow_rate


def compute_webster_timing(avg_data):
    """
    使用 Webster 方法計算週期和綠燈時間
    對於 4 相位系統，將總綠燈時間分配給 EB 和 SB
    
    回傳: (C_base, G_EB, G_SB, y_EB, y_SB, Y, flow_EB, flow_SB)
    """
    # 計算流量
    flow_EB = compute_flow_rate(avg_data['avg_vehicle_EB'], T_CONTROL)
    flow_SB = compute_flow_rate(avg_data['avg_vehicle_SB'], T_CONTROL)
    
    # 計算飽和流率（與 PPO/Max Pressure 一致）
    S_EB = SATURATION_FLOW_RATE * NUM_LANES_EB
    S_SB = SATURATION_FLOW_RATE * NUM_LANES_SB
    
    # 計算流量比
    y_EB = min(flow_EB / S_EB, 0.95) if S_EB > 0 else 0.1
    y_SB = min(flow_SB / S_SB, 0.95) if S_SB > 0 else 0.1
    
    # 總流量比
    Y = y_EB + y_SB
    
    # 避免過飽和
    if Y >= 0.99:
        Y = 0.99
    
    # 損失時間（2個主要相位 + 每個相位的黃燈過渡）
    # 4 相位系統：EB綠燈→黃燈→SB綠燈→黃燈，共 2 個黃燈過渡
    L = 2 * LOST_PER_PHASE
    
    # Webster 最佳週期公式
    C_base = (1.5 * L + 5.0) / max(1e-6, (1.0 - Y))
    
    # 限制週期範圍
    C_base = min(max(C_base, MIN_CYCLE), MAX_CYCLE)
    
    # 有效綠燈時間（扣除黃燈和損失時間）
    effective_green = C_base - L
    
    # 按流量比分配總綠燈時間
    if Y > 0:
        G_EB = (y_EB / Y) * effective_green
        G_SB = (y_SB / Y) * effective_green
    else:
        G_EB = effective_green / 2.0
        G_SB = effective_green / 2.0
    
    # 確保最小綠燈時間
    G_EB = max(G_EB, MIN_GREEN)
    G_SB = max(G_SB, MIN_GREEN)
    
    return C_base, G_EB, G_SB, y_EB, y_SB, Y, flow_EB, flow_SB


def pid_update(e_k, state, Kp, Ti, Td, T):
    """
    PID 控制器更新（位置式 PID）
    
    e_k: 當前誤差
    state: PID 狀態字典 {'e_prev', 'integral'}
    Kp, Ti, Td: PID 參數
    T: 控制週期（秒）
    
    回傳: u_k (控制輸出)
    """
    # 積分項
    state['integral'] += e_k * T
    
    # 防止積分飽和
    max_integral = 100.0
    state['integral'] = min(max(state['integral'], -max_integral), max_integral)
    
    # 微分項
    derivative = (e_k - state['e_prev']) / T if T > 0 else 0.0
    
    # PID 輸出
    u_k = Kp * (
        e_k + 
        (T / Ti) * state['integral'] + 
        Td * derivative
    )
    
    # 更新狀態
    state['e_prev'] = e_k
    
    return u_k


def apply_control_update(C_base, G_EB, G_SB, avg_data):
    """
    應用 Webster + PID 控制（4 相位系統）
    
    回傳: (C_new, G_EB_new, G_SB_new, u_k, U_current, e_k)
    """
    global current_eb_green, current_sb_green
    
    # 計算當前排隊長度指標（使用停等車輛數）
    U_current = (avg_data['avg_halting_EB'] + avg_data['avg_halting_SB']) / 2.0
    
    # 目標：無排隊
    U_target = 0.0
    
    # 誤差
    e_k = U_target - U_current
    
    # PID 計算調整量
    u_k = pid_update(e_k, pid_state, PID_Kp, PID_Ti, PID_Td, T_CONTROL)
    
    # 調整週期（限制調整幅度）
    max_adjustment = 30.0  # 最多調整 ±30 秒
    u_k = min(max(u_k, -max_adjustment), max_adjustment)
    
    C_new = C_base + u_k
    C_new = min(max(C_new, MIN_CYCLE), MAX_CYCLE)
    
    # 按比例調整所有綠燈時間
    if C_base > 0:
        scale = C_new / C_base
        G_EB_new = G_EB * scale
        G_SB_new = G_SB * scale
    else:
        G_EB_new = G_EB
        G_SB_new = G_SB
    
    # 更新全域變數
    current_eb_green = G_EB_new
    current_sb_green = G_SB_new
    
    return C_new, G_EB_new, G_SB_new, u_k, U_current, e_k



# ================================================
# 主控制迴圈
# ================================================

print("\n" + "="*70)
print("Starting Webster + PID Control Simulation (1800 sec)")
print("="*70 + "\n")

# 初始化統計
interval_stats = reset_interval_stats()

# 追蹤變數：記錄上一個step的相位，用來偵測相位切換
previous_phase = -1

for step in range(TOTAL_STEPS):
    # 1. 推進模擬
    traci.simulationStep()
    
    # 2. 讀取基本資訊
    current_phase = traci.trafficlight.getPhase(TLS_ID)
    ryg_state = traci.trafficlight.getRedYellowGreenState(TLS_ID)
    is_yellow = 'y' in ryg_state.lower()
    
    # 3. 偵測相位切換，立即設定綠燈時間
    if current_phase != previous_phase:
        if current_phase == EB_PHASE:  # Phase 0: EB 綠燈
            traci.trafficlight.setPhaseDuration(TLS_ID, current_eb_green)
        elif current_phase == SB_PHASE:  # Phase 2: SB 綠燈
            traci.trafficlight.setPhaseDuration(TLS_ID, current_sb_green)
        
        previous_phase = current_phase
    
    # 4. 收集統計數據
    q_EB, q_SB, h_EB, h_SB = collect_statistics(interval_stats)
    
    # 5. 全域統計
    if is_yellow:
        yellow_steps += 1
    
    if current_phase not in phase_counts:
        phase_counts[current_phase] = 0
    phase_counts[current_phase] += 1
    
    if current_phase == EB_PHASE:
        eb_green_steps += 1
    elif current_phase == SB_PHASE:
        sb_green_steps += 1
    
    total_arrived_vehicles += traci.simulation.getArrivedNumber()
    
    # 6. 累積統計
    sum_qEB += q_EB
    sum_qSB += q_SB
    sum_hEB += h_EB
    sum_hSB += h_SB
    sum_halting += (h_EB + h_SB)
    total_steps += 1
    
    # 7. 計算 reward (僅用於比較)
    reward = -(h_EB + h_SB)
    cumulative_reward += reward
    
    # 8. 定期執行 Webster + PID 控制
    if step > 0 and step % CONTROL_INTERVAL == 0:
        # 計算平均值
        avg_data = compute_averages(interval_stats)
        
        # 1. Webster 計算最佳週期和綠燈時間
        C_base, G_EB, G_SB, y_EB, y_SB, Y, flow_EB, flow_SB = compute_webster_timing(avg_data)
        
        # 2. PID 調整
        C_new, G_EB_new, G_SB_new, u_k, U_current, e_k = apply_control_update(
            C_base, G_EB, G_SB, avg_data
        )
        
        # 記錄控制歷史
        control_history.append({
            'step': step,
            'C_base': C_base,
            'C_new': C_new,
            'G_EB': G_EB_new,
            'G_SB': G_SB_new,
            'u_k': u_k,
            'error': e_k,
            'flow_EB': flow_EB,
            'flow_SB': flow_SB
        })
        
        # 重置統計
        interval_stats = reset_interval_stats()
        
        # 輸出日誌
        print(f"[Step {step}] Webster+PID Update:")
        print(f"  Flow: EB={flow_EB:.0f}, SB={flow_SB:.0f} (veh/hr)")
        print(f"  Webster: C={C_base:.1f}s, G_EB={G_EB:.1f}s, G_SB={G_SB:.1f}s")
        print(f"  PID: Error={e_k:.1f}, Adjustment={u_k:.1f}s")
        print(f"  Final: C={C_new:.1f}s, G_EB={G_EB_new:.1f}s, G_SB={G_SB_new:.1f}s")
        print("-" * 50)
    
    # 每 100 step 印一次狀態
    if step % 100 == 0:
        sim_time = step * STEP_LENGTH
        status_str = "Yellow" if is_yellow else "Green"
        print(f"[t={sim_time:4.1f}s] qEB={q_EB:3d}, qSB={q_SB:3d} | hEB={h_EB:3d}, hSB={h_SB:3d} ({status_str}), phase={current_phase}")

print("\nSimulation completed.\n")

# ====== 統計結果輸出（與 PPO/Max-Pressure 相同格式） ======
if total_steps > 0:
    avg_qEB = sum_qEB / total_steps
    avg_qSB = sum_qSB / total_steps
    avg_hEB = sum_hEB / total_steps
    avg_hSB = sum_hSB / total_steps

    eb_green_ratio = eb_green_steps / total_steps
    sb_green_ratio = sb_green_steps / total_steps

    # 總停等時間 = 累積的停等車輛數 * step_length
    total_waiting_time = sum_halting * STEP_LENGTH
    
    # 黃燈比例
    yellow_ratio = yellow_steps / total_steps
    
    # 平均延滯時間 (Average Delay Time) = 總停等時間 / 總到達車輛數
    if total_arrived_vehicles > 0:
        avg_delay_time = total_waiting_time / total_arrived_vehicles
    else:
        avg_delay_time = 0.0
        
    # 平均停等長度 (Average Queue Length) - 總平均
    avg_queue_total = (sum_qEB + sum_qSB) / total_steps

else:
    avg_qEB = avg_qSB = 0.0
    avg_hEB = avg_hSB = 0.0
    eb_green_ratio = sb_green_ratio = 0.0
    total_waiting_time = 0.0
    yellow_ratio = 0.0
    avg_delay_time = 0.0
    avg_queue_total = 0.0

print("\n========== Webster + PID Summary (Objective: Min Waiting Time) ==========")
print(f"EB 平均排隊 (Queue) = {avg_qEB:.2f} veh")
print(f"SB 平均排隊 (Queue) = {avg_qSB:.2f} veh")
print(f"Total Avg Queue     = {avg_queue_total:.2f} veh")
print(f"EB 平均停等 (Halt)  = {avg_hEB:.2f} veh")
print(f"SB 平均停等 (Halt)  = {avg_hSB:.2f} veh")
print("")
print(f"EB 綠燈比例         = {eb_green_ratio:.2f}")
print(f"SB 綠燈比例         = {sb_green_ratio:.2f}")
print(f"黃燈比例 (Yellow)   = {yellow_ratio:.2f}")
print("")
print("各時相比例 (Phase Ratios):")
for p_idx in sorted(phase_counts.keys()):
    ratio = phase_counts[p_idx] / total_steps
    print(f"  Phase {p_idx}: {ratio:.2f}")
print("")
print(f"Total Arrived Veh   = {total_arrived_vehicles}")
print(f"Avg Delay Time      = {avg_delay_time:.2f} sec/veh")
print(f"Total Waiting Time  = {total_waiting_time:.2f} veh·sec")
print(f"Cumulative reward   = {cumulative_reward:.2f}")
print("========================================================================\n")

