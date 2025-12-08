# ================================
# 固定時制 4 相位交通號誌 (含左轉保護)
# ================================

import os
import sys
from collections import defaultdict

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
# 參數設定
# ================================================

STEP_LENGTH = 0.1           # 秒/step
SIM_TIME = 1800.0           # 總模擬時間（秒）
TOTAL_STEPS = int(SIM_TIME / STEP_LENGTH)

# 定義 4 相位循環 (含黃燈)
# 索引對應:
# SB(0-4), WB(5-9), NB(10-14), EB(15-19)
# Straight: GGGGr (Right+Straight Green, Left Red)
# Left: rrrrG (Left Green, others Red)

PHASE_CYCLE = [
    # Phase 0: NS Straight (Green) - 30s
    {"duration": 300, "state": "GGGGrrrrrrGGGGrrrrrr", "name": "NS Straight"},
    # Phase 1: NS Straight (Yellow) - 3s
    {"duration": 30,  "state": "yyyyrrrrrryyyyrrrrrr", "name": "NS Straight (Y)"},
    
    # Phase 2: NS Left (Green) - 15s
    {"duration": 150, "state": "rrrrGrrrrrrrrrGrrrrr", "name": "NS Left"},
    # Phase 3: NS Left (Yellow) - 3s
    {"duration": 30,  "state": "rrrryrrrrrrrrryrrrrr", "name": "NS Left (Y)"},
    
    # Phase 4: EW Straight (Green) - 30s
    # 0-4(SB): rrrrr, 5-9(WB): GGGGr, 10-14(NB): rrrrr, 15-19(EB): GGGGr
    {"duration": 300, "state": "rrrrrGGGGrrrrrGGGGrr", "name": "EW Straight"}, 
    
    # Phase 5: EW Straight (Yellow) - 3s
    {"duration": 30,  "state": "rrrrryyyyrrrrryyyyrr", "name": "EW Straight (Y)"},
    
    # Phase 6: EW Left (Green) - 15s
    {"duration": 150, "state": "rrrrrrrrrGrrrrrrrrrG", "name": "EW Left"},
    # Phase 7: EW Left (Yellow) - 3s
    {"duration": 30,  "state": "rrrrrrrrryrrrrrrrrry", "name": "EW Left (Y)"}
]

# 統計變數
sum_qEB = 0.0
sum_qSB = 0.0
sum_qWB = 0.0
sum_qNB = 0.0
sum_hEB = 0.0
sum_hSB = 0.0
sum_hWB = 0.0
sum_hNB = 0.0
sum_halting = 0.0
total_arrived_vehicles = 0
total_steps = 0
cumulative_reward = 0.0

# 計算每個時相佔的比例
phase_counts = defaultdict(int)

# ================================================
# 主迴圈
# ================================================

print("\n" + "="*70)
print("Starting Fixed 4-Phase (Protected Left) Simulation (1800 sec)")
print("="*70 + "\n")

current_phase_idx = 0
steps_in_phase = 0

# 設定初始相位
traci.trafficlight.setRedYellowGreenState(TLS_ID, PHASE_CYCLE[current_phase_idx]["state"])

for step in range(TOTAL_STEPS):
    # 推進模擬
    traci.simulationStep()
    
    # 更新相位計時與切換
    steps_in_phase += 1
    if steps_in_phase >= PHASE_CYCLE[current_phase_idx]["duration"]:
        current_phase_idx = (current_phase_idx + 1) % len(PHASE_CYCLE)
        steps_in_phase = 0
        traci.trafficlight.setRedYellowGreenState(TLS_ID, PHASE_CYCLE[current_phase_idx]["state"])
    
    # 強制執行車道紀律（每 10 steps = 1秒 執行一次）
    if step % 10 == 0:
        target_edges = ["Node1_2_EB", "Node2_3_WB", "Node2_4_SB", "Node2_5_NB"]
        for edge in target_edges:
            veh_ids = traci.edge.getLastStepVehicleIDs(edge)
            for veh_id in veh_ids:
                try:
                    route = traci.vehicle.getRoute(veh_id)
                    if len(route) < 2:
                        continue
                        
                    try:
                        current_index = route.index(edge)
                    except ValueError:
                        continue
                        
                    if current_index + 1 >= len(route):
                        continue
                        
                    next_edge = route[current_index + 1]
                    current_lane_index = traci.vehicle.getLaneIndex(veh_id)
                    
                    # 判斷轉向類型
                    is_straight = (
                        (edge == "Node1_2_EB" and next_edge == "Node2_3_EB") or
                        (edge == "Node2_3_WB" and next_edge == "Node1_2_WB") or
                        (edge == "Node2_4_SB" and next_edge == "Node2_5_SB") or
                        (edge == "Node2_5_NB" and next_edge == "Node2_4_NB")
                    )
                    
                    is_left_turn = (
                        (edge == "Node1_2_EB" and next_edge == "Node2_4_NB") or
                        (edge == "Node2_3_WB" and next_edge == "Node2_5_SB") or
                        (edge == "Node2_4_SB" and next_edge == "Node2_3_EB") or
                        (edge == "Node2_5_NB" and next_edge == "Node1_2_WB")
                    )
                    
                    # 強制執行車道紀律
                    if is_straight and current_lane_index == 2:
                        traci.vehicle.changeLane(veh_id, 1, 3.0)
                    elif is_left_turn and current_lane_index != 2:
                        traci.vehicle.changeLane(veh_id, 2, 3.0)
                        
                except Exception:
                    pass
    
    # 收集統計數據
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
    
    # WB 方向（西向）
    q_WB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_3_WB_0")
    q_WB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_3_WB_1")
    q_WB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_3_WB_2")
    
    h_WB_0 = traci.lanearea.getLastStepHaltingNumber("Node2_3_WB_0")
    h_WB_1 = traci.lanearea.getLastStepHaltingNumber("Node2_3_WB_1")
    h_WB_2 = traci.lanearea.getLastStepHaltingNumber("Node2_3_WB_2")
    
    # NB 方向（北向）
    q_NB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_5_NB_0")
    q_NB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_5_NB_1")
    q_NB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_5_NB_2")
    
    h_NB_0 = traci.lanearea.getLastStepHaltingNumber("Node2_5_NB_0")
    h_NB_1 = traci.lanearea.getLastStepHaltingNumber("Node2_5_NB_1")
    h_NB_2 = traci.lanearea.getLastStepHaltingNumber("Node2_5_NB_2")
    
    # 累積統計
    q_EB = q_EB_0 + q_EB_1 + q_EB_2
    q_SB = q_SB_0 + q_SB_1 + q_SB_2
    q_WB = q_WB_0 + q_WB_1 + q_WB_2
    q_NB = q_NB_0 + q_NB_1 + q_NB_2
    
    h_EB = h_EB_0 + h_EB_1 + h_EB_2
    h_SB = h_SB_0 + h_SB_1 + h_SB_2
    h_WB = h_WB_0 + h_WB_1 + h_WB_2
    h_NB = h_NB_0 + h_NB_1 + h_NB_2
    
    sum_qEB += q_EB
    sum_qSB += q_SB
    sum_qWB += q_WB
    sum_qNB += q_NB
    sum_hEB += h_EB
    sum_hSB += h_SB
    sum_hWB += h_WB
    sum_hNB += h_NB
    sum_halting += (h_EB + h_SB + h_WB + h_NB)
    total_steps += 1
    
    total_arrived_vehicles += traci.simulation.getArrivedNumber()
    
    # 計算 reward
    reward = -(h_EB + h_SB + h_WB + h_NB)
    cumulative_reward += reward
    
    # 統計時相佔比
    phase_counts[PHASE_CYCLE[current_phase_idx]["name"]] += 1

    # 每 100 step 印一次狀態
    if step % 100 == 0:
        sim_time = step * STEP_LENGTH
        print(f"[t={sim_time:4.1f}s] Phase: {PHASE_CYCLE[current_phase_idx]['name']} | qEB={q_EB:3d}, qWB={q_WB:3d}, qSB={q_SB:3d}, qNB={q_NB:3d}")

print("\nSimulation completed.\n")

# ====== 統計結果輸出 ======
if total_steps > 0:
    avg_qEB = sum_qEB / total_steps
    avg_qSB = sum_qSB / total_steps
    avg_qWB = sum_qWB / total_steps
    avg_qNB = sum_qNB / total_steps
    avg_hEB = sum_hEB / total_steps
    avg_hSB = sum_hSB / total_steps
    avg_hWB = sum_hWB / total_steps
    avg_hNB = sum_hNB / total_steps

    # 輸出每個時相的佔比
    print("\n=== Phase Proportions ===")
    for name, cnt in phase_counts.items():
        proportion = cnt / total_steps * 100 if total_steps > 0 else 0
        print(f"{name}: {proportion:.2f}% ({cnt} steps)")

    total_waiting_time = sum_halting * STEP_LENGTH
    
    if total_arrived_vehicles > 0:
        avg_delay_time = total_waiting_time / total_arrived_vehicles
    else:
        avg_delay_time = 0.0
        
    avg_queue_total = (sum_qEB + sum_qSB + sum_qWB + sum_qNB) / total_steps

else:
    avg_qEB = avg_qSB = avg_qWB = avg_qNB = 0.0
    avg_hEB = avg_hSB = avg_hWB = avg_hNB = 0.0
    total_waiting_time = 0.0
    avg_delay_time = 0.0
    avg_queue_total = 0.0

print("\n========== Fixed 4-Phase Traffic Signal Summary (Protected Left) ==========")
print(f"EB 平均排隊 (Queue) = {avg_qEB:.2f} veh")
print(f"WB 平均排隊 (Queue) = {avg_qWB:.2f} veh")
print(f"SB 平均排隊 (Queue) = {avg_qSB:.2f} veh")
print(f"NB 平均排隊 (Queue) = {avg_qNB:.2f} veh")
print(f"Total Avg Queue     = {avg_queue_total:.2f} veh")
print(f"EB 平均停等 (Halt)  = {avg_hEB:.2f} veh")
print(f"WB 平均停等 (Halt)  = {avg_hWB:.2f} veh")
print(f"SB 平均停等 (Halt)  = {avg_hSB:.2f} veh")
print(f"NB 平均停等 (Halt)  = {avg_hNB:.2f} veh")
print("")
print(f"Total Arrived Veh   = {total_arrived_vehicles}")
print(f"Avg Delay Time      = {avg_delay_time:.2f} sec/veh")
print(f"Total Waiting Time  = {total_waiting_time:.2f} veh·sec")
print(f"Cumulative reward   = {cumulative_reward:.2f}")
print("==========================================================================\n")

traci.close()
