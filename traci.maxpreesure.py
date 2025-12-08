# ================================
# Max-Pressure Traffic Light Control (Protected Left, 8 Phases, Safe)
# ================================

import os
import sys
from collections import defaultdict

# ★ Script directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Step 1: SUMO_HOME
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 2: import TraCI
import traci

# Step 3: Load SUMO config
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

TLS_ID = "Node2"

# -------------------------
# 模擬參數
# -------------------------

STEP_LENGTH = 0.10
SIM_TIME = 1800.0
TOTAL_STEPS = int(SIM_TIME / STEP_LENGTH)

# PHASE_CYCLE（完全由程式端控制，不使用 SUMO 內建號誌程序）
# 索引對應: SB(0-4), WB(5-9), NB(10-14), EB(15-19)
PHASE_CYCLE = [
    # 0: NS Straight (Green)
    {"duration": 300, "state": "GGGGrrrrrrGGGGrrrrrr", "name": "NS Straight"},
    # 1: NS Straight (Yellow)
    {"duration": 30,  "state": "yyyyrrrrrryyyyrrrrrr", "name": "NS Straight (Y)"},
    # 2: NS Left (Green)
    {"duration": 150, "state": "rrrrGrrrrrrrrrGrrrrr", "name": "NS Left"},
    # 3: NS Left (Yellow)
    {"duration": 30,  "state": "rrrryrrrrrrrrryrrrrr", "name": "NS Left (Y)"},
    # 4: EW Straight (Green)
    {"duration": 300, "state": "rrrrrGGGGrrrrrGGGGrr", "name": "EW Straight"},
    # 5: EW Straight (Yellow)
    {"duration": 30,  "state": "rrrrryyyyrrrrryyyyrr", "name": "EW Straight (Y)"},
    # 6: EW Left (Green)
    {"duration": 150, "state": "rrrrrrrrrGrrrrrrrrrG", "name": "EW Left"},
    # 7: EW Left (Yellow)
    {"duration": 30,  "state": "rrrrrrrrryrrrrrrrrry", "name": "EW Left (Y)"}
]

# NS / EW group 定義
NS_GROUP_PHASES = [0, 1, 2, 3]
EW_GROUP_PHASES = [4, 5, 6, 7]

# 壓力差門檻
PRESSURE_DIFF_THRESHOLD = 10  # vehicles

# -------------------------
# 工具函式
# -------------------------

def enforce_lane_discipline():
    """依照固定規則強制車道切換（左轉/直行）。"""
    edges = ["Node1_2_EB", "Node2_3_WB", "Node2_4_SB", "Node2_5_NB"]
    for edge in edges:
        for vid in traci.edge.getLastStepVehicleIDs(edge):
            try:
                route = traci.vehicle.getRoute(vid)
                if len(route) < 2:
                    continue
                cur_idx = route.index(edge)
                if cur_idx + 1 >= len(route):
                    continue
                nxt = route[cur_idx + 1]
                lane = traci.vehicle.getLaneIndex(vid)

                # 直行判斷
                straight = (
                    (edge == "Node1_2_EB" and nxt == "Node2_3_EB") or
                    (edge == "Node2_3_WB" and nxt == "Node1_2_WB") or
                    (edge == "Node2_4_SB" and nxt == "Node2_5_SB") or
                    (edge == "Node2_5_NB" and nxt == "Node2_4_NB")
                )
                # 左轉判斷
                left = (
                    (edge == "Node1_2_EB" and nxt == "Node2_4_NB") or
                    (edge == "Node2_3_WB" and nxt == "Node2_5_SB") or
                    (edge == "Node2_4_SB" and nxt == "Node2_3_EB") or
                    (edge == "Node2_5_NB" and nxt == "Node1_2_WB")
                )

                if straight and lane == 2:
                    traci.vehicle.changeLane(vid, 1, 3.0)
                elif left and lane != 2:
                    traci.vehicle.changeLane(vid, 2, 3.0)
            except Exception:
                pass


def get_queues_and_halting():
    """讀取四個方向的 queue / halting，回傳 dict。"""
    # EB
    q_EB = sum(traci.lanearea.getLastStepVehicleNumber(f"Node1_2_EB_{i}") for i in range(3))
    h_EB = sum(traci.lanearea.getLastStepHaltingNumber(f"Node1_2_EB_{i}") for i in range(3))

    # SB
    q_SB = sum(traci.lanearea.getLastStepVehicleNumber(f"Node2_4_SB_{i}") for i in range(3))
    h_SB = sum(traci.lanearea.getLastStepHaltingNumber(f"Node2_4_SB_{i}") for i in range(3))

    # WB
    q_WB = sum(traci.lanearea.getLastStepVehicleNumber(f"Node2_3_WB_{i}") for i in range(3))
    h_WB = sum(traci.lanearea.getLastStepHaltingNumber(f"Node2_3_WB_{i}") for i in range(3))

    # NB
    q_NB = sum(traci.lanearea.getLastStepVehicleNumber(f"Node2_5_NB_{i}") for i in range(3))
    h_NB = sum(traci.lanearea.getLastStepHaltingNumber(f"Node2_5_NB_{i}") for i in range(3))

    return {
        "q_EB": q_EB, "h_EB": h_EB,
        "q_SB": q_SB, "h_SB": h_SB,
        "q_WB": q_WB, "h_WB": h_WB,
        "q_NB": q_NB, "h_NB": h_NB,
    }


def get_reward_from_halting(stats):
    """reward = - (全部停等車輛數)，目標：最小化停等時間。"""
    return -float(stats["h_EB"] + stats["h_SB"] + stats["h_WB"] + stats["h_NB"])


# -------------------------
# 主程式：Max-Pressure 控制
# -------------------------

def main():
    traci.start(Sumo_config)
    try:
        traci.gui.setSchema("View #0", "real world")
    except Exception:
        pass

    print("\n=== Starting Max-Pressure (Protected Left, 8 Phases) Simulation (1800 sec) ===\n")

    # 初始狀態：先跑 NS group
    current_group = "NS"  # 或 "EW"
    current_phase_idx = NS_GROUP_PHASES[0]
    steps_in_phase = 0

    traci.trafficlight.setRedYellowGreenState(TLS_ID, PHASE_CYCLE[current_phase_idx]["state"])

    # 統計變數
    sum_qEB = sum_qSB = 0.0
    sum_hEB = sum_hSB = 0.0
    sum_halting = 0.0

    total_arrived_vehicles = 0
    total_steps = 0
    cumulative_reward = 0.0

    yellow_steps = 0
    phase_counts = defaultdict(int)
    eb_green_steps = 0
    sb_green_steps = 0

    for step in range(TOTAL_STEPS):
        # 每秒強制一次車道紀律
        if step % int(1.0 / STEP_LENGTH) == 0:
            enforce_lane_discipline()

        # 推進 SUMO
        traci.simulationStep()

        # 讀取 queue / halting
        stats = get_queues_and_halting()
        q_EB = stats["q_EB"]
        q_SB = stats["q_SB"]
        q_WB = stats["q_WB"]
        q_NB = stats["q_NB"]
        h_EB = stats["h_EB"]
        h_SB = stats["h_SB"]
        h_WB = stats["h_WB"]
        h_NB = stats["h_NB"]

        # 累計
        sum_qEB += q_EB
        sum_qSB += q_SB
        sum_hEB += h_EB
        sum_hSB += h_SB
        sum_halting += (h_EB + h_SB + h_WB + h_NB)
        total_arrived_vehicles += traci.simulation.getArrivedNumber()
        total_steps += 1

        # Reward
        reward = get_reward_from_halting(stats)
        cumulative_reward += reward

        # Phase 統計
        phase_name = PHASE_CYCLE[current_phase_idx]["name"]
        phase_counts[phase_name] += 1

        # 黃燈判斷
        if current_phase_idx in [1, 3, 5, 7]:
            yellow_steps += 1

        # 綠燈比例：SB 看 NS Straight / NS Left；EB 看 EW Straight / EW Left
        if current_phase_idx in [0, 2]:
            sb_green_steps += 1
        if current_phase_idx in [4, 6]:
            eb_green_steps += 1

        # 每 100 step 印一次
        if step % 100 == 0:
            sim_time = step * STEP_LENGTH
            print(f"[t={sim_time:4.1f}s] Phase: {phase_name} | "
                  f"qEB={q_EB:3d}, qWB={q_WB:3d}, qSB={q_SB:3d}, qNB={q_NB:3d}")

        # 更新相位時間
        steps_in_phase += 1
        phase_duration = PHASE_CYCLE[current_phase_idx]["duration"]

        if steps_in_phase >= phase_duration:
            # 結束當前 phase
            steps_in_phase = 0

            # 判斷是否為 group 結尾（NS Left (Y) or EW Left (Y)）
            if current_phase_idx in [3, 7]:
                # 計算 NS / EW 壓力
                q_NS = q_SB + q_NB
                q_EW = q_EB + q_WB

                if q_NS - q_EW > PRESSURE_DIFF_THRESHOLD:
                    next_group = "NS"
                elif q_EW - q_NS > PRESSURE_DIFF_THRESHOLD:
                    next_group = "EW"
                else:
                    # 壓力差不大 → 維持原 group
                    next_group = current_group

                current_group = next_group

                if current_group == "NS":
                    current_phase_idx = NS_GROUP_PHASES[0]
                else:
                    current_phase_idx = EW_GROUP_PHASES[0]
            else:
                # group 內部，照順序跳下一個 phase
                if current_group == "NS":
                    group = NS_GROUP_PHASES
                else:
                    group = EW_GROUP_PHASES
                pos = group.index(current_phase_idx)
                next_pos = (pos + 1) % len(group)
                current_phase_idx = group[next_pos]

            # 套用新 phase 的 RYG state
            traci.trafficlight.setRedYellowGreenState(
                TLS_ID, PHASE_CYCLE[current_phase_idx]["state"]
            )

    traci.close()
    print("\nSimulation completed.\n")

    # ====== 統計結果輸出（Max-Pressure Summary 格式） ======
    if total_steps > 0:
        avg_qEB = sum_qEB / total_steps
        avg_qSB = sum_qSB / total_steps
        avg_hEB = sum_hEB / total_steps
        avg_hSB = sum_hSB / total_steps

        eb_green_ratio = eb_green_steps / total_steps
        sb_green_ratio = sb_green_steps / total_steps

        total_waiting_time = sum_halting * STEP_LENGTH
        yellow_ratio = yellow_steps / total_steps

        if total_arrived_vehicles > 0:
            avg_delay_time = total_waiting_time / total_arrived_vehicles
        else:
            avg_delay_time = 0.0

        avg_queue_total = (sum_qEB + sum_qSB) / total_steps
    else:
        avg_qEB = avg_qSB = 0.0
        avg_hEB = avg_hSB = 0.0
        eb_green_ratio = sb_green_ratio = 0.0
        total_waiting_time = 0.0
        yellow_ratio = 0.0
        avg_delay_time = 0.0
        avg_queue_total = 0.0

    print("\n========== Max-Pressure Summary (Objective: Min Waiting Time) ==========")
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
    for name, cnt in phase_counts.items():
        ratio = cnt / total_steps
        print(f"  Phase {name}: {ratio:.2f}")
    print("")
    print(f"Total Arrived Veh   = {total_arrived_vehicles}")
    print(f"Avg Delay Time      = {avg_delay_time:.2f} sec/veh")
    print(f"Total Waiting Time  = {total_waiting_time:.2f} veh·sec")
    print(f"Cumulative reward   = {cumulative_reward:.2f}")
    print("========================================================================\n")


if __name__ == "__main__":
    main()
