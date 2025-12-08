# -*- coding: utf-8 -*-
"""
Webster Fixed-Time Control (Protected Left, via RYG States)

- 固定時制號誌控制（Webster 設定）
- 與 PPO / Max-Pressure 使用相同的 PHASE_CYCLE（含左轉保護）
- 目標式：最小停等時間（以統計指標比較）
"""

import os
import sys
from collections import defaultdict

# -------------------------
# SUMO / TraCI 初始化
# -------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

import traci  # noqa: E402

sumocfg_path = os.path.join(SCRIPT_DIR, 'Traci.sumocfg')
if not os.path.exists(sumocfg_path):
    sys.exit(f"找不到 SUMO 設定檔: {sumocfg_path}，請確認檔名與位置是否正確。")

STEP_LENGTH = 0.10
SIM_TIME = 1800.0
TOTAL_STEPS = int(SIM_TIME / STEP_LENGTH)

Sumo_config = [
    'sumo-gui',
    '-c', sumocfg_path,
    '--step-length', str(STEP_LENGTH),
    '--delay', '0',
    '--lateral-resolution', '0',
    '--end', str(int(SIM_TIME))
]

TLS_ID = "Node2"

# -------------------------
# Webster 固定時制：保護左轉 8 相位
# -------------------------
# 這裡的 duration 單位是「step」，1 step = 0.1 秒
# 對應你原本固定時制程式：
#   NS 直行 G: 30s → 300 steps
#   NS 直行 Y:  3s →  30 steps
#   NS 左轉 G: 15s → 150 steps
#   NS 左轉 Y:  3s →  30 steps
#   EW 直行 G: 30s → 300 steps
#   EW 直行 Y:  3s →  30 steps
#   EW 左轉 G: 15s → 150 steps
#   EW 左轉 Y:  3s →  30 steps

PHASE_CYCLE = [
    {
        "name": "NS Straight",
        "state": "GGGGrrrrrrGGGGrrrrrr",
        "group": "NS",
        "is_green": True,
        "is_yellow": False,
        "duration": 300,
    },
    {
        "name": "NS Straight (Y)",
        "state": "yyyyrrrrrryyyyrrrrrr",
        "group": "NS",
        "is_green": False,
        "is_yellow": True,
        "duration": 30,
    },
    {
        "name": "NS Left",
        "state": "rrrrGrrrrrrrrrGrrrrr",
        "group": "NS",
        "is_green": True,
        "is_yellow": False,
        "duration": 150,
    },
    {
        "name": "NS Left (Y)",
        "state": "rrrryrrrrrrrrryrrrrr",
        "group": "NS",
        "is_green": False,
        "is_yellow": True,
        "duration": 30,
    },
    {
        "name": "EW Straight",
        "state": "rrrrrGGGGrrrrrGGGGrr",
        "group": "EW",
        "is_green": True,
        "is_yellow": False,
        "duration": 300,
    },
    {
        "name": "EW Straight (Y)",
        "state": "rrrrryyyyrrrrryyyyrr",
        "group": "EW",
        "is_green": False,
        "is_yellow": True,
        "duration": 30,
    },
    {
        "name": "EW Left",
        "state": "rrrrrrrrrGrrrrrrrrrG",
        "group": "EW",
        "is_green": True,
        "is_yellow": False,
        "duration": 150,
    },
    {
        "name": "EW Left (Y)",
        "state": "rrrrrrrrryrrrrrrrrry",
        "group": "EW",
        "is_green": False,
        "is_yellow": True,
        "duration": 30,
    },
]

# -------------------------
# 統計變數（比照 Max-Pressure / PPO）
# -------------------------

sum_qEB = 0.0
sum_qSB = 0.0
sum_hEB = 0.0
sum_hSB = 0.0
sum_halting = 0.0

yellow_steps = 0
phase_counts = defaultdict(int)
total_arrived_vehicles = 0
total_steps = 0

eb_green_steps = 0
sb_green_steps = 0

cumulative_reward = 0.0

# -------------------------
# 工具函式
# -------------------------

def enforce_lane_discipline():
    """
    依照你原本的車道紀律：
    - 直行車不在左轉車道
    - 左轉車導向左轉車道
    """
    edges = ["Node1_2_EB", "Node2_3_WB", "Node2_4_SB", "Node2_5_NB"]
    for edge in edges:
        veh_ids = traci.edge.getLastStepVehicleIDs(edge)
        for vid in veh_ids:
            try:
                route = traci.vehicle.getRoute(vid)
                if len(route) < 2:
                    continue

                try:
                    cur_idx = route.index(edge)
                except ValueError:
                    continue

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


def get_state():
    """
    讀取 EB / SB 的排隊與停等（與 Max-Pressure / PPO 對齊）
    """
    # EB queue
    q_EB_0 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_0")
    q_EB_1 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_1")
    q_EB_2 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_2")

    # SB queue
    q_SB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_0")
    q_SB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_1")
    q_SB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_2")

    # EB halting
    h_EB_0 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_0")
    h_EB_1 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_1")
    h_EB_2 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_2")

    # SB halting
    h_SB_0 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_0")
    h_SB_1 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_1")
    h_SB_2 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_2")

    q_EB = q_EB_0 + q_EB_1 + q_EB_2
    q_SB = q_SB_0 + q_SB_1 + q_SB_2
    h_EB = h_EB_0 + h_EB_1 + h_EB_2
    h_SB = h_SB_0 + h_SB_1 + h_SB_2

    return q_EB, q_SB, h_EB, h_SB


def get_reward(q_EB, q_SB):
    """
    reward = - (EB + SB 總排隊車數)
    """
    total_queue = q_EB + q_SB
    return -float(total_queue)


# -------------------------
# 主程式：Webster 固定時制控制
# -------------------------

def main():
    global sum_qEB, sum_qSB, sum_hEB, sum_hSB, sum_halting
    global yellow_steps, total_arrived_vehicles, total_steps
    global eb_green_steps, sb_green_steps, cumulative_reward

    traci.start(Sumo_config)
    traci.gui.setSchema("View #0", "real world")

    print("\n=== Starting Webster Fixed-Time Control (Protected Left, 1800 sec) ===\n")

    current_phase_idx = 0
    phase_elapsed = 0

    # 設定初始相位
    traci.trafficlight.setRedYellowGreenState(
        TLS_ID, PHASE_CYCLE[current_phase_idx]["state"]
    )

    for step in range(TOTAL_STEPS):
        # 每 1 秒執行一次車道紀律
        if step % int(1.0 / STEP_LENGTH) == 0:
            enforce_lane_discipline()

        # 讀取狀態
        q_EB, q_SB, h_EB, h_SB = get_state()

        # 統計
        sum_qEB += q_EB
        sum_qSB += q_SB
        sum_hEB += h_EB
        sum_hSB += h_SB
        sum_halting += (h_EB + h_SB)

        total_arrived_vehicles += traci.simulation.getArrivedNumber()
        total_steps += 1

        phase_info = PHASE_CYCLE[current_phase_idx]

        # 綠燈比例：EW 算 EB 綠燈，NS 算 SB 綠燈
        if phase_info["is_green"]:
            if phase_info["group"] == "EW":
                eb_green_steps += 1
            elif phase_info["group"] == "NS":
                sb_green_steps += 1

        # 黃燈步數
        if phase_info["is_yellow"]:
            yellow_steps += 1

        # Phase 使用比例
        phase_counts[current_phase_idx] += 1

        # reward
        reward = get_reward(q_EB, q_SB)
        cumulative_reward += reward

        # 每 100 step 印一次狀態
        if step % 100 == 0:
            sim_time = step * STEP_LENGTH
            print(
                f"[t={sim_time:4.1f}s] Phase: {PHASE_CYCLE[current_phase_idx]['name']} | "
                f"qEB={q_EB:3d}, qSB={q_SB:3d} | hEB={h_EB:3d}, hSB={h_SB:3d}"
            )

        # 時制邏輯：固定時間到就切換下一個 phase（Webster）
        phase_elapsed += 1
        if phase_elapsed >= PHASE_CYCLE[current_phase_idx]["duration"]:
            current_phase_idx = (current_phase_idx + 1) % len(PHASE_CYCLE)
            phase_elapsed = 0
            traci.trafficlight.setRedYellowGreenState(
                TLS_ID, PHASE_CYCLE[current_phase_idx]["state"]
            )

        # 推進 SUMO 一步
        traci.simulationStep()

    traci.close()
    print("\nSimulation completed.\n")

    # ====== 統計結果輸出（對齊 Max-Pressure / PPO Summary） ======
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

    print("\n========== Webster Summary (Objective: Min Waiting Time) ==========")
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
        print(f"  Phase {p_idx} ({PHASE_CYCLE[p_idx]['name']}): {ratio:.2f}")
    print("")
    print(f"Total Arrived Veh   = {total_arrived_vehicles}")
    print(f"Avg Delay Time      = {avg_delay_time:.2f} sec/veh")
    print(f"Total Waiting Time  = {total_waiting_time:.2f} veh·sec")
    print(f"Cumulative reward   = {cumulative_reward:.2f}")
    print("========================================================================\n")


if __name__ == "__main__":
    main()
