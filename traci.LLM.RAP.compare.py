# ================================
# LLM-Based Traffic Signal Control (Protected Left, 8 Phases)
# - LLM 每一個 group 結束時選「下一個 group」
# - group = {NS_STRAIGHT, NS_LEFT, EW_STRAIGHT, EW_LEFT}
# - 直接控制 8 相位 RYG（跟你 PPO 版本同一組 PHASE_CYCLE）
# - 結尾輸出平均排隊 / 停等 / Delay 等統計
# ================================

import os
import sys
import time
import json
import re
from collections import defaultdict

import google.generativeai as genai

# -------------------------
# 基本參數
# -------------------------

SIM_TIME = 1800.0
STEP_LENGTH = 0.10
TOTAL_STEPS = int(SIM_TIME / STEP_LENGTH)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# SUMO 路徑
if "SUMO_HOME" not in os.environ:
    sys.exit("請先設定環境變數 SUMO_HOME")
tools = os.path.join(os.environ["SUMO_HOME"], "tools")
sys.path.append(tools)

import traci  # noqa

sumocfg_path = os.path.join(SCRIPT_DIR, "Traci.sumocfg")
if not os.path.exists(sumocfg_path):
    sys.exit(f"找不到 SUMO 設定檔: {sumocfg_path}")

SUMO_CONFIG = [
    "sumo-gui",
    "-c", sumocfg_path,
    "--step-length", str(STEP_LENGTH),
    "--delay", "0",
    "--lateral-resolution", "0"
]

TLS_ID = "Node2"

# -------------------------
# 8 相位（完全沿用你 PPO 的設定）
# duration 單位 = simulation steps (0.1s)
# -------------------------

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

NS_STRAIGHT_PHASES = [0, 1]
NS_LEFT_PHASES     = [2, 3]
EW_STRAIGHT_PHASES = [4, 5]
EW_LEFT_PHASES     = [6, 7]

GROUP_TO_PHASE = {
    "NS_STRAIGHT": 0,
    "NS_LEFT": 2,
    "EW_STRAIGHT": 4,
    "EW_LEFT": 6
}

GROUP_LIST = ["NS_STRAIGHT", "NS_LEFT", "EW_STRAIGHT", "EW_LEFT"]

# -------------------------
# LLM 設定（依你要求寫死 key）
# -------------------------

genai.configure(api_key="AIzaSyDqT-0MfSvqSYCqw4RrRKE5j9GIHCGvLG0")
LLM_MODEL = "gemini-2.5-flash"
model = genai.GenerativeModel(LLM_MODEL)

api_call_count = 0
fallback_count = 0
total_api_time = 0.0

# -------------------------
# 車道紀律（跟 PPO 一樣）
# -------------------------

def enforce_lane_discipline():
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

                straight = (
                    (edge == "Node1_2_EB" and nxt == "Node2_3_EB") or
                    (edge == "Node2_3_WB" and nxt == "Node1_2_WB") or
                    (edge == "Node2_4_SB" and nxt == "Node2_5_SB") or
                    (edge == "Node2_5_NB" and nxt == "Node2_4_NB")
                )
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

# -------------------------
# 狀態讀取（含直行/左轉、各向 queue / halting）
# -------------------------

def get_full_stats():
    # EB
    q_EB_0 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_0")
    q_EB_1 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_1")
    q_EB_2 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_2")
    h_EB_0 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_0")
    h_EB_1 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_1")
    h_EB_2 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_2")

    # WB
    q_WB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_3_WB_0")
    q_WB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_3_WB_1")
    q_WB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_3_WB_2")
    h_WB_0 = traci.lanearea.getLastStepHaltingNumber("Node2_3_WB_0")
    h_WB_1 = traci.lanearea.getLastStepHaltingNumber("Node2_3_WB_1")
    h_WB_2 = traci.lanearea.getLastStepHaltingNumber("Node2_3_WB_2")

    # SB
    q_SB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_0")
    q_SB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_1")
    q_SB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_2")
    h_SB_0 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_0")
    h_SB_1 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_1")
    h_SB_2 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_2")

    # NB
    q_NB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_5_NB_0")
    q_NB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_5_NB_1")
    q_NB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_5_NB_2")
    h_NB_0 = traci.lanearea.getLastStepHaltingNumber("Node2_5_NB_0")
    h_NB_1 = traci.lanearea.getLastStepHaltingNumber("Node2_5_NB_1")
    h_NB_2 = traci.lanearea.getLastStepHaltingNumber("Node2_5_NB_2")

    q_EB_s = q_EB_0 + q_EB_1
    q_EB_l = q_EB_2
    q_WB_s = q_WB_0 + q_WB_1
    q_WB_l = q_WB_2
    q_SB_s = q_SB_0 + q_SB_1
    q_SB_l = q_SB_2
    q_NB_s = q_NB_0 + q_NB_1
    q_NB_l = q_NB_2

    h_EB_s = h_EB_0 + h_EB_1
    h_EB_l = h_EB_2
    h_WB_s = h_WB_0 + h_WB_1
    h_WB_l = h_WB_2
    h_SB_s = h_SB_0 + h_SB_1
    h_SB_l = h_SB_2
    h_NB_s = h_NB_0 + h_NB_1
    h_NB_l = h_NB_2

    q_EB = q_EB_s + q_EB_l
    q_WB = q_WB_s + q_WB_l
    q_SB = q_SB_s + q_SB_l
    q_NB = q_NB_s + q_NB_l

    h_EB = h_EB_s + h_EB_l
    h_WB = h_WB_s + h_WB_l
    h_SB = h_SB_s + h_SB_l
    h_NB = h_NB_s + h_NB_l

    stats = {
        "q_EB": q_EB, "q_WB": q_WB, "q_SB": q_SB, "q_NB": q_NB,
        "h_EB": h_EB, "h_WB": h_WB, "h_SB": h_SB, "h_NB": h_NB,
        "q_EB_s": q_EB_s, "q_EB_l": q_EB_l,
        "q_WB_s": q_WB_s, "q_WB_l": q_WB_l,
        "q_SB_s": q_SB_s, "q_SB_l": q_SB_l,
        "q_NB_s": q_NB_s, "q_NB_l": q_NB_l,
        "h_EB_s": h_EB_s, "h_EB_l": h_EB_l,
        "h_WB_s": h_WB_s, "h_WB_l": h_WB_l,
        "h_SB_s": h_SB_s, "h_SB_l": h_SB_l,
        "h_NB_s": h_NB_s, "h_NB_l": h_NB_l,
    }

    stats["q_NS_s"] = q_SB_s + q_NB_s
    stats["q_NS_l"] = q_SB_l + q_NB_l
    stats["q_EW_s"] = q_EB_s + q_WB_s
    stats["q_EW_l"] = q_EB_l + q_WB_l

    stats["q_total"] = q_EB + q_WB + q_SB + q_NB
    stats["h_total"] = h_EB + h_WB + h_SB + h_NB

    return stats

# -------------------------
# LLM 部分
# -------------------------

def build_llm_prompt(stats, current_group, sim_time):
    return f"""
You control a 4-approach intersection with protected left-turn phases (8 total phases).

Current time: {sim_time:.1f} s
Current group: {current_group}

Queues (vehicles):
  NS_STRAIGHT = {stats['q_NS_s']}
  NS_LEFT     = {stats['q_NS_l']}
  EW_STRAIGHT = {stats['q_EW_s']}
  EW_LEFT     = {stats['q_EW_l']}

Goal:
- Minimize total queue and stopped vehicles over the whole simulation.
- For S1 (balanced) scenario, all directions have similar demand.
- At each decision, choose the movement group that should receive the NEXT full green+yellow cycle.

Valid options (exact strings):
  "NS_STRAIGHT"
  "NS_LEFT"
  "EW_STRAIGHT"
  "EW_LEFT"

Return ONLY one JSON object, no explanation, no extra text:
{{"group": "NS_STRAIGHT"}}
or
{{"group": "NS_LEFT"}}
or
{{"group": "EW_STRAIGHT"}}
or
{{"group": "EW_LEFT"}}
"""

def call_llm(prompt):
    global api_call_count, total_api_time
    try:
        start = time.time()
        resp = model.generate_content(
            prompt,
            generation_config={"temperature": 0.2, "max_output_tokens": 32}
        )
        total_api_time += (time.time() - start)
        api_call_count += 1
    except Exception as e:
        print("⚠️ LLM Error:", e)
        return None

    text = ""
    if resp.candidates:
        for c in resp.candidates:
            if c.content and c.content.parts:
                for p in c.content.parts:
                    if hasattr(p, "text") and p.text:
                        text += p.text.strip()
    return text.strip() if text.strip() else None

def parse_llm_group(resp_text):
    if not resp_text:
        return None
    try:
        m = re.search(r"\{.*\}", resp_text, re.DOTALL)
        if not m:
            return None
        obj = json.loads(m.group(0))
        g = obj.get("group", "").strip().upper()
        if g in GROUP_TO_PHASE:
            return g
        return None
    except Exception:
        return None

# -------------------------
# fallback：Max-Pressure-like
# -------------------------

def fallback_group(stats):
    global fallback_count
    fallback_count += 1
    pressures = {
        "NS_STRAIGHT": stats["q_NS_s"],
        "NS_LEFT":     stats["q_NS_l"],
        "EW_STRAIGHT": stats["q_EW_s"],
        "EW_LEFT":     stats["q_EW_l"],
    }
    return max(pressures, key=pressures.get)

# -------------------------
# 主程式
# -------------------------

def main():
    global api_call_count, fallback_count, total_api_time

    if traci.isLoaded():
        traci.close()
    traci.start(SUMO_CONFIG)

    try:
        traci.gui.setSchema("View #0", "real world")
    except Exception:
        pass

    current_group = "NS_STRAIGHT"
    current_phase = GROUP_TO_PHASE[current_group]
    steps_in_phase = 0

    traci.trafficlight.setRedYellowGreenState(TLS_ID, PHASE_CYCLE[current_phase]["state"])

    # 統計變數
    sum_qEB = sum_qWB = sum_qSB = sum_qNB = 0.0
    sum_hEB = sum_hWB = sum_hSB = sum_hNB = 0.0
    sum_halting = 0.0
    total_steps = 0
    total_arrived_vehicles = 0
    cumulative_reward = 0.0

    ns_green_steps = 0
    ew_green_steps = 0
    yellow_steps = 0
    phase_counts = defaultdict(int)

    for step in range(TOTAL_STEPS):
        sim_time = step * STEP_LENGTH

        if step % int(1.0 / STEP_LENGTH) == 0:
            enforce_lane_discipline()

        traci.simulationStep()
        total_steps += 1
        total_arrived_vehicles += traci.simulation.getArrivedNumber()

        stats = get_full_stats()

        sum_qEB += stats["q_EB"]
        sum_qWB += stats["q_WB"]
        sum_qSB += stats["q_SB"]
        sum_qNB += stats["q_NB"]

        sum_hEB += stats["h_EB"]
        sum_hWB += stats["h_WB"]
        sum_hSB += stats["h_SB"]
        sum_hNB += stats["h_NB"]

        sum_halting += stats["h_total"]
        cumulative_reward += -float(stats["h_total"])

        phase_name = PHASE_CYCLE[current_phase]["name"]
        phase_counts[phase_name] += 1

        if current_phase in [0, 2]:   # NS 綠燈
            ns_green_steps += 1
        if current_phase in [4, 6]:   # EW 綠燈
            ew_green_steps += 1
        if current_phase in [1, 3, 5, 7]:
            yellow_steps += 1

        if step % 100 == 0:
            print(f"[t={sim_time:4.1f}s] Phase={phase_name} | "
                  f"qEB={stats['q_EB']:2d}, qWB={stats['q_WB']:2d}, "
                  f"qSB={stats['q_SB']:2d}, qNB={stats['q_NB']:2d}")

        steps_in_phase += 1
        phase_dur = PHASE_CYCLE[current_phase]["duration"]

        if steps_in_phase >= phase_dur:
            steps_in_phase = 0

            # group 結尾（黃燈結束）→ LLM 決策下一個 group
            if current_phase in [1, 3, 5, 7]:
                print(f"\n[t={sim_time:4.1f}s] === Group {current_group} finished, deciding NEXT group ===")
                print(f"   Queues: NS_s={stats['q_NS_s']}, NS_l={stats['q_NS_l']}, "
                      f"EW_s={stats['q_EW_s']}, EW_l={stats['q_EW_l']}")

                prompt = build_llm_prompt(stats, current_group, sim_time)
                resp_text = call_llm(prompt)
                chosen_group = parse_llm_group(resp_text)

                if chosen_group is None:
                    chosen_group = fallback_group(stats)
                    print(f"   → Fallback selected group: {chosen_group}")
                else:
                    print(f"   → LLM selected group: {chosen_group}")

                current_group = chosen_group
                current_phase = GROUP_TO_PHASE[current_group]

            else:
                if current_phase in NS_STRAIGHT_PHASES:
                    seq = NS_STRAIGHT_PHASES
                elif current_phase in NS_LEFT_PHASES:
                    seq = NS_LEFT_PHASES
                elif current_phase in EW_STRAIGHT_PHASES:
                    seq = EW_STRAIGHT_PHASES
                else:
                    seq = EW_LEFT_PHASES

                idx = seq.index(current_phase)
                current_phase = seq[(idx + 1) % len(seq)]

            traci.trafficlight.setRedYellowGreenState(TLS_ID, PHASE_CYCLE[current_phase]["state"])

    traci.close()

    print("\n=== Simulation completed. ===\n")

    if total_steps > 0:
        avg_qEB = sum_qEB / total_steps
        avg_qWB = sum_qWB / total_steps
        avg_qSB = sum_qSB / total_steps
        avg_qNB = sum_qNB / total_steps

        avg_hEB = sum_hEB / total_steps
        avg_hWB = sum_hWB / total_steps
        avg_hSB = sum_hSB / total_steps
        avg_hNB = sum_hNB / total_steps

        avg_queue_total = (sum_qEB + sum_qWB + sum_qSB + sum_qNB) / total_steps
        total_waiting_time = sum_halting * STEP_LENGTH

        if total_arrived_vehicles > 0:
            avg_delay_time = total_waiting_time / total_arrived_vehicles
        else:
            avg_delay_time = 0.0

        ns_green_ratio = ns_green_steps / total_steps
        ew_green_ratio = ew_green_steps / total_steps
        yellow_ratio = yellow_steps / total_steps
    else:
        avg_qEB = avg_qWB = avg_qSB = avg_qNB = 0.0
        avg_hEB = avg_hWB = avg_hSB = avg_hNB = 0.0
        avg_queue_total = 0.0
        total_waiting_time = 0.0
        avg_delay_time = 0.0
        ns_green_ratio = ew_green_ratio = yellow_ratio = 0.0

    print("========== LLM Protected-Left Summary ==========")
    print(f"EB 平均排隊 (Queue) = {avg_qEB:.2f} veh")
    print(f"WB 平均排隊 (Queue) = {avg_qWB:.2f} veh")
    print(f"SB 平均排隊 (Queue) = {avg_qSB:.2f} veh")
    print(f"NB 平均排隊 (Queue) = {avg_qNB:.2f} veh")
    print(f"Total Avg Queue     = {avg_queue_total:.2f} veh")
    print("")
    print(f"EB 平均停等 (Halt)  = {avg_hEB:.2f} veh")
    print(f"WB 平均停等 (Halt)  = {avg_hWB:.2f} veh")
    print(f"SB 平均停等 (Halt)  = {avg_hSB:.2f} veh")
    print(f"NB 平均停等 (Halt)  = {avg_hNB:.2f} veh")
    print("")
    print(f"NS 綠燈比例         = {ns_green_ratio:.2f}")
    print(f"EW 綠燈比例         = {ew_green_ratio:.2f}")
    print(f"黃燈比例 (Yellow)   = {yellow_ratio:.2f}")
    print("")
    print("各時相比例 (Phase Ratios):")
    for name, cnt in phase_counts.items():
        ratio = cnt / total_steps
        print(f"  {name}: {ratio:.2f}")
    print("")
    print(f"Total Arrived Veh   = {total_arrived_vehicles}")
    print(f"Avg Delay Time      = {avg_delay_time:.2f} sec/veh")
    print(f"Total Waiting Time  = {total_waiting_time:.2f} veh·sec")
    print(f"Cumulative reward   = {cumulative_reward:.2f}")
    print("")
    print("--- LLM Performance ---")
    print(f"Total API Calls     = {api_call_count}")
    print(f"Fallback Count      = {fallback_count}")
    if api_call_count > 0:
        print(f"Avg API Latency     = {total_api_time/api_call_count:.2f} sec")
    else:
        print("Avg API Latency     = 0.00 sec")
    print("===============================================\n")


if __name__ == "__main__":
    main()
