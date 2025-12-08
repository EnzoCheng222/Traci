# -*- coding: utf-8 -*-
"""
PPO Traffic Light Control (Protected Left, 8 Phases, Safe)

- 使用自寫 PPO（numpy）
- 由程式端直接控制 8 個相位的 RYG state（含左轉保護）
- 不使用 SUMO 內建號誌程序，避免衝突號誌
"""

import os
import sys
from collections import defaultdict

import numpy as np

# -------------------------
# 模擬 / PPO 參數
# -------------------------

SIM_TIME = 1800.0
STEP_LENGTH = 0.10
TOTAL_STEPS = int(SIM_TIME / STEP_LENGTH)

# 決策層級：在「一個方向的 Straight+Left 結束」這種 group 邊界，決定下一輪主方向
# action = 0 → 保持當前主方向；1 → 切換主方向
ACTIONS = [0, 1]
ACTION_DIM = len(ACTIONS)

# state = 4*3 queue + 4*3 halting + current_phase_idx = 25
STATE_DIM = 25

# PPO 超參數（簡易版）
GAMMA = 0.9
LAMBDA = 0.95
POLICY_LR = 1e-4
VALUE_LR = 1e-4
PPO_EPOCHS = 4
PPO_BATCH_SIZE = 64
PPO_UPDATE_INTERVAL = 512

# Policy / Value 網路（線性）
policy_W = np.random.randn(STATE_DIM, ACTION_DIM).astype(np.float32) * 0.01
policy_b = np.zeros(ACTION_DIM, dtype=np.float32)

value_W = np.random.randn(STATE_DIM).astype(np.float32) * 0.01
value_b = 0.0

# PPO 緩衝區
states_buffer = []
actions_buffer = []
rewards_buffer = []
old_logprobs_buffer = []
values_buffer = []

# -------------------------
# SUMO / TraCI 初始化
# -------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

if "SUMO_HOME" not in os.environ:
    sys.exit("請先設定環境變數 SUMO_HOME")
tools = os.path.join(os.environ["SUMO_HOME"], "tools")
sys.path.append(tools)

import traci  # noqa: E402

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
# 8 相位（含左轉保護）
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

NS_GROUP_PHASES = [0, 1, 2, 3]
EW_GROUP_PHASES = [4, 5, 6, 7]


# -------------------------
# 工具函式
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


def stable_softmax(logits: np.ndarray) -> np.ndarray:
    logits = logits - np.max(logits)
    exp = np.exp(logits)
    return exp / np.sum(exp)


def get_queues_and_halting():
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


def get_state(current_phase_idx: int):
    """
    state = [EB 3 lanes q, SB 3 lanes q, WB 3 lanes q, NB 3 lanes q,
             EB 3 lanes h, SB 3 lanes h, WB 3 lanes h, NB 3 lanes h,
             current_phase_idx]
    """
    q = [
        *(traci.lanearea.getLastStepVehicleNumber(f"Node1_2_EB_{i}") for i in range(3)),
        *(traci.lanearea.getLastStepVehicleNumber(f"Node2_4_SB_{i}") for i in range(3)),
        *(traci.lanearea.getLastStepVehicleNumber(f"Node2_3_WB_{i}") for i in range(3)),
        *(traci.lanearea.getLastStepVehicleNumber(f"Node2_5_NB_{i}") for i in range(3)),
    ]
    h = [
        *(traci.lanearea.getLastStepHaltingNumber(f"Node1_2_EB_{i}") for i in range(3)),
        *(traci.lanearea.getLastStepHaltingNumber(f"Node2_4_SB_{i}") for i in range(3)),
        *(traci.lanearea.getLastStepHaltingNumber(f"Node2_3_WB_{i}") for i in range(3)),
        *(traci.lanearea.getLastStepHaltingNumber(f"Node2_5_NB_{i}") for i in range(3)),
    ]
    return tuple(q + h + [current_phase_idx])


def get_reward_from_halting(stats):
    return -float(stats["h_EB"] + stats["h_SB"] + stats["h_WB"] + stats["h_NB"])


def get_action_from_policy(state):
    s = np.array(state, dtype=np.float32)
    logits = s @ policy_W + policy_b
    probs = stable_softmax(logits)
    act_idx = np.random.choice(len(ACTIONS), p=probs)
    logprob = float(np.log(probs[act_idx] + 1e-8))
    value = float(s @ value_W + value_b)
    return ACTIONS[act_idx], logprob, value


def ppo_update():
    global policy_W, policy_b, value_W, value_b

    if len(states_buffer) == 0:
        return

    states = np.array(states_buffer, dtype=np.float32)
    actions_idx = np.array(actions_buffer, dtype=np.int64)
    rewards = np.array(rewards_buffer, dtype=np.float32)
    old_logprobs = np.array(old_logprobs_buffer, dtype=np.float32)
    values = np.array(values_buffer, dtype=np.float32)

    N = len(rewards)
    advantages = np.zeros(N, dtype=np.float32)
    gae = 0.0
    for t in reversed(range(N)):
        nxt = values[t + 1] if t < N - 1 else 0.0
        delta = rewards[t] + GAMMA * nxt - values[t]
        gae = delta + GAMMA * LAMBDA * gae
        advantages[t] = gae

    returns = advantages + values
    adv_norm = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

    for _ in range(PPO_EPOCHS):
        idx = np.arange(N)
        np.random.shuffle(idx)

        for start in range(0, N, PPO_BATCH_SIZE):
            batch = idx[start:start + PPO_BATCH_SIZE]
            if len(batch) == 0:
                continue

            b_states = states[batch]
            b_actions = actions_idx[batch]
            b_adv = adv_norm[batch]
            b_returns = returns[batch]

            logits = b_states @ policy_W + policy_b
            probs = np.array([stable_softmax(l) for l in logits])
            act_probs = probs[np.arange(len(b_actions)), b_actions]
            logprobs = np.log(act_probs + 1e-8)

            # 簡化版 Policy Gradient（沒有做 clipping）
            one_hot = np.zeros_like(probs)
            one_hot[np.arange(len(b_actions)), b_actions] = 1
            diff = one_hot - probs

            grad_W = -(b_states.T @ (diff * b_adv[:, None])) / len(b_states)
            grad_b = -np.mean(diff * b_adv[:, None], axis=0)

            policy_W -= POLICY_LR * grad_W
            policy_b -= POLICY_LR * grad_b

            # Value 更新
            v_pred = b_states @ value_W + value_b
            err = v_pred - b_returns
            grad_v_W = (2.0 / len(b_states)) * (b_states.T @ err)
            grad_v_b = 2.0 * np.mean(err)

            value_W -= VALUE_LR * grad_v_W
            value_b -= VALUE_LR * grad_v_b

    states_buffer.clear()
    actions_buffer.clear()
    rewards_buffer.clear()
    old_logprobs_buffer.clear()
    values_buffer.clear()


# -------------------------
# 主程式：單一 episode
# -------------------------

def main():
    EPISODES = 1
    print("\n=== PPO Training ({} Episode) ===".format(EPISODES))

    for ep in range(1, EPISODES + 1):
        print("\n--- Episode {}/{} ---".format(ep, EPISODES))

        if traci.isLoaded():
            traci.close()
        traci.start(SUMO_CONFIG)

        try:
            traci.gui.setSchema("View #0", "real world")
        except Exception:
            pass

        # 初始主方向：NS
        current_group = "NS"
        current_phase_idx = NS_GROUP_PHASES[0]
        steps_in_phase = 0

        traci.trafficlight.setRedYellowGreenState(
            TLS_ID, PHASE_CYCLE[current_phase_idx]["state"]
        )

        # 統計變數（與 Max-Pressure 相同格式）
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

            # reward（步階）
            reward = get_reward_from_halting(stats)
            cumulative_reward += reward

            # 目前 state（用 PHASE_CYCLE index 當作 phase）
            state = get_state(current_phase_idx)

            # phase 統計
            phase_name = PHASE_CYCLE[current_phase_idx]["name"]
            phase_counts[phase_name] += 1

            if current_phase_idx in [1, 3, 5, 7]:
                yellow_steps += 1

            if current_phase_idx in [0, 2]:
                sb_green_steps += 1
            if current_phase_idx in [4, 6]:
                eb_green_steps += 1

            # 存入 PPO buffer
            states_buffer.append(np.array(state, dtype=np.float32))
            actions_buffer.append(0)  # 先暫存，後面會覆蓋成實際 action index
            rewards_buffer.append(reward)
            old_logprobs_buffer.append(0.0)
            values_buffer.append(0.0)

            # 每 100 step 印一次
            if step % 100 == 0:
                sim_time = step * STEP_LENGTH
                print(f"[t={sim_time:4.1f}s] Phase: {phase_name} | "
                      f"qEB={q_EB:3d}, qWB={q_WB:3d}, qSB={q_SB:3d}, qNB={q_NB:3d}")

            # 取得 PPO action（每一步都算，但只在 group 邊界生效）
            action, logprob, value = get_action_from_policy(state)

            # 覆蓋最後一筆 buffer 的 action / logprob / value
            actions_buffer[-1] = ACTIONS.index(action)
            old_logprobs_buffer[-1] = logprob
            values_buffer[-1] = value

            # Phase 時間更新
            steps_in_phase += 1
            phase_duration = PHASE_CYCLE[current_phase_idx]["duration"]

            if steps_in_phase >= phase_duration:
                steps_in_phase = 0

                # group 結尾（NS Left (Y) 或 EW Left (Y)）
                if current_phase_idx in [3, 7]:
                    # action = 0 → 保持 current_group；1 → 切換
                    if action == 1:
                        current_group = "EW" if current_group == "NS" else "NS"

                    # 根據 current_group 決定新的 phase index
                    if current_group == "NS":
                        current_phase_idx = NS_GROUP_PHASES[0]
                    else:
                        current_phase_idx = EW_GROUP_PHASES[0]
                else:
                    # group 內部，照順序
                    if current_group == "NS":
                        group = NS_GROUP_PHASES
                    else:
                        group = EW_GROUP_PHASES

                    pos = group.index(current_phase_idx)
                    next_pos = (pos + 1) % len(group)
                    current_phase_idx = group[next_pos]

                traci.trafficlight.setRedYellowGreenState(
                    TLS_ID, PHASE_CYCLE[current_phase_idx]["state"]
                )

            # 依照 buffer 大小更新一次 PPO（簡化版）
            if len(states_buffer) >= PPO_UPDATE_INTERVAL:
                ppo_update()

        # Episode 結束後，做最後一次 update
        if len(states_buffer) > 0:
            ppo_update()

        traci.close()
        print("\nSimulation completed.\n")

        # ====== Summary（跟 Max-Pressure / 之前 PPO 一樣格式） ======
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

        print("\n========== PPO Summary (Objective: Min Waiting Time) ==========")
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
