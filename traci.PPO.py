# ================================
# PPO Traffic Light Control for Node2 (No Q-table)
# ================================

# Step 1: Add modules to provide access to specific libraries and functions
import os
import sys
import random
import numpy as np

# ★ 取得這支程式所在的資料夾，之後用來組 sumocfg 的絕對路徑
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Step 2: Establish path to SUMO (SUMO_HOME)
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 3: Add Traci module
import traci

# Step 4: Define Sumo configuration
sumocfg_path = os.path.join(SCRIPT_DIR, 'Traci.sumocfg')

if not os.path.exists(sumocfg_path):
    sys.exit(f"找不到 SUMO 設定檔: {sumocfg_path}，請確認檔名與位置是否正確。")

Sumo_config = [
    'sumo-gui',
    '-c', sumocfg_path,
    '--step-length', '0.10',   # 0.1 s
    '--delay', '0',
    '--lateral-resolution', '0'
]

# Step 5: Open connection between SUMO and Traci
traci.start(Sumo_config)

# 如果有 GUI，就用真實世界配色
try:
    traci.gui.setSchema("View #0", "real world")
except Exception:
    pass

# -------------------------
# Step 6: Define Variables
# -------------------------

# Detector queues
q_EB_0 = q_EB_1 = q_EB_2 = 0
q_SB_0 = q_SB_1 = q_SB_2 = 0
current_phase = 0
current_simulation_step = 0

# ---- Only simulate 1800 seconds ----
SIM_TIME = 1800        # 秒
STEP_LENGTH = 0.10     # 秒
TOTAL_STEPS = int(SIM_TIME / STEP_LENGTH)   # = 18000 steps

# 控制最低綠燈時間（以 step 為單位）
MIN_GREEN_STEPS = 100          # 100 steps * 0.1 s = 10 秒
last_switch_step = -MIN_GREEN_STEPS

# Action space: 0 = 保持當前 phase, 1 = 切換到下一個 phase
ACTIONS = [0, 1]
STATE_DIM = 7
ACTION_DIM = len(ACTIONS)

# PPO / RL hyperparameters
GAMMA = 0.9
LAMBDA = 0.95
CLIP_EPS = 0.2
POLICY_LR = 1e-4
VALUE_LR = 1e-4
PPO_EPOCHS = 4
PPO_BATCH_SIZE = 64
PPO_UPDATE_INTERVAL = 512

# Policy network 參數 (linear: state -> logits over actions)
policy_W = np.random.randn(STATE_DIM, ACTION_DIM).astype(np.float32) * 0.01
policy_b = np.zeros(ACTION_DIM, dtype=np.float32)

# Value network 參數 (linear: state -> value)
value_W = np.random.randn(STATE_DIM).astype(np.float32) * 0.01
value_b = 0.0

# PPO buffers
states_buffer = []
actions_buffer = []
rewards_buffer = []
old_logprobs_buffer = []
dones_buffer = []
values_buffer = []

# ------ 相位分類 & 統計用變數 ------
EB_PHASES = []  # 東向(EB)綠燈相位 index
SB_PHASES = []  # 南向(SB)綠燈相位 index

sum_qEB = 0.0
sum_qSB = 0.0
sum_queue = 0.0
eb_green_steps = 0
sb_green_steps = 0
total_steps = 0


# -------------------------
# Step 7: Utility Functions
# -------------------------

def stable_softmax(logits: np.ndarray) -> np.ndarray:
    """
    支援 1D (單一樣本) 與 2D (batch) 的 softmax，並且數值穩定。
    """
    logits = np.array(logits, dtype=np.float32)
    logits = np.clip(logits, -50.0, 50.0)

    if logits.ndim == 1:
        m = np.max(logits)
        exps = np.exp(logits - m)
        s = np.sum(exps)
        if s <= 0 or not np.isfinite(s):
            return np.ones_like(logits) / logits.shape[0]
        return exps / s
    elif logits.ndim == 2:
        m = np.max(logits, axis=1, keepdims=True)
        exps = np.exp(logits - m)
        s = np.sum(exps, axis=1, keepdims=True)
        s[s <= 0] = 1.0
        probs = exps / s
        return probs
    else:
        raise ValueError("logits 維度必須是 1 或 2")


def get_reward(state):
    """
    Reward = - (總排隊長度)
    state: (q_EB0, q_EB1, q_EB2, q_SB0, q_SB1, q_SB2, phase)
    """
    total_queue = sum(state[:-1])  # 最後一個是 phase，不算進 queue
    return -float(total_queue)


def get_state():
    """
    從偵測器和號誌讀取 state。
    """
    global q_EB_0, q_EB_1, q_EB_2
    global q_SB_0, q_SB_1, q_SB_2
    global current_phase

    # 你現在用的偵測器 id
    q_EB_0 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_0")
    q_EB_1 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_1")
    q_EB_2 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_2")

    q_SB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_0")
    q_SB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_1")
    q_SB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_2")

    current_phase = traci.trafficlight.getPhase("Node2")

    return (q_EB_0, q_EB_1, q_EB_2,
            q_SB_0, q_SB_1, q_SB_2,
            current_phase)


def get_action_from_policy(state):
    """
    根據目前 policy 對給定的 state 選出一個動作 (0 或 1)，
    同時回傳 logprob 與 value，用於 PPO 更新。
    """
    global policy_W, policy_b, value_W, value_b

    s = np.array(state, dtype=np.float32)
    logits = s @ policy_W + policy_b          # shape: (ACTION_DIM,)
    probs = stable_softmax(logits)            # shape: (ACTION_DIM,)

    # 保險：避免數值錯亂導致機率不是合法分佈
    if np.any(probs < 0) or not np.isfinite(probs).all():
        probs = np.ones_like(probs) / len(probs)
    if probs.sum() <= 0:
        probs = np.ones_like(probs) / len(probs)
    probs = probs / probs.sum()

    value = float(s @ value_W + value_b)

    action_index = np.random.choice(len(ACTIONS), p=probs)
    logprob = float(np.log(probs[action_index] + 1e-8))

    return ACTIONS[action_index], logprob, value


def ppo_update():
    """
    用 buffer 內收集到的 (s, a, r, logprob_old, value) 做一次 PPO 更新。
    """
    global states_buffer, actions_buffer, rewards_buffer
    global old_logprobs_buffer, values_buffer, dones_buffer
    global policy_W, policy_b, value_W, value_b

    if len(states_buffer) == 0:
        return

    states = np.array(states_buffer, dtype=np.float32)     # (N, STATE_DIM)
    actions_idx = np.array(actions_buffer, dtype=np.int64) # (N,)
    rewards = np.array(rewards_buffer, dtype=np.float32)   # (N,)
    old_logprobs = np.array(old_logprobs_buffer, dtype=np.float32) # (N,)
    values = np.array(values_buffer, dtype=np.float32)     # (N,)

    N = len(rewards)

    # ------- 計算 GAE Advantage -------
    advantages = np.zeros(N, dtype=np.float32)
    gae = 0.0

    for t in reversed(range(N)):
        next_value = values[t + 1] if t < N - 1 else 0.0
        delta = rewards[t] + GAMMA * next_value - values[t]
        gae = delta + GAMMA * LAMBDA * gae
        advantages[t] = gae

    returns = advantages + values

    # 標準化 advantages
    adv_mean = np.mean(advantages)
    adv_std = np.std(advantages)
    advantages = (advantages - adv_mean) / (adv_std + 1e-8)

    # ------- PPO 多輪 epoch 更新 -------
    for _ in range(PPO_EPOCHS):
        idx = np.arange(N)
        np.random.shuffle(idx)

        for start in range(0, N, PPO_BATCH_SIZE):
            batch = idx[start:start + PPO_BATCH_SIZE]
            if len(batch) == 0:
                continue

            b_states = states[batch]           # (B, STATE_DIM)
            b_actions = actions_idx[batch]     # (B,)
            b_advantages = advantages[batch]   # (B,)
            b_returns = returns[batch]         # (B,)
            b_old_logprobs = old_logprobs[batch]  # (B,)

            # Policy 前向
            logits = b_states @ policy_W + policy_b   # (B, ACTION_DIM)
            probs = stable_softmax(logits)            # (B, ACTION_DIM)

            action_probs = probs[np.arange(len(batch)), b_actions]
            logprobs = np.log(action_probs + 1e-8)
            ratios = np.exp(logprobs - b_old_logprobs)

            # PPO clipping objective（簡化版）
            surr1 = ratios * b_advantages
            surr2 = np.clip(ratios, 1 - CLIP_EPS, 1 + CLIP_EPS) * b_advantages

            # one-hot for actions
            one_hot = np.zeros_like(probs)
            one_hot[np.arange(len(batch)), b_actions] = 1.0

            # 這個 diff * advantage 是 policy gradient 的方向 (簡化寫法)
            diff = one_hot - probs   # (B, ACTION_DIM)

            # Policy gradient
            grad_W = -(b_states.T @ (diff * b_advantages[:, None])) / len(batch)
            grad_b = -np.mean(diff * b_advantages[:, None], axis=0)

            policy_W -= POLICY_LR * grad_W
            policy_b -= POLICY_LR * grad_b

            # Value function update (MSE)
            values_pred = b_states @ value_W + value_b
            errors = values_pred - b_returns

            grad_value_W = (2.0 / len(batch)) * (b_states.T @ errors)
            grad_value_b = 2.0 * np.mean(errors)

            value_W -= VALUE_LR * grad_value_W
            value_b -= VALUE_LR * grad_value_b

    # 清空 buffer
    states_buffer.clear()
    actions_buffer.clear()
    rewards_buffer.clear()
    old_logprobs_buffer.clear()
    dones_buffer.clear()
    values_buffer.clear()


def build_approach_signal_indices():
    """
    用 controlledLinks 自動偵測：
    - 哪些 phase 是 EB 綠燈
    - 哪些 phase 是 SB 綠燈
    只是用來算綠燈比例，不影響 PPO 控制邏輯。
    """
    global EB_PHASES, SB_PHASES

    controlled_links = traci.trafficlight.getControlledLinks(TLS_ID)

    EB_signal_indices = set()
    SB_signal_indices = set()

    for sig_idx, conn_list in enumerate(controlled_links):
        for inLane, outLane, via in conn_list:
            if "Node1_2_EB" in inLane:
                EB_signal_indices.add(sig_idx)
            if "Node2_4_SB" in inLane:
                SB_signal_indices.add(sig_idx)

    complete_logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(TLS_ID)[0]
    phases = complete_logic.phases

    eb_phases = set()
    sb_phases = set()

    for i, p in enumerate(phases):
        state = p.state

        for idx in EB_signal_indices:
            if idx < len(state) and state[idx] in ("G", "g"):
                eb_phases.add(i)
                break

        for idx in SB_signal_indices:
            if idx < len(state) and state[idx] in ("G", "g"):
                sb_phases.add(i)
                break

    EB_PHASES = sorted(list(eb_phases))
    SB_PHASES = sorted(list(sb_phases))

    # safety fallback
    if not EB_PHASES:
        EB_PHASES.append(0)
    if not SB_PHASES:
        SB_PHASES.append(2)


def apply_action(action, tls_id="Node2", qEB=None, qSB=None):
    """
    action = 0: 保持目前 phase
    action = 1: 切換到下一個 phase (若已滿足 MIN_GREEN_STEPS)
    只改輸出格式，不改控制邏輯。
    """
    global last_switch_step, current_simulation_step

    # 不切換
    if action == 0:
        return

    # 還沒撐滿最小綠燈時間，不切
    if current_simulation_step - last_switch_step < MIN_GREEN_STEPS:
        return

    # 讀目前 program 的 phase 數量
    program = traci.trafficlight.getAllProgramLogics(tls_id)[0]
    num_phases = len(program.phases)

    curr_phase = traci.trafficlight.getPhase(tls_id)
    next_phase = (curr_phase + 1) % num_phases

    # 切到下一個 phase，並把 phase duration 鎖很長
    traci.trafficlight.setPhase(tls_id, next_phase)
    traci.trafficlight.setPhaseDuration(tls_id, 99999)  # 把內建計時器凍住

    last_switch_step = current_simulation_step

    # 新格式：
    if qEB is None:
        qEB = -1
    if qSB is None:
        qSB = -1
    print(f"Step {current_simulation_step}: phase {curr_phase} → {next_phase} (qEB={qEB}, qSB={qSB})")


# -------------------------
# Step 8: 初始化：搶回號誌控制權
# -------------------------

TLS_ID = "Node2"

# 不管 net.xml 裡面是 static / actuated，一律改成由我們控制 phase duration
try:
    current_program = traci.trafficlight.getProgram(TLS_ID)
except Exception:
    current_program = "0"

traci.trafficlight.setProgram(TLS_ID, current_program)
traci.trafficlight.setPhaseDuration(TLS_ID, 99999)

# 建立 EB / SB 綠燈相位的 index（用來算綠燈比例）
build_approach_signal_indices()

# -------------------------
# Step 9: PPO Training Loop
# -------------------------

cumulative_reward = 0.0

print("\n=== Starting Fully Online PPO Continuous Learning (Node2 fully controlled by RL) ===")
for step in range(TOTAL_STEPS):
    current_simulation_step = step

    # 讀取當前狀態
    state = get_state()

    # 從 state 拆出 queue 與 phase（方便 log & 統計）
    qEB = int(state[0] + state[1] + state[2])
    qSB = int(state[3] + state[4] + state[5])
    phase_before = state[6]

    # 每 100 step 印一次你指定的格式
    if step % 100 == 0:
        sim_t = step * STEP_LENGTH
        print(f"[t={sim_t:4.1f}s] qEB={qEB:3d}, qSB={qSB:3d}, phase={phase_before}")

    # 從 policy 選動作
    action, logprob, value = get_action_from_policy(state)

    # 依照動作決定是否切換 phase（把 queue 傳給 log 用）
    apply_action(action, tls_id=TLS_ID, qEB=qEB, qSB=qSB)

    # 前進 SUMO 一步
    traci.simulationStep()

    # 取得下一步 state，用它計算 reward & 統計
    new_state = get_state()
    reward = get_reward(new_state)
    cumulative_reward += reward

    # 統計用（用「控制後」的新狀態）
    new_qEB = int(new_state[0] + new_state[1] + new_state[2])
    new_qSB = int(new_state[3] + new_state[4] + new_state[5])
    phase_after = new_state[6]

    sum_qEB += new_qEB
    sum_qSB += new_qSB
    sum_queue += (new_qEB + new_qSB)
    total_steps += 1

    if phase_after in EB_PHASES:
        eb_green_steps += 1
    if phase_after in SB_PHASES:
        sb_green_steps += 1

    # 儲存到 buffer（PPO 演算法本體不改）
    states_buffer.append(np.array(state, dtype=np.float32))
    actions_buffer.append(ACTIONS.index(action))
    rewards_buffer.append(reward)
    old_logprobs_buffer.append(logprob)
    values_buffer.append(value)
    dones_buffer.append(False)   # 這裡先不處理 episode 結束

    # 定期做 PPO 更新
    if len(states_buffer) >= PPO_UPDATE_INTERVAL:
        ppo_update()

# 最後再補一次更新
if len(states_buffer) > 0:
    ppo_update()

# -------------------------
# Step 10: Close SUMO & Summary
# -------------------------
traci.close()

# ====== 統計結果輸出（你指定的格式） ======
if total_steps > 0:
    avg_qEB = sum_qEB / total_steps
    avg_qSB = sum_qSB / total_steps

    eb_green_ratio = eb_green_steps / total_steps
    sb_green_ratio = sb_green_steps / total_steps

    total_veh_sec = sum_queue * STEP_LENGTH
else:
    avg_qEB = avg_qSB = 0.0
    eb_green_ratio = sb_green_ratio = 0.0
    total_veh_sec = 0.0

print("\n========== PPO Summary ==========")
print(f"EB 平均排隊        = {avg_qEB:.2f} veh")
print(f"SB 平均排隊        = {avg_qSB:.2f} veh")
print("")
print(f"EB 綠燈比例         = {eb_green_ratio:.2f}")
print(f"SB 綠燈比例         = {sb_green_ratio:.2f}")
print("")
print(f"Total queue time    = {total_veh_sec:.2f} veh·sec")
print(f"Cumulative reward   = {cumulative_reward:.2f}")
print("==========================================\n")

print("Training completed.")
