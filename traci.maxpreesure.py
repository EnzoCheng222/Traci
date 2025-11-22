# Step 1: Add modules to provide access to specific libraries and functions
import os  # Module provides functions to handle file paths, directories, environment variables
import sys  # Module provides access to Python-specific system parameters and functions
import matplotlib.pyplot as plt  # Visualization（這版不畫圖，保留也沒關係）

# ★ 取得這支程式所在的資料夾，之後用來組 sumocfg 的絕對路徑
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Step 2: Establish path to SUMO (SUMO_HOME)
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 3: Add Traci module to provide access to specific libraries and functions
import traci  # Static network information (such as reading and analyzing network files)

# Step 4: Define Sumo configuration
sumocfg_path = os.path.join(SCRIPT_DIR, 'Traci.sumocfg')

if not os.path.exists(sumocfg_path):
    sys.exit(f"找不到 SUMO 設定檔: {sumocfg_path}，請確認檔名與位置是否正確。")

Sumo_config = [
    'sumo-gui',
    '-c', sumocfg_path,
    '--step-length', '0.10',
    '--delay', '1000',
    '--lateral-resolution', '0'
]

# Step 5: Open connection between SUMO and Traci
traci.start(Sumo_config)
traci.gui.setSchema("View #0", "real world")

TLS_ID = "Node2"

# -------------------------
# Step 6: Define Variables
# -------------------------

# detector queue (sumo 會回報每個 lane area 當下的車輛數)
q_EB_0 = 0
q_EB_1 = 0
q_EB_2 = 0
q_SB_0 = 0
q_SB_1 = 0
q_SB_2 = 0
current_phase = 0

# 模擬時間設定：1800 秒，step-length = 0.1 秒 → 18000 steps
STEP_LENGTH = 0.10
SIM_TIME = 1800.0
TOTAL_STEPS = int(SIM_TIME / STEP_LENGTH)

# Min-green（以 step 為單位）
MIN_GREEN_STEPS = 100
last_switch_step = -MIN_GREEN_STEPS

# Max-Pressure 用到的 phase 群組（下面會自動偵測）
EB_PHASES = []  # 東西向直行相關 phase index
SB_PHASES = []  # 南北向直行相關 phase index

# 統計用變數
eb_green_steps = 0
sb_green_steps = 0

sum_qEB = 0.0
sum_qSB = 0.0
sum_queue = 0.0

cumulative_reward = 0.0


# -------------------------
# Step 7: Define Functions
# -------------------------

def build_approach_signal_indices():
    """
    使用 traci.trafficlight.getControlledLinks() 反推：
    - 哪些 signal index 負責 Node1_2_EB_*（東向）
    - 哪些 signal index 負責 Node2_4_SB_*（南向）

    然後再用 phase.state[signal_index] 是否為 'G' 或 'g' 來判斷：
    - 哪些 phase 屬於 EB 綠燈
    - 哪些 phase 屬於 SB 綠燈
    """
    global EB_PHASES, SB_PHASES

    controlled_links = traci.trafficlight.getControlledLinks(TLS_ID)
    # controlled_links 是 list，長度 = phase.state 的長度
    # 每個 element 是 [(inLane, outLane, viaLane), ...] 的 list

    EB_signal_indices = set()
    SB_signal_indices = set()

    for sig_idx, conn_list in enumerate(controlled_links):
        for inLane, outLane, via in conn_list:
            # 依你的命名：Node1_2_EB_* 是東向 / Node2_4_SB_* 是南向
            if "Node1_2_EB" in inLane:
                EB_signal_indices.add(sig_idx)
            if "Node2_4_SB" in inLane:
                SB_signal_indices.add(sig_idx)

    complete_logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(TLS_ID)[0]
    phases = complete_logic.phases

    print("\n=== TLS Phase & Signal Mapping ===")
    print("EB_signal_indices =", sorted(EB_signal_indices))
    print("SB_signal_indices =", sorted(SB_signal_indices))

    eb_phases = set()
    sb_phases = set()

    for i, p in enumerate(phases):
        state = p.state  # 每一個 char 對應一個 signal index

        # 判斷這個 phase 是否讓 EB 任一個 signal index 變綠
        for idx in EB_signal_indices:
            if idx < len(state) and state[idx] in ("G", "g"):
                eb_phases.add(i)
                break

        # 判斷這個 phase 是否讓 SB 任一個 signal index 變綠
        for idx in SB_signal_indices:
            if idx < len(state) and state[idx] in ("G", "g"):
                sb_phases.add(i)
                break

        print(f"Phase {i}: state={state}")

    EB_PHASES = sorted(list(eb_phases))
    SB_PHASES = sorted(list(sb_phases))

    print("\n→ 偵測到東西向綠燈相位 =", EB_PHASES)
    print("→ 偵測到南北向綠燈相位 =", SB_PHASES)

    # safety fallback
    if not EB_PHASES:
        EB_PHASES.append(0)
    if not SB_PHASES:
        # 如果完全偵測不到 SB，暫時假設 2 是 SB 主綠
        SB_PHASES.append(2)

    print("\n最終使用相位：")
    print("EB_PHASES =", EB_PHASES)
    print("SB_PHASES =", SB_PHASES)
    print("=================================\n")


def get_state():
    """
    從 SUMO 讀取目前偵測到的排隊車數以及目前號誌相位，
    回傳：(q_EB, q_SB, phase)
    """
    global q_EB_0, q_EB_1, q_EB_2, q_SB_0, q_SB_1, q_SB_2, current_phase

    # Eastbound detectors
    q_EB_0 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_0")
    q_EB_1 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_1")
    q_EB_2 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_2")

    # Southbound detectors
    q_SB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_0")
    q_SB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_1")
    q_SB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_2")

    current_phase = traci.trafficlight.getPhase(TLS_ID)

    q_EB = q_EB_0 + q_EB_1 + q_EB_2
    q_SB = q_SB_0 + q_SB_1 + q_SB_2

    return q_EB, q_SB, current_phase


def get_reward(q_EB, q_SB):
    """
    reward = - (EB + SB 總排隊車數)
    純粹拿來當作系統壓力指標，用來比較不同控制策略。
    """
    total_queue = q_EB + q_SB
    return -float(total_queue)


def max_pressure_control(q_EB, q_SB, curr_phase, step):
    """
    簡化版 Max-Pressure：
    - q_EB > q_SB → 優先 EB 綠燈（若目前不是 EB 相位則切換）
    - q_SB > q_EB → 優先 SB 綠燈
    - 持續遵守 MIN_GREEN_STEPS
    """
    global last_switch_step

    target_phase = curr_phase

    # Max-Pressure 決策
    if q_EB > q_SB:
        if curr_phase not in EB_PHASES:
            target_phase = EB_PHASES[0]
    elif q_SB > q_EB:
        if curr_phase not in SB_PHASES:
            target_phase = SB_PHASES[0]
    # 否則同壓力 → 維持原相位

    # min-green 限制
    if (step - last_switch_step) >= MIN_GREEN_STEPS:
        if target_phase != curr_phase:
            print(f"Step {step}: phase {curr_phase} → {target_phase} (qEB={q_EB}, qSB={q_SB})")
            traci.trafficlight.setPhase(TLS_ID, target_phase)
            last_switch_step = step


# -------------------------
# Step 8: Max-Pressure Control Loop
# -------------------------

# 先根據目前 Node2 的號誌設定，自動偵測 EB / SB phase
build_approach_signal_indices()

print("\n=== Starting Max-Pressure Control Simulation (1800 sec) ===")

for step in range(TOTAL_STEPS):
    current_simulation_step = step

    q_EB, q_SB, phase = get_state()

    # 控制（Max-Pressure）
    max_pressure_control(q_EB, q_SB, phase, step)

    # 推進 SUMO 一步
    traci.simulationStep()

    # 統計指標
    sum_qEB += q_EB
    sum_qSB += q_SB
    sum_queue += (q_EB + q_SB)

    if phase in EB_PHASES:
        eb_green_steps += 1
    if phase in SB_PHASES:
        sb_green_steps += 1

    reward = get_reward(q_EB, q_SB)
    cumulative_reward += reward

    # Log（每 100 step 約 10 秒印一次）
    if step % 100 == 0:
        sim_time = step * STEP_LENGTH
        print(f"[t={sim_time:5.1f}s] qEB={q_EB:3d}, qSB={q_SB:3d}, phase={phase}")

# -------------------------
# Step 9: Close SUMO
# -------------------------
traci.close()

print("\nSimulation completed.\n")

# ====== 統計結果輸出 ======
avg_qEB = sum_qEB / TOTAL_STEPS
avg_qSB = sum_qSB / TOTAL_STEPS

eb_green_ratio = eb_green_steps / TOTAL_STEPS
sb_green_ratio = sb_green_steps / TOTAL_STEPS

total_veh_sec = sum_queue * STEP_LENGTH

print("========== Max-Pressure Summary ==========")
print(f"EB 平均排隊        = {avg_qEB:.2f} veh")
print(f"SB 平均排隊        = {avg_qSB:.2f} veh")
print("")
print(f"EB 綠燈比例         = {eb_green_ratio:.2f}")
print(f"SB 綠燈比例         = {sb_green_ratio:.2f}")
print("")
print(f"Total queue time    = {total_veh_sec:.2f} veh·sec")
print(f"Cumulative reward   = {cumulative_reward:.2f}")
print("==========================================\n")
