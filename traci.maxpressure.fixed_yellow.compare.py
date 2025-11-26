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
    '--delay', '0',
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

# detector halting (停等車輛數)
h_EB_0 = 0
h_EB_1 = 0
h_EB_2 = 0
h_SB_0 = 0
h_SB_1 = 0
h_SB_2 = 0

current_phase = 0

# 模擬時間設定：1800 秒，step-length = 0.1 秒 → 18000 steps
STEP_LENGTH = 0.10
SIM_TIME = 1800.0
TOTAL_STEPS = int(SIM_TIME / STEP_LENGTH)

# Min-green（以 step 為單位）
MIN_GREEN_STEPS = 100
last_switch_step = -MIN_GREEN_STEPS

# 黃燈時間設定
YELLOW_DURATION_STEPS = 30  # 3 seconds

# Max-Pressure 用到的 phase 群組（下面會自動偵測）
EB_PHASES = []  # 東西向直行相關 phase index
SB_PHASES = []  # 南北向直行相關 phase index

# 統計用變數
eb_green_steps = 0
sb_green_steps = 0

sum_qEB = 0.0
sum_qSB = 0.0
sum_hEB = 0.0
sum_hSB = 0.0
sum_halting = 0.0  # 總停等車輛數累計

# 新增統計變數
yellow_steps = 0
phase_counts = {}
total_arrived_vehicles = 0
total_steps = 0

cumulative_reward = 0.0

# 黃燈控制變數
in_yellow_phase = False
yellow_start_step = -1


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
    從 SUMO 讀取目前偵測到的排隊車數、停等車數以及目前號誌相位，
    回傳：(q_EB, q_SB, h_EB, h_SB, phase)
    """
    global q_EB_0, q_EB_1, q_EB_2, q_SB_0, q_SB_1, q_SB_2
    global h_EB_0, h_EB_1, h_EB_2, h_SB_0, h_SB_1, h_SB_2
    global current_phase

    # Eastbound detectors - Vehicle Number
    q_EB_0 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_0")
    q_EB_1 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_1")
    q_EB_2 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_2")

    # Southbound detectors - Vehicle Number
    q_SB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_0")
    q_SB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_1")
    q_SB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_2")

    # Eastbound detectors - Halting Number
    h_EB_0 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_0")
    h_EB_1 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_1")
    h_EB_2 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_2")

    # Southbound detectors - Halting Number
    h_SB_0 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_0")
    h_SB_1 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_1")
    h_SB_2 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_2")

    current_phase = traci.trafficlight.getPhase(TLS_ID)

    q_EB = q_EB_0 + q_EB_1 + q_EB_2
    q_SB = q_SB_0 + q_SB_1 + q_SB_2
    h_EB = h_EB_0 + h_EB_1 + h_EB_2
    h_SB = h_SB_0 + h_SB_1 + h_SB_2

    return q_EB, q_SB, h_EB, h_SB, current_phase


def get_reward(q_EB, q_SB):
    """
    reward = - (EB + SB 總排隊車數)
    純粹拿來當作系統壓力指標，用來比較不同控制策略。
    """
    total_queue = q_EB + q_SB
    return -float(total_queue)


def max_pressure_control(q_EB, q_SB, curr_phase, step):
    """
    改進版 Max-Pressure：
    - 加入完整的黃燈過渡機制
    - q_EB > q_SB → 優先 EB 綠燈（若目前不是 EB 相位則切換）
    - q_SB > q_EB → 優先 SB 綠燈
    - 持續遵守 MIN_GREEN_STEPS
    - 切換時先進入黃燈相位,等待 YELLOW_DURATION_STEPS 後才切換到目標綠燈相位
    """
    global last_switch_step, in_yellow_phase, yellow_start_step

    # 如果正在黃燈階段
    if in_yellow_phase:
        # 檢查黃燈時間是否已經結束
        if step - yellow_start_step >= YELLOW_DURATION_STEPS:
            # 黃燈結束,切換到下一個綠燈相位
            program = traci.trafficlight.getAllProgramLogics(TLS_ID)[0]
            num_phases = len(program.phases)
            next_phase = (curr_phase + 1) % num_phases
            
            traci.trafficlight.setPhase(TLS_ID, next_phase)
            traci.trafficlight.setPhaseDuration(TLS_ID, 99999)
            
            in_yellow_phase = False
            last_switch_step = step
            print(f"Step {step}: Yellow -> Green (Phase {next_phase})")
        return

    # 綠燈階段：Max-Pressure 決策
    target_phase = curr_phase
    SWITCH_THRESHOLD = 0  # 壓力差閾值：0 輛車 (標準 Max-Pressure)
    
    # 計算壓力差
    pressure_diff = abs(q_EB - q_SB)

    # Max-Pressure 決策 (加入閾值判斷)
    if pressure_diff >= SWITCH_THRESHOLD:
        if q_EB > q_SB:
            if curr_phase not in EB_PHASES:
                target_phase = EB_PHASES[0]
        elif q_SB > q_EB:
            if curr_phase not in SB_PHASES:
                target_phase = SB_PHASES[0]
    # 否則壓力差不夠大 → 維持原相位

    # min-green 限制
    if (step - last_switch_step) >= MIN_GREEN_STEPS:
        if target_phase != curr_phase:
            # 需要切換相位,先進入黃燈
            program = traci.trafficlight.getAllProgramLogics(TLS_ID)[0]
            num_phases = len(program.phases)
            next_phase = (curr_phase + 1) % num_phases
            
            traci.trafficlight.setPhase(TLS_ID, next_phase)
            traci.trafficlight.setPhaseDuration(TLS_ID, 99999)
            
            in_yellow_phase = True
            yellow_start_step = step
            print(f"Step {step}: Green -> Yellow (Phase {next_phase}, qEB={q_EB}, qSB={q_SB}, diff={pressure_diff})")


# -------------------------
# Step 8: 初始化：搶回號誌控制權
# -------------------------

# 不管 net.xml 裡面是 static / actuated，一律改成由我們控制 phase duration
try:
    current_program = traci.trafficlight.getProgram(TLS_ID)
except Exception:
    current_program = "0"

traci.trafficlight.setProgram(TLS_ID, current_program)
traci.trafficlight.setPhaseDuration(TLS_ID, 99999)

# 先根據目前 Node2 的號誌設定，自動偵測 EB / SB phase
build_approach_signal_indices()

# -------------------------
# Step 9: Max-Pressure Control Loop
# -------------------------

print("\n=== Starting Max-Pressure Control with Proper Yellow Phase (1800 sec) ===")

for step in range(TOTAL_STEPS):
    current_simulation_step = step

    # 1. 讀取基本資訊 (Phase & RYG)
    current_phase = traci.trafficlight.getPhase(TLS_ID)
    ryg_state = traci.trafficlight.getRedYellowGreenState(TLS_ID)
    is_yellow = 'y' in ryg_state.lower() or 'Y' in ryg_state

    # 2. 統計數據收集
    q_EB, q_SB, h_EB, h_SB, phase = get_state()

    if is_yellow:
        yellow_steps += 1
    
    if phase not in phase_counts:
        phase_counts[phase] = 0
    phase_counts[phase] += 1
    total_arrived_vehicles += traci.simulation.getArrivedNumber()

    # 統計累加
    sum_qEB += q_EB
    sum_qSB += q_SB
    sum_hEB += h_EB
    sum_hSB += h_SB
    sum_halting += (h_EB + h_SB)
    total_steps += 1

    if phase in EB_PHASES:
        eb_green_steps += 1
    if phase in SB_PHASES:
        sb_green_steps += 1

    # 控制（Max-Pressure with Yellow Phase）
    max_pressure_control(q_EB, q_SB, phase, step)

    # 推進 SUMO 一步
    traci.simulationStep()

    reward = -(h_EB + h_SB)  # 與 PPO 一致：最小化停等車輛數
    cumulative_reward += reward

    # Log（每 100 step 約 10 秒印一次）
    if step % 100 == 0:
        sim_time = step * STEP_LENGTH
        status_str = "Yellow" if is_yellow else "Green"
        print(f"[t={sim_time:4.1f}s] qEB={q_EB:3d}, qSB={q_SB:3d} | hEB={h_EB:3d}, hSB={h_SB:3d} ({status_str}), phase={phase}")

# -------------------------
# Step 10: Close SUMO
# -------------------------
traci.close()

print("\nSimulation completed.\n")

# ====== 統計結果輸出（與 PPO.update.py 相同格式） ======
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
for p_idx in sorted(phase_counts.keys()):
    ratio = phase_counts[p_idx] / total_steps
    print(f"  Phase {p_idx}: {ratio:.2f}")
print("")
print(f"Total Arrived Veh   = {total_arrived_vehicles}")
print(f"Avg Delay Time      = {avg_delay_time:.2f} sec/veh")
print(f"Total Waiting Time  = {total_waiting_time:.2f} veh·sec")
print(f"Cumulative reward   = {cumulative_reward:.2f}")
print("========================================================================\n")
