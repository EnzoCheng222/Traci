# ================================
# LLM-Based Traffic Signal Control (RAP Method)
# Based on "Large Language Models as Traffic Signal Control Agents" (arXiv:2312.16044v5)
# ================================

import os
import sys
import json
import time
import google.generativeai as genai

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

# ================================================
# Google Gemini Configuration
# ================================================

# ⚠️ IMPORTANT: Set your Google Gemini API key
# Get your free API key from: https://makersuite.google.com/app/apikey
# Set it as an environment variable: GEMINI_API_KEY
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

if not GEMINI_API_KEY:
    print("⚠️  ERROR: GEMINI_API_KEY environment variable not set!")
    print("   Step 1: Go to https://makersuite.google.com/app/apikey")
    print("   Step 2: Create a new API key (FREE)")
    print("   Step 3: Set environment variable GEMINI_API_KEY")
    print("   Example (Windows PowerShell):")
    print("      $env:GEMINI_API_KEY='your-api-key-here'")
    print("   Example (Linux/Mac):")
    print("      export GEMINI_API_KEY='your-api-key-here'")
    sys.exit(1)

# Configure Gemini
genai.configure(api_key=GEMINI_API_KEY)

# LLM Model (Using the latest available model)
LLM_MODEL = "gemini-2.5-flash"

# Initialize Gemini model
model = genai.GenerativeModel(LLM_MODEL)

# Control interval (how often to query LLM, in seconds)
LLM_CONTROL_INTERVAL = 20.0  # Fixed 20s interval

# ================================================
# Traffic Signal Control Parameters
# ================================================

TLS_ID = "Node2"
STEP_LENGTH = 0.1           # 秒/step
SIM_TIME = 1800.0           # 總模擬時間（秒）
TOTAL_STEPS = int(SIM_TIME / STEP_LENGTH)

# Phase definitions (4-phase system)
EB_PHASE = 0   # 東西向綠燈
SB_PHASE = 2   # 南北向綠燈

PHASE_MAP = {
    "EB": 0,
    "SB": 2
}

PHASE_NAMES = {
    0: "EB (East-West)",
    2: "SB (South-North)"
}

# Green time constraints
MIN_GREEN = 10.0   # 最小綠燈時間（秒）
MAX_GREEN = 30.0   # 最大綠燈時間（秒）
YELLOW_TIME = 3.0  # 黃燈時間（秒）

# ================================================
# State Encoder
# ================================================

def encode_traffic_state():
    """
    將當前交通狀態編碼為自然語言描述
    
    Returns:
        dict: 包含交通狀態和文字描述的字典
    """
    # 讀取 EB 方向數據
    q_EB_0 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_0")
    q_EB_1 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_1")
    q_EB_2 = traci.lanearea.getLastStepVehicleNumber("Node1_2_EB_2")
    
    h_EB_0 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_0")
    h_EB_1 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_1")
    h_EB_2 = traci.lanearea.getLastStepHaltingNumber("Node1_2_EB_2")
    
    # 讀取 SB 方向數據
    q_SB_0 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_0")
    q_SB_1 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_1")
    q_SB_2 = traci.lanearea.getLastStepVehicleNumber("Node2_4_SB_2")
    
    h_SB_0 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_0")
    h_SB_1 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_1")
    h_SB_2 = traci.lanearea.getLastStepHaltingNumber("Node2_4_SB_2")
    
    # 總計
    q_EB = q_EB_0 + q_EB_1 + q_EB_2
    q_SB = q_SB_0 + q_SB_1 + q_SB_2
    h_EB = h_EB_0 + h_EB_1 + h_EB_2
    h_SB = h_SB_0 + h_SB_1 + h_SB_2
    
    # 當前相位和持續時間
    current_phase = traci.trafficlight.getPhase(TLS_ID)
    
    state = {
        "queue_EB": q_EB,
        "queue_SB": q_SB,
        "halting_EB": h_EB,
        "halting_SB": h_SB,
        "current_phase": current_phase,
        "current_phase_name": PHASE_NAMES.get(current_phase, f"Phase {current_phase}")
    }
    
    # 生成自然語言描述
    text_description = f"""Traffic Condition at Node2:

East-West Direction (EB):
  - Queue length: {q_EB} vehicles
  - Halting vehicles: {h_EB} vehicles

South-North Direction (SB):
  - Queue length: {q_SB} vehicles
  - Halting vehicles: {h_SB} vehicles

Current Signal Phase: {state['current_phase_name']}

Available Actions:
  - "EB": Give green light to East-West direction
  - "SB": Give green light to South-North direction
"""
    
    state["text_description"] = text_description
    return state


# ================================================
# LLM Decision Module (RAP Method)
# ================================================

def build_RAP_prompt(state):
    """
    構建 Reasoning + Action Prompt (RAP) - Simplified
    """
    prompt = f"""You are an intelligent traffic signal controller.

Traffic Status:
{state['text_description']}

Goal: Minimize queue length and delay.

Task: Decide which phase should be active for the next 20 seconds.
- Option 1: "EB" (East-West Green)
- Option 2: "SB" (South-North Green)

Instructions:
1. Analyze the queue and halting vehicles.
2. If the current green phase still has a long queue, keep it (Extension).
3. If the waiting phase has a much longer queue, switch to it.
4. Output JSON only.

Format:
Thought: <reasoning>
Action: {{"phase": "EB" or "SB"}}
"""
    return prompt

def query_LLM(prompt, max_retries=3):
    """
    查詢 Gemini LLM 並獲取回應
    
    Args:
        prompt: 完整的 prompt
        max_retries: 最大重試次數
        
    Returns:
        str: LLM 的回應文字
    """
    for attempt in range(max_retries):
        try:
            # Gemini API call
            response = model.generate_content(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.3,  # Lower temperature for more consistent outputs
                    max_output_tokens=500,
                )
            )
            
            # 成功後稍微等待，避免太快發送下一個請求
            time.sleep(1.0)
            return response.text
            
        except Exception as e:
            print(f"⚠️  Gemini API Error (attempt {attempt + 1}/{max_retries}): {e}")
            
            # 如果是 Rate Limit (429)，不要長時間等待，直接 Fallback
            if "429" in str(e):
                print(f"   ⚠️ Rate Limit hit! Skipping LLM for this step (Fallback to Max Pressure).")
                return None
            else:
                if attempt < max_retries - 1:
                    time.sleep(2)  # Wait before retry
                else:
                    return None
    
    return None

def parse_LLM_response(response_text):
    """
    解析 LLM 回應，只提取 phase
    """
    try:
        # 提取 Thought 和 Action
        lines = response_text.split("\n")
        thought = ""
        action_json = ""
        
        in_action = False
        for line in lines:
            if line.strip().startswith("Thought:"):
                thought = line.replace("Thought:", "").strip()
            elif line.strip().startswith("Action:"):
                action_json = line.replace("Action:", "").strip()
                in_action = True
            elif in_action and line.strip():
                action_json += " " + line.strip()
        
        if not action_json:
            import re
            json_match = re.search(r'\{[^}]+\}', response_text)
            if json_match:
                action_json = json_match.group(0)
        
        action = json.loads(action_json)
        phase = action.get("phase", "").upper()
        
        if phase not in PHASE_MAP:
            return None, thought
        
        return phase, thought
        
    except Exception as e:
        print(f"⚠️  Failed to parse: {e}")
        return None, None

# ================================================
# Fallback Controller (Max Pressure)
# ================================================

def fallback_max_pressure(state):
    """
    Fallback 到 Max Pressure 演算法
    
    Args:
        state: 交通狀態
        
    Returns:
        tuple: (phase, duration)
    """
    q_EB = state["queue_EB"]
    q_SB = state["queue_SB"]
    
    # Max Pressure: 選擇壓力大的方向
    if q_EB > q_SB:
        return "EB", MIN_GREEN + 10.0
    else:
        return "SB", MIN_GREEN + 10.0

def LLM_controller(state):
    """
    LLM 控制函數 - 只返回目標相位
    """
    try:
        prompt = build_RAP_prompt(state)
        response = query_LLM(prompt)
        
        if response is None:
            phase = fallback_max_pressure(state) # Fallback returns ("EB", time)
            return PHASE_MAP[phase[0]], "Fallback (API Fail)", True
        
        phase, thought = parse_LLM_response(response)
        
        if phase is None:
            phase = fallback_max_pressure(state)
            return PHASE_MAP[phase[0]], "Fallback (Parse Fail)", True
        
        return PHASE_MAP[phase], thought, False
        
    except Exception as e:
        print(f"⚠️  Error: {e}")
        phase = fallback_max_pressure(state)
        return PHASE_MAP[phase[0]], f"Error: {e}", True

def execute_phase_fixed_interval(target_phase):
    """
    執行固定的 20 秒區間
    """
    global sum_qEB, sum_qSB, sum_hEB, sum_hSB, sum_halting
    global eb_green_steps, sb_green_steps, yellow_steps
    global total_arrived_vehicles, total_steps, cumulative_reward
    global phase_counts
    
    current_phase = traci.trafficlight.getPhase(TLS_ID)
    steps_to_run = int(LLM_CONTROL_INTERVAL / STEP_LENGTH)
    
    # 定義收集數據函數
    def step_and_collect():
        global sum_qEB, sum_qSB, sum_hEB, sum_hSB, sum_halting
        global eb_green_steps, sb_green_steps, yellow_steps
        global total_arrived_vehicles, total_steps, cumulative_reward
        global phase_counts
        
        traci.simulationStep()
        total_steps += 1
        
        state = encode_traffic_state()
        curr_p = state["current_phase"]
        
        # 累積統計
        sum_qEB += state["queue_EB"]
        sum_qSB += state["queue_SB"]
        sum_hEB += state["halting_EB"]
        sum_hSB += state["halting_SB"]
        sum_halting += (state["halting_EB"] + state["halting_SB"])
        
        if curr_p not in phase_counts: phase_counts[curr_p] = 0
        phase_counts[curr_p] += 1
        
        if curr_p == EB_PHASE: eb_green_steps += 1
        elif curr_p == SB_PHASE: sb_green_steps += 1
        else: yellow_steps += 1
            
        cumulative_reward += -(state["halting_EB"] + state["halting_SB"])
        total_arrived_vehicles += traci.simulation.getArrivedNumber()

    # 邏輯：
    # 1. 如果 Target == Current: 直接跑 20s (Extension)
    # 2. 如果 Target != Current: 切換 (3s 黃燈) + (17s 綠燈)
    
    if current_phase == target_phase:
        # 保持當前相位
        for _ in range(steps_to_run):
            step_and_collect()
    else:
        # 切換相位
        # 1. 黃燈 (3s)
        if current_phase in [0, 2]:
            yellow_phase = current_phase + 1
            traci.trafficlight.setPhase(TLS_ID, yellow_phase)
            yellow_steps_duration = int(YELLOW_TIME / STEP_LENGTH)
            for _ in range(yellow_steps_duration):
                step_and_collect()
            steps_to_run -= yellow_steps_duration
            
        # 2. 新綠燈 (剩餘時間, 約 17s)
        traci.trafficlight.setPhase(TLS_ID, target_phase)
        for _ in range(steps_to_run):
            step_and_collect()


# ================================================
# 統計變數
# ================================================

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

# LLM 控制歷史
control_history = []
llm_decisions = []
fallback_count = 0
api_call_count = 0
total_api_time = 0.0


# ================================================
# 主控制迴圈
# ================================================

print("\n" + "="*70)
print("Starting LLM-Based Traffic Signal Control (RAP Method)")
print("="*70)
print(f"Model: Google {LLM_MODEL}")
print(f"Control Interval: Fixed {LLM_CONTROL_INTERVAL}s")
print("="*70 + "\n")

# 啟動 SUMO
traci.start(Sumo_config)
traci.gui.setSchema("View #0", "real world")

step = 0
next_control_time = 0.0

while step < TOTAL_STEPS:
    current_time = step * STEP_LENGTH
    
    # 1. Encode state
    state = encode_traffic_state()
    
    # 2. Query LLM (Every 20s)
    print(f"\n[t={current_time:.1f}s] Querying LLM...")
    print(f"  Current: qEB={state['queue_EB']}, qSB={state['queue_SB']}")
    
    api_start = time.time()
    # LLM now only returns target phase
    phase_index, thought, is_fallback = LLM_controller(state)
    api_time = time.time() - api_start
    
    total_api_time += api_time
    api_call_count += 1
    if is_fallback:
        fallback_count += 1
    
    # 記錄決策
    decision = {
        "time": current_time,
        "state": state,
        "phase": PHASE_NAMES[phase_index],
        "thought": thought,
        "is_fallback": is_fallback,
        "api_time": api_time
    }
    llm_decisions.append(decision)
    
    print(f"  Decision: {PHASE_NAMES[phase_index]}")
    print(f"  Thought: {thought[:100]}...")
    if is_fallback:
        print(f"  ⚠️  FALLBACK")
    print(f"  API time: {api_time:.2f}s")
    
    # 3. Execute Fixed Interval (20s)
    # This function handles simulation steps and data collection
    execute_phase_fixed_interval(phase_index)
    
    # 更新 step
    step = int(traci.simulation.getTime() / STEP_LENGTH)


# ================================================
# 結束模擬
# ================================================

traci.close()

print("\n" + "="*70)
print("Simulation completed.")
print("="*70 + "\n")


# ================================================
# 輸出統計結果
# ================================================

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
    
    # 平均延滯時間
    if total_arrived_vehicles > 0:
        avg_delay_time = total_waiting_time / total_arrived_vehicles
    else:
        avg_delay_time = 0.0
        
    # 平均停等長度
    avg_queue_total = (sum_qEB + sum_qSB) / total_steps

else:
    avg_qEB = avg_qSB = 0.0
    avg_hEB = avg_hSB = 0.0
    eb_green_ratio = sb_green_ratio = 0.0
    total_waiting_time = 0.0
    yellow_ratio = 0.0
    avg_delay_time = 0.0
    avg_queue_total = 0.0

print("\n========== LLM-RAP Summary (Objective: Min Waiting Time) ==========")
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
print("")
print("--- LLM Performance Metrics ---")
print(f"Total API Calls     = {api_call_count}")
print(f"Fallback Count      = {fallback_count} ({fallback_count/api_call_count*100:.1f}%)")
print(f"Avg API Latency     = {total_api_time/api_call_count:.2f} sec")
print(f"Total API Time      = {total_api_time:.2f} sec")
print("========================================================================\n")
