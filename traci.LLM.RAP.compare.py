# ================================
# LLM-Based Traffic Signal Control (RAP Method)
# Based on "Large Language Models as Traffic Signal Control Agents" (arXiv:2312.16044v5)
# ================================

import os
import sys
import json
import time
from openai import OpenAI

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
# LLM Configuration
# ================================================

# ⚠️ IMPORTANT: Set your OpenAI API key
# Option 1: Environment variable (recommended)
# Option 2: Hardcode here (not recommended for production)
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "your-api-key-here")

if OPENAI_API_KEY == "your-api-key-here":
    print("⚠️  WARNING: Please set your OpenAI API key!")
    print("   Option 1: Set environment variable OPENAI_API_KEY")
    print("   Option 2: Edit the script and replace 'your-api-key-here'")
    sys.exit(1)

# Initialize OpenAI client
client = OpenAI(api_key=OPENAI_API_KEY)

# LLM Model
LLM_MODEL = "gpt-4-turbo"  # or "gpt-4", "gpt-3.5-turbo"

# Control interval (how often to query LLM, in seconds)
LLM_CONTROL_INTERVAL = 20.0  # Query LLM every 20 seconds

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
MAX_GREEN = 60.0   # 最大綠燈時間（秒）
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
    構建 Reasoning + Action Prompt (RAP)
    
    Args:
        state: 交通狀態字典
        
    Returns:
        str: 完整的 prompt
    """
    prompt = f"""You are an intelligent traffic signal controller for an urban intersection.

Your objective is to minimize traffic congestion by reducing queue length and vehicle delay.

{state['text_description']}

Instructions:
1. Analyze the current traffic condition
2. Compare the two possible actions (EB vs SB)
3. Choose the best phase to minimize total queue and halting vehicles
4. Suggest a green light duration between {MIN_GREEN} and {MAX_GREEN} seconds

Think step-by-step and output your decision in the following format:

Thought: <Your step-by-step reasoning>
Action: {{"phase": "EB" or "SB", "duration": <number between {MIN_GREEN} and {MAX_GREEN}>}}

IMPORTANT: Your final output MUST include a valid JSON in the Action field.
"""
    return prompt


def query_LLM(prompt, max_retries=3):
    """
    查詢 LLM 並獲取回應
    
    Args:
        prompt: 完整的 prompt
        max_retries: 最大重試次數
        
    Returns:
        str: LLM 的回應文字
    """
    for attempt in range(max_retries):
        try:
            response = client.chat.completions.create(
                model=LLM_MODEL,
                messages=[
                    {"role": "system", "content": "You are a traffic signal control expert."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,  # Lower temperature for more consistent outputs
                max_tokens=500
            )
            
            return response.choices[0].message.content
            
        except Exception as e:
            print(f"⚠️  LLM API Error (attempt {attempt + 1}/{max_retries}): {e}")
            if attempt < max_retries - 1:
                time.sleep(2)  # Wait before retry
            else:
                raise
    
    return None


def parse_LLM_response(response_text):
    """
    解析 LLM 回應，提取 phase 和 duration
    
    Args:
        response_text: LLM 的完整回應
        
    Returns:
        tuple: (phase, duration, thought) 或 (None, None, None) 如果解析失敗
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
        
        # 嘗試從多種格式中提取 JSON
        if not action_json:
            # 嘗試直接搜索 JSON
            import re
            json_match = re.search(r'\{[^}]+\}', response_text)
            if json_match:
                action_json = json_match.group(0)
        
        # 解析 JSON
        action = json.loads(action_json)
        phase = action.get("phase", "").upper()
        duration = float(action.get("duration", MIN_GREEN))
        
        # 驗證 phase
        if phase not in PHASE_MAP:
            print(f"⚠️  Invalid phase from LLM: {phase}")
            return None, None, thought
        
        # Clamp duration
        duration = max(MIN_GREEN, min(MAX_GREEN, duration))
        
        return phase, duration, thought
        
    except Exception as e:
        print(f"⚠️  Failed to parse LLM response: {e}")
        print(f"   Response: {response_text[:200]}...")
        return None, None, None


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


# ================================================
# LLM Controller
# ================================================

def LLM_controller(state):
    """
    主要的 LLM 控制函數
    
    Args:
        state: 交通狀態
        
    Returns:
        tuple: (phase_index, duration, thought, is_fallback)
    """
    try:
        # 1. Build prompt
        prompt = build_RAP_prompt(state)
        
        # 2. Query LLM
        response = query_LLM(prompt)
        
        if response is None:
            print("⚠️  LLM query failed, using fallback...")
            phase, duration = fallback_max_pressure(state)
            return PHASE_MAP[phase], duration, "Fallback to Max Pressure", True
        
        # 3. Parse response
        phase, duration, thought = parse_LLM_response(response)
        
        if phase is None:
            print("⚠️  LLM response parsing failed, using fallback...")
            phase, duration = fallback_max_pressure(state)
            return PHASE_MAP[phase], duration, "Fallback to Max Pressure", True
        
        # 4. Return valid decision
        return PHASE_MAP[phase], duration, thought, False
        
    except Exception as e:
        print(f"⚠️  LLM Controller Error: {e}")
        phase, duration = fallback_max_pressure(state)
        return PHASE_MAP[phase], duration, f"Error fallback: {e}", True


# ================================================
# Phase Execution
# ================================================

def execute_phase(target_phase, duration):
    """
    執行指定的相位和持續時間
    
    Args:
        target_phase: 目標相位索引 (0 or 2)
        duration: 綠燈持續時間（秒）
        
    Returns:
        int: 實際執行的 steps 數
    """
    current_phase = traci.trafficlight.getPhase(TLS_ID)
    
    # 如果需要切換相位，先切換到黃燈
    if current_phase != target_phase:
        # 切換到對應的黃燈相位
        yellow_phase = target_phase + 1 if target_phase in [0, 2] else target_phase - 1
        
        # 執行黃燈
        traci.trafficlight.setPhase(TLS_ID, yellow_phase)
        yellow_steps = int(YELLOW_TIME / STEP_LENGTH)
        for _ in range(yellow_steps):
            traci.simulationStep()
        
        # 切換到目標綠燈
        traci.trafficlight.setPhase(TLS_ID, target_phase)
    
    # 執行綠燈階段
    green_steps = int(duration / STEP_LENGTH)
    for _ in range(green_steps):
        traci.simulationStep()
    
    return green_steps


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
print(f"Model: {LLM_MODEL}")
print(f"Control Interval: {LLM_CONTROL_INTERVAL}s")
print("="*70 + "\n")

# 啟動 SUMO
traci.start(Sumo_config)
traci.gui.setSchema("View #0", "real world")

step = 0
next_control_time = 0.0

while step < TOTAL_STEPS:
    current_time = step * STEP_LENGTH
    
    # 檢查是否需要進行 LLM 決策
    if current_time >= next_control_time:
        # 1. Encode state
        state = encode_traffic_state()
        
        # 2. Query LLM
        print(f"\n[t={current_time:.1f}s] Querying LLM...")
        print(f"  Current: qEB={state['queue_EB']}, qSB={state['queue_SB']}")
        
        api_start = time.time()
        phase_index, duration, thought, is_fallback = LLM_controller(state)
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
            "duration": duration,
            "thought": thought,
            "is_fallback": is_fallback,
            "api_time": api_time
        }
        llm_decisions.append(decision)
        
        print(f"  Decision: {PHASE_NAMES[phase_index]}, duration={duration:.1f}s")
        print(f"  Thought: {thought[:100]}...")
        if is_fallback:
            print(f"  ⚠️  FALLBACK")
        print(f"  API time: {api_time:.2f}s")
        
        # 3. Execute phase
        execute_phase(phase_index, duration)
        
        # 更新下次控制時間
        next_control_time = current_time + LLM_CONTROL_INTERVAL + duration
        
        # 更新 step（因為 execute_phase 已經推進了模擬）
        step = int(traci.simulation.getTime() / STEP_LENGTH)
    else:
        # 繼續執行當前相位
        traci.simulationStep()
        step += 1
    
    # 收集統計數據
    state = encode_traffic_state()
    q_EB = state["queue_EB"]
    q_SB = state["queue_SB"]
    h_EB = state["halting_EB"]
    h_SB = state["halting_SB"]
    current_phase = state["current_phase"]
    
    # 累積統計
    sum_qEB += q_EB
    sum_qSB += q_SB
    sum_hEB += h_EB
    sum_hSB += h_SB
    sum_halting += (h_EB + h_SB)
    total_steps += 1
    
    # 相位統計
    if current_phase not in phase_counts:
        phase_counts[current_phase] = 0
    phase_counts[current_phase] += 1
    
    if current_phase == EB_PHASE:
        eb_green_steps += 1
    elif current_phase == SB_PHASE:
        sb_green_steps += 1
    else:
        yellow_steps += 1
    
    # Reward
    reward = -(h_EB + h_SB)
    cumulative_reward += reward
    
    total_arrived_vehicles += traci.simulation.getArrivedNumber()


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
