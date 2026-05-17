import serial
import math
import time
import numpy as np

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# =========================================
# LIDAR START
# =========================================
lidar_ser.write(bytes([0xA5, 0x40]))   # Reset
time.sleep(2)
lidar_ser.reset_input_buffer()

lidar_ser.write(bytes([0xA5, 0x20]))   # Scan Start
lidar_ser.read(7)                       # 응답 헤더 소비

print("LIDAR START")

# =========================================
# ROBOT PHYSICAL PARAMETER
# =========================================
ROBOT_RADIUS = 17.0   # 물리 반지름 + 측면 안전 마진 (cm)
WHEEL_BASE   = 17.0   # 차동구동 휠 베이스 (cm)

# =========================================
# DRIVE PARAMETER (선속도 고정 / 각속도 하향)
# =========================================
MAX_SPEED    = 0.18   # 최대 선속도 0.18 m/s (초당 18cm 고정)
MIN_SPEED    = 0.14   # [수정] 속도가 뚝 떨어지는 걸 방지하기 위해 하한선을 0.14 m/s로 상향
MAX_W        = 0.8    # [수정] 최대 각속도를 1.6 -> 0.8 rad/s로 낮춰 급격한 털림 방지
TURN_GAIN    = 1.0    # [수정] 조향 민감도를 1.8 -> 1.0으로 낮춰 부드러운 조향 유도

SCAN_LIMIT   = 150    # 유효 인식 거리 (cm)
FRONT_RANGE  = 60     # 탐색 반경 (±60°)

# =========================================
# FILTER PARAMETER
# =========================================
EMA_ALPHA    = 0.3    # EMA 필터 계수
MEDIAN_K     = 2      # 중앙값 필터 범위

# =========================================
# SMOOTHING PARAMETER
# =========================================
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.20
DANGER_DIST      = 24     # 위험 감지 거리 (cm)

# =========================================
# GAP & INFLATION PARAMETER (부드러운 큰 선회를 위한 마진 확장)
# =========================================
SAFE_DIST          = 18   # Gap 유효 최소 거리 (cm)
INFLATION_MAX_DIST = 30   # [수정] 회전 반경이 커졌으므로, 더 멀리서(30cm) 장애물을 부풀려 크게 돌게 함

FRONT_CLEAR_DIST   = 28   # 직진 편향 판단 거리 (cm)
FRONT_CLEAR_RANGE  = 15   # 직진 편향 범위 (±15°)

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1      

EMERGENCY_DIST    = 8      # 고속 주행 기준 긴급회피 거리
REVERSE_DURATION  = 0.25   # 회전 반경 보상을 위해 후진 시간 탈출 마진 소폭 확대
ROTATE_DURATION   = 1.10   # 각속도가 낮아졌으므로 제자리 회전 시간을 조금 늘려 각도 확보
REVERSE_SPEED     = -0.12  
ROTATE_W          = 0.8    # 제자리 회전 시에도 부드럽게 돌도록 하향 조정

# =========================================
# LOOP TRAP MEMORY (원형 교차로 뺑뺑이 방지)
# =========================================
loop_counter      = 0.0    
LOOP_THRESHOLD    = 18.0   
VIRTUAL_WALL_DIST = 15.0   

# =========================================
# STATE DATA
# =========================================
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0

# =========================================
# UTIL & FILTER
# =========================================
def apply_ema(angle, new_dist_cm):
    scan_data[angle] = ((1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm)

def apply_median_filter():
    k = MEDIAN_K
    window = 2 * k + 1
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        indices = [(i + d) % 360 for d in range(-k, k + 1)]
        values  = np.sort(scan_data[indices])
        filtered[i] = values[window // 2]
    scan_data[:] = filtered

# =========================================
# OBSTACLE INFLATION
# =========================================
def inflate_obstacles(dists):
    proc = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= INFLATION_MAX_DIST:
            continue
        alpha = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists) - 1, int(i + alpha))
        proc[start_idx : end_idx + 1] = 0.0
    return proc

# =========================================
# GAP SEARCH
# =========================================
def find_gaps(proc_dists, angles):
    gaps = []
    gap_start = None
    for i, d in enumerate(proc_dists):
        if d > SAFE_DIST:
            if gap_start is None: gap_start = i
        else:
            if gap_start is not None:
                gaps.append((gap_start, i - 1))
                gap_start = None
    if gap_start is not None: gaps.append((gap_start, len(proc_dists) - 1))
    return gaps

def score_gap(gap, proc_dists, angles):
    start, end = gap
    width = end - start
    center_i = (start + end) / 2.0
    center_angle = angles[int(center_i)]
    avg_dist = np.mean(proc_dists[start : end + 1])
    return (width * 0.5 + avg_dist * 1.2 - abs(center_angle) * 0.4)

def select_best_gap(gaps, proc_dists, angles):
    best_gap, best_score = None, -1e9
    for gap in gaps:
        s = score_gap(gap, proc_dists, angles)
        if s > best_score:
            best_score, best_gap = s, gap
    return best_gap

# =========================================
# PLANNING
# =========================================
def find_best_direction(smoothing):
    global prev_angle, loop_counter
    
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    local_scan = scan_data.copy()
    
    if abs(loop_counter) > LOOP_THRESHOLD:
        if loop_counter > 0:
            for a in range(15, FRONT_RANGE + 1):
                local_scan[a % 360] = min(local_scan[a % 360], VIRTUAL_WALL_DIST)
        else:
            for a in range(-FRONT_RANGE, -14):
                local_scan[a % 360] = min(local_scan[a % 360], VIRTUAL_WALL_DIST)

    dists = np.array([local_scan[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    gaps = find_gaps(proc_dists, angles)

    if not gaps: return None

    best_gap = select_best_gap(gaps, proc_dists, angles)
    start, end = best_gap
    gap_angle = float(angles[int((start + end) / 2.0)])

    front_clear = float(np.min(local_scan[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))
    
    if front_clear > FRONT_CLEAR_DIST:         
        target = gap_angle * 0.2
        bias_label = "STRAIGHT"
    elif front_clear > EMERGENCY_DIST * 2:     
        target = gap_angle * 1.0               
        smoothing = SMOOTHING_DANGER           
        bias_label = "GAP"
    else:                                      
        target = gap_angle * 1.0
        smoothing = 0.0                        
        bias_label = "CRITICAL"

    target = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    
    return target, bias_label, front_clear

# =========================================
# CONTROL (곡선 주행 최적화 밸런싱)
# =========================================
ALIGN_THRESHOLD = 15  # [수정] 각속도가 느려졌으므로 조향 중 직진 판정 마진을 15도로 완화

def compute_cmd(target_angle):
    global loop_counter
    
    # 1. 조향 계산 (제한된 MAX_W와 둔감해진 TURN_GAIN 반영)
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    loop_counter = loop_counter * 0.98 + target_angle * 0.02

    # 2. 측면 감시 범위
    search_indices = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    relevant_min = float(np.min(scan_data[search_indices]))

    # 3. 조향 중심 상태 판단 (회전 각도가 클 때)
    if abs(target_angle) > ALIGN_THRESHOLD:
        # [수정] 제자리에서 멈춰 도는 대신, v=0.08을 주어 둥글고 부드러운 호를 그리며 전진 회전하도록 유도
        v = 0.08 if relevant_min > 20.0 else 0.0
        return v, w

    # 4. 직진 속도 계산 (하한선이 0.14로 잡혀 웬만하면 속도가 유지됨)
    obstacle_scale = min(relevant_min / 30.0, 1.0)
    speed = max(MAX_SPEED * obstacle_scale, MIN_SPEED)

    return speed, w

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

def choose_avoid_direction():
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    return 1 if left_avg >= right_avg else -1

# =========================================
# MAIN LOOP
# =========================================
print("NAVIGATION START (0.18 m/s Smooth & Cruise Dynamic Mode)")
try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5: continue

        s_flag = raw[0] & 0x01
        if (raw[0] & 0x02) >> 1 != (1 - s_flag): continue
        if (raw[1] & 0x01) != 1: continue
        if (raw[0] >> 2) < 3: continue

        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < SCAN_LIMIT:
            apply_ema(angle, dist_cm)

        if s_flag != 1: continue
        apply_median_filter()
        now = time.time()

        # --- STATE MACHINE ---
        if state == STATE_REVERSE:
            if now < maneuver_end_time: send_cmd(REVERSE_SPEED, 0.0)
            else:
                state, maneuver_end_time = STATE_ROTATE, now + ROTATE_DURATION
            continue

        if state == STATE_ROTATE:
            if now < maneuver_end_time: send_cmd(0.0, ROTATE_W * rotate_dir)
            else:
                rotate_dir *= -1
                state, prev_angle, loop_counter = STATE_NORMAL, 0.0, 0.0
                for a in range(-45, 46): scan_data[a % 360] = float(SCAN_LIMIT)
            continue

        # --- STATE_NORMAL ---
        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))
        if front_min < EMERGENCY_DIST:
            rotate_dir = choose_avoid_direction()
            state, maneuver_end_time = STATE_REVERSE, now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result = find_best_direction(smoothing)

        if result is None:
            rotate_dir = choose_avoid_direction()
            state, maneuver_end_time = STATE_REVERSE, now + REVERSE_DURATION
            continue

        target_angle, bias_label, front_clear = result
        v, w = compute_cmd(target_angle)
        send_cmd(v, w)

        print(f"TRG:{target_angle:5.1f}° | v:{v:.2f} | w:{w:.2f} | f_min:{front_min:.1f} | LOOP_CTR:{loop_counter:+.1f}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
