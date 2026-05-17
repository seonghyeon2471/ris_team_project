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
# DRIVE PARAMETER (상한선 0.18 m/s 상향 및 하한선 조정)
# =========================================
MAX_SPEED    = 0.18   # [수정] 최대 선속도 상향 (0.14 -> 0.18 m/s)
MIN_SPEED    = 0.12   # [수정] 최고 속도에 맞춰 최소 속도 하한선도 0.12 m/s로 밸런싱
MAX_W        = 1.6    # [수정] 속도 증가에 따라 민첩한 회전을 위해 최대 각속도 살짝 상향 (1.5 -> 1.6)
TURN_GAIN    = 1.8    # 조향 게인

SCAN_LIMIT   = 150    # 유효 인식 거리 (cm)
FRONT_RANGE  = 60     # 탐색 반경 (±60°) - 이 범위 내 측면 충돌 감시

# =========================================
# FILTER PARAMETER
# =========================================
EMA_ALPHA    = 0.3    # EMA 필터 계수
MEDIAN_K     = 2      # 중앙값 필터 범위

# =========================================
# SMOOTHING PARAMETER (안전 마진 상향)
# =========================================
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.20
DANGER_DIST      = 24     # [수정] 속도가 빨라진 만큼 위험 감지 거리를 늘림 (18cm -> 24cm)

# =========================================
# GAP & INFLATION PARAMETER (고속 대비 확장)
# =========================================
SAFE_DIST          = 18   # [수정] 진입 안전 마진 확장 (17cm -> 18cm)
INFLATION_MAX_DIST = 28   # [수정] 더 멀리서부터 장애물을 불려 우회 궤적을 크게 그림 (25cm -> 28cm)

FRONT_CLEAR_DIST   = 28   # [수정] 직진 편향 판단 거리 상향 (23cm -> 28cm)
FRONT_CLEAR_RANGE  = 15   # 직진 편향 범위 (±15°)

# =========================================
# STATE MACHINE (탈출 머누버 마진 확보)
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1      # +1: 좌회전, -1: 우회전

EMERGENCY_DIST    = 8      # [수정] 고속 급제동 거리를 고려해 긴급회피 거리 상향 (6cm -> 8cm)
REVERSE_DURATION  = 0.22   # [수정] 후진 시간 소폭 확대
ROTATE_DURATION   = 1.00
REVERSE_SPEED     = -0.12  # [수정] 후진 속도 가속 (-0.10 -> -0.12)
ROTATE_W          = 1.0    # [수정] 제자리 회전 속도 보강 (0.9 -> 1.0)

# =========================================
# LOOP TRAP MEMORY (원형 교차로 뺑뺑이 방지)
# =========================================
loop_counter      = 0.0    
LOOP_THRESHOLD    = 18.0   # [수정] 고속 선회 시 일시적 오작동 방지를 위해 임계값 상향 (15.0 -> 18.0)
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

# =========================================
# PLANNING
# =========================================
def find_best_direction(smoothing):
    global prev_angle, loop_counter
    
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    local_scan = scan_data.copy()
    
    # 원형 교차로 뺑뺑이 탈출 로직 (가상 벽)
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

def select_best_gap(gaps, proc_dists, angles):
    best_gap, best_score = None, -1e9
    for gap in gaps:
        s = score_gap(gap, proc_dists, angles)
        if s > best_score:
            best_score, best_gap = s, gap
    return best_gap

# =========================================
# CONTROL (0.18 m/s 맞춤형 감속 스케일)
# =========================================
ALIGN_THRESHOLD = 10

def compute_cmd(target_angle):
    global loop_counter
    
    # 1. 조향 계산
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    # 조향 누적 필터 업데이트
    loop_counter = loop_counter * 0.98 + target_angle * 0.02

    # 2. 측면 감시 범위
    search_indices = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    relevant_min = float(np.min(scan_data[search_indices]))

    # 3. 정렬 상태 판단
    if abs(target_angle) > ALIGN_THRESHOLD:
        # 고속 주행 중 회전 시 구동력 확보를 위해 최소 전진 동력도 약간 보강
        v = 0.06 if relevant_min > 20.0 else 0.0
        return v, w

    # 4. 직진 속도 계산
    # 속도가 빨라졌으므로 30cm 거리에서부터 부드럽게 감속 수식이 개입하도록 설정
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
print("NAVIGATION START (0.18 m/s High-Velocity Tuned Mode)")
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
