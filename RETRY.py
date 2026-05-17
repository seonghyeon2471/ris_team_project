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
# DRIVE PARAMETER (하한선 대폭 상향 유지)
# =========================================
MAX_SPEED    = 0.14   # 최대 선속도 (m/s)
MIN_SPEED    = 0.11   # 최소 속도 하한선 유지 (감속 거의 안 함)
MAX_W        = 0.8    # 최대 각속도 (rad/s)
TURN_GAIN    = 1.8    # 조향 게인

SCAN_LIMIT   = 150    # 유효 인식 거리 (cm)
FRONT_RANGE  = 65     # 탐색 반경 (±60°) - 이 범위 내 측면 충돌 감시

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
DANGER_DIST      = 15     # 위험 감지 거리 (cm)

# =========================================
# GAP & INFLATION PARAMETER
# =========================================
SAFE_DIST          = 14   # Gap 유효 최소 거리 (cm)
INFLATION_MAX_DIST = 25   # 팽창 적용 최대 거리 (cm)

FRONT_CLEAR_DIST   = 12   # 직진 편향 판단 거리 (cm)
FRONT_CLEAR_RANGE  = 15   # 직진 편향 범위 (±15°)

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1      # +1: 좌회전, -1: 우회전

EMERGENCY_DIST    = 6      # 긴급회피 거리 (cm)
REVERSE_DURATION  = 0.18
ROTATE_DURATION   = 1.00
REVERSE_SPEED     = -0.10
ROTATE_W          = 0.9

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
# OBSTACLE INFLATION (강화된 반지름 적용)
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
# GAP SEARCH & WINDOW SCANNING (알고리즘 고도화)
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
    
    # 정면 선호 가중치를 살짝 강화하여 대각선 쏠림 현상 1차 억제
    return (width * 0.5 + avg_dist * 1.2 - abs(center_angle) * 0.6)

def select_best_gap(gaps, proc_dists, angles):
    best_gap, best_score = None, -1e9
    for gap in gaps:
        s = score_gap(gap, proc_dists, angles)
        if s > best_score:
            best_score, best_gap = s, gap
    return best_gap

def find_best_index_in_gap(best_gap, proc_dists, angles):
    """
    [대각선 돌진 버그 수정 핵심]
    선택된 넓은 Gap 내부에서 단순 기하학적 중앙을 잡지 않고,
    양쪽 장애물 경계면(Wall)으로부터 마진이 균등하게 확보되면서 
    정면(0도)에 가장 가까운 '진짜 안전한 베스트 각도'의 인덱스를 반환합니다.
    """
    start, end = best_gap
    best_idx = int((start + end) / 2)
    max_local_score = -1e9
    
    for i in range(start, end + 1):
        dist = proc_dists[i]
        angle = angles[i]
        
        # Gap 양쪽 끝단(장애물 부풀리기 경계)으로부터의 인덱스 마진 계산
        left_margin = i - start
        right_margin = end - i
        margin = min(left_margin, right_margin)
        
        # 내부 스코어링 수식: 
        # 장애물까지의 실제 거리(안전) + 좌우 여유폭(마진) - 정면 지향성 가중치
        # 이 계산을 거치면 대각선 장애물 옆 빈 공간보다 정면 개활지 중앙이 가장 높은 점수를 받게 됨
        local_score = (dist * 1.0) + (margin * 0.8) - (abs(angle) * 1.5)
        
        if local_score > max_local_score:
            max_local_score = local_score
            best_idx = i
            
    return best_idx

# =========================================
# PLANNING (개선된 Gap 및 스무딩 제어 적용)
# =========================================
def find_best_direction(smoothing):
    global prev_angle
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    gaps = find_gaps(proc_dists, angles)

    if not gaps: return None

    best_gap = select_best_gap(gaps, proc_dists, angles)
    
    # [수정] 단순 평균 수식이 아닌, 갭 내부 정밀 윈도우 스캔 함수 적용
    best_idx = find_best_index_in_gap(best_gap, proc_dists, angles)
    gap_angle = float(angles[best_idx])

    front_clear = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))
    
    if front_clear > FRONT_CLEAR_DIST:         # 23cm 초과: 직진 편향
        target = gap_angle * 0.2
        bias_label = "STRAIGHT"
        
    elif front_clear > EMERGENCY_DIST * 2:     # 12~23cm: 완전 Gap 추종
        target = gap_angle * 1.0               
        smoothing = SMOOTHING_DANGER           
        bias_label = "GAP"
        
    else:                                      # 12cm 미만: 즉시 최대 선회
        target = gap_angle * 1.0
        smoothing = 0.0                        
        bias_label = "CRITICAL"

    target = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    
    return target, bias_label, front_clear

# =========================================
# CONTROL (상향된 하한선 반영 및 추가 감속 제거)
# =========================================
ALIGN_THRESHOLD = 10

def compute_cmd(target_angle):
    # 1. 조향 계산
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    # 2. 측면 감시 범위 확대 (±60° 전체 최소 거리 확인)
    search_indices = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    relevant_min = float(np.min(scan_data[search_indices]))

    # 3. 정렬 상태 판단
    if abs(target_angle) > ALIGN_THRESHOLD:

       if relevant_min > 20.0:
           v = 0.05

       elif relevant_min > 10.0:
           v = 0.03

       else:
           v = 0.015

       return v, w

    # 4. 직진 속도 계산 (고속 지향형 튜닝)
    # 감속 진입 시점을 완전히 늦춤 (25cm 이내일 때만 수식 작동)
    obstacle_scale = min(relevant_min / 25.0, 1.0)
    
    # 아무리 좁은 곳이라도 상향된 MIN_SPEED(0.11m/s) 이하로는 속도가 떨어지지 않음
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
print("NAVIGATION START (High-Speed & High-Minimum Drive Mode)")
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
                state, prev_angle = STATE_NORMAL, 0.0
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

        print(f"TRG:{target_angle:5.1f}° | v:{v:.2f} | w:{w:.2f} | f_min:{front_min:.1f} | bias:{bias_label}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
