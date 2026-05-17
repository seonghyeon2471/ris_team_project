import serial
import math
import time
import numpy as np

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

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
# ★ 좁은 길 통과를 위해 안전 마진을 타이트하게 조정 (필요시 환경에 맞춰 15.0~17.0 조절)
ROBOT_RADIUS = 15.5   # 물리 반지름 + 최소 안전 마진 (cm)
WHEEL_BASE   = 17.0   # 차동구동 휠 베이스 (cm)

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED    = 0.25   # 최대 선속도 (m/s)
MIN_SPEED    = 0.13   # 최소 속도 하한선
MAX_W        = 1.3    # 최대 각속도 (rad/s)
TURN_GAIN    = 1.8    # 조향 게인

SCAN_LIMIT   = 150    # 유효 인식 거리 (cm)
FRONT_RANGE  = 65     # 탐색 반경 (±65°)

# =========================================
# FILTER PARAMETER
# =========================================
EMA_ALPHA    = 0.3
MEDIAN_K     = 2

# =========================================
# SMOOTHING PARAMETER
# =========================================
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.20
DANGER_DIST      = 12

# =========================================
# GAP & INFLATION PARAMETER
# =========================================
# ★ 차체보다 살짝 넓은 틈새도 인식할 수 있도록 최소 갭 기준 완화
SAFE_DIST          = 5
# ★ 25cm 제한을 풀고 SCAN_LIMIT까지 부풀리기를 적용하여 좁은 길 진입 전 튕기는 현상 방지
INFLATION_MAX_DIST = SCAN_LIMIT

FRONT_CLEAR_DIST   = 23   
FRONT_CLEAR_RANGE  = 8

# =========================================
# GOAL PARAMETER
# =========================================
GOAL_ANGLE  = 0      # 목적지 방향 (정면 = 0°)
GOAL_WEIGHT = 1.5    # 목적지 방향 가산점 가중치

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST    = 6
REVERSE_DURATION  = 0.20
ROTATE_DURATION   = 1.00
REVERSE_SPEED     = -0.10
ROTATE_W          = 0.6

# =========================================
# STATE DATA
# =========================================
scan_data  = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0

# =========================================
# UTIL & FILTER
# =========================================
def apply_ema(angle, new_dist_cm):
    scan_data[angle] = ((1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm)

def apply_median_filter():
    k      = MEDIAN_K
    window = 2 * k + 1
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        indices  = [(i + d) % 360 for d in range(-k, k + 1)]
        values   = np.sort(scan_data[indices])
        filtered[i] = values[window // 2]
    scan_data[:] = filtered

# =========================================
# OBSTACLE INFLATION
# =========================================
def inflate_obstacles(dists):
    proc = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        # 원거리 벽면까지 정확하게 내 몸집만큼 부풀림
        if d < 5.0 or d >= INFLATION_MAX_DIST:
            continue
        try:
            alpha = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
        except ValueError:
            alpha = 45.0
            
        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists) - 1, int(i + alpha))
        proc[start_idx : end_idx + 1] = 0.0
    return proc

# =========================================
# GAP SEARCH
# =========================================
def find_gaps(proc_dists, angles):
    gaps      = []
    gap_start = None
    for i, d in enumerate(proc_dists):
        if d > SAFE_DIST:
            if gap_start is None: gap_start = i
        else:
            if gap_start is not None:
                gaps.append((gap_start, i - 1))
                gap_start = None
    if gap_start is not None:
        gaps.append((gap_start, len(proc_dists) - 1))
    return gaps

def score_gap(gap, proc_dists, angles):
    start, end   = gap
    width        = end - start
    center_i     = (start + end) / 2.0
    center_angle = angles[int(center_i)]
    avg_dist     = np.mean(proc_dists[start : end + 1])

    goal_diff  = abs(center_angle - GOAL_ANGLE)
    goal_bonus = max(0.0, (FRONT_RANGE - goal_diff) / FRONT_RANGE) * GOAL_WEIGHT

    return (width    * 0.5
            + avg_dist * 1.2
            - abs(center_angle) * 0.6
            + goal_bonus)

# ★ 차체보다 살짝 넓은 길의 정확한 '중앙값'을 찾아 조향하도록 수정된 함수
def find_best_index_in_gap(best_gap, proc_dists, angles):
    start, end = best_gap
    
    # 기본 타겟은 갭의 기하학적 정중앙 인덱스
    center_idx = int((start + end) / 2)
    
    best_idx = center_idx
    max_local_score = -1e9
    
    for i in range(start, end + 1):
        dist  = proc_dists[i]
        angle = angles[i]
        
        # 좌우 양쪽 벽(0이 된 지점)으로부터 확보한 각도 마진 계산
        left_margin  = i - start
        right_margin = end - i
        min_margin   = min(left_margin, right_margin)
        
        # [스코어링 밸런스 조정] 
        # min_margin 가중치를 높여 벽 정중앙(중앙값)을 찌르도록 유도하고, 각도 불일치 단위 보정
        local_score = (min_margin * 3.0) + (dist * 0.5) - (abs(angle) * 1.0)
        
        if local_score > max_local_score:
            max_local_score = local_score
            best_idx = i
            
    return best_idx

# =========================================
# PLANNING
# =========================================
def find_best_direction(smoothing):
    global prev_angle
    angles     = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists      = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    gaps       = find_gaps(proc_dists, angles)

    if not gaps: return None

    best_gap  = select_best_gap(gaps, proc_dists, angles)
    best_idx  = find_best_index_in_gap(best_gap, proc_dists, angles)
    gap_angle = float(angles[best_idx])

    front_clear = float(np.min(
        scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]
    ))

    CRITICAL_DIST = EMERGENCY_DIST * 2   # 12cm

    if front_clear > FRONT_CLEAR_DIST:   # > 23cm : 직진 편향
        target     = gap_angle * 0.2
        bias_label = "STRAIGHT"
    elif front_clear > CRITICAL_DIST:    # 12~23cm : Gap 완전 추종
        target     = gap_angle * 1.0
        smoothing  = SMOOTHING_DANGER
        bias_label = "GAP"
    else:                                # ≤ 12cm : 즉각 반응
        target     = gap_angle * 1.0
        smoothing  = 0.0
        bias_label = "CRITICAL"

    target     = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    return target, bias_label, front_clear

# =========================================
# CONTROL
# =========================================
# ★ 울컥거림을 유발하던 고정 임계값(ALIGN_THRESHOLD) 제거 후 선형 감속 적용
def compute_cmd(target_angle):
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    search_indices = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    relevant_min   = float(np.min(scan_data[search_indices]))

    # [선형 감속 알고리즘]
    # 조향각이 커질수록(즉, 좁은 길을 통과하려고 핸들을 많이 꺾을수록) 선속도를 부드럽게 감속
    angle_filter = max(0.0, 1.0 - (abs(target_angle) / FRONT_RANGE))
    
    obstacle_scale = min(relevant_min / 25.0, 1.0)
    base_speed     = max(MAX_SPEED * obstacle_scale, MIN_SPEED)
    
    # 정면 주행 시 고속, 급조향(좁은 문 진입 등) 시 안전하게 MIN_SPEED까지 선형 감속
    speed = MIN_SPEED + (base_speed - MIN_SPEED) * angle_filter
    
    return float(speed), w

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
print("NAVIGATION START")
try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5: continue

        s_flag = raw[0] & 0x01
        if (raw[0] & 0x02) >> 1 != (1 - s_flag): continue
        if (raw[1] & 0x01) != 1: continue
        if (raw[0] >> 2) < 3: continue

        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
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
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result    = find_best_direction(smoothing)

        if result is None:
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        target_angle, bias_label, front_clear = result
        v, w = compute_cmd(target_angle)
        send_cmd(v, w)

        print(f"TRG:{target_angle:5.1f}° | v:{v:.2f} | w:{w:.2f} | "
              f"f_min:{front_min:.1f} | clear:{front_clear:.1f} | bias:{bias_label}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
