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
# DRIVE PARAMETER
# =========================================
MAX_SPEED    = 0.14   # 최대 선속도 (m/s)
MIN_SPEED    = 0.09   # [수정] 최소 속도 정상화 (0.9 -> 0.09)
MAX_W        = 1.5    # 최대 각속도 (rad/s)
TURN_GAIN    = 2.2    # 조향 게인

SCAN_LIMIT   = 150    # 유효 인식 거리 (cm)
FRONT_RANGE  = 60     # 탐색 반경 (±60°)

# =========================================
# FILTER & SMOOTHING PARAMETER
# =========================================
EMA_ALPHA        = 0.3
MEDIAN_K         = 2
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.10
DANGER_DIST      = 14

SAFE_DIST          = 17
INFLATION_MAX_DIST = 25
FRONT_CLEAR_DIST   = 23
FRONT_CLEAR_RANGE  = 15

# =========================================
# GLOBAL DIRECTION PARAMETERS (단위: rad)
# =========================================
pose_theta = 0.0  # rad (시작할 때 정면 방향을 0으로 기준 잡음)
GLOBAL_GOAL_THETA = 0.0  # rad
GOAL_WEIGHT = 2.5 # 목적지 방향 틈새에 줄 가중치

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
REVERSE_DURATION  = 0.15
ROTATE_DURATION   = 1.00
REVERSE_SPEED     = -0.10
ROTATE_W          = 0.9

# =========================================
# STATE DATA
# =========================================
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0
last_odom_time = time.time() 

# 회전 직후 가짜 프리패스 방지용 플래그
is_recovering = False

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
# HEADING UPDATE (각도 누적 적분)
# =========================================
def update_heading(w_radps):
    global pose_theta, last_odom_time
    now = time.time()
    dt = now - last_odom_time
    last_odom_time = now
    
    if dt <= 0 or dt > 0.5:
        return

    pose_theta += w_radps * dt
    pose_theta = (pose_theta + math.pi) % (2 * math.pi) - math.pi

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
# GAP SEARCH & SCORED WITH DIRECTION BIAS
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

def score_gap(gap, proc_dists, angles, is_critical=False):
    start, end = gap
    width = end - start
    center_i = (start + end) / 2.0
    center_angle = angles[int(center_i)] 
    avg_dist = np.mean(proc_dists[start : end + 1])

    base_score = (width * 0.5 + avg_dist * 1.2 - abs(center_angle) * 0.4)
    
    # [버그 수정 1] 박기 직전(is_critical)일 때는 방향 가중치를 배제하여
    # 목표 방향보다 '눈앞의 장애물 회피'를 무조건 최우선으로 잡도록 만듭니다.
    if is_critical:
        return base_score

    relative_goal_angle = math.degrees(GLOBAL_GOAL_THETA - pose_theta)
    relative_goal_angle = (relative_goal_angle + 180) % 360 - 180

    angle_diff = abs(center_angle - relative_goal_angle)
    angle_diff = (angle_diff + 180) % 360 - 180
    angle_diff = abs(angle_diff)

    goal_bonus = (180.0 - angle_diff) / 180.0 * GOAL_WEIGHT
    return base_score + goal_bonus

def select_best_gap(gaps, proc_dists, angles, is_critical=False):
    best_gap, best_score = None, -1e9
    for gap in gaps:
        s = score_gap(gap, proc_dists, angles, is_critical)
        if s > best_score:
            best_score, best_gap = s, gap
    return best_gap

# =========================================
# PLANNING
# =========================================
def find_best_direction(smoothing):
    global prev_angle, is_recovering
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    
    gaps = find_gaps(proc_dists, angles)
    if not gaps: return None

    front_clear = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))
    
    if front_clear > FRONT_CLEAR_DIST:
        is_critical = False
        best_gap = select_best_gap(gaps, proc_dists, angles, is_critical)
        target = float(angles[int((best_gap[0] + best_gap[1]) / 2.0)]) * 0.2
        bias_label = "STRAIGHT"
    elif front_clear > EMERGENCY_DIST * 2:
        is_critical = False
        best_gap = select_best_gap(gaps, proc_dists, angles, is_critical)
        target = float(angles[int((best_gap[0] + best_gap[1]) / 2.0)]) * 1.0               
        smoothing = SMOOTHING_DANGER           
        bias_label = "GAP"
    else:
        is_critical = True
        best_gap = select_best_gap(gaps, proc_dists, angles, is_critical)
        target = float(angles[int((best_gap[0] + best_gap[1]) / 2.0)]) * 1.0
        smoothing = 0.0                        
        bias_label = "CRITICAL"

    # 회전 제어 복귀 직후 루프에서 급격히 핸들을 꺾는 현상 방지
    if is_recovering:
        smoothing = 0.8
        is_recovering = False

    target = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    
    return target, bias_label, front_clear

# =========================================
# CONTROL
# =========================================
ALIGN_THRESHOLD = 7

def compute_cmd(target_angle):
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    search_indices = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    relevant_min = float(np.min(scan_data[search_indices]))

    if abs(target_angle) > ALIGN_THRESHOLD:
        v = 0.05 if relevant_min > 20.0 else 0.0
        return v, w

    # [버그 수정 2] MIN_SPEED 수식 교정
    # 장애물이 다가올 때 MAX_SPEED를 줄이되, 수정된 MIN_SPEED(0.09) 미만으로 내려가지 않게 제한합니다.
    obstacle_scale = min(relevant_min / 25.0, 1.0)
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
print("NAVIGATION START (Global Direction Aware & Bug Fixed Mode)")
try:
    last_odom_time = time.time()
    v_active, w_active = 0.0, 0.0

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

        update_heading(w_active)

        # --- STATE MACHINE ---
        if state == STATE_REVERSE:
            if now < maneuver_end_time: 
                v_active, w_active = REVERSE_SPEED, 0.0
                send_cmd(v_active, w_active)
            else:
                state, maneuver_end_time = STATE_ROTATE, now + ROTATE_DURATION
            continue

        if state == STATE_ROTATE:
            if now < maneuver_end_time: 
                v_active, w_active = 0.0, ROTATE_W * rotate_dir
                send_cmd(v_active, w_active)
            else:
                rotate_dir *= -1
                state, prev_angle = STATE_NORMAL, 0.0
                is_recovering = True  
                
                # [버그 수정 3] 150cm 가짜 벽 착시현상 해결
                # 회전 직후 라이다 값이 다 채워지기 전에 뻥 뚫렸다고 판단해 돌진하는 문제를 막기 위해,
                # 임시로 안전 거리 경계값(SAFE_DIST)으로 채워 센서가 갱신될 때까지 보수적으로 움직이게 유도합니다.
                for a in range(-45, 46): 
                    scan_data[a % 360] = float(SAFE_DIST)
            continue

        # --- STATE_NORMAL ---
        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))
        if front_min < EMERGENCY_DIST:
            rotate_dir = choose_avoid_direction()
            state, maneuver_end_time = STATE_REVERSE, now + REVERSE_DURATION
            v_active, w_active = REVERSE_SPEED, 0.0
            send_cmd(v_active, w_active)
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result = find_best_direction(smoothing)

        if result is None:
            rotate_dir = choose_avoid_direction()
            state, maneuver_end_time = STATE_REVERSE, now + REVERSE_DURATION
            continue

        target_angle, bias_label, front_clear = result
        v_active, w_active = compute_cmd(target_angle)
        send_cmd(v_active, w_active)

        print(f"Heading:{math.degrees(pose_theta):6.1f}° | TRG:{target_angle:5.1f}° | v:{v_active:.2f} | w:{w_active:.2f} | bias:{bias_label}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
