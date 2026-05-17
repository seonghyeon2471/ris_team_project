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
MIN_SPEED    = 0.9    # 최소 속도
MAX_W        = 1.2    # 최대 각속도 (rad/s)
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
DANGER_DIST      = 18

SAFE_DIST          = 17
INFLATION_MAX_DIST = 25
FRONT_CLEAR_DIST   = 23
FRONT_CLEAR_RANGE  = 15

# =========================================
# GLOBAL GOAL & ODOMETRY PARAMETERS (단위: cm, rad, sec)
# =========================================
# 로봇의 현재 추정 자세 (시작점 기반 데드 레코닝)
pose_x = 0.0      # cm
pose_y = 0.0      # cm
pose_theta = 0.0  # rad (로봇의 현재 헤딩)

# 목표 지점 세팅: 시작 지점 정면 방향(X축)으로 직선거리 3.2m (320cm)
GOAL_X = 320.0    # cm
GOAL_Y = 0.0      # cm

# 가중치 튜닝 변수
GOAL_WEIGHT = 2.5 # [핵심] 목적지 방향 틈새에 줄 가중치

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2
STATE_ARRIVED = 3  # 목표 도착(통과) 상태

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST    = 6
REVERSE_DURATION  = 0.18
ROTATE_DURATION   = 1.00
REVERSE_SPEED     = -0.06
ROTATE_W          = 0.9

# =========================================
# STATE DATA
# =========================================
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0
last_odom_time = time.time() # 오도메트리 적분용 시간 기록

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
# ODOMETRY UPDATE (데드 레코닝)
# =========================================
def update_odometry(v_mps, w_radps):
    """실제 나간 속도 제어 명령을 바탕으로 가상 오도메트리를 누적합니다."""
    global pose_x, pose_y, pose_theta, last_odom_time
    now = time.time()
    dt = now - last_odom_time
    last_odom_time = now
    
    if dt <= 0 or dt > 0.5: # 루프가 너무 늘어졌을 때의 예외 처리
        return

    # m/s -> cm/s 변환
    v_cmps = v_mps * 100.0

    # 차동 구동 로봇 Kinematics 기반 단순 적분
    if abs(w_radps) < 1e-5:
        pose_x += v_cmps * math.cos(pose_theta) * dt
        pose_y += v_cmps * math.sin(pose_theta) * dt
    else:
        # 회전 반경 반영 모델
        r = v_cmps / w_radps
        pose_x += r * (math.sin(pose_theta + w_radps * dt) - math.sin(pose_theta))
        pose_y += r * (math.cos(pose_theta) - math.cos(pose_theta + w_radps * dt))
        pose_theta += w_radps * dt

    # 각도 정규화 (-PI ~ PI)
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
# GAP SEARCH & SCORED WITH GOAL BIAS
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
    center_angle = angles[int(center_i)] # 로봇 정면 기준 틈새의 상대 각도 (도)
    avg_dist = np.mean(proc_dists[start : end + 1])

    # 1. 목적지 방향과 현재 로봇 좌표 사이의 전역 각도 계산
    dx = GOAL_X - pose_x
    dy = GOAL_Y - pose_y
    global_goal_angle = math.atan2(dy, dx) # 시작 월드 좌표계 기준 목적지 각도

    # 2. 로봇 기준 상대적 목적지 각도 구하기 (단위: 도)
    relative_goal_angle = math.degrees(global_goal_angle - pose_theta)
    # -180 ~ 180도 범위 정규화
    relative_goal_angle = (relative_goal_angle + 180) % 360 - 180

    # 3. 이 틈새의 중심 각도가 '목적지 각도'와 얼마나 일치하는지 차이 계산
    angle_diff = abs(center_angle - relative_goal_angle)
    angle_diff = (angle_diff + 180) % 360 - 180
    angle_diff = abs(angle_diff)

    # 기본 Gap 점수 (너비 + 깊이 - 정면편향)
    base_score = (width * 0.5 + avg_dist * 1.2 - abs(center_angle) * 0.4)
    
    # 목적지와의 각도 오차가 작을수록 높은 보너스 점수 부여
    goal_bonus = (180.0 - angle_diff) / 180.0 * GOAL_WEIGHT

    return base_score + goal_bonus

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
    global prev_angle
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    gaps = find_gaps(proc_dists, angles)

    if not gaps: return None

    best_gap = select_best_gap(gaps, proc_dists, angles)
    start, end = best_gap
    gap_angle = float(angles[int((start + end) / 2.0)])

    front_clear = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))
    
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
print("NAVIGATION START (Global Goal Aware & High-Speed Drive Mode)")
try:
    # 최초 실행 시간 동기화
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

        # [필수] 루프 주기마다 로봇의 오도메트리 위치를 누적 추정
        update_odometry(v_active, w_active)

        # 현재 위치와 전역 목적지 사이의 남은 거리 계산 (출력용)
        dist_to_goal = math.hypot(GOAL_X - pose_x, GOAL_Y - pose_y)

        # --- GOAL DISTANCE CHECK [수정] ---
        # 로봇의 현재 X 좌표가 목적지 X 라인(320cm)을 넘어섰을 때 확실하게 미션 성공 및 멈춤 처리
        if pose_x >= GOAL_X or state == STATE_ARRIVED:
            state = STATE_ARRIVED
            stop_robot()
            v_active, w_active = 0.0, 0.0
            print(f"🎉 GOAL PASSED! Position: ({pose_x:.1f}, {pose_y:.1f})")
            time.sleep(0.5)
            continue

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
                for a in range(-45, 46): scan_data[a % 360] = float(SCAN_LIMIT)
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

        # 디버그 터미널 출력에 현재 위치와 목적지까지 남은 거리 표시
        print(f"Pos:({pose_x:5.1f},{pose_y:5.1f})| Rem:{dist_to_goal:5.1f}cm | TRG:{target_angle:5.1f}° | v:{v_active:.2f} | w:{w_active:.2f} | bias:{bias_label}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
