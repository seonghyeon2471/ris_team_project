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
ROBOT_RADIUS = 12.0   # 로봇 물리 반지름 (cm)

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED    = 0.18   # 최대 선속도 (m/s)
MIN_SPEED    = 0.10   # 최소 속도 (m/s)
MAX_W        = 1.8    # 최대 각속도 (rad/s)
TURN_GAIN    = 2.2    # 조향 게인

SCAN_LIMIT   = 150    # 유효 인식 거리 (cm)
FRONT_RANGE  = 55     # 탐색 반경 (±55°)

# =========================================
# FILTER & SMOOTHING PARAMETER
# =========================================
EMA_ALPHA        = 0.3
MEDIAN_K         = 2
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.10
DANGER_DIST      = 15     # 감속 기준 거리 (cm)

# [핵심 수정] 물리 반지름보다 최소 3cm의 여유를 주어야 벽에 처박지 않고 스무스하게 돌아 나갑니다.
SAFE_DIST          = 15.0   # 기존 12.0 -> 15.0 (벽 충돌 방지 안전 마진 확보)
INFLATION_MAX_DIST = 50.0   # 부풀리기를 적용할 최대 거리
FRONT_CLEAR_DIST   = 22.0   
FRONT_CLEAR_RANGE  = 12     

# =========================================
# GLOBAL DIRECTION PARAMETERS (단위: rad)
# =========================================
pose_theta = 0.0  
GLOBAL_GOAL_THETA = 0.0  
GOAL_WEIGHT = 2.5 

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST    = 8.0    # 처박기 전에 멈추도록 비상 정지 거리 상향
REVERSE_DURATION  = 0.45   
ROTATE_DURATION   = 0.65   
REVERSE_SPEED     = -0.12  
ROTATE_W          = 1.0    

# =========================================
# STATE DATA
# =========================================
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0
last_odom_time = time.time() 

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
# HEADING UPDATE
# =========================================
def update_heading(w_radps):
    global pose_theta, last_odom_time
    now = time.time()
    dt = now - last_odom_time
    last_odom_time = now
    
    if dt <= 0 or dt > 0.5:
        return

    pose_theta += w_radps * dt
    pose_theta = (math.pi + pose_theta) % (2 * math.pi) - math.pi

# =========================================
# OBSTACLE INFLATION (배열 인덱스 버그 수정본)
# =========================================
def inflate_obstacles(dists, angles):
    """ 탐색 범위 내의 장애물을 로봇 크기만큼 확실하게 부풀려 0.0으로 만듭니다. """
    proc = dists.copy()
    num_elements = len(dists)
    
    for i in range(num_elements):
        real_angle = angles[i] % 360
        # 실제 원본 데이터(scan_data)에서 거리를 가져와 판별
        d = scan_data[real_angle]
        
        if d < 4.0 or d >= INFLATION_MAX_DIST:
            continue
            
        # 장애물 각도 스크리닝 부풀리기 연산
        alpha = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
        
        # 현재 슬라이스(angles) 내에서 부풀려야 할 인덱스 범위를 계산
        for j in range(num_elements):
            angle_diff = (angles[j] - angles[i] + 180) % 360 - 180
            if abs(angle_diff) <= alpha:
                proc[j] = 0.0  # 안전거리 이하 지역은 강제로 틈새 탐색에서 제외
                
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

def score_gap(gap, proc_dists, angles, is_critical=False):
    start, end = gap
    width = end - start
    center_i = (start + end) / 2.0
    center_angle = angles[int(center_i)] 
    avg_dist = np.mean(proc_dists[start : end + 1])

    base_score = (width * 0.5 + avg_dist * 1.2 - abs(center_angle) * 0.4)
    
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
    
    # [수정] 인덱스 깨짐 방지를 위해 angles 배열을 인자로 같이 전달합니다.
    proc_dists = inflate_obstacles(dists, angles)
    
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

    if is_recovering:
        smoothing = 0.8
        is_recovering = False

    target = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    
    return target, bias_label, front_clear

# =========================================
# CONTROL
# =========================================
ALIGN_THRESHOLD = 10  

def compute_cmd(target_angle):
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    search_indices = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    relevant_min = float(np.min(scan_data[search_indices]))

    if abs(target_angle) > ALIGN_THRESHOLD:
        return MIN_SPEED, w

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
print("NAVIGATION START (Wall-Collision Fix Mode)")
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
                stop_robot()
                lidar_ser.reset_input_buffer()
                time.sleep(0.1) 
                
                state, prev_angle = STATE_NORMAL, 0.0
                is_recovering = True  
            continue

        # --- STATE_NORMAL ---
        emergency_indices = np.arange(-90, 91) % 360
        side_front_min = float(np.min(scan_data[emergency_indices]))
        
        if side_front_min < EMERGENCY_DIST:
            rotate_dir = choose_avoid_direction()
            state, maneuver_end_time = STATE_REVERSE, now + REVERSE_DURATION
            v_active, w_active = REVERSE_SPEED, 0.0
            send_cmd(v_active, w_active)
            continue

        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))
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
