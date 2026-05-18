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
ROBOT_RADIUS        = 12.0   # 로봇 물리 반지름 (cm)
ROBOT_DIAMETER      = ROBOT_RADIUS * 2          # 24.0 cm
INFLATION_BUFFER    = 2.5    # 센서 노이즈·위치 오차 보정 버퍼 (cm)
INFLATION_RADIUS    = ROBOT_RADIUS + INFLATION_BUFFER  # 실질 팽창 반지름 (cm)

# ★ 통과 가능 최소 물리 너비
#   = 로봇 지름 + 양쪽 여유(각 2 cm) → 28 cm
#   이보다 좁은 틈은 "진입 불가"로 판단
MIN_GAP_PHYSICAL_CM = ROBOT_DIAMETER + 4.0      # 28.0 cm

# ★ "좁은 통로" 판단 기준
#   이 거리 이내로 좌우 장애물이 접근하면 좁은 통로로 판단
NARROW_PASSAGE_SIDE_DIST = 35.0   # cm

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED    = 0.22   # 최대 선속도 (m/s)
MIN_SPEED    = 0.12   # 최소 속도 (m/s)
NARROW_SPEED = 0.09   # ★ 좁은 통로 진입 속도 (m/s)
MAX_W        = 1.5    # 최대 각속도 (rad/s)
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
DANGER_DIST      = 15

# ★ SAFE_DIST: INFLATION_RADIUS보다 약간 크게 설정
#   (팽창 후 남은 공간을 실제 통과 가능 공간으로 인정)
SAFE_DIST          = INFLATION_RADIUS + 1.0     # ≈ 15.5 cm  (기존 15.0과 유사하지만 근거 명확)
INFLATION_MAX_DIST = 50.0
FRONT_CLEAR_DIST   = 22.0
FRONT_CLEAR_RANGE  = 12

# =========================================
# GLOBAL DIRECTION PARAMETERS (단위: rad)
# =========================================
pose_theta        = 0.0
GLOBAL_GOAL_THETA = 0.0
GOAL_WEIGHT       = 2.5

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST   = 8.0
REVERSE_DURATION = 0.45
ROTATE_DURATION  = 0.60
REVERSE_SPEED    = -0.12
ROTATE_W         = 1.0

# =========================================
# STATE DATA
# =========================================
scan_data      = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle     = 0.0
last_odom_time = time.time()
is_recovering  = False

# =========================================
# UTIL & FILTER
# =========================================
def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

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
# HEADING UPDATE
# =========================================
def update_heading(w_radps):
    global pose_theta, last_odom_time
    now = time.time()
    dt  = now - last_odom_time
    last_odom_time = now
    if dt <= 0 or dt > 0.5:
        return
    pose_theta += w_radps * dt
    pose_theta  = (math.pi + pose_theta) % (2 * math.pi) - math.pi

# =========================================
# NARROW PASSAGE DETECTION  ★ NEW
# =========================================
def detect_narrow_passage():
    """
    좌우 60~120° 범위에서 가장 가까운 장애물 거리를 확인.
    양쪽 모두 NARROW_PASSAGE_SIDE_DIST 이내면 좁은 통로로 판단.
    """
    left_min  = float(np.min(scan_data[np.arange(50,  130) % 360]))
    right_min = float(np.min(scan_data[np.arange(230, 310) % 360]))
    in_narrow = (left_min < NARROW_PASSAGE_SIDE_DIST and
                 right_min < NARROW_PASSAGE_SIDE_DIST)
    passage_width = left_min + right_min  # 좌우 합산 ≈ 통로 너비
    return in_narrow, passage_width

# =========================================
# OBSTACLE INFLATION  ★ INFLATION_RADIUS 사용
# =========================================
def inflate_obstacles(dists, angles):
    proc = dists.copy()
    num_elements = len(dists)
    for i in range(num_elements):
        real_angle = angles[i] % 360
        d = scan_data[real_angle]
        if d < 4.0 or d >= INFLATION_MAX_DIST:
            continue
        # 실질 팽창 반지름 = ROBOT_RADIUS + INFLATION_BUFFER
        alpha = math.degrees(math.asin(min(INFLATION_RADIUS / d, 1.0)))
        for j in range(num_elements):
            angle_diff = (angles[j] - angles[i] + 180) % 360 - 180
            if abs(angle_diff) <= alpha:
                proc[j] = 0.0
    return proc

# =========================================
# GAP PHYSICAL WIDTH  ★ NEW
# =========================================
def gap_physical_width(gap, proc_dists, angles):
    """
    갭의 실제 물리 너비(cm)를 추정.

    원리: 갭 중심까지의 평균 거리 D와 각도 폭 θ(rad)을 이용해
    현(chord) 길이로 근사.
        width ≈ 2 * D * sin(θ / 2)

    보수적 추정을 위해 평균 거리 대신 최솟값(가장 가까운 지점)을
    사용한다 — 좁은 부분을 기준으로 판단.
    """
    start, end = gap
    gap_dists   = proc_dists[start:end + 1]
    gap_angles  = angles[start:end + 1]

    valid = gap_dists[gap_dists > SAFE_DIST]
    if len(valid) == 0:
        return 0.0

    # 보수적 거리: 갭 내 유효 거리의 최솟값
    conservative_dist = float(np.min(valid))

    angle_span_deg = abs(float(gap_angles[-1]) - float(gap_angles[0]))
    angle_span_rad = math.radians(angle_span_deg)

    physical_width = 2.0 * conservative_dist * math.sin(angle_span_rad / 2.0)
    return physical_width

# =========================================
# GAP SEARCH  ★ 물리 너비 검증 추가
# =========================================
def find_gaps(proc_dists, angles):
    """
    갭을 탐색하되, 실제 물리 너비가 MIN_GAP_PHYSICAL_CM 이상인
    갭만 반환한다.
    너무 좁은 틈을 잘못 인식해 박는 경우를 원천 차단.
    """
    gaps      = []
    gap_start = None

    for i, d in enumerate(proc_dists):
        if d > SAFE_DIST:
            if gap_start is None:
                gap_start = i
        else:
            if gap_start is not None:
                candidate = (gap_start, i - 1)
                w = gap_physical_width(candidate, proc_dists, angles)
                if w >= MIN_GAP_PHYSICAL_CM:   # ★ 물리 너비 필터
                    gaps.append(candidate)
                gap_start = None

    if gap_start is not None:
        candidate = (gap_start, len(proc_dists) - 1)
        w = gap_physical_width(candidate, proc_dists, angles)
        if w >= MIN_GAP_PHYSICAL_CM:           # ★ 물리 너비 필터
            gaps.append(candidate)

    return gaps

# =========================================
# GAP SCORING
# =========================================
def score_gap(gap, proc_dists, angles, is_critical=False):
    start, end   = gap
    width        = end - start
    center_i     = (start + end) / 2.0
    center_angle = angles[int(center_i)]
    avg_dist     = np.mean(proc_dists[start:end + 1])

    # ★ 물리 너비 보너스: 더 넓은 틈에 가점 (좁은 틈 패널티 효과)
    phys_width = gap_physical_width(gap, proc_dists, angles)
    width_bonus = (phys_width - MIN_GAP_PHYSICAL_CM) * 0.05  # 넓을수록 가점

    base_score = (width * 0.5 + avg_dist * 1.2
                  - abs(center_angle) * 0.4
                  + width_bonus)

    if is_critical:
        return base_score

    relative_goal_angle = math.degrees(GLOBAL_GOAL_THETA - pose_theta)
    relative_goal_angle = (relative_goal_angle + 180) % 360 - 180
    angle_diff          = abs(center_angle - relative_goal_angle)
    angle_diff          = abs((angle_diff + 180) % 360 - 180)

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
    dists  = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)

    proc_dists = inflate_obstacles(dists, angles)

    gaps = find_gaps(proc_dists, angles)
    if not gaps:
        return None

    front_clear = float(np.min(scan_data[
        np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))

    if front_clear > FRONT_CLEAR_DIST:
        is_critical  = False
        best_gap     = select_best_gap(gaps, proc_dists, angles, is_critical)
        target       = float(angles[int((best_gap[0] + best_gap[1]) / 2.0)]) * 0.2
        bias_label   = "STRAIGHT"
    elif front_clear > EMERGENCY_DIST * 2:
        is_critical  = False
        best_gap     = select_best_gap(gaps, proc_dists, angles, is_critical)
        target       = float(angles[int((best_gap[0] + best_gap[1]) / 2.0)]) * 1.0
        smoothing    = SMOOTHING_DANGER
        bias_label   = "GAP"
    else:
        is_critical  = True
        best_gap     = select_best_gap(gaps, proc_dists, angles, is_critical)
        target       = float(angles[int((best_gap[0] + best_gap[1]) / 2.0)]) * 1.0
        smoothing    = 0.0
        bias_label   = "CRITICAL"

    if is_recovering:
        smoothing     = 0.8
        is_recovering = False

    target     = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target

    return target, bias_label, front_clear

# =========================================
# CONTROL  ★ 좁은 통로 속도 제한 추가
# =========================================
ALIGN_THRESHOLD = 10

def compute_cmd(target_angle):
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    search_indices = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    relevant_min   = float(np.min(scan_data[search_indices]))

    # ★ 좁은 통로 감지: 속도 상한을 NARROW_SPEED로 제한
    in_narrow, passage_width = detect_narrow_passage()

    if abs(target_angle) > ALIGN_THRESHOLD:
        speed = NARROW_SPEED if in_narrow else MIN_SPEED
        return speed, w

    obstacle_scale = min(relevant_min / 25.0, 1.0)
    speed = max(MAX_SPEED * obstacle_scale, MIN_SPEED)

    if in_narrow:
        speed = min(speed, NARROW_SPEED)   # ★ 좁은 통로에서 속도 캡

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
print("NAVIGATION START (Narrow-Gap Pass & Physical-Width Validation Mode)")
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

        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
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
                stop_robot()
                time.sleep(0.15)
                state, maneuver_end_time = STATE_ROTATE, time.time() + ROTATE_DURATION
            continue

        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                v_active, w_active = 0.0, ROTATE_W * rotate_dir
                send_cmd(v_active, w_active)
            else:
                stop_robot()
                lidar_ser.reset_input_buffer()
                time.sleep(0.15)
                state, prev_angle = STATE_NORMAL, 0.0
                is_recovering = True
            continue

        # --- STATE_NORMAL ---
        emergency_indices = np.arange(-90, 91) % 360
        side_front_min    = float(np.min(scan_data[emergency_indices]))

        if side_front_min < EMERGENCY_DIST:
            stop_robot()
            time.sleep(0.1)
            rotate_dir = choose_avoid_direction()
            state, maneuver_end_time = STATE_REVERSE, time.time() + REVERSE_DURATION
            v_active, w_active = REVERSE_SPEED, 0.0
            send_cmd(v_active, w_active)
            continue

        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))
        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result    = find_best_direction(smoothing)

        if result is None:
            rotate_dir = choose_avoid_direction()
            state, maneuver_end_time = STATE_REVERSE, time.time() + REVERSE_DURATION
            continue

        target_angle, bias_label, front_clear = result
        v_active, w_active = compute_cmd(target_angle)
        send_cmd(v_active, w_active)

        in_narrow, pw = detect_narrow_passage()
        narrow_tag = f" [NARROW pw={pw:.0f}cm]" if in_narrow else ""

        print(f"Heading:{math.degrees(pose_theta):6.1f}° | TRG:{target_angle:5.1f}° | "
              f"v:{v_active:.2f} | w:{w_active:.2f} | bias:{bias_label}{narrow_tag}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
