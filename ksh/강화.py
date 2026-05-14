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
ROBOT_RADIUS       = 17.0   # 물리 반지름 + 측면 안전 마진 (cm)  폭 20cm → 반폭 10 + 마진 7
WHEEL_BASE         = 17.0   # 차동구동 휠 베이스 (cm)

# 라이다 ~ 뒷축 기구학 파라미터 (실측 기반)
# - 로봇 길이 20cm, 라이다: 맨앞에서 2.5cm 뒤
# - 뒷바퀴 축: 뒤에서 4cm 앞  →  앞에서 16cm 지점
# - 라이다 기준 뒷축까지: 16 - 2.5 = 13.5cm
LIDAR_TO_REAR_AXLE = 13.5   # 라이다 ~ 뒷축 직선거리 (cm)

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED    = 0.14   # 최대 선속도 (m/s)
MIN_SPEED    = 0.05   # 최소 선속도 (m/s)
MAX_W        = 1.5    # 최대 각속도 (rad/s)
TURN_GAIN    = 1.8    # 조향 게인

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
DANGER_DIST      = 18     # 위험 감지 거리 (cm)

# =========================================
# GAP & INFLATION PARAMETER
# =========================================
SAFE_DIST          = 20   # Gap 유효 최소 거리 (cm)  17 → 20 (뒷바퀴 마진 반영)
INFLATION_MAX_DIST = 30   # 팽창 적용 최대 거리 (cm) 25 → 30

FRONT_CLEAR_DIST   = 23   # 직진 편향 판단 거리 (cm)
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
scan_data  = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0
v_cmd      = MIN_SPEED   # 이전 선속도 명령 (동적 팽창 계산용)
w_cmd      = 0.0         # 이전 각속도 명령 (동적 팽창 계산용)

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
        indices   = [(i + d) % 360 for d in range(-k, k + 1)]
        values    = np.sort(scan_data[indices])
        filtered[i] = values[window // 2]
    scan_data[:] = filtered

# =========================================
# DYNAMIC INFLATION RADIUS
# 회전 시 뒷바퀴가 스윙하는 궤적을 고려한 동적 팽창 반경
# =========================================
def get_turn_inflation_radius(v, w):
    """
    현재 v, w 명령 기준으로 뒷바퀴 스윙을 포함한 유효 팽창 반경 반환.

    원리:
      - ICC(순간회전중심)까지 거리 R = v / w
      - 라이다 원점 기준 뒷축이 그리는 궤도 반경 = sqrt(R² + L²)
        (L = LIDAR_TO_REAR_AXLE)
      - inner_cut = rear_orbit - R  : 뒷축이 라이다 원점 안쪽으로 파고드는 거리
      - 유효 반경 = ROBOT_RADIUS + inner_cut
    """
    if abs(w) < 0.05:
        return ROBOT_RADIUS   # 직진: 기본 반지름만 사용

    R          = abs(v / w)
    rear_orbit = math.sqrt(R ** 2 + LIDAR_TO_REAR_AXLE ** 2)
    inner_cut  = rear_orbit - R
    return ROBOT_RADIUS + inner_cut

# =========================================
# OBSTACLE INFLATION (동적 반경 적용)
# =========================================
def inflate_obstacles(dists, radius):
    proc = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= INFLATION_MAX_DIST:
            continue
        alpha     = math.degrees(math.asin(min(radius / d, 1.0)))
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
            if gap_start is None:
                gap_start = i
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
    return width * 0.5 + avg_dist * 1.2 - abs(center_angle) * 0.4

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
def find_best_direction(smoothing, v_prev=0.0, w_prev=0.0):
    global prev_angle

    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists  = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)

    # 이전 명령 기반 동적 팽창 반경 결정
    eff_radius = get_turn_inflation_radius(v_prev, w_prev)
    proc_dists = inflate_obstacles(dists, eff_radius)

    gaps = find_gaps(proc_dists, angles)
    if not gaps:
        return None

    best_gap        = select_best_gap(gaps, proc_dists, angles)
    start, end      = best_gap
    gap_angle       = float(angles[int((start + end) / 2.0)])

    front_clear = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))
    if front_clear > FRONT_CLEAR_DIST:
        target, bias_label = gap_angle * 0.3, "STRAIGHT"
    else:
        target, bias_label = gap_angle * 0.7, "GAP"

    target      = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle  = target
    return target, bias_label, front_clear

# =========================================
# CONTROL
# =========================================
ALIGN_THRESHOLD = 10

def compute_cmd(target_angle):
    # 1. 조향 계산
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    # 2. 전방 ±60° 최소 거리
    search_indices = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    relevant_min   = float(np.min(scan_data[search_indices]))

    # 3. 정렬 중: 제자리 회전 (측면 여유 있을 때만 미세 전진)
    if abs(target_angle) > ALIGN_THRESHOLD:
        v = 0.02 if relevant_min > 20.0 else 0.0
        return v, w

    # 4. 직진 속도 계산
    obstacle_scale = min(relevant_min / 40.0, 1.0)
    speed = max(MAX_SPEED * obstacle_scale, MIN_SPEED)

    if relevant_min < 22.0:
        speed *= 0.8

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
print("NAVIGATION START (Rear Wheel Protection Enabled)")
print(f"  LIDAR_TO_REAR_AXLE = {LIDAR_TO_REAR_AXLE} cm")
print(f"  ROBOT_RADIUS       = {ROBOT_RADIUS} cm")
print(f"  SAFE_DIST          = {SAFE_DIST} cm")

try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

        s_flag = raw[0] & 0x01
        if (raw[0] & 0x02) >> 1 != (1 - s_flag):
            continue
        if (raw[1] & 0x01) != 1:
            continue
        if (raw[0] >> 2) < 3:
            continue

        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < SCAN_LIMIT:
            apply_ema(angle, dist_cm)

        if s_flag != 1:
            continue
        apply_median_filter()
        now = time.time()

        # --- STATE MACHINE ---
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
            else:
                state, maneuver_end_time = STATE_ROTATE, now + ROTATE_DURATION
            continue

        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
            else:
                rotate_dir *= -1
                state, prev_angle = STATE_NORMAL, 0.0
                for a in range(-45, 46):
                    scan_data[a % 360] = float(SCAN_LIMIT)
            continue

        # --- STATE_NORMAL ---
        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))
        if front_min < EMERGENCY_DIST:
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            v_cmd, w_cmd = REVERSE_SPEED, 0.0
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result    = find_best_direction(smoothing, v_prev=v_cmd, w_prev=w_cmd)

        if result is None:
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            v_cmd, w_cmd = REVERSE_SPEED, 0.0
            continue

        target_angle, bias_label, front_clear = result
        v_cmd, w_cmd = compute_cmd(target_angle)
        send_cmd(v_cmd, w_cmd)

        # 동적 팽창 반경 로그 출력 (디버깅용)
        eff_r = get_turn_inflation_radius(v_cmd, w_cmd)
        print(
            f"TRG:{target_angle:5.1f}° | v:{v_cmd:.2f} | w:{w_cmd:.2f} | "
            f"f_min:{front_min:.1f} | eff_r:{eff_r:.1f} | bias:{bias_label}"
        )

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
