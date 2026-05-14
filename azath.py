import serial
import math
import time
import numpy as np

# =========================================
# SERIAL & LIDAR SETUP
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

lidar_ser.write(bytes([0xA5, 0x40]))   # Reset
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))   # Scan Start
lidar_ser.read(7)
print("LIDAR START (Smooth Moving Turn Mode)")

# =========================================
# PARAMETERS
# =========================================
ROBOT_RADIUS = 17.0   
MAX_SPEED    = 0.14
MIN_SPEED    = 0.05
MAX_W        = 1.5
TURN_GAIN    = 1.5    # 부드러운 조향을 위해 약간 하향 (1.8 -> 1.5)

SCAN_LIMIT   = 150
FRONT_RANGE  = 60

EMA_ALPHA    = 0.3
MEDIAN_K     = 2
SMOOTHING_NORMAL = 0.40 # 반응성 향상을 위해 약간 하향 (0.55 -> 0.4)
SMOOTHING_DANGER = 0.20
DANGER_DIST      = 18

SAFE_DIST          = 17
INFLATION_MAX_DIST = 25
FRONT_CLEAR_DIST   = 23
FRONT_CLEAR_RANGE  = 15

# State Machine
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1
target_angle      = 0.0  # 전역 변수화하여 루프마다 명령 전송

EMERGENCY_DIST    = 6
REVERSE_DURATION  = 0.18
ROTATE_DURATION   = 0.8
REVERSE_SPEED     = -0.10
ROTATE_W          = 0.9

scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0

# =========================================
# CORE FUNCTIONS
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

def inflate_obstacles(dists):
    proc = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= INFLATION_MAX_DIST: continue
        alpha = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists) - 1, int(i + alpha))
        proc[start_idx : end_idx + 1] = 0.0
    return proc

def find_best_direction(smoothing):
    global prev_angle
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    
    gaps = []
    gap_start = None
    for i, d in enumerate(proc_dists):
        if d > SAFE_DIST:
            if gap_start is None: gap_start = i
        else:
            if gap_start is not None:
                gaps.append((gap_start, i - 1)); gap_start = None
    if gap_start is not None: gaps.append((gap_start, len(proc_dists) - 1))

    if not gaps: return None

    # Best Gap Scoring
    best_gap, best_score = None, -1e9
    for gap in gaps:
        start, end = gap
        width = end - start
        center_a = angles[int((start + end) / 2.0)]
        avg_d = np.mean(proc_dists[start : end + 1])
        score = (width * 0.5 + avg_d * 1.2 - abs(center_a) * 0.4)
        if score > best_score:
            best_score, best_gap = score, gap

    gap_angle = float(angles[int((best_gap[0] + best_gap[1]) / 2.0)])
    front_clear = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))
    
    target = gap_angle * (0.3 if front_clear > FRONT_CLEAR_DIST else 0.7)
    target = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    return target

# =========================================
# [수정] CONTROL (Moving Turn 로직)
# =========================================
def compute_cmd(t_angle):
    # 1. 각속도(w) 산출
    w = math.radians(t_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    # 2. 주변 장애물 감시
    relevant_min = float(np.min(scan_data[np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360]))

    # 3. 곡선 주행 속도 제어
    # 목표 각도가 클수록 속도를 줄임 (60도 차이 시 속도 0에 수렴하도록 설정)
    angle_factor = max(0, 1.0 - (abs(t_angle) / 60.0))
    
    # 장애물이 가까울수록 속도를 줄임 (40cm 기준)
    dist_factor = min(relevant_min / 40.0, 1.0)
    
    # 기본 선속도 결정
    speed = MAX_SPEED * angle_factor * dist_factor
    
    # 최소 주행 속도 보장 (너무 멈추지 않게)
    if relevant_min > 20.0:
        speed = max(speed, MIN_SPEED)
    else:
        speed = max(speed, 0.0) # 아주 가까우면 멈춤

    # 4. 극단적인 회전 상황 (예: 45도 초과)에서만 제자리 회전
    if abs(t_angle) > 45:
        speed = 0.0

    return speed, w

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def choose_avoid_direction():
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    return 1 if left_avg >= right_avg else -1

# =========================================
# MAIN LOOP
# =========================================
try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) == 5:
            s_flag = raw[0] & 0x01
            # 데이터 파싱 및 필터 적용
            angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
            dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
            if 3 < dist_cm < SCAN_LIMIT:
                apply_ema(angle, dist_cm)

            if s_flag == 1: # 1회전 완료 시점에만 경로 재계산
                apply_median_filter()
                now = time.time()

                if state == STATE_NORMAL:
                    front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))
                    if front_min < EMERGENCY_DIST:
                        rotate_dir = choose_avoid_direction()
                        state, maneuver_end_time = STATE_REVERSE, now + REVERSE_DURATION
                    else:
                        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
                        res = find_best_direction(smoothing)
                        if res is not None: target_angle = res
                        else:
                            rotate_dir = choose_avoid_direction()
                            state, maneuver_end_time = STATE_REVERSE, now + REVERSE_DURATION

                elif state == STATE_REVERSE:
                    if now >= maneuver_end_time:
                        state, maneuver_end_time = STATE_ROTATE, now + ROTATE_DURATION

                elif state == STATE_ROTATE:
                    if now >= maneuver_end_time:
                        rotate_dir *= -1
                        state, prev_angle = STATE_NORMAL, 0.0

        # --- 명령 전송 (라이다 수신과 무관하게 매 루프 실행하여 끊김 방지) ---
        now = time.time()
        if state == STATE_NORMAL:
            v, w = compute_cmd(target_angle)
            send_cmd(v, w)
        elif state == STATE_REVERSE:
            send_cmd(REVERSE_SPEED, 0.0)
        elif state == STATE_ROTATE:
            send_cmd(0.0, ROTATE_W * rotate_dir)

except KeyboardInterrupt:
    print("STOP")
finally:
    send_cmd(0.0, 0.0)
    lidar_ser.write(bytes([0xA5, 0x25]))
