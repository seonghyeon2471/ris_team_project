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
ROBOT_RADIUS = 17.0
WHEEL_BASE   = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED    = 0.14
MIN_SPEED    = 0.11
MAX_W        = 1.5
TURN_GAIN    = 1.8

SCAN_LIMIT   = 150
FRONT_RANGE  = 60

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
DANGER_DIST      = 18

# =========================================
# GAP & INFLATION PARAMETER
# =========================================
SAFE_DIST          = 17
INFLATION_MAX_DIST = 25

FRONT_CLEAR_DIST   = 23
FRONT_CLEAR_RANGE  = 15

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
REVERSE_DURATION  = 0.18
ROTATE_DURATION   = 1.00
REVERSE_SPEED     = -0.10
ROTATE_W          = 0.9

# =========================================
# [NEW] EMERGENCY 연속 확인 카운터
# 순간 노이즈(경사면 등)에 의한 과민 반응 방지
# EMERGENCY_CONFIRM 프레임 연속으로 감지될 때만 실제 emergency 진입
# =========================================
emergency_counter   = 0
EMERGENCY_CONFIRM   = 3   # 연속 3프레임 이상 감지 시에만 진입

# =========================================
# LOOP TRAP MEMORY
# =========================================
loop_counter      = 0.0
LOOP_THRESHOLD    = 15.0
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
# CONTROL
# =========================================
ALIGN_THRESHOLD = 10

def compute_cmd(target_angle):
    global loop_counter
    
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    loop_counter = loop_counter * 0.98 + target_angle * 0.02

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

# =========================================
# [개선] choose_avoid_direction()
# 단순 평균 대신 역수 가중합(위험도 점수)으로
# 실제로 더 막힌 쪽을 정확히 판단해 반대로 회전
# =========================================
def choose_avoid_direction():
    def danger_score(indices):
        dists = scan_data[np.array(indices) % 360]
        valid = dists[dists > 0.5]
        if len(valid) == 0:
            return 999.0   # 아무것도 없으면 극도로 위험(=막힌 것으로 간주)
        return float(np.sum(1.0 / valid))  # 가까울수록 위험도 높음

    # 좌측(1~90°)과 우측(270~359°)의 위험도 비교
    left_danger  = danger_score(np.arange(1, 91))
    right_danger = danger_score(np.arange(270, 360))

    # 위험도가 낮은(더 열린) 쪽으로 회전
    return 1 if left_danger <= right_danger else -1

# =========================================
# MAIN LOOP
# =========================================
print("NAVIGATION START (Smooth + Improved Avoid Direction)")
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

        # [개선] emergency 연속 확인: 순간 노이즈는 무시, 진짜 막힌 경우만 반응
        if front_min < EMERGENCY_DIST:
            emergency_counter += 1
        else:
            emergency_counter = 0   # 한 번이라도 정상이면 카운터 리셋

        if emergency_counter >= EMERGENCY_CONFIRM:
            emergency_counter = 0
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

        print(f"[{bias_label:8s}] TRG:{target_angle:5.1f}° | v:{v:.2f} | w:{w:.2f} | f_min:{front_min:.1f} | EMG_CNT:{emergency_counter} | LOOP:{loop_counter:+.1f}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
