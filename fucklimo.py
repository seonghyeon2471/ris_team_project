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
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR START")

# =========================================
# ROBOT PHYSICAL PARAMETER
# =========================================
ROBOT_RADIUS = 17.0
WHEEL_BASE   = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED    = 0.5
MIN_SPEED    = 0.50
MAX_W        = 1.0
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
SMOOTHING_NORMAL = 0.7
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
# EMERGENCY 감지 박스
# =========================================
EMERGENCY_FRONT = 10.0
EMERGENCY_SIDE  = 8.0

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0

# =========================================
# ROTATE: 목표각도 기반
# =========================================
rotate_dir         = 1
rotate_target_deg  = 90.0
rotate_accumulated = 0.0
rotate_last_time   = 0.0

REVERSE_DURATION = 0.1
REVERSE_SPEED    = -0.10
ROTATE_W         = 0.8

# =========================================
# SCAN DATA
# =========================================
scan_data  = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0

# =========================================
# UTIL & FILTER
# =========================================
def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def apply_median_filter():
    k        = MEDIAN_K
    window   = 2 * k + 1
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        indices     = [(i + d) % 360 for d in range(-k, k + 1)]
        values      = np.sort(scan_data[indices])
        filtered[i] = values[window // 2]
    scan_data[:] = filtered

# =========================================
# EMERGENCY 감지 (사각형 박스)
# =========================================
def check_emergency():
    triggered = False
    for i in range(-90, 91):
        angle = i % 360
        d = scan_data[angle]
        if d >= SCAN_LIMIT:
            continue
        rad = math.radians(i)
        x = d * math.sin(rad)
        y = d * math.cos(rad)
        if 0 < y < EMERGENCY_FRONT and abs(x) < EMERGENCY_SIDE:
            triggered = True
            break
    front_min = float(np.min(scan_data[np.arange(-60, 61) % 360]))
    return triggered, front_min

# =========================================
# OBSTACLE INFLATION
# =========================================
def inflate_obstacles(dists):
    proc = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= INFLATION_MAX_DIST:
            continue
        alpha     = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
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

# 일반 주행용: 정면 가까울수록 가산점
def score_gap_normal(gap, proc_dists, angles):
    start, end   = gap
    center_i     = (start + end) / 2.0
    center_angle = angles[int(center_i)]
    avg_dist     = np.mean(proc_dists[start : end + 1])
    width        = end - start
    return (width * 0.5 + avg_dist * 1.2 - abs(center_angle) * 0.4)

# Emergency용: 정면 바이어스 없음, 넓고 먼 갭 우선
def score_gap_emergency(gap, proc_dists, angles):
    start, end = gap
    avg_dist   = np.mean(proc_dists[start : end + 1])
    width      = end - start
    return (width * 0.7 + avg_dist * 1.3)

def select_best_gap(gaps, proc_dists, angles, score_fn):
    best_gap, best_score = None, -1e9
    for gap in gaps:
        s = score_fn(gap, proc_dists, angles)
        if s > best_score:
            best_score, best_gap = s, gap
    return best_gap

# =========================================
# Emergency용 180도 갭 탐색
# =========================================
def find_best_direction_180():
    angles = np.arange(-90, 91)
    dists  = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc   = inflate_obstacles(dists)
    gaps   = find_gaps(proc, angles)

    if not gaps:
        left_avg  = float(np.mean(scan_data[271:360]))
        right_avg = float(np.mean(scan_data[1:91]))
        if left_avg >= right_avg:
            print("[180 SCAN] No gap → fallback LEFT 90°")
            return +1, 90.0
        else:
            print("[180 SCAN] No gap → fallback RIGHT 90°")
            return -1, 90.0

    best_gap     = select_best_gap(gaps, proc, angles, score_gap_emergency)
    start, end   = best_gap
    center_i     = int((start + end) / 2.0)
    target_angle = float(angles[center_i])

    rotate_dir_out = 1 if target_angle >= 0 else -1
    target_degrees = min(abs(target_angle), 90.0)
    target_degrees = max(target_degrees, 20.0)

    direction_str = "LEFT" if rotate_dir_out > 0 else "RIGHT"
    print(f"[180 SCAN] Best gap → {direction_str} {target_degrees:.1f}°")
    return rotate_dir_out, target_degrees

# =========================================
# PLANNING (STATE_NORMAL용)
# =========================================
def find_best_direction(smoothing):
    global prev_angle
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists  = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc   = inflate_obstacles(dists)
    gaps   = find_gaps(proc, angles)

    if not gaps:
        return None

    best_gap    = select_best_gap(gaps, proc, angles, score_gap_normal)
    start, end  = best_gap
    gap_angle   = float(angles[int((start + end) / 2.0)])
    front_clear = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))

    if front_clear > FRONT_CLEAR_DIST:
        target     = gap_angle * 0.2
        bias_label = "STRAIGHT"
    elif front_clear > 12:
        target     = gap_angle * 1.0
        smoothing  = SMOOTHING_DANGER
        bias_label = "GAP"
    else:
        target     = gap_angle * 1.0
        smoothing  = 0.0
        bias_label = "CRITICAL"

    target     = prev_angle * smoothing + target * (1.0 - smoothing)
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
    relevant_min   = float(np.min(scan_data[search_indices]))

    if abs(target_angle) > ALIGN_THRESHOLD:
        v = 0.05 if relevant_min > 20.0 else 0.03
        return v, w

    obstacle_scale = min(relevant_min / 25.0, 1.0)
    speed          = max(MAX_SPEED * obstacle_scale, MIN_SPEED)
    return speed, w

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# MAIN LOOP
# =========================================
print("NAVIGATION START (180-degree Gap Emergency, No Center Bias)")
print(f"Emergency box: front={EMERGENCY_FRONT}cm x side=±{EMERGENCY_SIDE}cm")

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

        # --------------------------------------------------
        # STATE_REVERSE: 후진
        # --------------------------------------------------
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
            else:
                rotate_dir, rotate_target_deg = find_best_direction_180()
                rotate_accumulated = 0.0
                rotate_last_time   = now
                state = STATE_ROTATE
            continue

        # --------------------------------------------------
        # STATE_ROTATE: 목표각도까지 회전 (emergency 중첩 없음)
        # --------------------------------------------------
        if state == STATE_ROTATE:
            dt                  = now - rotate_last_time
            rotate_last_time    = now
            rotate_accumulated += math.degrees(ROTATE_W * dt)

            if rotate_accumulated < rotate_target_deg:
                send_cmd(0.0, ROTATE_W * rotate_dir)
            else:
                state      = STATE_NORMAL
                prev_angle = 0.0
                rotate_accumulated = 0.0
                # [핵심 수정] 회전 중 쌓인 스캔 데이터 전부 초기화
                # 기존 -45~45 → -90~90으로 확장
                for a in range(-90, 91):
                    scan_data[a % 360] = float(SCAN_LIMIT)
                print("[ROTATE DONE] → NORMAL")
            continue

        # --------------------------------------------------
        # STATE_NORMAL: 일반 주행
        # --------------------------------------------------
        is_emergency, front_min = check_emergency()

        if is_emergency:
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            print(f"[EMERGENCY] front_min={front_min:.1f}cm → REVERSE")
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result    = find_best_direction(smoothing)

        if result is None:
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            print("[NO GAP] → REVERSE")
            continue

        target_angle, bias_label, front_clear = result
        v, w = compute_cmd(target_angle)
        send_cmd(v, w)

        print(f"[{bias_label:8s}] TRG:{target_angle:5.1f}° | v:{v:.2f} | w:{w:.2f} | f_min:{front_min:.1f}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
