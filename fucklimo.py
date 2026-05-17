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
MAX_SPEED    = 0.28
MIN_SPEED    = 0.18
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
# 사각형 Emergency 감지 박스
# =========================================
EMERGENCY_FRONT = 9.0
EMERGENCY_SIDE  = 8.0

# =========================================
# 실패 방향 패널티
# emergency 발생 직전에 향하던 GAP 방향을 기억해
# 일정 시간 동안 해당 방향 GAP 점수를 깎음
# → 같은 방향 GAP 반복 선택 루프 방지
# =========================================
PENALTY_WEIGHT   = 8.0   # GAP 점수 감점량 (클수록 회피 강도 ↑)
PENALTY_ANGLE    = 30.0  # 패널티 적용 각도 범위 (°), 실패 방향 ±30° 이내
PENALTY_DURATION = 3.0   # 패널티 유지 시간 (초)

penalty_angle = None  # 실패한 GAP 중심 각도 (°)
penalty_until = 0.0   # 패널티 만료 시각

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

REVERSE_DURATION  = 0.20
ROTATE_DURATION   = 1.00
REVERSE_SPEED     = -0.10
ROTATE_W          = 0.9

# =========================================
# STATE DATA
# =========================================
scan_data      = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle     = 0.0
last_gap_angle = 0.0  # 직전에 향하던 GAP 각도

# =========================================
# UTIL & FILTER
# =========================================
def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

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
# EMERGENCY 감지
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

def score_gap(gap, proc_dists, angles, now):
    """
    GAP 점수 계산.
    패널티 활성 중이면 실패 방향 ±PENALTY_ANGLE° 이내 GAP에 감점.
    가까울수록 더 많이 깎임 (선형 감쇠).
    """
    start, end   = gap
    width        = end - start
    center_i     = (start + end) / 2.0
    center_angle = float(angles[round(center_i)])
    avg_dist     = np.mean(proc_dists[start : end + 1])

    score = width * 0.5 + avg_dist * 1.2 - abs(center_angle) * 0.4

    if penalty_angle is not None and now < penalty_until:
        diff = abs(center_angle - penalty_angle)
        if diff < PENALTY_ANGLE:
            score -= PENALTY_WEIGHT * (1.0 - diff / PENALTY_ANGLE)

    return score

def select_best_gap(gaps, proc_dists, angles, now):
    best_gap, best_score = None, -1e9
    for gap in gaps:
        s = score_gap(gap, proc_dists, angles, now)
        if s > best_score:
            best_score, best_gap = s, gap
    return best_gap

# =========================================
# PLANNING
# =========================================
def find_best_direction(smoothing, now):
    global prev_angle
    angles     = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists      = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    gaps       = find_gaps(proc_dists, angles)

    if not gaps:
        return None

    best_gap     = select_best_gap(gaps, proc_dists, angles, now)
    start, end   = best_gap
    gap_angle    = float(angles[round((start + end) / 2.0)])
    front_clear  = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))

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
    return target, bias_label, front_clear, gap_angle

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
print("NAVIGATION START (Rect-based Emergency + Gap Penalty)")
print(f"Emergency box: front={EMERGENCY_FRONT}cm x side=±{EMERGENCY_SIDE}cm")
print(f"Penalty: weight={PENALTY_WEIGHT}, range=±{PENALTY_ANGLE}°, duration={PENALTY_DURATION}s")

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
                for a in range(-45, 46): scan_data[a % 360] = float(SCAN_LIMIT)
            continue

        # --- STATE_NORMAL ---
        is_emergency, front_min = check_emergency()

        if is_emergency:
            # 직전에 향하던 GAP 방향을 실패로 기록
            penalty_angle = last_gap_angle
            penalty_until = now + PENALTY_DURATION
            print(f"[EMERGENCY] GAP {penalty_angle:.1f}° 실패 → {PENALTY_DURATION}s 패널티")

            rotate_dir = choose_avoid_direction()
            state, maneuver_end_time = STATE_REVERSE, now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result = find_best_direction(smoothing, now)

        if result is None:
            rotate_dir = choose_avoid_direction()
            state, maneuver_end_time = STATE_REVERSE, now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        target_angle, bias_label, front_clear, gap_angle = result
        last_gap_angle = gap_angle  # 현재 향하는 GAP 각도 저장

        v, w = compute_cmd(target_angle)
        send_cmd(v, w)

        pen_str = f" [PEN:{penalty_angle:.0f}°]" if (penalty_angle is not None and now < penalty_until) else ""
        print(f"[{bias_label:8s}] TRG:{target_angle:5.1f}° | v:{v:.2f} | w:{w:.2f} | f_min:{front_min:.1f}{pen_str}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
