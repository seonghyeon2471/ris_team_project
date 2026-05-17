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
ROBOT_RADIUS = 10.0   # 반지름 10cm (지름 20cm)
WHEEL_BASE   = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED  = 0.28
MIN_SPEED  = 0.18
MAX_W      = 1.0
TURN_GAIN  = 1.8

SCAN_LIMIT  = 150
FRONT_RANGE = 60    # 정상 주행 탐색 범위 ±60°
WIDE_RANGE  = 120   # 탈출 탐색 범위 ±120°

# =========================================
# FILTER PARAMETER
# =========================================
EMA_ALPHA = 0.3
MEDIAN_K  = 2

# =========================================
# SMOOTHING PARAMETER
# =========================================
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.20
DANGER_DIST      = 18

# =========================================
# GAP & INFLATION PARAMETER
# =========================================
SAFE_DIST          = 20
INFLATION_MAX_DIST = 25
FRONT_CLEAR_DIST   = 23
FRONT_CLEAR_RANGE  = 15

# =========================================
# EMERGENCY 감지 박스
# =========================================
EMERGENCY_FRONT = 9.0
EMERGENCY_SIDE  = 8.0

# =========================================
# EMERGENCY 쿨다운 (연속 재트리거 방지)
# =========================================
EMERGENCY_COOLDOWN  = 0.5
last_emergency_time = 0.0

# =========================================
# 실패 방향 패널티
# =========================================
PENALTY_WEIGHT   = 10.0
PENALTY_ANGLE    = 35.0
PENALTY_DURATION = 4.0

penalty_angle = None
penalty_until = 0.0

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

REVERSE_DURATION   = 0.20
REVERSE_SPEED      = -0.10
ROTATE_W           = 0.7
ROTATE_TARGET_DEG  = 75.0
ROTATE_DURATION    = math.radians(ROTATE_TARGET_DEG) / ROTATE_W  # ≈ 1.87s

# =========================================
# STATE DATA
# =========================================
scan_data      = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle     = 0.0
last_gap_angle = 0.0

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
        indices      = [(i + d) % 360 for d in range(-k, k + 1)]
        values       = np.sort(scan_data[indices])
        filtered[i]  = values[window // 2]
    scan_data[:] = filtered

# =========================================
# EMERGENCY 감지
# =========================================
def check_emergency():
    triggered = False
    for i in range(-90, 91):
        angle = i % 360
        d     = scan_data[angle]
        if d >= SCAN_LIMIT:
            continue
        rad = math.radians(i)
        x   = d * math.sin(rad)
        y   = d * math.cos(rad)
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
# 로봇 통과 가능 최소 GAP 각도 (지름 20cm 기준)
# =========================================
def min_gap_angle(dist_cm):
    if dist_cm <= ROBOT_RADIUS:
        return 180.0
    return math.degrees(2.0 * math.asin(min(ROBOT_RADIUS / dist_cm, 1.0)))

# =========================================
# GAP SEARCH
# strict=True  → 로봇이 실제 통과 가능한 GAP만 인정
# strict=False → 폭 무시, 방향 힌트용 (막힌 상황 탈출)
# =========================================
def find_gaps(proc_dists, angles, strict=True):
    gaps      = []
    gap_start = None

    for i, d in enumerate(proc_dists):
        if d > SAFE_DIST:
            if gap_start is None:
                gap_start = i
        else:
            if gap_start is not None:
                gap_end   = i - 1
                width_deg = gap_end - gap_start
                avg_dist  = float(np.mean(proc_dists[gap_start : gap_end + 1]))
                if not strict or width_deg >= min_gap_angle(avg_dist):
                    gaps.append((gap_start, gap_end))
                gap_start = None

    if gap_start is not None:
        gap_end   = len(proc_dists) - 1
        width_deg = gap_end - gap_start
        avg_dist  = float(np.mean(proc_dists[gap_start : gap_end + 1]))
        if not strict or width_deg >= min_gap_angle(avg_dist):
            gaps.append((gap_start, gap_end))

    return gaps

# =========================================
# GAP SCORING
# =========================================
def score_gap(gap, proc_dists, angles, now):
    start, end   = gap
    width        = end - start
    center_i     = (start + end) / 2.0
    center_angle = float(angles[round(center_i)])
    avg_dist     = float(np.mean(proc_dists[start : end + 1]))

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
# 1단계: ±60°  strict  → 통과 가능 GAP (정상 주행)
# 2단계: ±120° strict  → 넓은 범위 통과 가능 GAP
# 3단계: ±120° loose   → 폭 무시, 방향 힌트만 (막힌 상황)
# 3단계도 실패 → None → ROTATE
# =========================================
def find_best_direction(smoothing, now):
    global prev_angle

    for front_range, strict, label in [
        (FRONT_RANGE, True,  None),
        (WIDE_RANGE,  True,  "WIDE GAP"),
        (WIDE_RANGE,  False, "LOOSE GAP"),
    ]:
        angles     = np.arange(-front_range, front_range + 1)
        dists      = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
        proc_dists = inflate_obstacles(dists)
        gaps       = find_gaps(proc_dists, angles, strict=strict)

        if not gaps:
            continue

        best_gap    = select_best_gap(gaps, proc_dists, angles, now)
        start, end  = best_gap
        gap_angle   = float(angles[round((start + end) / 2.0)])
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

        if label:
            bias_label = label

        target     = prev_angle * smoothing + target * (1.0 - smoothing)
        prev_angle = target
        return target, bias_label, front_clear, gap_angle

    return None

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

def choose_avoid_direction():
    """좌우 중 더 열린 방향. 매번 새로 계산 (누적 반전 없음)"""
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    return 1 if left_avg >= right_avg else -1

# =========================================
# MAIN LOOP
# =========================================
print("NAVIGATION START")
print(f"Robot diameter : {ROBOT_RADIUS * 2:.0f}cm")
print(f"Emergency box  : front={EMERGENCY_FRONT}cm x side=±{EMERGENCY_SIDE}cm")
print(f"Rotate         : {ROTATE_TARGET_DEG:.0f}° / {ROTATE_DURATION:.2f}s")
print(f"Penalty        : ±{PENALTY_ANGLE}°, {PENALTY_DURATION}s")

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

        # --- STATE: REVERSE ---
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
            else:
                state             = STATE_ROTATE
                maneuver_end_time = now + ROTATE_DURATION
            continue

        # --- STATE: ROTATE ---
        # 목표 각도(75°)만큼 딱 돌고 복귀
        # GAP 체크로 멈추지 않음 → 벽 옆에서 무한대기 방지
        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
            else:
                state      = STATE_NORMAL
                prev_angle = 0.0
                # 전방만 초기화 (측면·후방 벽 정보는 유지)
                for a in range(-FRONT_RANGE, FRONT_RANGE + 1):
                    scan_data[a % 360] = float(SCAN_LIMIT)
                print("[ROTATE] 완료 → NORMAL")
            continue

        # --- STATE: NORMAL ---
        is_emergency, front_min = check_emergency()

        if is_emergency:
            # 쿨다운 중 → 주행 유지하며 GAP 재탐색
            if now - last_emergency_time < EMERGENCY_COOLDOWN:
                smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
                result    = find_best_direction(smoothing, now)
                if result:
                    target_angle, _, _, gap_angle = result
                    last_gap_angle = gap_angle
                    v, w = compute_cmd(target_angle)
                    send_cmd(v, w)
                continue

            penalty_angle       = last_gap_angle
            penalty_until       = now + PENALTY_DURATION
            last_emergency_time = now

            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            print(f"[EMERGENCY] GAP {penalty_angle:.1f}° 실패 | {'L' if rotate_dir > 0 else 'R'}")
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result    = find_best_direction(smoothing, now)

        if result is None:
            # 3단계 탐색 모두 실패 → ROTATE
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            print("[NO GAP] 사방 막힘 → ROTATE")
            continue

        target_angle, bias_label, front_clear, gap_angle = result
        last_gap_angle = gap_angle

        v, w = compute_cmd(target_angle)
        send_cmd(v, w)

        pen_str = f" [PEN:{penalty_angle:.0f}°]" if (penalty_angle is not None and now < penalty_until) else ""
        print(f"[{bias_label:8s}] TRG:{target_angle:5.1f}° | v:{v:.2f} | w:{w:.2f} | f:{front_min:.1f}{pen_str}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
