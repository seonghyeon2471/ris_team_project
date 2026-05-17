import serial
import math
import time
import numpy as np

# =========================================
# SERIAL
# =========================================

arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

USE_IMU = True
try:
    import smbus2
    imu_bus  = smbus2.SMBus(1)
    IMU_ADDR = 0x68
    imu_bus.write_byte_data(IMU_ADDR, 0x6B, 0)
except Exception:
    USE_IMU = False
    print("[WARN] IMU 초기화 실패 → IMU 비활성화")

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
# PARAMETERS
# =========================================

ROBOT_RADIUS = 14.0
WHEEL_BASE   = 17.0

MAX_SPEED = 0.20   # 20cm/s
MIN_SPEED = 0.10   # 10cm/s
MAX_W     = 1.8
TURN_GAIN = 1.2

SCAN_LIMIT  = 120
FRONT_RANGE = 40

EMA_ALPHA = 0.5
MEDIAN_K  = 2

SMOOTHING_NORMAL = 0.70
SMOOTHING_DANGER = 0.25
DANGER_DIST      = 30

SAFE_DIST          = 20
INFLATION_MAX_DIST = 60
FRONT_CLEAR_DIST   = 25
FRONT_CLEAR_RANGE  = 15

ROBOT_WIDTH        = 20.0
PASS_MARGIN        = 5.0

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2
STATE_RAMP    = 3
STATE_CREEP   = 4

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST   = 10
EMERGENCY_RANGE  = 30
REVERSE_DURATION = 0.35
ROTATE_DURATION  = 0.50
REVERSE_SPEED    = -0.10
ROTATE_W         = 0.9

NO_GAP_CREEP_SPEED = 0.07
NO_GAP_CREEP_TIME  = 0.3

RAMP_PITCH_THRESH  = 8.0
RAMP_EXIT_PITCH    = 3.0
LIDAR_DROP_THRESH  = 30.0
RAMP_LIDAR_TIMEOUT = 3.0
RAMP_SPEED         = 0.12
RAMP_INFLATION_MAX = 10
RAMP_SAFE_DIST     = 8

WALL_FOLLOW_TARGET = 16
WALL_FOLLOW_KP     = 0.3
WALL_FOLLOW_MAX_W  = 0.8
WALL_RAMP_DIST     = 23

RAMP_MODE_NORMAL     = 0
RAMP_MODE_WALL_LEFT  = 2
RAMP_MODE_WALL_RIGHT = 1

ALIGN_THRESHOLD = 25
DEADBAND_ANGLE  = 5

scan_data       = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle      = 0.0
prev_front_avg  = float(SCAN_LIMIT)
ramp_start_time = 0.0
ramp_mode       = RAMP_MODE_NORMAL

# =========================================
# 무한루프 탈출 변수
# =========================================

consec_emergency      = 0        # 연속 emergency 횟수
last_emergency_angle  = None     # 마지막 emergency 때 로봇 heading (rad)
rotate_duration       = ROTATE_DURATION  # 현재 회전 지속시간 (누적 증가)
force_flip_next       = False    # 다음 emergency 때 방향 강제 반전
MAX_ROTATE_DURATION   = 1.5     # 회전 최대 누적 한도(초)
LOOP_ANGLE_THRESH     = 0.5     # 루프 판정 각도 임계값(rad, ~28도)
ROTATE_INCREMENT      = 0.35    # 루프 감지 시 회전 추가량(초)

# =========================================
# IMU
# =========================================

def read_imu_pitch() -> float:
    if not USE_IMU:
        return 0.0
    try:
        def read_word(reg):
            h = imu_bus.read_byte_data(IMU_ADDR, reg)
            l = imu_bus.read_byte_data(IMU_ADDR, reg + 1)
            val = (h << 8) | l
            return val - 65536 if val >= 0x8000 else val
        ax = read_word(0x3B) / 16384.0
        ay = read_word(0x3D) / 16384.0
        az = read_word(0x3F) / 16384.0
        return abs(math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2))))
    except Exception:
        return 0.0

# =========================================
# RAMP
# =========================================

def detect_ramp(front_avg: float) -> bool:
    global prev_front_avg
    if USE_IMU:
        prev_front_avg = front_avg
        return read_imu_pitch() >= RAMP_PITCH_THRESH
    drop = front_avg - prev_front_avg
    prev_front_avg = front_avg
    return drop >= LIDAR_DROP_THRESH

def ramp_exited() -> bool:
    if USE_IMU:
        return read_imu_pitch() < RAMP_EXIT_PITCH
    return (time.time() - ramp_start_time) > RAMP_LIDAR_TIMEOUT

def detect_side_ramp():
    left_dist  = float(np.mean(scan_data[85:96]))
    right_dist = float(np.mean(scan_data[265:276]))
    if left_dist < WALL_RAMP_DIST and right_dist >= WALL_RAMP_DIST:
        return RAMP_MODE_WALL_LEFT
    if right_dist < WALL_RAMP_DIST and left_dist >= WALL_RAMP_DIST:
        return RAMP_MODE_WALL_RIGHT
    return RAMP_MODE_NORMAL

def compute_wall_follow_cmd(mode):
    if mode == RAMP_MODE_WALL_LEFT:
        wall_dist = float(np.mean(scan_data[85:96]))
        error = WALL_FOLLOW_TARGET - wall_dist
        w = float(np.clip(-error * WALL_FOLLOW_KP, -WALL_FOLLOW_MAX_W, WALL_FOLLOW_MAX_W))
        label = "WALL_FOLLOW_LEFT"
    elif mode == RAMP_MODE_WALL_RIGHT:
        wall_dist = float(np.mean(scan_data[265:276]))
        error = WALL_FOLLOW_TARGET - wall_dist
        w = float(np.clip(error * WALL_FOLLOW_KP, -WALL_FOLLOW_MAX_W, WALL_FOLLOW_MAX_W))
        label = "WALL_FOLLOW_RIGHT"
    else:
        wall_dist = 0.0
        w = 0.0
        label = "WALL_FOLLOW_NONE"
    return RAMP_SPEED, w, wall_dist, label

# =========================================
# FILTER
# =========================================

def drain_lidar_buffer():
    MAX_DRAIN = 30
    count = 0
    while count < MAX_DRAIN and lidar_ser.in_waiting >= 5:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            break
        s_flag     = raw[0] & 0x01
        s_inv_flag = (raw[0] & 0x02) >> 1
        if s_inv_flag != (1 - s_flag):
            count += 1
            continue
        if (raw[1] & 0x01) != 1:
            count += 1
            continue
        if (raw[0] >> 2) < 3:
            count += 1
            continue
        angle_raw = (raw[1] >> 1) | (raw[2] << 7)
        angle     = int(angle_raw / 64.0) % 360
        dist_cm   = (raw[3] | (raw[4] << 8)) / 4.0 / 10.0
        if 3 <= dist_cm <= SCAN_LIMIT:
            apply_ema(angle, dist_cm)
        count += 1

def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def apply_median_filter():
    k = MEDIAN_K
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        indices = [(i + d) % 360 for d in range(-k, k + 1)]
        filtered[i] = np.sort(scan_data[indices])[k]
    scan_data[:] = filtered

# =========================================
# OBSTACLE INFLATION
# =========================================

def inflate_obstacles(dists, inflation_max=None):
    if inflation_max is None:
        inflation_max = INFLATION_MAX_DIST
    proc = dists.copy()
    n = len(dists)
    for i in range(n):
        d = dists[i]
        if d < 5 or d >= inflation_max:
            continue
        ratio = ROBOT_RADIUS / d
        if ratio >= 1.0:
            alpha_deg = 45.0
        else:
            alpha_deg = math.degrees(math.asin(ratio))
        start_i = max(0, int(i - alpha_deg))
        end_i   = min(n - 1, int(i + alpha_deg))
        proc[start_i : end_i + 1] = 0.0
    return proc

def is_gap_passable(gap, proc_dists, angles):
    start_i, end_i = gap
    if start_i == end_i:
        return False, 0.0
    a_start_rad = math.radians(float(angles[start_i]))
    a_end_rad   = math.radians(float(angles[end_i]))
    d_start     = float(proc_dists[start_i])
    d_end       = float(proc_dists[end_i])
    if d_start <= 0 or d_end <= 0:
        return False, 0.0
    x1 = d_start * math.sin(a_start_rad)
    y1 = d_start * math.cos(a_start_rad)
    x2 = d_end   * math.sin(a_end_rad)
    y2 = d_end   * math.cos(a_end_rad)
    physical_width = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    required_width = ROBOT_WIDTH + PASS_MARGIN
    return physical_width >= required_width, physical_width

# =========================================
# GAP
# =========================================

def find_gaps(proc_dists, angles, safe_dist=None):
    if safe_dist is None:
        safe_dist = SAFE_DIST
    gaps, gap_start = [], None
    for i, d in enumerate(proc_dists):
        if d > safe_dist:
            if gap_start is None:
                gap_start = i
        else:
            if gap_start is not None:
                gaps.append((gap_start, i - 1))
                gap_start = None
    if gap_start is not None:
        gaps.append((gap_start, len(proc_dists) - 1))

    passable_gaps = []
    for g in gaps:
        result = is_gap_passable(g, proc_dists, angles)
        if result is not False:
            ok, width = result
            if ok:
                passable_gaps.append(g)
            else:
                print(f"  [GAP FILTER] 폭 {width:.1f}cm < {ROBOT_WIDTH+PASS_MARGIN:.0f}cm → 제거")
    return passable_gaps

def score_gap(gap, proc_dists, angles):
    start, end   = gap
    center_angle = angles[int((start + end) / 2)]
    avg_dist     = np.mean(proc_dists[start : end + 1])
    min_dist     = np.min(proc_dists[start : end + 1])
    return (end - start) * 1.5 + avg_dist * 2.0 + min_dist * 1.0 - abs(center_angle) * 0.15

def select_best_gap(gaps, proc_dists, angles):
    return max(gaps, key=lambda g: score_gap(g, proc_dists, angles))

# =========================================
# PLANNING
# =========================================

def find_best_direction(smoothing, on_ramp=False):
    global prev_angle
    angles     = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists      = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    infl_max   = RAMP_INFLATION_MAX if on_ramp else INFLATION_MAX_DIST
    safe_dist  = RAMP_SAFE_DIST     if on_ramp else SAFE_DIST
    proc_dists = inflate_obstacles(dists, inflation_max=infl_max)
    gaps       = find_gaps(proc_dists, angles, safe_dist=safe_dist)
    if not gaps:
        return None
    best_gap    = select_best_gap(gaps, proc_dists, angles)
    gap_angle   = float(angles[int((best_gap[0] + best_gap[1]) / 2)])
    _, gap_width = is_gap_passable(best_gap, proc_dists, angles)
    front_clear = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))
    if front_clear > FRONT_CLEAR_DIST:
        target, bias_label = gap_angle * 0.2, "STRAIGHT"
    else:
        target, bias_label = gap_angle * 1.0, "GAP"
    target     = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    return target, bias_label, front_clear, gap_width

# =========================================
# CONTROL
# =========================================

def compute_cmd(target_angle, on_ramp=False):
    w = float(np.clip(math.radians(target_angle) * TURN_GAIN, -MAX_W, MAX_W))
    if abs(target_angle) <= DEADBAND_ANGLE:
        w = 0.0
    if on_ramp:
        return RAMP_SPEED, w
    front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))
    if abs(target_angle) > ALIGN_THRESHOLD:
        return MIN_SPEED, w
    speed = max(MAX_SPEED * min(front_min / 60.0, 1.0), MIN_SPEED)
    return speed, w

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# AVOID DIRECTION (무한루프 탈출 포함)
# =========================================

def choose_avoid_direction():
    """
    좌/우 평균 거리 기반 회피 방향 결정.
    연속 emergency 시 루프 감지 → 방향 강제 반전 + 회전량 증가.
    """
    global rotate_dir, consec_emergency, last_emergency_angle
    global rotate_duration, force_flip_next

    left_avg  = float(np.mean(scan_data[np.arange(1,  61) % 360]))
    right_avg = float(np.mean(scan_data[np.arange(300, 360) % 360]))

    # 루프 감지: 이전 emergency 각도와 현재 각도 비교
    import math as _math
    current_heading = 0.0  # heading은 외부에서 추적 불가, scan 패턴으로 대체
    # → force_flip_next 플래그로 대신 처리

    if force_flip_next:
        # 강제 반전
        new_dir = -rotate_dir
        rotate_duration = min(rotate_duration + ROTATE_INCREMENT, MAX_ROTATE_DURATION)
        force_flip_next = False
        print(f"  [LOOP ESCAPE] 강제반전 dir:{'+' if new_dir>0 else '-'} rot_dur:{rotate_duration:.2f}s")
    else:
        new_dir = 1 if left_avg >= right_avg else -1
        consec_emergency += 1
        if consec_emergency >= 2:
            # 2회 연속이면 다음엔 강제 반전
            force_flip_next = True
            rotate_duration = min(rotate_duration + ROTATE_INCREMENT, MAX_ROTATE_DURATION)
            print(f"  [LOOP WARN] 연속{consec_emergency}회 → 다음 강제반전 예약, rot_dur:{rotate_duration:.2f}s")
        print(f"  [AVOID DIR] {'LEFT' if new_dir>0 else 'RIGHT'} (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")

    rotate_dir = new_dir
    return new_dir

def reset_emergency_state():
    """정상 주행 구간에서 emergency 카운터 리셋."""
    global consec_emergency, rotate_duration, force_flip_next
    consec_emergency = 0
    rotate_duration  = ROTATE_DURATION
    force_flip_next  = False

# =========================================
# MAIN LOOP
# =========================================

print("NAVIGATION START")

creep_end_time = 0.0

try:
    while True:

        drain_lidar_buffer()

        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue
        s_flag     = raw[0] & 0x01
        s_inv_flag = (raw[0] & 0x02) >> 1
        if s_inv_flag != (1 - s_flag):
            continue
        if (raw[1] & 0x01) != 1:
            continue
        if (raw[0] >> 2) < 3:
            continue
        angle_raw = (raw[1] >> 1) | (raw[2] << 7)
        angle     = int(angle_raw / 64.0) % 360
        dist_cm   = (raw[3] | (raw[4] << 8)) / 4.0 / 10.0
        if dist_cm < 3 or dist_cm > SCAN_LIMIT:
            continue

        apply_ema(angle, dist_cm)

        if s_flag != 1:
            continue

        apply_median_filter()

        now        = time.time()
        front_avg  = float(np.mean(scan_data[np.arange(-10, 11) % 360]))
        front_min  = float(np.min(scan_data[np.arange(-10, 11) % 360]))

        # ── STATE_REVERSE ──
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
                print(f"  [REVERSE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                state = STATE_ROTATE
                maneuver_end_time = now + rotate_duration  # 누적된 rotate_duration 사용
                print(f"  [ROTATE START] dir:{'+' if rotate_dir > 0 else '-'} dur:{rotate_duration:.2f}s")
            continue

        # ── STATE_ROTATE ──
        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
                print(f"  [ROTATE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                # 회전 후 갭 재탐색
                result = find_best_direction(SMOOTHING_DANGER)
                if result is not None:
                    state          = STATE_NORMAL
                    prev_angle     = 0.0
                    prev_front_avg = front_avg
                    reset_emergency_state()
                    print("  [NORMAL] 갭 확보 → 주행 재개")
                else:
                    # 갭 없으면 추가 회전
                    maneuver_end_time = now + 0.35
                    print("  [ROTATE] 갭 없음 → 추가 회전 0.35s")
            continue

        # ── STATE_CREEP ──
        if state == STATE_CREEP:
            if now < creep_end_time:
                send_cmd(NO_GAP_CREEP_SPEED, 0.0)
                print(f"  [CREEP] remaining:{creep_end_time - now:.2f}s")
            else:
                result = find_best_direction(SMOOTHING_DANGER)
                if result is not None:
                    state = STATE_NORMAL
                    print("  [CREEP→NORMAL] 갭 발견")
                else:
                    choose_avoid_direction()
                    state             = STATE_REVERSE
                    maneuver_end_time = now + REVERSE_DURATION
                    print("  [CREEP→REVERSE] 갭 없음")
                    send_cmd(REVERSE_SPEED, 0.0)
            continue

        # ── STATE_RAMP ──
        if state == STATE_RAMP:
            if ramp_exited():
                state          = STATE_NORMAL
                ramp_mode      = RAMP_MODE_NORMAL
                prev_front_avg = float(SCAN_LIMIT)
                print("  [RAMP→NORMAL] 경사 통과 완료")
                send_cmd(RAMP_SPEED, 0.0)
                continue

            emergency_min = float(np.min(scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE + 1) % 360]))
            if emergency_min < EMERGENCY_DIST:
                choose_avoid_direction()
                state             = STATE_REVERSE
                ramp_mode         = RAMP_MODE_NORMAL
                maneuver_end_time = now + REVERSE_DURATION
                print(f"  [RAMP] EMERGENCY! front:{emergency_min:.1f}cm → REVERSE")
                send_cmd(REVERSE_SPEED, 0.0)
                continue

            detected_mode = detect_side_ramp()
            ramp_mode = detected_mode

            if ramp_mode != RAMP_MODE_NORMAL:
                v, w, wall_dist, label = compute_wall_follow_cmd(ramp_mode)
                send_cmd(v, w)
                print(f"  [RAMP/{label}] wall:{wall_dist:.1f}cm v:{v:.2f} w:{w:.2f}")
            else:
                result = find_best_direction(SMOOTHING_NORMAL, on_ramp=True)
                if result is not None:
                    target_angle, bias_label, front_clear, gap_width = result
                    v, w = compute_cmd(target_angle, on_ramp=True)
                    send_cmd(v, w)
                    print(f"  [RAMP/GAP] target:{target_angle:5.1f}° v:{v:.2f} w:{w:.2f} pitch:{read_imu_pitch():.1f}° front:{front_min:.1f}cm gap_w:{gap_width:.1f}cm")
                else:
                    send_cmd(RAMP_SPEED, 0.0)
                    print("  [RAMP] NO GAP → 서행 직진 유지")
            continue

        # ── STATE_NORMAL ──

        emergency_min = float(np.min(scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE + 1) % 360]))
        if emergency_min < EMERGENCY_DIST:
            choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            print(f"EMERGENCY! front:{emergency_min:.1f}cm → REVERSE")
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        if detect_ramp(front_avg):
            state           = STATE_RAMP
            ramp_start_time = now
            print(f"RAMP DETECTED! pitch:{read_imu_pitch():.1f}° front_avg:{front_avg:.1f}cm → STATE_RAMP")
            send_cmd(RAMP_SPEED, 0.0)
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result    = find_best_direction(smoothing)

        if result is None:
            state          = STATE_CREEP
            creep_end_time = now + NO_GAP_CREEP_TIME
            print("NO GAP! → CREEP 시도")
            send_cmd(NO_GAP_CREEP_SPEED, 0.0)
            continue

        # 정상 주행 중 emergency 카운터 리셋
        reset_emergency_state()

        target_angle, bias_label, front_clear, gap_width = result
        v, w = compute_cmd(target_angle)
        send_cmd(v, w)
        print(
            f"TARGET:{target_angle:6.1f}° | v:{v:.2f}m/s | w:{w:.2f}r/s | "
            f"front:{front_min:.1f}cm | clear:{front_clear:.1f}cm | "
            f"gap_w:{gap_width:.1f}cm | bias:{bias_label} | smooth:{smoothing:.2f}"
        )

        prev_front_avg = front_avg

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
