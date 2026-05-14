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
# ROBOT PHYSICAL PARAMETER
# =========================================

ROBOT_RADIUS = 14.0
WHEEL_BASE   = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================

MAX_SPEED = 0.20
MIN_SPEED = 0.07
MAX_W     = 1.8
TURN_GAIN = 1.2

SCAN_LIMIT  = 150
FRONT_RANGE = 60

# =========================================
# FILTER PARAMETER
# =========================================

EMA_ALPHA = 0.5
MEDIAN_K  = 2

# =========================================
# SMOOTHING PARAMETER
# =========================================

SMOOTHING_NORMAL = 0.70
SMOOTHING_DANGER = 0.25
DANGER_DIST      = 30

# =========================================
# GAP PARAMETER
# =========================================

SAFE_DIST          = 17
INFLATION_MAX_DIST = 25

FRONT_CLEAR_DIST  = 25
FRONT_CLEAR_RANGE = 15

# =========================================
# STATE MACHINE
# =========================================

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2
STATE_RAMP    = 3

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST   = 8
EMERGENCY_RANGE  = 5
REVERSE_DURATION = 0.25
REVERSE_SPEED    = -0.10

# ── [수정 2] 회전 각도 축소 ──────────────────────────
ROTATE_DURATION  = 0.65   # 1.00 → 0.65s
ROTATE_W         = 0.70   # 0.90 → 0.70 rad/s
# ─────────────────────────────────────────────────────

# =========================================
# RAMP PARAMETER
# =========================================

RAMP_PITCH_THRESH  = 8.0
RAMP_EXIT_PITCH    = 3.0
LIDAR_DROP_THRESH  = 30.0
RAMP_LIDAR_TIMEOUT = 3.0
RAMP_SPEED         = 0.12
RAMP_INFLATION_MAX = 10
RAMP_SAFE_DIST     = 8

WALL_FOLLOW_TARGET = 25
WALL_FOLLOW_KP     = 0.4
WALL_FOLLOW_MAX_W  = 0.8
WALL_RAMP_DIST     = 20

RAMP_MODE_NORMAL     = 0
RAMP_MODE_WALL_LEFT  = 1
RAMP_MODE_WALL_RIGHT = 2

# =========================================
# STATE
# =========================================

scan_data       = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle      = 0.0
prev_front_avg  = float(SCAN_LIMIT)
ramp_start_time = 0.0
ramp_mode       = RAMP_MODE_NORMAL

# Emergency 회전 방향 우선순위:
# 마지막으로 보낸 non-zero w의 반대 방향으로 회전
last_nonzero_w  = 0.0   # send_cmd 호출 시 갱신

# =========================================
# UTIL
# =========================================

def normalize_angle(angle):
    return int(angle) % 360


# =========================================
# 측면 경사 감지 + 벽 추종
# =========================================

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
        wall_dist = float(np.mean(scan_data[265:276]))
        error = WALL_FOLLOW_TARGET - wall_dist
        w = float(np.clip(-error * WALL_FOLLOW_KP, -WALL_FOLLOW_MAX_W, WALL_FOLLOW_MAX_W))
        label = "WALL_FOLLOW_L(right_wall)"
    else:
        wall_dist = float(np.mean(scan_data[85:96]))
        error = WALL_FOLLOW_TARGET - wall_dist
        w = float(np.clip(error * WALL_FOLLOW_KP, -WALL_FOLLOW_MAX_W, WALL_FOLLOW_MAX_W))
        label = "WALL_FOLLOW_R(left_wall)"

    return RAMP_SPEED, w, wall_dist, label


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
        pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
        return abs(pitch)
    except Exception:
        return 0.0


# =========================================
# 경사면 감지
# =========================================

def detect_ramp(front_avg: float) -> bool:
    global prev_front_avg

    if USE_IMU:
        pitch = read_imu_pitch()
        prev_front_avg = front_avg
        return pitch >= RAMP_PITCH_THRESH

    drop = front_avg - prev_front_avg
    prev_front_avg = front_avg
    return drop >= LIDAR_DROP_THRESH


def ramp_exited() -> bool:
    if USE_IMU:
        return read_imu_pitch() < RAMP_EXIT_PITCH
    return (time.time() - ramp_start_time) > RAMP_LIDAR_TIMEOUT


# =========================================
# FILTER
# =========================================

def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (
        (1.0 - EMA_ALPHA) * scan_data[angle]
        + EMA_ALPHA * new_dist_cm
    )


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
# OBSTACLE INFLATION
# =========================================

def inflate_obstacles(dists, inflation_max=None):
    if inflation_max is None:
        inflation_max = INFLATION_MAX_DIST

    proc = dists.copy()

    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= inflation_max:
            continue
        alpha     = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists) - 1, int(i + alpha))
        proc[start_idx : end_idx + 1] = 0.0

    return proc


# =========================================
# GAP SEARCH
# =========================================

def find_gaps(proc_dists, angles, safe_dist=None):
    if safe_dist is None:
        safe_dist = SAFE_DIST

    gaps      = []
    gap_start = None

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

    return gaps


def score_gap(gap, proc_dists, angles):
    start, end   = gap
    width        = end - start
    center_i     = (start + end) / 2.0
    center_angle = angles[int(center_i)]
    avg_dist     = np.mean(proc_dists[start : end + 1])
    min_dist     = np.min(proc_dists[start : end + 1])

    score = (
        width               * 1.5
        + avg_dist          * 2.0
        + min_dist          * 1.0
        - abs(center_angle) * 0.15
    )
    return score


def select_best_gap(gaps, proc_dists, angles):
    best_gap   = None
    best_score = -1e9

    for gap in gaps:
        s = score_gap(gap, proc_dists, angles)
        if s > best_score:
            best_score = s
            best_gap   = gap

    return best_gap


# =========================================
# PLANNING
# =========================================

def find_best_direction(smoothing, on_ramp=False):
    global prev_angle

    angles    = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists     = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)

    infl_max  = RAMP_INFLATION_MAX if on_ramp else INFLATION_MAX_DIST
    safe_dist = RAMP_SAFE_DIST     if on_ramp else SAFE_DIST

    proc_dists = inflate_obstacles(dists, inflation_max=infl_max)
    gaps       = find_gaps(proc_dists, angles, safe_dist=safe_dist)

    if not gaps:
        return None

    best_gap             = select_best_gap(gaps, proc_dists, angles)
    start, end           = best_gap
    center_i             = (start + end) / 2.0
    gap_angle            = float(angles[int(center_i)])

    front_clear = float(np.min(
        scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]
    ))

    if front_clear > FRONT_CLEAR_DIST:
        target     = gap_angle * 0.2
        bias_label = "STRAIGHT"
    else:
        target     = gap_angle * 1.0
        bias_label = "GAP"

    target     = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target

    return target, bias_label, front_clear


# =========================================
# CONTROL
# =========================================

ALIGN_THRESHOLD = 25
DEADBAND_ANGLE  =  5

def compute_cmd(target_angle, on_ramp=False):
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    if abs(target_angle) <= DEADBAND_ANGLE:
        w = 0.0

    if on_ramp:
        return RAMP_SPEED, w

    if abs(target_angle) > ALIGN_THRESHOLD:
        return MIN_SPEED, w

    front_min      = float(np.min(scan_data[np.arange(-10, 11) % 360]))
    obstacle_scale = min(front_min / 60.0, 1.0)
    speed          = max(MAX_SPEED * obstacle_scale, MIN_SPEED)

    return speed, w


def send_cmd(v, w):
    global last_nonzero_w
    if w != 0.0:
        last_nonzero_w = w
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())


def stop_robot():
    send_cmd(0.0, 0.0)


# =========================================
# AVOID DIRECTION
# =========================================

def choose_avoid_direction():
    """
    회전 방향 결정 우선순위:
    1순위 — 마지막 non-zero w의 반대 방향
             (직전 주행 방향 기억 → 왔던 방향으로 돌아감)
    2순위 — 좌우 스캔 평균 거리가 더 넓은 쪽
             (last_nonzero_w == 0 이거나 양쪽 차이 없을 때 fallback)
    """
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))

    if last_nonzero_w != 0.0:
        # w 양수 = 왼쪽 회전 중이었음 → 반대(오른쪽, -1)로
        # w 음수 = 오른쪽 회전 중이었음 → 반대(왼쪽, +1)로
        direction = -1 if last_nonzero_w > 0 else 1
        label     = "LEFT" if direction > 0 else "RIGHT"
        print(
            f"  [AVOID DIR] {label} (last_w:{last_nonzero_w:+.2f} → 반대방향) "
            f"L:{left_avg:.1f}cm R:{right_avg:.1f}cm"
        )
        return direction

    # fallback: 공간이 넓은 쪽
    if left_avg >= right_avg:
        print(f"  [AVOID DIR] LEFT  (fallback L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return 1
    else:
        print(f"  [AVOID DIR] RIGHT (fallback L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return -1


# =========================================
# MAIN LOOP
# =========================================

print("NAVIGATION START")

try:
    while True:

        # ---------------------------------
        # LiDAR 패킷 수신 + 검증
        # ---------------------------------
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

        s_flag     = raw[0] & 0x01
        s_inv_flag = (raw[0] & 0x02) >> 1

        if s_inv_flag != (1 - s_flag):
            continue

        if (raw[1] & 0x01) != 1:
            continue

        quality = raw[0] >> 2
        if quality < 3:
            continue

        angle_raw = (raw[1] >> 1) | (raw[2] << 7)
        angle     = int(angle_raw / 64.0) % 360

        dist_raw = raw[3] | (raw[4] << 8)
        dist_cm  = (dist_raw / 4.0) / 10.0

        if dist_cm < 3 or dist_cm > SCAN_LIMIT:
            continue

        apply_ema(angle, dist_cm)

        # ── [수정 3] Emergency 체크를 스캔 완료(s_flag==1) 전에 배치 ──────
        # 매 패킷마다 정면 ±EMERGENCY_RANGE° 최솟값을 확인하여
        # s_flag 조건과 무관하게 즉각 반응
        if state == STATE_NORMAL:
            emergency_min = float(np.min(
                scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE + 1) % 360]
            ))
            if emergency_min < EMERGENCY_DIST:
                rotate_dir        = choose_avoid_direction()
                state             = STATE_REVERSE
                maneuver_end_time = time.time() + REVERSE_DURATION
                print(f"EMERGENCY(early)! front:{emergency_min:.1f}cm → REVERSE")
                send_cmd(REVERSE_SPEED, 0.0)
                continue
        # ─────────────────────────────────────────────────────────────────

        if s_flag != 1:
            continue

        apply_median_filter()

        now = time.time()

        front_avg = float(np.mean(scan_data[np.arange(-10, 11) % 360]))
        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))

        # =================================
        # STATE MACHINE
        # =================================

        # ── STATE_REVERSE ──
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
                print(f"  [REVERSE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                state = STATE_ROTATE
                maneuver_end_time = now + ROTATE_DURATION
                print(f"  [ROTATE START] dir:{'+' if rotate_dir > 0 else '-'}")
            continue

        # ── STATE_ROTATE ──
        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
                print(f"  [ROTATE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                rotate_dir *= -1
                state       = STATE_NORMAL
                prev_angle  = 0.0

                for a in range(-45, 46):
                    scan_data[a % 360] = float(SCAN_LIMIT)

                print("  [NORMAL] maneuver done / scan reset ±45°")
            continue

        # ── STATE_RAMP ──
        if state == STATE_RAMP:
            # [수정 1] global 키워드 제거 → 모듈 수준 ramp_mode 직접 참조
            # (루프 안 대입은 모듈 변수를 바꾸므로 global 불필요)
            if ramp_exited():
                state          = STATE_NORMAL
                ramp_mode      = RAMP_MODE_NORMAL   # ← global 없이 정상 작동
                prev_front_avg = float(SCAN_LIMIT)
                print("  [RAMP→NORMAL] 경사 통과 완료")
                send_cmd(RAMP_SPEED, 0.0)
                continue

            emergency_min = float(np.min(
                scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE + 1) % 360]
            ))
            if emergency_min < EMERGENCY_DIST:
                rotate_dir        = choose_avoid_direction()
                state             = STATE_REVERSE
                ramp_mode         = RAMP_MODE_NORMAL
                maneuver_end_time = now + REVERSE_DURATION
                print(f"  [RAMP] EMERGENCY! front:{emergency_min:.1f}cm → REVERSE")
                send_cmd(REVERSE_SPEED, 0.0)
                continue

            detected_mode = detect_side_ramp()
            if detected_mode != RAMP_MODE_NORMAL:
                ramp_mode = detected_mode

            if ramp_mode != RAMP_MODE_NORMAL:
                v, w, wall_dist, label = compute_wall_follow_cmd(ramp_mode)
                send_cmd(v, w)
                print(f"  [RAMP/{label}] wall:{wall_dist:.1f}cm v:{v:.2f} w:{w:.2f}")
            else:
                result = find_best_direction(SMOOTHING_NORMAL, on_ramp=True)
                if result is not None:
                    target_angle, bias_label, front_clear = result
                    v, w = compute_cmd(target_angle, on_ramp=True)
                    send_cmd(v, w)
                    pitch = read_imu_pitch()
                    print(
                        f"  [RAMP/GAP] target:{target_angle:5.1f}° "
                        f"v:{v:.2f} w:{w:.2f} "
                        f"pitch:{pitch:.1f}° "
                        f"front:{front_min:.1f}cm"
                    )
                else:
                    send_cmd(RAMP_SPEED, 0.0)
                    print("  [RAMP] NO GAP → 서행 직진 유지")
            continue

        # ── STATE_NORMAL ──

        # Emergency (스캔 완료 후 재확인 — 필터 적용된 값으로 정밀 판단)
        emergency_min = float(np.min(
            scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE + 1) % 360]
        ))
        if emergency_min < EMERGENCY_DIST:
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            print(f"EMERGENCY! front:{emergency_min:.1f}cm → REVERSE")
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        if detect_ramp(front_avg):
            state           = STATE_RAMP
            ramp_start_time = now
            pitch           = read_imu_pitch()
            print(f"RAMP DETECTED! pitch:{pitch:.1f}° front_avg:{front_avg:.1f}cm → STATE_RAMP")
            send_cmd(RAMP_SPEED, 0.0)
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL

        result = find_best_direction(smoothing)

        if result is None:
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            print("NO GAP! → REVERSE")
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        target_angle, bias_label, front_clear = result

        v, w = compute_cmd(target_angle)
        send_cmd(v, w)

        print(
            f"TARGET:{target_angle:6.1f}° | "
            f"v:{v:.2f}m/s | w:{w:.2f}r/s | "
            f"front:{front_min:.1f}cm | "
            f"clear:{front_clear:.1f}cm | "
            f"bias:{bias_label} | "
            f"smooth:{smoothing:.2f}"
        )

        prev_front_avg = front_avg

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
