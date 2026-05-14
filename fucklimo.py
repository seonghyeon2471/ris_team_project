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

MAX_SPEED = 0.20
MIN_SPEED = 0.07
MAX_W     = 1.8
TURN_GAIN = 1.2

SCAN_LIMIT  = 150
FRONT_RANGE = 60

EMA_ALPHA = 0.5
MEDIAN_K  = 2

SMOOTHING_NORMAL = 0.70
SMOOTHING_DANGER = 0.25
DANGER_DIST      = 30

SAFE_DIST          = 17
INFLATION_MAX_DIST = 25
FRONT_CLEAR_DIST   = 25
FRONT_CLEAR_RANGE  = 15

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2
STATE_RAMP    = 3

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

# ★ 수정: 5 → 30 (±50° 측면 장애물도 긴급 회피 트리거)
EMERGENCY_DIST   = 8
EMERGENCY_RANGE  = 30
REVERSE_DURATION = 0.25
ROTATE_DURATION  = 1.00
REVERSE_SPEED    = -0.10
ROTATE_W         = 0.9

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
        # 왼쪽 벽 기준 추종 (85~96도)
        wall_dist = float(np.mean(scan_data[85:96]))
        error = WALL_FOLLOW_TARGET - wall_dist
        
        # 거리가 너무 가까우면(error > 0) 우회전(-w) 해야 하므로 부호는 마이너스(-)
        w = float(np.clip(-error * WALL_FOLLOW_KP, -WALL_FOLLOW_MAX_W, WALL_FOLLOW_MAX_W))
        label = "WALL_FOLLOW_LEFT"
        
    elif mode == RAMP_MODE_WALL_RIGHT:
        # 오른쪽 벽 기준 추종 (265~276도)
        wall_dist = float(np.mean(scan_data[265:276]))
        error = WALL_FOLLOW_TARGET - wall_dist
        
        # 거리가 너무 가까우면(error > 0) 좌회전(+w) 해야 하므로 부호는 플러스(+)
        w = float(np.clip(error * WALL_FOLLOW_KP, -WALL_FOLLOW_MAX_W, WALL_FOLLOW_MAX_W))
        label = "WALL_FOLLOW_RIGHT"
        
    else:
        # RAMP_MODE_NORMAL 예외 처리
        wall_dist = 0.0
        w = 0.0
        label = "WALL_FOLLOW_NONE"
        
    return RAMP_SPEED, w, wall_dist, label

# =========================================
# FILTER
# =========================================

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
# OBSTACLE INFLATION  ★ 인덱스 버그 수정
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
        alpha_deg = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
        # ★ 수정: alpha를 각도(인덱스 단위)로 직접 사용해 슬라이싱
        #    기존엔 int(i ± alpha) 슬라이싱이 배열 끝에서 잘려 50° 근처 팽창 누락
        start_i = max(0, int(i - alpha_deg))
        end_i   = min(n - 1, int(i + alpha_deg))
        proc[start_i : end_i + 1] = 0.0
    return proc

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
    return gaps

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
    front_clear = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))
    if front_clear > FRONT_CLEAR_DIST:
        target, bias_label = gap_angle * 0.2, "STRAIGHT"
    else:
        target, bias_label = gap_angle * 1.0, "GAP"
    target     = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    return target, bias_label, front_clear

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
# AVOID DIRECTION  ★ 전방 ±60° 기준으로 수정
# =========================================

def choose_avoid_direction():
    # ★ 수정: 전체 좌/우 평균 대신 전방 ±60° 내 좌/우만 비교
    #    기존엔 1~89°, 271~359° 전체 평균이라 측면/후방 정보가 희석됨
    left_avg  = float(np.mean(scan_data[np.arange(1,  61) % 360]))
    right_avg = float(np.mean(scan_data[np.arange(300, 360) % 360]))
    if left_avg >= right_avg:
        print(f"  [AVOID DIR] LEFT  (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return 1
    else:
        print(f"  [AVOID DIR] RIGHT (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return -1

# =========================================
# MAIN LOOP
# =========================================

print("NAVIGATION START")

try:
    while True:

        # LiDAR 수신
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
            if ramp_exited():
                state          = STATE_NORMAL
                ramp_mode      = RAMP_MODE_NORMAL
                prev_front_avg = float(SCAN_LIMIT)
                print("  [RAMP→NORMAL] 경사 통과 완료")
                send_cmd(RAMP_SPEED, 0.0)
                continue

            emergency_min = float(np.min(scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE + 1) % 360]))
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
                    print(f"  [RAMP/GAP] target:{target_angle:5.1f}° v:{v:.2f} w:{w:.2f} pitch:{read_imu_pitch():.1f}° front:{front_min:.1f}cm")
                else:
                    send_cmd(RAMP_SPEED, 0.0)
                    print("  [RAMP] NO GAP → 서행 직진 유지")
            continue

        # ── STATE_NORMAL ──

        emergency_min = float(np.min(scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE + 1) % 360]))
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
            print(f"RAMP DETECTED! pitch:{read_imu_pitch():.1f}° front_avg:{front_avg:.1f}cm → STATE_RAMP")
            send_cmd(RAMP_SPEED, 0.0)
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result    = find_best_direction(smoothing)

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
            f"TARGET:{target_angle:6.1f}° | v:{v:.2f}m/s | w:{w:.2f}r/s | "
            f"front:{front_min:.1f}cm | clear:{front_clear:.1f}cm | "
            f"bias:{bias_label} | smooth:{smoothing:.2f}"
        )

        prev_front_avg = front_avg

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
