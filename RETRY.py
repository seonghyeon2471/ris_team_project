import serial
import math
import time
import numpy as np

# =========================================
# SERIAL
# =========================================

arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# IMU (MPU-6050 등) I2C — smbus2 사용
# 없으면 아래 두 줄 주석 처리 후 USE_IMU = False
USE_IMU = True
try:
    import smbus2
    imu_bus  = smbus2.SMBus(1)
    IMU_ADDR = 0x68
    imu_bus.write_byte_data(IMU_ADDR, 0x6B, 0)   # wake up
except Exception:
    USE_IMU = False
    print("[WARN] IMU 초기화 실패 → IMU 비활성화")

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
# ROBOT PHYSICAL PARAMETER  (단위: cm)
# =========================================

ROBOT_RADIUS = 11.0
WHEEL_BASE   = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================

MAX_SPEED = 0.20
MIN_SPEED = 0.07
MAX_W     = 1.5
TURN_GAIN = 1.8

SCAN_LIMIT   = 150
FRONT_RANGE  = 60

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
# GAP PARAMETER  (단위: cm)
# =========================================

SAFE_DIST          = 17
INFLATION_MAX_DIST = 25

FRONT_CLEAR_DIST  = 23
FRONT_CLEAR_RANGE = 15

# =========================================
# STATE MACHINE
# =========================================

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2
STATE_RAMP    = 3   # ★ 경사면 통과 상태 추가

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST   =  6
REVERSE_DURATION = 0.18
ROTATE_DURATION  = 1.00
REVERSE_SPEED    = -0.10
ROTATE_W         =  0.9

# =========================================
# ★ RAMP PARAMETER
# =========================================

# IMU 기반 경사 감지
RAMP_PITCH_THRESH  = 8.0    # 경사 진입 판단 피치각 (°) — 조정 가능
RAMP_EXIT_PITCH    = 3.0    # 경사 종료 판단 피치각 (°)

# IMU 없을 때: LiDAR 패턴으로 경사 추정
# 전방 ±10° 평균이 갑자기 LIDAR_DROP_THRESH 이상 감소하면 경사로 판단
LIDAR_DROP_THRESH  = 30.0   # cm — 한 스캔 사이 전방 거리 급증량
RAMP_LIDAR_TIMEOUT = 3.0    # 경사 상태 최대 유지 시간 (초)

# 경사 통과 중 주행 파라미터
RAMP_SPEED         = 0.14   # 경사 통과 선속도 (m/s) — 감속
RAMP_INFLATION_MAX = 10     # 경사 중 팽창 최대 거리 (cm) — 과팽창 억제
RAMP_SAFE_DIST     = 8      # 경사 중 Gap 최소 거리 (cm) — 기준 완화

# =========================================
# STATE
# =========================================

scan_data       = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle      = 0.0
prev_front_avg  = float(SCAN_LIMIT)   # LiDAR 경사 감지용 이전 전방 평균
ramp_start_time = 0.0                 # 경사 진입 시각

# =========================================
# UTIL
# =========================================

def normalize_angle(angle):
    return int(angle) % 360


# =========================================
# ★ IMU 읽기
# =========================================

def read_imu_pitch() -> float:
    """
    MPU-6050 가속도계로 피치각(°) 반환.
    IMU 없으면 0.0 반환.
    축 방향은 실제 장착 방향에 맞게 조정 필요.
    """
    if not USE_IMU:
        return 0.0
    try:
        def read_word(reg):
            h = imu_bus.read_byte_data(IMU_ADDR, reg)
            l = imu_bus.read_byte_data(IMU_ADDR, reg + 1)
            val = (h << 8) | l
            return val - 65536 if val >= 0x8000 else val

        ax = read_word(0x3B) / 16384.0   # ±2g 스케일
        ay = read_word(0x3D) / 16384.0
        az = read_word(0x3F) / 16384.0

        # 전후 피치각: atan2(ax, sqrt(ay²+az²))
        pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
        return abs(pitch)
    except Exception:
        return 0.0


# =========================================
# ★ 경사면 감지
# =========================================

def detect_ramp(front_avg: float) -> bool:
    """
    IMU 우선, 없으면 LiDAR 전방 거리 급증으로 판단.
    front_avg: 현재 전방 ±10° 평균 거리 (cm)
    """
    global prev_front_avg

    # 1) IMU 피치 기반
    pitch = read_imu_pitch()
    prev_front_avg = front_avg

    if USE_IMU:
        return pitch >= RAMP_PITCH_THRESH

    # 2) LiDAR 패턴 기반 (fallback)
    #    경사를 오를 때 LiDAR가 허공을 보게 되어 전방 거리가 급증
    drop = front_avg - prev_front_avg
    return drop >= LIDAR_DROP_THRESH


def ramp_exited() -> bool:
    """경사면 통과 완료 판단"""
    if USE_IMU:
        return read_imu_pitch() < RAMP_EXIT_PITCH
    # LiDAR fallback: 타임아웃으로만 판단
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
# inflation_max: 경사 상태에 따라 동적으로 전달받음
# =========================================

def inflate_obstacles(dists, inflation_max=None):
    if inflation_max is None:
        inflation_max = INFLATION_MAX_DIST

    proc = dists.copy()

    for i in range(len(dists)):
        d = dists[i]

        if d < 5 or d >= inflation_max:
            continue

        alpha = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))

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

    # ★ 개선된 점수 계산
    # - width 가중치 상향:      0.5 → 1.5  (좁은 Gap 선호 억제)
    # - avg_dist 가중치 상향:   1.2 → 2.0  (실제 열린 공간 우선)
    # - min_dist 추가:          × 1.0      (Gap 내 최소 거리 — 막힌 틈 억제)
    # - 정면 편향 패널티 완화:  0.4 → 0.15 (먼 방향도 공정하게 평가)
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
    """
    on_ramp=True 이면 팽창·Gap 기준을 완화하여
    경사면을 장애물로 오인하지 않도록 처리
    """
    global prev_angle

    angles     = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists      = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)

    # ★ 경사 중: 팽창 최대 거리·Safe 거리 완화
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
        # 전방 열림 → 직진 편향 (Gap 방향 20%만 반영)
        target     = gap_angle * 0.2
        bias_label = "STRAIGHT"
    else:
        # 전방 막힘 → Gap 방향 100% 추종
        # 정면+오른쪽 막혀있을 때 왼쪽 Gap을 확실히 추종
        target     = gap_angle * 1.0
        bias_label = "GAP"

    target     = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target

    return target, bias_label, front_clear


# =========================================
# CONTROL
# =========================================

ALIGN_THRESHOLD = 15   # ★ 10 → 15° (Gap 방향 선회 중 직진 빠르게 재개)

def compute_cmd(target_angle, on_ramp=False):
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    if abs(target_angle) > ALIGN_THRESHOLD:
        return 0.0, w

    front_min      = float(np.min(scan_data[np.arange(-10, 11) % 360]))

    # ★ 경사 중: 속도 고정 (obstacle_scale 미적용)
    if on_ramp:
        return RAMP_SPEED, w

    obstacle_scale = min(front_min / 40.0, 1.0)
    speed          = max(MAX_SPEED * obstacle_scale, MIN_SPEED)

    return speed, w


def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())


def stop_robot():
    send_cmd(0.0, 0.0)


# =========================================
# AVOID DIRECTION
# =========================================

def choose_avoid_direction():
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))

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

        if s_flag != 1:
            continue

        apply_median_filter()

        now = time.time()

        # 현재 전방 평균 (경사 감지용)
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

        # ── ★ STATE_RAMP : 경사면 통과 ──
        if state == STATE_RAMP:
            if ramp_exited():
                state = STATE_NORMAL
                print("  [RAMP→NORMAL] 경사 통과 완료")
            else:
                # 경사 중: 완화된 파라미터로 직진 유지
                result = find_best_direction(SMOOTHING_NORMAL, on_ramp=True)
                if result is not None:
                    target_angle, bias_label, front_clear = result
                    v, w = compute_cmd(target_angle, on_ramp=True)
                    send_cmd(v, w)
                    pitch = read_imu_pitch()
                    print(
                        f"  [RAMP] target:{target_angle:5.1f}° "
                        f"v:{v:.2f} w:{w:.2f} "
                        f"pitch:{pitch:.1f}° "
                        f"front:{front_min:.1f}cm"
                    )
                else:
                    # 경사 중에도 완전히 막히면 서행 직진 유지
                    send_cmd(RAMP_SPEED, 0.0)
                    print("  [RAMP] NO GAP → 서행 직진 유지")
            continue

        # ── STATE_NORMAL ──

        # 긴급회피 진입
        if front_min < EMERGENCY_DIST:
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            print(f"EMERGENCY! front:{front_min:.1f}cm → REVERSE")
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        # ★ 경사 감지 → STATE_RAMP 진입
        if detect_ramp(front_avg):
            state           = STATE_RAMP
            ramp_start_time = now
            prev_front_avg  = front_avg
            pitch           = read_imu_pitch()
            print(f"RAMP DETECTED! pitch:{pitch:.1f}° front_avg:{front_avg:.1f}cm → STATE_RAMP")
            continue

        # 위험 거리 기반 스무딩 전환
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

        prev_front_avg = front_avg   # LiDAR fallback용 갱신

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
