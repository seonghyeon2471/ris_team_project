"""
navigation_final.py
====================
좁은 코스(1.1m × 3.1m) 장애물 회피 자율주행 코드
버그 수정 목록:
  [FIX-1] global ramp_mode → 메인루프 전역변수로 정상 처리
  [FIX-2] ROTATE 완료 후 스캔 초기화 제거 → ramp 오탐 방지
  [FIX-3] rotate_dir 반전 제거 → 회피 방향 역전 버그 수정
  [FIX-4] compute_wall_follow_cmd() 좌우 벽 기준 통일
  [FIX-5] is_gap_passable() 물리 폭 검증 복원
  [FIX-6] EMERGENCY_RANGE 5° → 30° (측면 충돌 방지)
  [FIX-7] drain_lidar_buffer() 추가 → 버퍼 밀림 방지
  [FIX-8] STATE_CREEP 추가 → 갭 없을 때 저속 전진 후 재탐색
  [FIX-9] prev_front_avg ramp 오탐 방지 리셋 추가
"""

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
# ROBOT PHYSICAL PARAMETERS  (단위: cm)
# =========================================

ROBOT_RADIUS = 14.0
WHEEL_BASE   = 17.0
ROBOT_WIDTH  = 20.0   # 차폭
PASS_MARGIN  =  5.0   # 통과 여유 마진 → 실제 필요 폭 = ROBOT_WIDTH + PASS_MARGIN = 25cm

# =========================================
# DRIVE PARAMETERS
# =========================================

MAX_SPEED = 0.15   # 좁은 코스 대응 (0.20 → 0.15)
MIN_SPEED = 0.07
MAX_W     = 1.8
TURN_GAIN = 1.2

# =========================================
# SCAN PARAMETERS
# =========================================

SCAN_LIMIT  = 120   # 코스 최대폭 110cm 반영 (150 → 120)
FRONT_RANGE =  40   # 탐색각 좁힘 → 벽 혼입 방지 (60 → 40)

# =========================================
# FILTER PARAMETERS
# =========================================

EMA_ALPHA = 0.5
MEDIAN_K  = 2

# =========================================
# SMOOTHING PARAMETERS
# =========================================

SMOOTHING_NORMAL = 0.70
SMOOTHING_DANGER = 0.25
DANGER_DIST      = 30

# =========================================
# GAP PARAMETERS  (단위: cm)
# =========================================

SAFE_DIST          = 20   # 보수적 (17 → 20)
INFLATION_MAX_DIST = 60   # 과팽창 방지 (100 → 60)
FRONT_CLEAR_DIST   = 25
FRONT_CLEAR_RANGE  = 15

# =========================================
# CONTROL PARAMETERS
# =========================================

ALIGN_THRESHOLD = 25
DEADBAND_ANGLE  =  5

# =========================================
# STATE MACHINE
# =========================================

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2
STATE_RAMP    = 3
STATE_CREEP   = 4   # [FIX-8] 갭 없을 때 저속 전진 후 재탐색

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1
creep_end_time    = 0.0

# =========================================
# EMERGENCY / MANEUVER PARAMETERS
# =========================================

EMERGENCY_DIST   = 10     # 여유 확보 (8 → 10)
EMERGENCY_RANGE  = 30     # [FIX-6] 5° → 30° (측면 충돌 방지)
REVERSE_DURATION = 0.35   # 후진 시간 (0.25 → 0.35)
ROTATE_DURATION  = 0.50   # [FIX-2] 좁은 공간 과회전 방지 (1.00 → 0.50)
REVERSE_SPEED    = -0.10
ROTATE_W         =  0.70  # 회전 속도 감소 (0.9 → 0.7)

# [FIX-8] 크리프 파라미터
NO_GAP_CREEP_SPEED = 0.07
NO_GAP_CREEP_TIME  = 0.30

# =========================================
# RAMP PARAMETERS
# =========================================

RAMP_PITCH_THRESH  =  8.0
RAMP_EXIT_PITCH    =  3.0
LIDAR_DROP_THRESH  = 30.0
RAMP_LIDAR_TIMEOUT =  3.0
RAMP_SPEED         =  0.12
RAMP_INFLATION_MAX = 10
RAMP_SAFE_DIST     =  8
RAMP_NOGAP_TIMEOUT =  1.5   # 경사로 중 갭 없을 때 서행 직진 최대 허용 시간 (초)

# 측면 벽 추종 파라미터
WALL_FOLLOW_TARGET = 16    # 목표 측면 거리 (cm)
WALL_FOLLOW_KP     =  0.3
WALL_FOLLOW_MAX_W  =  0.8
WALL_RAMP_DIST     = 23    # 측면 경사 감지 임계 거리 (cm)

RAMP_MODE_NORMAL     = 0
RAMP_MODE_WALL_LEFT  = 1   # 왼쪽에 벽 → 왼쪽 벽 기준 추종
RAMP_MODE_WALL_RIGHT = 2   # 오른쪽에 벽 → 오른쪽 벽 기준 추종

# =========================================
# GLOBAL STATE
# =========================================

scan_data      = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle     = 0.0
prev_front_avg = float(SCAN_LIMIT)
ramp_start_time    = 0.0
ramp_mode          = RAMP_MODE_NORMAL   # [FIX-1] 전역변수로 정상 선언
ramp_nogap_start   = 0.0               # 경사로 갭 없음 서행 시작 시간

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
# RAMP DETECTION
# =========================================

def detect_ramp(front_avg: float) -> bool:
    global prev_front_avg
    if USE_IMU:
        prev_front_avg = front_avg
        return read_imu_pitch() >= RAMP_PITCH_THRESH
    # LiDAR fallback: 비교 먼저 → 갱신 (순서 버그 수정됨)
    drop = front_avg - prev_front_avg
    prev_front_avg = front_avg
    return drop >= LIDAR_DROP_THRESH

def ramp_exited() -> bool:
    if USE_IMU:
        return read_imu_pitch() < RAMP_EXIT_PITCH
    return (time.time() - ramp_start_time) > RAMP_LIDAR_TIMEOUT

# =========================================
# SIDE RAMP DETECTION & WALL FOLLOW
# =========================================

def detect_side_ramp() -> int:
    """
    좌(85~95°) / 우(265~275°) 평균 거리로 측면 경사 벽 감지.
    반환: RAMP_MODE_WALL_LEFT  → 왼쪽 벽 가까움
          RAMP_MODE_WALL_RIGHT → 오른쪽 벽 가까움
          RAMP_MODE_NORMAL     → 벽 없음
    """
    left_dist  = float(np.mean(scan_data[85:96]))
    right_dist = float(np.mean(scan_data[265:276]))

    if left_dist < WALL_RAMP_DIST and right_dist >= WALL_RAMP_DIST:
        return RAMP_MODE_WALL_LEFT
    if right_dist < WALL_RAMP_DIST and left_dist >= WALL_RAMP_DIST:
        return RAMP_MODE_WALL_RIGHT
    return RAMP_MODE_NORMAL

def compute_wall_follow_cmd(mode: int):
    """
    [FIX-4] 좌우 벽 기준 통일:
      WALL_LEFT  → 왼쪽 벽(90°)이 가까우므로 왼쪽 벽 거리 유지
      WALL_RIGHT → 오른쪽 벽(270°)이 가까우므로 오른쪽 벽 거리 유지
    error > 0 (너무 가까움) → 반대쪽으로 조향
    """
    if mode == RAMP_MODE_WALL_LEFT:
        wall_dist = float(np.mean(scan_data[85:96]))        # 왼쪽 벽
        error = WALL_FOLLOW_TARGET - wall_dist
        # 왼쪽이 가까우면 오른쪽으로(w 음수), 멀면 왼쪽으로(w 양수)
        w = float(np.clip(error * WALL_FOLLOW_KP, -WALL_FOLLOW_MAX_W, WALL_FOLLOW_MAX_W))
        label = "WALL_LEFT"
    else:  # RAMP_MODE_WALL_RIGHT
        wall_dist = float(np.mean(scan_data[265:276]))      # 오른쪽 벽
        error = WALL_FOLLOW_TARGET - wall_dist
        # 오른쪽이 가까우면 왼쪽으로(w 양수), 멀면 오른쪽으로(w 음수)
        w = float(np.clip(-error * WALL_FOLLOW_KP, -WALL_FOLLOW_MAX_W, WALL_FOLLOW_MAX_W))
        label = "WALL_RIGHT"

    return RAMP_SPEED, w, wall_dist, label

# =========================================
# LIDAR BUFFER DRAIN  [FIX-7]
# =========================================

def drain_lidar_buffer():
    """
    제어 루프 지연 방지: 버퍼에 쌓인 패킷 최대 30개 소진 후 scan_data 갱신.
    좁은 코스 고속 회전 시 반응 지연 방지에 필수.
    """
    MAX_DRAIN = 30
    count = 0
    while count < MAX_DRAIN and lidar_ser.in_waiting >= 5:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            break
        s_flag     = raw[0] & 0x01
        s_inv_flag = (raw[0] & 0x02) >> 1
        if s_inv_flag != (1 - s_flag):
            count += 1; continue
        if (raw[1] & 0x01) != 1:
            count += 1; continue
        if (raw[0] >> 2) < 3:
            count += 1; continue
        angle_raw = (raw[1] >> 1) | (raw[2] << 7)
        angle     = int(angle_raw / 64.0) % 360
        dist_cm   = (raw[3] | (raw[4] << 8)) / 4.0 / 10.0
        if 3 <= dist_cm <= SCAN_LIMIT:
            apply_ema(angle, dist_cm)
        count += 1

# =========================================
# FILTER
# =========================================

def apply_ema(angle: int, new_dist_cm: float):
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

def inflate_obstacles(dists: np.ndarray, inflation_max: float = None) -> np.ndarray:
    if inflation_max is None:
        inflation_max = INFLATION_MAX_DIST
    proc = dists.copy()
    n = len(dists)
    for i in range(n):
        d = dists[i]
        if d < 5 or d >= inflation_max:
            continue
        ratio = ROBOT_RADIUS / d
        alpha_deg = 45.0 if ratio >= 1.0 else math.degrees(math.asin(ratio))
        start_i = max(0, int(i - alpha_deg))
        end_i   = min(n - 1, int(i + alpha_deg))
        proc[start_i : end_i + 1] = 0.0
    return proc

# =========================================
# GAP PASSABILITY CHECK  [FIX-5] 복원
# =========================================

def is_gap_passable(gap: tuple, proc_dists: np.ndarray, angles: np.ndarray):
    """
    갭 양 끝점의 물리적 직선거리로 실제 통과 폭 계산.
    반환: (True, width) 또는 (False, width)
    """
    start_i, end_i = gap
    if start_i == end_i:
        return False, 0.0

    d_s = float(proc_dists[start_i])
    d_e = float(proc_dists[end_i])
    if d_s <= 0 or d_e <= 0:
        return False, 0.0

    a_s = math.radians(float(angles[start_i]))
    a_e = math.radians(float(angles[end_i]))

    x1, y1 = d_s * math.sin(a_s), d_s * math.cos(a_s)
    x2, y2 = d_e * math.sin(a_e), d_e * math.cos(a_e)

    width = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    required = ROBOT_WIDTH + PASS_MARGIN
    return width >= required, width

# =========================================
# GAP SEARCH
# =========================================

def find_gaps(proc_dists: np.ndarray, angles: np.ndarray, safe_dist: float = None):
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

    # [FIX-5] 물리 폭 검증: 통과 불가 갭 제거
    passable = []
    for g in gaps:
        ok, width = is_gap_passable(g, proc_dists, angles)
        if ok:
            passable.append(g)
        else:
            print(f"  [GAP FILTER] 폭 {width:.1f}cm < {ROBOT_WIDTH+PASS_MARGIN:.0f}cm → 제거")
    return passable

def score_gap(gap: tuple, proc_dists: np.ndarray, angles: np.ndarray) -> float:
    start, end   = gap
    center_angle = angles[int((start + end) / 2)]
    avg_dist     = float(np.mean(proc_dists[start : end + 1]))
    min_dist     = float(np.min(proc_dists[start : end + 1]))
    return (end - start) * 1.5 + avg_dist * 2.0 + min_dist * 1.0 - abs(center_angle) * 0.15

def select_best_gap(gaps, proc_dists, angles):
    return max(gaps, key=lambda g: score_gap(g, proc_dists, angles))

# =========================================
# PLANNING
# =========================================

def find_best_direction(smoothing: float, on_ramp: bool = False):
    global prev_angle

    angles     = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists      = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    infl_max   = RAMP_INFLATION_MAX if on_ramp else INFLATION_MAX_DIST
    safe_dist  = RAMP_SAFE_DIST     if on_ramp else SAFE_DIST

    proc_dists = inflate_obstacles(dists, inflation_max=infl_max)
    gaps       = find_gaps(proc_dists, angles, safe_dist=safe_dist)

    if not gaps:
        return None

    best_gap             = select_best_gap(gaps, proc_dists, angles)
    gap_angle            = float(angles[int((best_gap[0] + best_gap[1]) / 2)])
    _, gap_width         = is_gap_passable(best_gap, proc_dists, angles)
    front_clear          = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))

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

def compute_cmd(target_angle: float, on_ramp: bool = False):
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

def send_cmd(v: float, w: float):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# AVOID DIRECTION
# =========================================

def choose_avoid_direction() -> int:
    """실시간 스캔 기반으로 더 열린 쪽 선택. 매번 독립 판단."""
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

        # [FIX-7] 버퍼 선 소진 → 제어 주기 지연 방지
        drain_lidar_buffer()

        # LiDAR 패킷 수신
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

        now       = time.time()
        front_avg = float(np.mean(scan_data[np.arange(-10, 11) % 360]))
        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))

        # ══════════════════════════════════
        # STATE_REVERSE
        # ══════════════════════════════════
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
                print(f"  [REVERSE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                state             = STATE_ROTATE
                maneuver_end_time = now + ROTATE_DURATION
                print(f"  [ROTATE START] dir:{'+' if rotate_dir > 0 else '-'}")
            continue

        # ══════════════════════════════════
        # STATE_ROTATE
        # ══════════════════════════════════
        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
                print(f"  [ROTATE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                # [FIX-2] 스캔 초기화 제거 → ramp 오탐 방지
                # [FIX-3] rotate_dir 반전 제거 → 다음 회피도 실시간 판단
                state          = STATE_NORMAL
                prev_angle     = 0.0
                prev_front_avg = front_avg   # [FIX-9] 현재 실측값 기준 리셋
                print("  [NORMAL] maneuver done")
            continue

        # ══════════════════════════════════
        # STATE_CREEP  [FIX-8]
        # 갭 없을 때 즉시 REVERSE 대신 저속 전진 → 시야 확보 후 재탐색
        # ══════════════════════════════════
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
                    rotate_dir        = choose_avoid_direction()
                    state             = STATE_REVERSE
                    maneuver_end_time = now + REVERSE_DURATION
                    print("  [CREEP→REVERSE] 갭 없음 → REVERSE")
                    send_cmd(REVERSE_SPEED, 0.0)
            continue

        # ══════════════════════════════════
        # STATE_RAMP
        # ══════════════════════════════════
        if state == STATE_RAMP:
            if ramp_exited():
                state          = STATE_NORMAL
                ramp_mode      = RAMP_MODE_NORMAL
                prev_front_avg = float(SCAN_LIMIT)
                print("  [RAMP→NORMAL] 경사 통과 완료")
                send_cmd(RAMP_SPEED, 0.0)
                continue

            # 경사 중에도 정면 긴급회피 유지
            emergency_min = float(np.min(scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE + 1) % 360]))
            if emergency_min < EMERGENCY_DIST:
                rotate_dir        = choose_avoid_direction()
                state             = STATE_REVERSE
                ramp_mode         = RAMP_MODE_NORMAL
                maneuver_end_time = now + REVERSE_DURATION
                print(f"  [RAMP] EMERGENCY! dist:{emergency_min:.1f}cm → REVERSE")
                send_cmd(REVERSE_SPEED, 0.0)
                continue

            # 측면 경사 감지: 감지될 때만 갱신, NORMAL이면 명시적 리셋
            detected_mode = detect_side_ramp()
            ramp_mode = detected_mode   # NORMAL 포함 항상 갱신 (고착 방지)

            if ramp_mode != RAMP_MODE_NORMAL:
                v, w, wall_dist, label = compute_wall_follow_cmd(ramp_mode)
                send_cmd(v, w)
                print(f"  [RAMP/{label}] wall:{wall_dist:.1f}cm v:{v:.2f} w:{w:.2f}")
            else:
                result = find_best_direction(SMOOTHING_NORMAL, on_ramp=True)
                if result is not None:
                    ramp_nogap_start = 0.0   # 갭 발견 시 타이머 리셋
                    target_angle, bias_label, front_clear, gap_width = result
                    v, w = compute_cmd(target_angle, on_ramp=True)
                    send_cmd(v, w)
                    print(
                        f"  [RAMP/GAP] target:{target_angle:5.1f}° "
                        f"v:{v:.2f} w:{w:.2f} "
                        f"pitch:{read_imu_pitch():.1f}° "
                        f"front:{front_min:.1f}cm gap_w:{gap_width:.1f}cm"
                    )
                else:
                    # 갭 없음: 최초 진입 시 타이머 시작
                    if ramp_nogap_start == 0.0:
                        ramp_nogap_start = now
                    elapsed = now - ramp_nogap_start
                    if elapsed < RAMP_NOGAP_TIMEOUT:
                        # 타임아웃 전: 서행 직진 유지
                        send_cmd(RAMP_SPEED, 0.0)
                        print(f"  [RAMP] NO GAP → 서행 직진 ({elapsed:.1f}s / {RAMP_NOGAP_TIMEOUT}s)")
                    else:
                        # 타임아웃: 경사로 강제 종료 후 REVERSE
                        ramp_nogap_start = 0.0
                        state            = STATE_NORMAL
                        ramp_mode        = RAMP_MODE_NORMAL
                        rotate_dir       = choose_avoid_direction()
                        state            = STATE_REVERSE
                        maneuver_end_time = now + REVERSE_DURATION
                        print(f"  [RAMP] NO GAP 타임아웃 {RAMP_NOGAP_TIMEOUT}s 초과 → REVERSE")
                        send_cmd(REVERSE_SPEED, 0.0)
            continue

        # ══════════════════════════════════
        # STATE_NORMAL
        # ══════════════════════════════════

        # [FIX-6] EMERGENCY_RANGE = 30° → 측면 충돌 방지
        emergency_min = float(np.min(scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE + 1) % 360]))
        if emergency_min < EMERGENCY_DIST:
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            print(f"EMERGENCY! dist:{emergency_min:.1f}cm → REVERSE")
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
            if front_min > SAFE_DIST * 2:
                # 장애물 멀다(40cm↑) → 저속 직진하며 접근, 갭 재탐색
                send_cmd(MIN_SPEED, 0.0)
                print(f"NO GAP BUT FAR ({front_min:.1f}cm) → 저속 직진 접근")
            elif front_min > SAFE_DIST:
                # 중간 거리(20~40cm) → CREEP으로 시야 확보 후 재탐색
                state          = STATE_CREEP
                creep_end_time = now + NO_GAP_CREEP_TIME
                print(f"NO GAP MEDIUM ({front_min:.1f}cm) → CREEP 시도")
                send_cmd(NO_GAP_CREEP_SPEED, 0.0)
            else:
                # 장애물 가깝다(20cm↓) → REVERSE
                rotate_dir        = choose_avoid_direction()
                state             = STATE_REVERSE
                maneuver_end_time = now + REVERSE_DURATION
                print(f"NO GAP CLOSE ({front_min:.1f}cm) → REVERSE")
                send_cmd(REVERSE_SPEED, 0.0)
            continue

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
