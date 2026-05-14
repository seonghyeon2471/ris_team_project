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
# LIDAR START (RPLIDAR C1)
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

ROBOT_LENGTH   = 22.0    # 로봇 길이 (cm)
ROBOT_WIDTH    = 20.0    # 로봇 너비 (cm)
LIDAR_OFFSET   = 2.5     # 라이다 → 로봇 앞단 오프셋 (cm, 앞에서 뒤로)
                         # 라이다가 앞에서 2.5cm 뒤에 위치
                         # 로봇 중심 기준: ROBOT_LENGTH/2 - LIDAR_OFFSET = 8.5cm 앞에 위치
ROBOT_HALF_W   = ROBOT_WIDTH  / 2.0   # 10.0 cm
ROBOT_HALF_L   = ROBOT_LENGTH / 2.0   # 11.0 cm

# 안전 마진 (로봇 폭 절반 + 여유)
SAFETY_MARGIN  = ROBOT_HALF_W + 3.0   # 13.0 cm

# =========================================
# SCAN PARAMETER
# =========================================

SCAN_LIMIT     = 200     # 2미터 (cm)
FRONT_RANGE    = 60      # ±60° = 전방 120도

# =========================================
# DRIVE PARAMETER
# =========================================

# 속도 15 PWM → v = 15 / PWM_SCALE(900) ≈ 0.0167 m/s
# 좀 더 여유있게 0.018로 설정
BASE_SPEED     = 0.018   # 정상 주행 선속도 (m/s)
SLOW_SPEED     = 0.012   # 위험 구간 감속
MAX_W          = 1.5     # 최대 각속도 (rad/s)

# =========================================
# GRP PARAMETER (논문 공식)
# =========================================
# φ_s = (α/d_min × φ_gap + β × φ_ref) / (α/d_min + β)
#
# α 크면 → 장애물 근접 시 gap 방향 비중 ↑ (안전)
# β 크면 → reference(직진) 방향 비중 ↑ (효율)

GRP_ALPHA      = 40.0    # 논문 권장값
GRP_BETA       = 1.0     # 논문 권장값

# =========================================
# WALL / GAP PARAMETER
# =========================================

WALL_DETECT_DIST    = 80.0   # 이 거리 이하면 벽으로 간주 (cm)
GAP_THRESHOLD       = SCAN_LIMIT  # threshold: scan_limit 이상이면 gap

# 두 갈래 판단: 두 gap의 너비 차이가 이 비율 이상이면 확실히 넓은 쪽 선택
TWO_PATH_WIDTH_MIN  = 5      # gap이 최소 이 각도 이상이어야 유효 gap으로 인정 (°)

DANGER_DIST         = 25.0   # 긴급 회피 기준 (cm)
EMERGENCY_DIST      = 8.0    # 즉시 정지/후진 (cm)

# =========================================
# SMOOTHING
# =========================================

EMA_ALPHA      = 0.3
SMOOTH_NORMAL  = 0.50
SMOOTH_DANGER  = 0.20

# =========================================
# STATE MACHINE
# =========================================

STATE_NORMAL   = 0
STATE_REVERSE  = 1
STATE_ROTATE   = 2

state              = STATE_NORMAL
maneuver_end_time  = 0.0
rotate_dir         = 1

REVERSE_DURATION   = 0.18
ROTATE_DURATION    = 0.90
REVERSE_SPEED      = -0.012
ROTATE_W           = 0.9

# =========================================
# GLOBAL STATE
# =========================================

scan_data   = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_w      = 0.0

# =========================================
# FILTER
# =========================================

def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (
        (1.0 - EMA_ALPHA) * scan_data[angle]
        + EMA_ALPHA * new_dist_cm
    )


def apply_median_filter():
    k = 2
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        indices = [(i + d) % 360 for d in range(-k, k + 1)]
        values  = np.sort(scan_data[indices])
        filtered[i] = values[k]   # 중앙값
    scan_data[:] = filtered


# =========================================
# 직사각형 로봇 기반 obstacle inflation
# 라이다가 앞에서 2.5cm 뒤에 있으므로
# 각도별로 로봇 경계까지 거리를 계산해 팽창
# =========================================

def get_robot_half_width_at_angle(angle_deg):
    """
    각도 angle_deg 방향으로 라이다 기준
    로봇 경계까지의 거리(cm)를 반환.
    직사각형 로봇: 앞뒤 방향 half_length, 좌우 half_width
    라이다는 로봇 중심에서 (ROBOT_HALF_L - LIDAR_OFFSET) 앞에 위치.
    """
    rad = math.radians(angle_deg)
    cos_a = abs(math.cos(rad))
    sin_a = abs(math.sin(rad))

    # 직사각형 경계까지 거리 (라이다 중심 기준)
    # 전방 방향(0°): ROBOT_HALF_L - LIDAR_OFFSET = 8.5cm
    # 측면 방향(90°): ROBOT_HALF_W = 10.0cm
    lidar_to_front = ROBOT_HALF_L - LIDAR_OFFSET   # 8.5 cm
    lidar_to_rear  = ROBOT_HALF_L + LIDAR_OFFSET   # 13.5 cm (사용 안함)
    lidar_to_side  = ROBOT_HALF_W                  # 10.0 cm

    if cos_a < 1e-6:
        return lidar_to_side + SAFETY_MARGIN - ROBOT_HALF_W
    if sin_a < 1e-6:
        return lidar_to_front + 3.0  # 앞 여유

    # 직사각형 경계 교점
    t_front = lidar_to_front / cos_a
    t_side  = lidar_to_side  / sin_a
    boundary = min(t_front, t_side)

    return boundary + 3.0   # 안전 여유 3cm


def inflate_obstacles(dists, angles):
    """
    직사각형 로봇 기반 obstacle inflation.
    각 장애물 점에서 로봇 폭에 해당하는 각도만큼 인접 각도를 0으로 마킹.
    """
    proc = dists.copy()
    n = len(dists)

    for i in range(n):
        d = dists[i]
        if d < 3 or d >= SCAN_LIMIT - 1:
            continue

        # 이 거리에서 로봇 반폭이 차지하는 각도
        half_w = SAFETY_MARGIN
        if d > 0:
            alpha_deg = math.degrees(math.asin(min(half_w / d, 1.0)))
        else:
            alpha_deg = 45.0

        span = max(1, int(alpha_deg))
        start = max(0, i - span)
        end   = min(n - 1, i + span)
        proc[start:end+1] = np.minimum(proc[start:end+1], d)
        # 장애물로 마킹 (0으로 완전히 막지 않고 실제 거리로 마킹해 gap 탐색에 활용)
        proc[start:end+1] = 0.0

    return proc


# =========================================
# GAP FINDING (FGM 방식)
# threshold보다 멀면 gap, 가까우면 장애물
# =========================================

def find_gaps(proc_dists, safe_dist):
    """
    연속된 열린 공간(gap) 구간 리스트 반환.
    반환: [(start_idx, end_idx), ...]
    """
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

    # 너무 좁은 gap 제거
    gaps = [g for g in gaps if (g[1] - g[0]) >= TWO_PATH_WIDTH_MIN]
    return gaps


# =========================================
# TWO-PATH vs ONE-PATH 판단
# =========================================

def classify_paths(gaps, proc_dists, angles):
    """
    gap이 2개 이상이면 두 갈래, 1개면 한 갈래.
    두 갈래: 더 넓은 gap 선택
    한 갈래: gap 정중앙 각도 반환
    반환: (target_angle, mode_str)
    """
    if len(gaps) == 0:
        return None, "NO_GAP"

    if len(gaps) == 1:
        # 한 갈래: 정중앙
        s, e       = gaps[0]
        center_i   = (s + e) / 2.0
        center_ang = float(angles[int(center_i)])
        width      = e - s
        return center_ang, f"ONE_PATH(width={width}°)"

    # 두 갈래 이상: 가장 넓은 gap 선택
    best_gap   = max(gaps, key=lambda g: g[1] - g[0])
    s, e       = best_gap
    center_i   = (s + e) / 2.0
    center_ang = float(angles[int(center_i)])
    width      = e - s

    widths = [g[1]-g[0] for g in gaps]
    return center_ang, f"TWO_PATH(widest={width}° of {widths})"


# =========================================
# GRP 조향각 공식 (논문)
# φ_s = (α/d_min × φ_gap + β × φ_ref) / (α/d_min + β)
# φ_ref = 0 (정면 직진 기준, reference path 없으므로)
# =========================================

def grp_steering(phi_gap, d_min):
    """
    GRP 논문 공식 적용.
    phi_ref = 0 (직진 기준)
    d_min 이 작을수록 gap 방향 비중 증가 (안전)
    d_min 이 클수록 직진 비중 증가 (효율)
    """
    phi_ref = 0.0   # 직진 reference

    if d_min < 1.0:
        d_min = 1.0

    weight_gap = GRP_ALPHA / d_min
    weight_ref = GRP_BETA

    phi_s = (weight_gap * phi_gap + weight_ref * phi_ref) / (weight_gap + weight_ref)
    return phi_s


# =========================================
# MAIN PLANNING
# 1. 전방 120도 스캔 추출
# 2. inflation
# 3. gap 찾기
# 4. 두 갈래 / 한 갈래 판단
# 5. GRP 조향각 계산
# =========================================

def plan(smoothing):
    global prev_w

    # 전방 ±60° 인덱스 및 거리 추출
    angles   = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)   # -60 ~ +60
    dists    = np.array([scan_data[int(a) % 360] for a in angles], dtype=np.float32)

    # obstacle inflation
    proc     = inflate_obstacles(dists, angles)

    # 전방 최소 거리
    front_min = float(np.min(dists[FRONT_RANGE-10 : FRONT_RANGE+11]))

    # safe_dist: 벽 감지 기준
    safe_dist = WALL_DETECT_DIST

    # gap 탐색
    gaps = find_gaps(proc, safe_dist)

    # 두 갈래 / 한 갈래 판단 및 목표각 결정
    phi_gap, mode = classify_paths(gaps, proc, angles)

    if phi_gap is None:
        return None, front_min, "NO_GAP"

    # d_min: 전방 ±30° 내 최소 거리 (GRP 공식용)
    d_min = float(np.min(dists[FRONT_RANGE-30 : FRONT_RANGE+31]))

    # GRP 조향각 계산
    phi_s = grp_steering(phi_gap, d_min)

    # 스무딩
    phi_s = prev_w * smoothing + phi_s * (1.0 - smoothing)
    prev_w = phi_s

    return phi_s, front_min, mode


# =========================================
# CONTROL: 각도 → v, w
# =========================================

def compute_cmd(phi_s, front_min):
    w = math.radians(phi_s) * 1.8
    w = float(np.clip(w, -MAX_W, MAX_W))

    # 전방이 막혀있으면 감속
    if front_min < DANGER_DIST:
        v = SLOW_SPEED
    else:
        v = BASE_SPEED

    # 크게 꺾어야 할 때는 직진 속도 줄임
    if abs(phi_s) > 20:
        v = SLOW_SPEED

    return v, w


def send_cmd(v, w):
    arduino_ser.write(f"{v:.4f},{-w:.4f}\n".encode())


def stop_robot():
    send_cmd(0.0, 0.0)


# =========================================
# AVOID DIRECTION
# =========================================

def choose_avoid_direction():
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    if left_avg >= right_avg:
        print(f"  [AVOID] LEFT  (L:{left_avg:.1f} R:{right_avg:.1f})")
        return 1
    else:
        print(f"  [AVOID] RIGHT (L:{left_avg:.1f} R:{right_avg:.1f})")
        return -1


# =========================================
# MAIN LOOP
# =========================================

print("NAVIGATION START")
print(f"  Robot: {ROBOT_LENGTH}x{ROBOT_WIDTH}cm | LiDAR offset: {LIDAR_OFFSET}cm")
print(f"  Scan: ±{FRONT_RANGE}° / {SCAN_LIMIT}cm | Speed PWM≈15 ({BASE_SPEED} m/s)")
print(f"  GRP α={GRP_ALPHA} β={GRP_BETA}")

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

        dist_raw  = raw[3] | (raw[4] << 8)
        dist_cm   = (dist_raw / 4.0) / 10.0

        # 전방 120도 외 범위는 무시
        # 0°=정면, 1~60°=우, 300~359°=좌 (RPLIDAR 기준)
        in_front = (angle <= FRONT_RANGE) or (angle >= 360 - FRONT_RANGE)
        if not in_front:
            continue

        if dist_cm < 3 or dist_cm > SCAN_LIMIT:
            continue

        apply_ema(angle, dist_cm)

        # 새 스캔 시작 시 처리
        if s_flag != 1:
            continue

        apply_median_filter()

        now = time.time()

        front_min = float(np.min(
            scan_data[np.arange(-10, 11) % 360]
        ))

        # =================================
        # STATE MACHINE
        # =================================

        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
                print(f"  [REVERSE] {maneuver_end_time - now:.2f}s")
            else:
                globals()['state']             = STATE_ROTATE
                globals()['maneuver_end_time'] = now + ROTATE_DURATION
                print(f"  [ROTATE START] dir:{'+' if rotate_dir > 0 else '-'}")
            continue

        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
                print(f"  [ROTATE] {maneuver_end_time - now:.2f}s")
            else:
                globals()['state']      = STATE_NORMAL
                globals()['prev_w']     = 0.0
                globals()['rotate_dir'] *= -1
                for a in range(-45, 46):
                    scan_data[a % 360] = float(SCAN_LIMIT)
                print("  [NORMAL] maneuver done")
            continue

        # STATE_NORMAL

        # 긴급 회피
        if front_min < EMERGENCY_DIST:
            rd = choose_avoid_direction()
            globals()['rotate_dir']        = rd
            globals()['state']             = STATE_REVERSE
            globals()['maneuver_end_time'] = now + REVERSE_DURATION
            print(f"EMERGENCY! {front_min:.1f}cm → REVERSE")
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        # 스무딩 선택
        smoothing = SMOOTH_DANGER if front_min < DANGER_DIST else SMOOTH_NORMAL

        # 플래닝
        phi_s, front_min, mode = plan(smoothing)

        if phi_s is None:
            rd = choose_avoid_direction()
            globals()['rotate_dir']        = rd
            globals()['state']             = STATE_REVERSE
            globals()['maneuver_end_time'] = now + REVERSE_DURATION
            print(f"NO GAP → REVERSE")
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        v, w = compute_cmd(phi_s, front_min)
        send_cmd(v, w)

        print(
            f"φ:{phi_s:6.1f}° | v:{v:.4f} w:{w:.3f} | "
            f"front:{front_min:.1f}cm | {mode} | smooth:{smoothing:.2f}"
        )

except KeyboardInterrupt:
    print("\nSTOP")

finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))   # Stop scan
    print("DONE")
