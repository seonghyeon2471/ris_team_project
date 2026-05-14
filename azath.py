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
# LIDAR START                 [Doc 3·4]
# =========================================

lidar_ser.write(bytes([0xA5, 0x40]))   # Reset
time.sleep(2)
lidar_ser.reset_input_buffer()

lidar_ser.write(bytes([0xA5, 0x20]))   # Scan Start
lidar_ser.read(7)                       # 응답 헤더 소비

print("LIDAR START")

# =========================================
# ROBOT PHYSICAL PARAMETER    [Doc 5]
# 단위: cm
# =========================================

ROBOT_RADIUS = 14.0   # 로봇 안전 반경 (cm) — arcsin 팽창에 사용
WHEEL_BASE   = 17.0   # 아두이노 차동구동과 일치 (cm)

# =========================================
# DRIVE PARAMETER
# 속도 단위: m/s (아두이노 전송 포맷 유지)
# =========================================

MAX_SPEED    = 0.14   # 최대 선속도 (m/s)
MIN_SPEED    = 0.05   # 최소 선속도 (m/s)
MAX_W        = 1.5    # 최대 각속도 (rad/s)
TURN_GAIN    = 1.8    # 조향 게인

SCAN_LIMIT   = 150    # 유효 인식 거리 (cm) — 맵 세로 310cm의 절반 수준
FRONT_RANGE  = 60     # 전방 탐색 반경 (±60°) — 측면 벽 간섭 방지

# =========================================
# FILTER PARAMETER             [Doc 4]
# =========================================

EMA_ALPHA    = 0.3    # EMA 새 데이터 반영 비율
MEDIAN_K     = 2      # 중앙값 필터 이웃 범위 (±k)

# =========================================
# SMOOTHING PARAMETER          [Doc 3]
# =========================================

SMOOTHING_NORMAL = 0.55   # 일반 주행
SMOOTHING_DANGER = 0.20   # 위험 근접 시 반응성 강화
DANGER_DIST      = 18     # 스무딩 전환 임계 거리 (cm)

# =========================================
# GAP PARAMETER                [Doc 4]
# 단위: cm
# =========================================

SAFE_DIST          = 17   # Gap 유효 최소 거리 (cm)
INFLATION_MAX_DIST = 25   # 팽창 적용 최대 거리 (cm)

FRONT_CLEAR_DIST   = 23   # 전방 열림 판단 거리 (cm)
FRONT_CLEAR_RANGE  = 15   # 전방 열림 판단 범위 (±°)

# =========================================
# STATE MACHINE                [Doc 3]
# =========================================

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1      # +1: 좌회전, -1: 우회전 (교번)

EMERGENCY_DIST   =  6     # 긴급회피 진입 거리 (cm)
REVERSE_DURATION = 0.35   # 후진 지속 시간 (초)
ROTATE_DURATION  = 1.00   # 회전 지속 시간 (초) — ≈52° 회전
REVERSE_SPEED    = -0.10  # 후진 선속도 (m/s)
ROTATE_W         =  0.9   # 회전 각속도 (rad/s) — 1.2 → 0.9 (69°→52° 감소)

# =========================================
# STATE
# =========================================

scan_data  = np.full(360, float(SCAN_LIMIT), dtype=np.float32)  # 단위: cm
prev_angle = 0.0

# =========================================
# UTIL
# =========================================

def normalize_angle(angle):
    return int(angle) % 360


# =========================================
# FILTER                        [Doc 4]
# =========================================

def apply_ema(angle, new_dist_cm):
    """EMA 필터: 이전값을 유지하며 새 측정값을 서서히 반영"""
    scan_data[angle] = (
        (1.0 - EMA_ALPHA) * scan_data[angle]
        + EMA_ALPHA * new_dist_cm
    )


def apply_median_filter():
    """중앙값 필터: 이웃 ±k개 중 중앙값으로 스파이크 노이즈 제거"""
    k      = MEDIAN_K
    window = 2 * k + 1
    filtered = np.empty(360, dtype=np.float32)

    for i in range(360):
        indices = [(i + d) % 360 for d in range(-k, k + 1)]
        values  = np.sort(scan_data[indices])
        filtered[i] = values[window // 2]

    scan_data[:] = filtered


# =========================================
# OBSTACLE INFLATION            [Doc 5]
# arcsin(R/D) 수학적 팽창 — 물리 기반 (cm)
# =========================================

def inflate_obstacles(dists):
    """
    각 장애물에서 arcsin(R/D) 로 팽창 각도 계산 후 마스킹
    dists 단위: cm
    """
    proc = dists.copy()

    for i in range(len(dists)):
        d = dists[i]

        if d < 5 or d >= INFLATION_MAX_DIST:   # 5cm 미만 / 80cm 초과 제외
            continue

        alpha = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))

        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists) - 1, int(i + alpha))

        proc[start_idx : end_idx + 1] = 0.0

    return proc


# =========================================
# GAP SEARCH                    [Doc 4]
# =========================================

def find_gaps(proc_dists, angles):
    """0이 아닌 연속 구간을 Gap으로 탐색"""
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


def score_gap(gap, proc_dists, angles):
    """Gap 점수: 너비·평균거리·중심 이탈 가중합"""
    start, end   = gap
    width        = end - start
    center_i     = (start + end) / 2.0
    center_angle = angles[int(center_i)]
    avg_dist     = np.mean(proc_dists[start : end + 1])

    score = (
        width              * 0.5
        + avg_dist         * 1.2
        - abs(center_angle) * 0.4
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
# PLANNING                      [Doc 3·4 통합]
# =========================================

def find_best_direction(smoothing):
    """
    전방 ±60° 스캔 데이터로 Gap을 탐색하고
    최적 방향각(도)을 반환
    """
    global prev_angle

    angles     = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists      = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    gaps       = find_gaps(proc_dists, angles)

    if not gaps:
        return None

    best_gap             = select_best_gap(gaps, proc_dists, angles)
    start, end           = best_gap
    center_i             = (start + end) / 2.0
    gap_angle            = float(angles[int(center_i)])

    # 전방 열림 여부 확인 (cm 기준)
    front_clear = float(np.min(
        scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]
    ))

    if front_clear > FRONT_CLEAR_DIST:
        # 전방 열림 → 직진 편향 70%
        target     = gap_angle * 0.3
        bias_label = "STRAIGHT"
    else:
        # 전방 막힘 → Gap 방향 편향 70%
        target     = gap_angle * 0.7
        bias_label = "GAP"

    # 스무딩   [Doc 3]
    target     = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target

    return target, bias_label, front_clear


# =========================================
# CONTROL
# =========================================

def compute_cmd(target_angle):
    """목표 각도로부터 v(m/s), w(rad/s) 계산"""

    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    # 회전각 기반 감속
    steering_ratio = min(abs(target_angle) / 90.0, 1.0)
    speed          = MAX_SPEED * (1.0 - steering_ratio * 0.75)

    # 전방 장애물 기반 추가 감속 (기준: 40cm)
    front_min      = float(np.min(scan_data[np.arange(-10, 11) % 360]))
    obstacle_scale = min(front_min / 40.0, 1.0)
    speed         *= obstacle_scale
    speed          = max(speed, MIN_SPEED)

    return speed, w


def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())


def stop_robot():
    send_cmd(0.0, 0.0)


# =========================================
# AVOID DIRECTION               [Doc 3]
# 좌우 평균거리 비교 → 넓은 방향으로 회전 (cm)
# =========================================

def choose_avoid_direction():
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))

    if left_avg >= right_avg:
        print(f"  [AVOID DIR] LEFT  (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return 1    # 좌회전
    else:
        print(f"  [AVOID DIR] RIGHT (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return -1   # 우회전


# =========================================
# MAIN LOOP
# =========================================

print("NAVIGATION START")

try:
    while True:

        # ---------------------------------
        # LiDAR 패킷 수신 + 검증   [Doc 3·4]
        # ---------------------------------
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

        s_flag     = raw[0] & 0x01
        s_inv_flag = (raw[0] & 0x02) >> 1

        if s_inv_flag != (1 - s_flag):   # 동기 비트 검증
            continue

        if (raw[1] & 0x01) != 1:          # 체크 비트 검증
            continue

        quality = raw[0] >> 2
        if quality < 3:                   # 저품질 필터링
            continue

        angle_raw = (raw[1] >> 1) | (raw[2] << 7)
        angle     = int(angle_raw / 64.0) % 360

        dist_raw = raw[3] | (raw[4] << 8)
        dist_cm  = (dist_raw / 4.0) / 10.0   # mm → cm

        if dist_cm < 3 or dist_cm > SCAN_LIMIT:   # 3cm 미만 / 100cm 초과 제외
            continue

        # EMA 필터 적용   [Doc 4]
        apply_ema(angle, dist_cm)

        # ---------------------------------
        # 1회전 완료 시점에 판단
        # ---------------------------------
        if s_flag != 1:
            continue

        # 중앙값 필터 적용   [Doc 4]
        apply_median_filter()

        now = time.time()

        # =================================
        # STATE MACHINE                [Doc 3]
        # =================================

        # ── STATE_REVERSE : 맹목 후진 ──
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
                print(f"  [REVERSE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                state = STATE_ROTATE
                maneuver_end_time = now + ROTATE_DURATION
                print(f"  [ROTATE START] dir:{'+' if rotate_dir > 0 else '-'}")
            continue

        # ── STATE_ROTATE : 맹목 회전 ──
        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
                print(f"  [ROTATE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                rotate_dir *= -1   # 교번: 다음 회피는 반대 방향
                state       = STATE_NORMAL
                prev_angle  = 0.0  # 스무딩 초기화

                # 전방 ±45° 잔존 데이터 리셋 (cm)
                for a in range(-45, 46):
                    scan_data[a % 360] = float(SCAN_LIMIT)

                print("  [NORMAL] maneuver done / scan reset ±45°")
            continue

        # ── STATE_NORMAL : 일반 주행 ──

        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))

        # 긴급회피 진입
        if front_min < EMERGENCY_DIST:
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            print(f"EMERGENCY! front:{front_min:.1f}cm → REVERSE")
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        # 위험 거리 기반 스무딩 전환   [Doc 3]
        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL

        result = find_best_direction(smoothing)

        # Gap을 찾지 못한 경우 긴급 전환
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

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
