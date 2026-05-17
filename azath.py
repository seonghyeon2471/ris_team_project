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
lidar_ser.write(bytes([0xA5, 0x40]))   # Reset
time.sleep(2)
lidar_ser.reset_input_buffer()

lidar_ser.write(bytes([0xA5, 0x20]))   # Scan Start
lidar_ser.read(7)                       # 응답 헤더 소비

print("LIDAR START")

# =========================================
# ROBOT PHYSICAL PARAMETER  (단위: cm)
# =========================================
# ★ 수정: ROBOT_RADIUS 18.0 — sqrt(15²+8.5²)≈17.3, 뒷바퀴 스윕 반경 반영
# ★ 추가: LIDAR_TO_AXLE — 라이다↔바퀴축 오프셋, inflate 등가 반경 계산에 사용
ROBOT_RADIUS  = 18.0   # 17.0 → 18.0
LIDAR_TO_AXLE = 15.0   # ★ 신규 — 라이다↔바퀴축 직선 거리 (cm)
WHEEL_BASE    = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================
# ★ 수정: MAX_SPEED 0.16 — 좁은 맵(110cm) 정밀 주행
# ★ 수정: TURN_GAIN 1.0  — 1.8은 과선회 유발, 110cm 폭에서 벽 충돌 위험
MAX_SPEED    = 0.16    # 0.14 → 0.16  (v2 원본이 이미 느렸으나 통일)
MIN_SPEED    = 0.05
MAX_W        = 1.1     # 1.5  → 1.1   | 과각속 억제
TURN_GAIN    = 1.0     # 1.8  → 1.0   | 좁은 맵 과선회 방지 ★핵심 수정

SCAN_LIMIT   = 130     # 150  → 130   | 맵 최대 310cm, 원거리 노이즈 차단
FRONT_RANGE  = 60      # 유지

# =========================================
# FILTER PARAMETER
# =========================================
EMA_ALPHA    = 0.3
MEDIAN_K     = 2

# =========================================
# SMOOTHING PARAMETER
# =========================================
SMOOTHING_NORMAL = 0.50    # 0.55 → 0.50 | 반응성 향상
SMOOTHING_DANGER = 0.20
DANGER_DIST      = 24      # 18   → 24   | 위험 판단 거리 상향 (뒷바퀴 여유)

# =========================================
# GAP & INFLATION PARAMETER
# =========================================
# ★ 수정: SAFE_DIST 19 — 뒷바퀴 안전 vs 좁은 맵 갭 확보 타협점
#   110cm 폭 기준: (110 - 장애물25 - 로봇20) / 2 ≈ 32cm → 최대 20cm 이내 제한
# ★ 수정: INFLATION_MAX_DIST 28 — ROBOT_RADIUS 증가 + 갭 소멸 방지 균형
SAFE_DIST          = 19    # 17   → 19
INFLATION_MAX_DIST = 28    # 25   → 28

# ★ 수정: FRONT_CLEAR_DIST 28 — 전방 개방 판단 보수적으로
# ★ 수정: FRONT_CLEAR_RANGE 12 — 좁은 복도, 전방 판단 범위 축소
FRONT_CLEAR_DIST   = 28    # 23   → 28
FRONT_CLEAR_RANGE  = 12    # 15   → 12

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1      # +1: 좌회전, -1: 우회전

# ★ 수정: EMERGENCY_DIST 9   — 조기 진입으로 선회 공간 선확보
# ★ 수정: REVERSE_DURATION 0.28 — 후진 거리 늘려 선회 전 충분한 공간 마련
# ★ 수정: ROTATE_DURATION 1.10 — 좁은 폭에서 충분한 방향 전환각 확보
# ★ 수정: REVERSE_SPEED -0.08 — 좁은 공간 후진 속도 감소
# ★ 수정: ROTATE_W 0.50      — 0.9는 너무 빠름, 뒷바퀴 간섭 감지 여유 필요
EMERGENCY_DIST    =  9      # 6    →  9
REVERSE_DURATION  =  0.28   # 0.18 →  0.28
ROTATE_DURATION   =  1.10   # 1.00 →  1.10
REVERSE_SPEED     = -0.08   # -0.10 → -0.08
ROTATE_W          =  0.50   # 0.9  →  0.50  ★핵심 수정

# =========================================
# STATE DATA
# =========================================
scan_data  = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0

# =========================================
# UTIL & FILTER
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
# ★ 수정: LIDAR_TO_AXLE 기반 등가 반경으로 뒷바퀴 스윕 반영
# =========================================
def inflate_obstacles(dists):
    proc = dists.copy()

    # 라이다↔바퀴축 오프셋을 반영한 등가 반경
    effective_radius = math.sqrt(ROBOT_RADIUS**2 + LIDAR_TO_AXLE**2)

    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= INFLATION_MAX_DIST:
            continue
        # 등가 반경 기준 팽창각 (뒷바퀴 스윕 간섭 영역 포함)
        alpha = math.degrees(math.asin(min(effective_radius / max(d, effective_radius), 1.0)))
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

def score_gap(gap, proc_dists, angles):
    start, end   = gap
    width        = end - start
    center_i     = (start + end) / 2.0
    center_angle = angles[int(center_i)]
    avg_dist     = np.mean(proc_dists[start : end + 1])
    min_dist     = np.min(proc_dists[start : end + 1])

    # ★ 수정: 가중치 전면 개선
    #   width   0.5  → 1.5  (좁은 갭 선호 억제)
    #   avg_dist 1.2 → 2.0  (실제 열린 공간 우선)
    #   min_dist 추가 × 1.0 (막힌 틈 억제)
    #   편향 패널티 0.4 → 0.30 (정면 편향 강화, 측면 오진입 억제)
    score = (
        width               * 1.5
        + avg_dist          * 2.0
        + min_dist          * 1.0
        - abs(center_angle) * 0.30
    )
    return score

def select_best_gap(gaps, proc_dists, angles):
    best_gap, best_score = None, -1e9
    for gap in gaps:
        s = score_gap(gap, proc_dists, angles)
        if s > best_score:
            best_score, best_gap = s, gap
    return best_gap

# =========================================
# PLANNING
# =========================================
def find_best_direction(smoothing):
    global prev_angle

    angles     = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists      = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    gaps       = find_gaps(proc_dists, angles)

    if not gaps:
        return None

    best_gap     = select_best_gap(gaps, proc_dists, angles)
    start, end   = best_gap
    gap_angle    = float(angles[int((start + end) / 2.0)])

    front_clear = float(np.min(
        scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]
    ))

    if front_clear > FRONT_CLEAR_DIST:
        # 전방 열림 → 직진 편향 (Gap 방향 20%만 반영)
        # ★ 수정: 0.3 → 0.2 (직진 편향 강화)
        target, bias_label = gap_angle * 0.2, "STRAIGHT"
    else:
        # 전방 막힘 → Gap 방향 100% 추종
        # ★ 수정: 0.7 → 1.0 (막혔을 때 Gap 완전 추종)
        target, bias_label = gap_angle * 1.0, "GAP"

    target     = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    return target, bias_label, front_clear

# =========================================
# CONTROL (측면 충돌 방지 유지 + 파라미터 정합)
# =========================================
ALIGN_THRESHOLD = 15   # 10 → 15 | Gap 방향 선회 중 직진 빠르게 재개

def compute_cmd(target_angle):
    # 1. 조향 계산
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    # 2. 측면 감시 (±FRONT_RANGE° 전체 최소 거리)
    search_indices = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    relevant_min   = float(np.min(scan_data[search_indices]))

    # 3. 정렬 중 — 측면 여유 있을 때만 미세 전진
    if abs(target_angle) > ALIGN_THRESHOLD:
        v = 0.02 if relevant_min > 20.0 else 0.0
        return v, w

    # 4. 직진 속도 계산 (측면/정면 통합 장애물 거리 반영)
    obstacle_scale = min(relevant_min / 40.0, 1.0)
    speed          = max(MAX_SPEED * obstacle_scale, MIN_SPEED)

    # 5. 좁은 공간/측면 근접 시 추가 감속
    if relevant_min < 22.0:
        speed *= 0.8

    return speed, w

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

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

        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360

        # ★ 버그 수정: /40.0 → /4.0/10.0
        #   raw 값은 1/4 mm 단위 → ÷4 = mm → ÷10 = cm
        #   기존 /40.0 은 우연히 근사되지만 소수점 오차 누적 발생
        dist_cm = (raw[3] | (raw[4] << 8)) / 4.0 / 10.0

        if 3 < dist_cm < SCAN_LIMIT:
            apply_ema(angle, dist_cm)

        if s_flag != 1:
            continue

        apply_median_filter()
        now = time.time()

        # --- STATE MACHINE ---
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
                print(f"  [REVERSE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                state             = STATE_ROTATE
                maneuver_end_time = now + ROTATE_DURATION
                print(f"  [ROTATE START] dir:{'+' if rotate_dir > 0 else '-'}")
            continue

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

        # --- STATE_NORMAL ---
        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))

        if front_min < EMERGENCY_DIST:
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            print(f"EMERGENCY! front:{front_min:.1f}cm → REVERSE")
            send_cmd(REVERSE_SPEED, 0.0)
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
