import cv2
import serial
import numpy as np
import time
import math
import threading

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0",  460800, timeout=0.1)

# ── CAMERA ────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR RAW ─────────────────────────────────────────────────────────
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 90
THRESH_SLOW = 55.0
THRESH_TURN = 30.0
THRESH_STOP = 18.0

_scan     = np.full(360, 150.0, dtype=np.float32)
_scan_pub = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

def _ema(a, d):
    if d > 0:
        _scan[a] = (1 - EMA_ALPHA) * _scan[a] + EMA_ALPHA * d

def _median():
    k = MEDIAN_K
    buf = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
        buf[i] = np.sort(_scan[idx])[k]
    _scan[:] = buf

def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5: continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150: _ema(angle, dist_cm)
        if sf == 1:
            _median()
            with scan_lock: _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock: return _scan_pub.copy()

def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

def side_dist(scan, side):
    if side == "L":
        idx = np.arange(85, 96) % 360
    else:
        idx = np.arange(265, 276) % 360
    return float(np.mean(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))


# ══════════════════════════════════════════════════════════════════════
#  논문 알고리즘: Right Triangle + Triangle Bisection 혼합 모드
# ══════════════════════════════════════════════════════════════════════

# ── 파라미터 ──────────────────────────────────────────────────────────
WALL_TARGET   = 30.0   # 목표 벽 거리 (cm)

# 직각삼각형 방식
RT_BASE       = 40.0   # 직각삼각형 밑변 길이 (cm) — 논문의 L_b
RT_KP         = 0.006  # 면적오차 PID 비례 게인
RT_KD         = 0.003  # 면적오차 PID 미분 게인
RT_V          = 0.22   # 직선 구간 기본 속도

# 삼각형 이등분 방식 (코너)
TB_KP         = 0.025  # 각도오차 PID 게인
TB_V          = 0.13   # 코너 구간 속도

# 코너 인식
CORNER_RATIO  = 2.0    # 불연속 판정 비율 (논문: d_max/d_neighbor > 2)
CORNER_SCAN_L = (60, 120)   # 왼쪽 탐색 범위 (도)
CORNER_SCAN_R = (240, 300)  # 오른쪽 탐색 범위 (도)

# wall-following 탐색
WALL_V          = RT_V
WALL_TURN_V     = 0.10
WALL_LOST_W     = 0.5
WALL_SEARCH_W   = 1.1
WALL_SCAN_DIST  = 150.0
WALL_APPROACH_V = 0.20


def _scan_to_xy(scan, a_start, a_end):
    """LIDAR 거리 데이터를 XY 좌표로 변환 (논문 식 1~2)"""
    angles_deg = np.arange(a_start, a_end + 1, dtype=np.float32)
    angles_rad = np.deg2rad(angles_deg)
    idx        = angles_deg.astype(int) % 360
    dist       = scan[idx]
    valid      = dist < 148.0
    x = dist * np.cos(angles_rad)
    y = dist * np.sin(angles_rad)
    return x[valid], y[valid]


def _linear_regression_wall(scan, follow_side):
    """
    선형회귀로 근사 벽면을 구한다 (논문 3.1절, 식 3~5).
    follow_side="R" → 0~45도 구간 사용
    follow_side="L" → 135~180도 구간 사용 (왼쪽 벽)
    반환: (slope, intercept) 또는 None
    """
    if follow_side == "R":
        xs, ys = _scan_to_xy(scan, 0, 45)
    else:
        xs, ys = _scan_to_xy(scan, 135, 180)

    n = len(xs)
    if n < 5:
        return None

    x_mean = np.mean(xs)
    y_mean = np.mean(ys)
    num    = np.sum((xs - x_mean) * (ys - y_mean))
    den    = np.sum((xs - x_mean) ** 2)
    if abs(den) < 1e-6:
        return None

    slope     = num / den          # 논문 식 (3)
    intercept = y_mean - slope * x_mean  # 논문 식 (4)
    return slope, intercept


def _area_error(slope, intercept, target_dist):
    """
    근사 벽면과 목표 직각삼각형 사이의 면적오차를 계산한다 (논문 3.2절, 식 6~8).
    dist_err : 벽까지 거리 오차 (δ_d)
    angle_err: 벽과의 각도 오차 (δ_θ, rad)
    S_total  : 직사각형 + 삼각형 합산 면적오차
    """
    # 원점에서 직선 ax - y + b = 0까지의 거리
    # 직선: y = slope*x + intercept  →  slope*x - y + intercept = 0
    dist_to_wall = abs(intercept) / math.sqrt(slope ** 2 + 1)
    dist_err     = dist_to_wall - target_dist           # δ_d (식 6 근사)

    angle_err    = math.atan(slope)                     # δ_θ ≈ atan(m)

    # S_rect ≈ L_b * δ_d,  S_tri ≈ (L_b^2 / 2) * sin(δ_θ)
    S_rect = RT_BASE * dist_err
    S_tri  = (RT_BASE ** 2 / 2.0) * math.sin(angle_err) if abs(angle_err) < math.pi / 2 else 0.0
    S_total = S_rect + S_tri                            # 식 (8)
    return S_total, dist_err, angle_err


def _corner_recognition(scan):
    """
    코너 인식 알고리즘 (논문 4.1절).
    정면 기준 왼쪽(CORNER_SCAN_L)·오른쪽(CORNER_SCAN_R) 구간에서
    최대거리 데이터와 인접 데이터 비율이 CORNER_RATIO 초과 시 불연속으로 판정.

    반환: ("L"|"R"|None, d_max, d_corner_coord)
      - "L"/"R": 코너가 감지된 방향
      - None   : 코너 없음
    """
    results = {}
    for side, (a0, a1) in [("L", CORNER_SCAN_L), ("R", CORNER_SCAN_R)]:
        idx_range = np.arange(a0, a1) % 360
        dists     = scan[idx_range]
        max_pos   = int(np.argmax(dists))
        d_max     = float(dists[max_pos])

        # 인접 데이터: max_pos ±1 중 더 작은 쪽
        neighbor_vals = []
        for offset in [-1, 1]:
            ni = (max_pos + offset) % len(dists)
            neighbor_vals.append(float(dists[ni]))
        d_neighbor = min(neighbor_vals)

        if d_neighbor > 0 and (d_max / d_neighbor) > CORNER_RATIO:
            # 불연속 감지 → 코너 존재
            abs_angle = (a0 + max_pos) % 360
            results[side] = (d_max, abs_angle)

    if "L" in results:
        return "L", results["L"]
    if "R" in results:
        return "R", results["R"]
    return None, None


def _triangle_bisection_w(scan):
    """
    삼각형 이등분 방식 목표경로 각도오차 계산 (논문 4.2절, 식 9~13).
    왼쪽 최대거리 좌표 P_L, 오른쪽 최대거리 좌표 P_R의 중점 방향으로 조향.
    반환: w (angular velocity)
    """
    # 왼쪽 최대거리
    idx_l   = np.arange(*CORNER_SCAN_L) % 360
    ml      = int(np.argmax(scan[idx_l]))
    d_L     = float(scan[idx_l[ml]])
    a_L_deg = float(CORNER_SCAN_L[0] + ml)

    # 오른쪽 최대거리
    idx_r   = np.arange(*CORNER_SCAN_R) % 360
    mr      = int(np.argmax(scan[idx_r]))
    d_R     = float(scan[idx_r[mr]])
    a_R_deg = float(CORNER_SCAN_R[0] + mr)

    # XY 좌표 변환 (식 9~12)
    a_L = math.radians(a_L_deg)
    a_R = math.radians(a_R_deg)
    xL  =  d_L * math.cos(a_L)
    yL  =  d_L * math.sin(a_L)
    xR  =  d_R * math.cos(a_R)
    yR  =  d_R * math.sin(a_R)

    # 중점
    xM  = (xL + xR) / 2.0
    yM  = (yL + yR) / 2.0

    # 목표경로 각도오차 (식 13): θ_err = atan2(yM, xM) - atan2(yL, xL) 근사
    # 논문에서는 로봇 전진 방향(x축)과 목표경로 사이 각도
    theta_err = math.atan2(yM, xM)   # x축 기준 중점 방향 각도
    w = float(np.clip(-TB_KP * math.degrees(theta_err), -0.9, 0.9))
    return w


# PID 상태 (면적오차용)
_rt_prev_err = 0.0
_rt_prev_t   = time.time()

def wall_follow(scan, fm, adir, follow_side):
    """
    논문의 혼합 모드 벽면추종 (논문 4.3절 / Fig. 8).

    1) 코너 인식 → 삼각형 이등분 방식
    2) 코너 없음 → 직각삼각형 방식 (선형회귀 + 면적오차 PID)
    3) 정면 위험 → 긴급 회피
    """
    global _rt_prev_err, _rt_prev_t

    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)

    # ── 긴급 정면 회피 ──────────────────────────────────────────────
    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    # ── 측면 충돌 위험 ──────────────────────────────────────────────
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

    # ── 코너 인식 (논문 4.1절) ──────────────────────────────────────
    corner_side, _ = _corner_recognition(scan)

    if corner_side is not None:
        # ── 삼각형 이등분 방식 (논문 4.2절) ────────────────────────
        w = _triangle_bisection_w(scan)
        v = TB_V
        if fm < THRESH_SLOW:
            blend = float(np.clip(
                (THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
            v = TB_V * (1.0 - 0.3 * blend)
        return (v, w)

    # ── 직각삼각형 방식 (논문 3절) ──────────────────────────────────
    reg = _linear_regression_wall(scan, follow_side)

    if reg is None:
        # 벽 데이터 부족 → 해당 방향으로 천천히 조향
        sign = 1 if follow_side == "L" else -1
        sd   = side_dist(scan, follow_side)
        if sd > WALL_TARGET * 2.0:
            return (WALL_V * 0.7, sign * WALL_LOST_W)
        return (WALL_V * 0.8, 0.0)

    slope, intercept = reg
    S_err, dist_err, angle_err = _area_error(slope, intercept, WALL_TARGET)

    # PID (면적오차 → 조향각)
    now  = time.time()
    dt   = max(now - _rt_prev_t, 1e-3)
    dS   = (S_err - _rt_prev_err) / dt
    _rt_prev_err = S_err
    _rt_prev_t   = now

    sign = 1 if follow_side == "L" else -1
    w    = sign * (RT_KP * S_err + RT_KD * dS)

    # 정면 감속 블렌딩
    if fm < THRESH_SLOW:
        blend = float(np.clip(
            (THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = RT_V * (1.0 - 0.4 * blend)
    else:
        v = RT_V

    w = float(np.clip(w, -0.9, 0.9))
    return (v, w)


# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]),
               "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]),
               "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 95], [138, 207, 246]),
               "hsv2": None,
               "bgr":  ([40,  0,   0], [255, 220, 220]), "draw": (255, 80, 0)},
}
MISSION = ["red", "yellow", "blue"]

def make_mask(frame, hsv, name):
    cfg = COLOR_CFG[name]
    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    m = cv2.inRange(hsv, lo1, hi1)
    if cfg["hsv2"]:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        m = cv2.bitwise_or(m, cv2.inRange(hsv, lo2, hi2))
    bm = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    return cv2.bitwise_and(m, bm)

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA       = 400
KP_ROT         = 0.030
W_MIN          = 0.20
APPROACH_V     = 0.17
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP        = int(240 * 0.85)
ARRIVE_X_MARGIN     = 30
ARRIVE_FORWARD_SEC  = 0.8
ARRIVE_FORWARD_V    = 0.15
ARRIVE_CONFIRM      = 8

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
mission_idx   = 0
detect_count  = 0
arrive_count  = 0
follow_side   = "L"
lidar_state   = "WALL_SEARCH"
park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
last_cmd      = (0.0, 0.0)

print(f"START | MISSION: {MISSION}")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame  = cv2.flip(frame, 1)
        H, W   = frame.shape[:2]
        cx_mid = W // 2
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan   = get_scan()
        fm     = front_min(scan)
        adir   = avoid_dir(scan)

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big   = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        cx_obj, cy_obj = -1, -1
        if found:
            M_mom = cv2.moments(big)
            if M_mom["m00"] > 0:
                cx_obj = int(M_mom["m10"] / M_mom["m00"])
                cy_obj = int(M_mom["m01"] / M_mom["m00"])

            bx, by_top, bw, bh = cv2.boundingRect(big)
            last_seen_x   = cx_obj
            last_bottom_y = min(by_top + bh, 239)

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)
            cv2.line(frame, (cx_obj, by_top), (cx_obj, by_top + bh), (0, 255, 255), 1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        # 코너 상태 표시 (디버그용)
        corner_side, _ = _corner_recognition(scan)
        corner_label = f"CORNER:{corner_side}" if corner_side else "STRAIGHT"

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and
                    cy_obj >= ARRIVE_Y_TOP)

        # ══ LIDAR 모드 ════════════════════════════════════════════════
        if mode == "LIDAR":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                print(f"[{target}] 발견 → 추적 시작")
                continue

            # ── A. 벽 탐색 (제자리 회전) ─────────────────────────
            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    lidar_state = "WALL_APPROACH"
                    print(f"[LIDAR] 벽 감지 fm:{fm:.0f}cm, follow={follow_side} → 접근")
                else:
                    send_cmd(0.0, WALL_SEARCH_W)
                    cv2.putText(frame, f"LIDAR-SEARCH fm:{fm:.0f}",
                                (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── B. 벽 접근 ────────────────────────────────────────
            elif lidar_state == "WALL_APPROACH":
                sd   = side_dist(scan, follow_side)
                sign = 1 if follow_side == "L" else -1
                if sd <= WALL_TARGET * 1.3:
                    lidar_state = "WALL_FOLLOW"
                    print(f"[LIDAR] 벽 도달 {follow_side}:{sd:.0f}cm → 추종 시작")
                else:
                    if fm < THRESH_STOP:
                        v, w = 0.08, adir * 1.0
                    elif fm < THRESH_TURN:
                        v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                    else:
                        v, w = WALL_APPROACH_V, sign * 0.3
                    send_cmd(v, w)
                    cv2.putText(frame, f"LIDAR-APPROACH {follow_side}:{sd:.0f}cm",
                                (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── C. 혼합 모드 벽면추종 (논문 알고리즘) ─────────────
            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                sd_disp = side_dist(scan, follow_side)
                cv2.putText(frame, f"LIDAR-FOLLOW {follow_side}:{sd_disp:.0f} {corner_label}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            cv2.putText(frame, "MODE: LIDAR", (10, 45), 0, 0.5, (255, 255, 255), 1)

        # ══ PARK 모드 ════════════════════════════════════════════════
        elif mode == "PARK":

            # ── 1. 도착 후 전진 ───────────────────────────────────
            if park_state == "FORWARD":
                elapsed = time.time() - park_t
                if elapsed >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t = time.time()
                    print(f"[{target}] 전진 완료 → 정차")
                else:
                    send_cmd(*last_cmd)
                cv2.putText(frame, f"FORWARD: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 2. 정차 ───────────────────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"
                        print(f"다음 미션 [{MISSION[mission_idx]}] wall-following 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 3-A. 벽 탐색 (제자리 회전) ───────────────────────
            elif park_state == "WALL_SEARCH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] 탐색 중 발견 → 추적")
                        continue
                else:
                    detect_count = 0

                if fm < WALL_SCAN_DIST:
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    park_state = "WALL_APPROACH"
                    print(f"벽 감지 fm:{fm:.0f}cm, follow={follow_side} → 접근")
                    continue

                send_cmd(0.0, WALL_SEARCH_W)
                cv2.putText(frame, f"WALL-SEARCH [{target}] fm:{fm:.0f}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── 3-B. 벽 접근 ──────────────────────────────────────
            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] 접근 중 발견 → 추적")
                        continue
                else:
                    detect_count = 0

                sd   = side_dist(scan, follow_side)
                sign = 1 if follow_side == "L" else -1
                if sd <= WALL_TARGET * 1.3:
                    park_state = "WALL_FOLLOW"
                    print(f"벽 도달 {follow_side}:{sd:.0f}cm → 추종")
                    continue

                if fm < THRESH_STOP:
                    v, w = 0.08, adir * 1.0
                elif fm < THRESH_TURN:
                    v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else:
                    v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)
                cv2.putText(frame, f"WALL-APPROACH [{target}] {follow_side}:{sd:.0f}cm",
                            (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── 3-C. 혼합 모드 벽면추종 (논문 알고리즘) ───────────
            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] wall-follow 중 발견 → 추적")
                        continue
                else:
                    detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                sd_disp = side_dist(scan, follow_side)
                cv2.putText(frame, f"WALL-FOLLOW [{target}] {follow_side}:{sd_disp:.0f} {corner_label}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── 4. 객체 추적 ──────────────────────────────────────
            elif found:
                park_state = "TRACK"
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    print(f"[{target}] centroid 확정 → {ARRIVE_FORWARD_SEC}초 전진")
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                    continue

                err_x     = cx_obj - cx_mid
                err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                reduced_v = APPROACH_V * (1.0 - err_ratio)

                def cam_w(ex):
                    raw = -KP_ROT * ex
                    if abs(raw) < W_MIN and ex != 0:
                        return -W_MIN if ex > 0 else W_MIN
                    return raw

                if fm >= THRESH_SLOW:
                    v = reduced_v
                    w = cam_w(err_x)
                else:
                    w_cam = cam_w(err_x)
                    w_lid = adir * 0.7
                    if fm < THRESH_STOP:
                        v, w = 0.09, w_lid
                    elif fm < THRESH_TURN:
                        v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                    else:
                        v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                last_cmd = (v, w)
                send_cmd(v, w)
                cv2.putText(frame, f"TRACKING: {target}", (10, 25), 0, 0.6, draw, 1)

            # ── 5. 객체 놓침 ──────────────────────────────────────
            else:
                park_state = "SEARCH"
                arrive_count = 0
                w = (-1.0 if last_seen_x > cx_mid else 1.0)
                send_cmd(0.0, w)
                cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
