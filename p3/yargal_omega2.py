import cv2
import serial
import numpy as np
import time
import math
import threading
from scipy.signal import savgol_filter

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

# ── LIDAR ─────────────────────────────────────────────────────────────
EMA_ALPHA    = 0.35
MEDIAN_K     = 2
FRONT_RANGE  = 90   # 정면 감지 범위 (±90도 = 총 180도)
THRESH_SLOW  = 55.0
THRESH_TURN  = 30.0
THRESH_STOP  = 18.0

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

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ══════════════════════════════════════════════════════════════════════
# ── BUBBLE AND RAIN 알고리즘 (논문 구현) ─────────────────────────────
# ══════════════════════════════════════════════════════════════════════
# 논문 개념 → 실시간 LiDAR 적용 방식:
#   - 경로점: 현재 로봇 위치를 기준으로 LiDAR 각도별 거리값을 경로점으로 사용
#   - Bubble: 각도별 측정 거리가 안전거리(WALL_TARGET)보다 가까우면 → 멀어지는 방향 보정
#   - Rain  : 안전거리보다 멀면 → 벽 방향으로 접근 보정
#   - Savitzky-Golay 필터로 보정된 각도별 거리값을 스무딩
#   - 스무딩된 거리 프로파일에서 follow_side 방향 오차를 계산해 제어 명령 생성

BNR_WINDOW_DEG  = 30    # 측면 감지 윈도우 (±도)
BNR_INTERP_N    = 60    # 보간 경로점 수 (논문: 100개 이상 권장, 실시간이므로 60개)
BNR_STEP        = 0.5   # Bubble/Rain 이동 단위 (cm)
BNR_MAX_ITER    = 60    # Bubble/Rain 최대 반복 횟수

# Savitzky-Golay 필터 파라미터 (논문 수식 5)
SG_WINDOW = 11   # 홀수여야 함
SG_POLY   = 2    # 다항식 차수


def scan_to_occupancy(scan, threshold=80.0):
    """
    LiDAR 스캔을 occupancy 배열로 변환.
    M(q) 역할: 거리가 threshold 이하면 장애물(1), 이상이면 자유공간(0).
    논문의 지도(map) 역할을 실시간 LiDAR로 대체.
    """
    occupancy = np.where(scan < threshold, 1.0, 0.0)
    return occupancy


def chi(occupancy, center_angle, radius_deg, tau=0.5):
    """
    논문 수식 (1): χ(p, r) 구현
    특정 각도(center_angle)를 중심으로 반지름(radius_deg) 범위 내에
    장애물(occupancy > tau)이 존재하는지 판별.
    각도 공간에서의 '가상의 원' 역할.
    """
    angles = np.arange(center_angle - radius_deg,
                       center_angle + radius_deg + 1, dtype=int) % 360
    return bool(np.any(occupancy[angles] > tau))


def bubble_step(occupancy, angle, radius_deg, u_dir, step=BNR_STEP, max_iter=BNR_MAX_ITER):
    """
    논문 Bubble 단계 (수식 2): α*(p) = min{α≥0 | χ(p + αu, r) = 0}
    장애물과 겹쳐 있는 경로점을 회피 방향(u_dir)으로 이동시켜
    가상의 원이 장애물에 닿지 않을 때까지의 최소 이동량 반환.
    각도 공간에서 angle을 u_dir 방향으로 이동.
    """
    alpha = 0.0
    cur_angle = angle
    for _ in range(max_iter):
        if not chi(occupancy, int(cur_angle) % 360, radius_deg):
            break
        cur_angle += u_dir * step
        alpha += step
    return alpha, cur_angle % 360


def rain_step(occupancy, angle, radius_deg, n_dir, step=BNR_STEP, max_iter=BNR_MAX_ITER):
    """
    논문 Rain 단계 (수식 3): β*(p) = min{β≥0 | χ(p + α*u + βn, r) = 1}
    Bubble로 벽에서 충분히 떨어진 경로점을 다시 벽 방향(n_dir)으로 이동시켜
    가상의 원이 벽 표면에 딱 접할 때까지의 최소 이동량 반환.
    """
    beta = 0.0
    cur_angle = angle
    for _ in range(max_iter):
        next_angle = (cur_angle + n_dir * step) % 360
        if chi(occupancy, int(next_angle) % 360, radius_deg):
            break
        cur_angle = next_angle
        beta += step
    return beta, cur_angle % 360


def bubble_and_rain_correct(scan, follow_side, wall_target=None):
    """
    논문 알고리즘 전체 파이프라인:
      1) LiDAR → occupancy 변환
      2) follow_side 방향 윈도우에서 경로점 보간 (BNR_INTERP_N개)
      3) 각 경로점에 Bubble → Rain 보정 적용 (수식 4: p* = p + α*u + β*n)
      4) Savitzky-Golay 필터로 스무딩 (수식 5)
      5) 스무딩된 결과에서 벽과의 거리 오차 계산 → (v, w) 반환
    """
    if wall_target is None:
        wall_target = WALL_TARGET

    occupancy = scan_to_occupancy(scan)

    # follow_side에 따라 중심 각도 및 방향 벡터 결정
    # n: 장애물 방향 벡터(벽 쪽), u: 회피 벡터 = -n (논문: u = -n)
    if follow_side == "L":
        center_angle = 90        # 왼쪽 (90도)
        n_dir = +1               # 벽 방향: 각도 증가
        u_dir = -1               # 회피 방향: 각도 감소
    else:
        center_angle = 270       # 오른쪽 (270도)
        n_dir = -1               # 벽 방향: 각도 감소
        u_dir = +1               # 회피 방향: 각도 증가

    # ① 경로 보간: center_angle 기준 ±BNR_WINDOW_DEG 범위를 BNR_INTERP_N개로 보간
    half = BNR_WINDOW_DEG
    raw_angles = np.linspace(center_angle - half, center_angle + half, BNR_INTERP_N)

    # ② 반지름: 안전거리를 각도 단위로 근사 (wall_target cm → 각도 범위)
    # LiDAR 해상도 1도 기준, wall_target cm에 해당하는 각도폭을 사용
    radius_deg = max(2, int(wall_target / 5.0))

    # ③ 각 경로점에 Bubble + Rain 보정 적용 (수식 4: p* = p + α*u + β*n)
    corrected_angles = []
    for ang in raw_angles:
        ang_i = int(ang) % 360

        # Bubble: 장애물에 겹쳐 있으면 회피 방향으로 이동
        alpha, ang_after_bubble = bubble_step(occupancy, ang_i, radius_deg, u_dir)

        # Rain: 벽 방향으로 접근하여 표면에 접하게 이동
        beta, ang_final = rain_step(occupancy, ang_after_bubble, radius_deg, n_dir)

        corrected_angles.append(ang_final)

    corrected_angles = np.array(corrected_angles)

    # ④ Savitzky-Golay 필터 스무딩 (논문 수식 5)
    # window_length는 데이터 길이보다 작아야 하며 홀수여야 함
    win = min(SG_WINDOW, len(corrected_angles) - 1)
    if win % 2 == 0:
        win -= 1
    if win >= SG_POLY + 1:
        smoothed_angles = savgol_filter(corrected_angles, win, SG_POLY)
    else:
        smoothed_angles = corrected_angles

    # ⑤ 스무딩된 결과에서 추종 각도의 실제 거리 오차 계산
    # 보정된 경로점 중 center_angle에 가장 가까운 인덱스의 각도에서 거리 측정
    mid_idx = BNR_INTERP_N // 2
    best_angle = int(round(smoothed_angles[mid_idx])) % 360
    corrected_dist = float(scan[best_angle])

    return corrected_dist, smoothed_angles


def wall_follow_bnr(scan, fm, adir, follow_side):
    """
    Bubble and Rain 기반 벽 추종 제어기.
    기존 wall_follow() 함수를 논문 알고리즘으로 완전 대체.

    흐름:
      1) Bubble and Rain으로 보정된 벽 거리 획득
      2) 보정 거리와 WALL_TARGET 차이(err)로 각속도(w) 계산
      3) 정면 장애물 거리(fm)에 따라 속도 블렌딩
    """
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1

    # ① 정면 위험 → 긴급 회피 (Bubble and Rain 생략, 즉각 반응)
    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    # ② 측면 너무 가까움 → 반대쪽으로 긴급 회피
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

    # ③ Bubble and Rain으로 보정된 거리 획득
    corrected_dist, _ = bubble_and_rain_correct(scan, follow_side, WALL_TARGET)

    # ④ 벽이 너무 멀면 (follow_side 방향에 벽 없음) → 벽 찾으러 회전
    if corrected_dist > WALL_TARGET * 2.0:
        return (WALL_V * 0.7, sign * WALL_LOST_W)

    # ⑤ 정상 Bubble and Rain 기반 wall-following
    # err: 보정 거리 - 목표 거리 (양수: 너무 멀음, 음수: 너무 가까움)
    err = corrected_dist - WALL_TARGET
    w   = sign * WALL_KP * err

    if fm < THRESH_SLOW:
        blend = float(np.clip(
            (THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0
        ))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V

    w = float(np.clip(w, -0.9, 0.9))
    return (v, w)

# ══════════════════════════════════════════════════════════════════════


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

ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8

# wall-following (Bubble and Rain)
WALL_TARGET    = 25.0
WALL_SCAN_DIST = 150.0
WALL_APPROACH_V = 0.20
WALL_KP        = 0.012
WALL_V         = 0.22
WALL_TURN_V    = 0.10
WALL_LOST_W    = 0.5
WALL_SEARCH_W  = 1.1

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

            # ── A. 벽 탐색 (WALL_SEARCH) ──────────────────────────
            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    lidar_state = "WALL_APPROACH"
                    print(f"[LIDAR] 벽 감지 fm:{fm:.0f}cm, follow_side={follow_side} → 접근")
                else:
                    send_cmd(0.0, WALL_SEARCH_W)
                    cv2.putText(frame, f"LIDAR-WALL-SEARCH fm:{fm:.0f}",
                                (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── B. 벽 접근 (WALL_APPROACH) ────────────────────────
            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    lidar_state = "WALL_FOLLOW"
                    print(f"[LIDAR] 벽 도달 {follow_side}:{sd:.0f}cm → BnR 시작")
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP:
                        v, w = 0.08, adir * 1.0
                    elif fm < THRESH_TURN:
                        v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                    else:
                        v, w = WALL_APPROACH_V, sign * 0.3
                    send_cmd(v, w)
                    cv2.putText(frame, f"LIDAR-WALL-APPROACH {follow_side}:{sd:.0f}cm",
                                (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── C. Bubble and Rain wall-following (WALL_FOLLOW) ───
            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow_bnr(scan, fm, adir, follow_side)
                send_cmd(v, w)
                corrected_dist, _ = bubble_and_rain_correct(scan, follow_side)
                cv2.putText(frame, f"LIDAR-BnR {follow_side} corr:{corrected_dist:.0f}cm",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            cv2.putText(frame, "MODE: LIDAR", (10, 45), 0, 0.5, (255, 255, 255), 1)

        # ══ PARK 모드 ════════════════════════════════════════════════
        elif mode == "PARK":

            # ── 1. 도착 후 전진 (FORWARD) ─────────────────────────
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

            # ── 2. 정차 (PARKING) ─────────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"
                        print(f"다음 미션 [{MISSION[mission_idx]}] BnR 탐색 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 3-A. 벽 탐색 (WALL_SEARCH) ───────────────────────
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
                    print(f"벽 감지 fm:{fm:.0f}cm, follow_side={follow_side} → 접근")
                    continue

                send_cmd(0.0, WALL_SEARCH_W)
                cv2.putText(frame, f"WALL-SEARCH [{target}] fm:{fm:.0f}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── 3-B. 벽 접근 (WALL_APPROACH) ─────────────────────
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

                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    park_state = "WALL_FOLLOW"
                    print(f"벽 도달 {follow_side}:{sd:.0f}cm → BnR 시작")
                    continue

                sign = 1 if follow_side == "L" else -1
                if fm < THRESH_STOP:
                    v, w = 0.08, adir * 1.0
                elif fm < THRESH_TURN:
                    v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else:
                    v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)
                cv2.putText(frame, f"WALL-APPROACH [{target}] {follow_side}:{sd:.0f}cm",
                            (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── 3-C. Bubble and Rain wall-following (WALL_FOLLOW) ─
            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] BnR 중 발견 → 추적")
                        continue
                else:
                    detect_count = 0

                v, w = wall_follow_bnr(scan, fm, adir, follow_side)
                send_cmd(v, w)
                corrected_dist, _ = bubble_and_rain_correct(scan, follow_side)
                cv2.putText(frame, f"BnR-FOLLOW [{target}] {follow_side} corr:{corrected_dist:.0f}cm",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── 4. 객체 추적 (TRACK) ──────────────────────────────
            elif found:
                park_state = "TRACK"
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    print(f"[{target}] 도착 확정 → {ARRIVE_FORWARD_SEC}초 전진")
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                    continue

                else:
                    err_x = cx_obj - cx_mid
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

            # ── 5. 객체 놓침 (SEARCH) ─────────────────────────────
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
