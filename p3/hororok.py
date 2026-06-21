import cv2
import serial
import numpy as np
import time
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

# ── LIDAR ─────────────────────────────────────────────────────────────
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

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# ── 벽 직선 추정 (핵심) ───────────────────────────────────────────────
#
# 로봇 좌표계: 전방 = +X, 왼쪽 = +Y
# 라이다 각도 0° = 전방, 반시계 방향으로 증가 (표준 RPLidar 관례 따름)
#
# 왼쪽 벽: 60°~120° 범위의 포인트 사용  (90°가 정측면)
# 오른쪽 벽: 240°~300° 범위의 포인트 사용 (270°가 정측면)
#
# 선형회귀로 벽의 기울기(wall_angle)와 수직거리(wall_dist) 추출:
#   wall_angle > 0 : 벽이 진행방향 기준 앞쪽이 더 가까움 (파고드는 중)
#   wall_angle < 0 : 벽이 진행방향 기준 뒤쪽이 더 가까움 (멀어지는 중)
#   wall_dist      : 90° (정측면) 기준 벽까지 추정 수직거리

WALL_SCAN_START_L = 55    # 왼쪽 벽 스캔 시작각
WALL_SCAN_END_L   = 125   # 왼쪽 벽 스캔 끝각
WALL_SCAN_START_R = 235   # 오른쪽 벽 스캔 시작각
WALL_SCAN_END_R   = 305   # 오른쪽 벽 스캔 끝각
WALL_MAX_DIST     = 80.0  # 이 거리 이상이면 벽 포인트에서 제외 (cm)
WALL_MIN_POINTS   = 5     # 선형회귀에 필요한 최소 포인트 수

def estimate_wall(scan, side):
    """
    지정된 측면의 라이다 포인트를 직교좌표로 변환 후
    선형회귀로 벽 직선을 추정한다.

    반환값: (wall_dist, wall_angle, valid)
      wall_dist  : 로봇 무게중심 ~ 벽 수직거리 (cm)
      wall_angle : 벽 직선과 로봇 X축(전방)이 이루는 각도 오차 (rad)
                   양수 = 앞머리가 벽으로 파고드는 방향
      valid      : 추정 신뢰도 (포인트 부족/이상치 많으면 False)
    """
    if side == "L":
        angles_deg = np.arange(WALL_SCAN_START_L, WALL_SCAN_END_L + 1)
    else:
        angles_deg = np.arange(WALL_SCAN_START_R, WALL_SCAN_END_R + 1)

    angles_rad = np.deg2rad(angles_deg)
    dists      = scan[angles_deg % 360].astype(np.float32)

    # 유효 포인트만 사용 (너무 멀거나 최대값인 포인트 제외)
    valid_mask = (dists > 2.0) & (dists < WALL_MAX_DIST)
    if np.sum(valid_mask) < WALL_MIN_POINTS:
        return 0.0, 0.0, False

    a = angles_rad[valid_mask]
    d = dists[valid_mask]

    # 극좌표 → 직교좌표 (로봇 기준: 전방 X+, 왼쪽 Y+)
    # 라이다 각도 θ: 전방=0, 왼쪽=90, 후방=180, 오른쪽=270
    px =  d * np.cos(a)   # 전방 성분
    py =  d * np.sin(a)   # 측면 성분 (왼쪽 양수)

    # 선형회귀: py = m * px + b  (벽이 X축과 이루는 직선)
    # 포인트가 거의 Y=const 직선 형태이므로 px를 독립변수로
    A = np.vstack([px, np.ones(len(px))]).T
    try:
        result  = np.linalg.lstsq(A, py, rcond=None)
        m, b    = result[0]
    except Exception:
        return 0.0, 0.0, False

    # 벽 직선의 기울기 m = dy/dx
    # 로봇이 벽과 완전히 평행하면 m ≈ 0 (벽이 Y = const)
    # wall_angle: 벽 법선과 로봇 측면(Y축)이 이루는 각 = atan(m)
    # 양수면 앞머리가 벽 쪽으로 기울어진 것
    wall_angle = float(np.arctan(m))   # rad, 작은 값

    # 수직거리: 90도 방향의 회귀선 Y값 = b (px=0일 때)
    # 오른쪽이면 부호 반전해서 항상 양수 거리로 반환
    wall_dist = abs(float(b))

    return wall_dist, wall_angle, True


# ── 벽 추종 제어기 (Stanley 스타일) ──────────────────────────────────
#
# Stanley controller 공식 응용:
#   w = Kh * heading_error + atan(Kd * dist_error / v)
#
# heading_error : 벽과 로봇 진행방향의 각도 차이 (= wall_angle)
#   → 이것만 보정해도 평행에 가까워지지만, 거리 오차가 남음
#
# dist_error    : 목표거리 - 실제거리
#   → atan으로 감싸면 거리오차가 클 때는 강하게, 수렴할수록 부드럽게
#   → 속도(v)로 나누면 저속일수록 조향각 크게 (안전)
#
# 미래 예측:
#   LOOKAHEAD_TIME초 후 위치에서의 벽 거리를 추정해
#   현재 조향을 선제적으로 보정 → 오버슈트 방지

WALL_TARGET      = 20.0   # 목표 벽 거리 (cm)
WALL_KH          = 1.20   # 헤딩(각도) 오차 게인
WALL_KD          = 0.06   # 거리 오차 게인
WALL_V           = 0.22   # 기본 전진 속도
WALL_TURN_V      = 0.10   # 전방 장애물 회피 속도
WALL_APPROACH_V  = 0.20   # 접근 속도
WALL_LOST_W      = 0.45   # 벽 유실 시 탐색 조향
WALL_SEARCH_W    = 1.1    # 벽 없을 때 회전 탐색
LOOKAHEAD_TIME   = 0.35   # 미래 예측 시간 (초)
MAX_W            = 0.90   # 최대 조향 속도

def wall_follow(scan, fm, adir, follow_side, current_v=None):
    """
    벽 직선을 선형회귀로 추정하고 Stanley 스타일로 부드럽게 추종.

    조향 성분:
      1) heading_error (KH): 벽과 평행하지 않은 각도 → 즉각 보정
      2) dist_error (KD):    목표 거리와의 차이 → atan으로 부드럽게 보정
      3) lookahead 보정:     현재 w로 LOOKAHEAD_TIME 후 예상 위치의
                             거리 변화를 미리 계산해 오버슈트 억제

    side 부호 규칙:
      왼쪽(L): 거리 오차 양수 → 왼쪽(양수 w)으로 조향
      오른쪽(R): 거리 오차 양수 → 오른쪽(음수 w)으로 조향
    """
    if current_v is None:
        current_v = WALL_V

    sign = 1.0 if follow_side == "L" else -1.0

    # ── 전방/측방 장애물 회피 최우선 ─────────────────────────────────
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)

    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

    # ── 벽 직선 추정 ─────────────────────────────────────────────────
    wall_dist, wall_angle, valid = estimate_wall(scan, follow_side)

    if not valid:
        # 벽 포인트 부족 → 벽 쪽으로 천천히 회전하며 탐색
        if fm < THRESH_STOP:
            return (0.08, adir * 1.1)
        return (WALL_V * 0.6, sign * WALL_LOST_W)

    # ── Stanley 조향 계산 ─────────────────────────────────────────────
    # 1) 헤딩 오차: 벽과 평행하지 않은 정도
    #    wall_angle > 0 → 앞머리가 벽으로 기울어짐 → 벽에서 멀어지도록 조향
    #    부호: 왼쪽 벽이면 앞머리가 벽으로 기울면 오른쪽(음수)으로 조향
    heading_term = -sign * WALL_KH * wall_angle

    # 2) 거리 오차: 목표 거리와의 차이
    #    dist_error > 0 → 벽에서 멀어짐 → 벽 쪽(sign 방향)으로 조향
    dist_error = wall_dist - WALL_TARGET
    v_safe     = max(abs(current_v), 0.05)   # 0 나눔 방지
    dist_term  = sign * float(np.arctan(WALL_KD * dist_error / v_safe))

    w_raw = heading_term + dist_term

    # 3) Lookahead 오버슈트 억제
    #    현재 w_raw로 LOOKAHEAD_TIME 후 로봇이 얼마나 옆으로 이동할지 추정
    #    그만큼 거리 오차가 줄어드므로 현재 조향을 미리 줄여 오버슈트 방지
    #    횡이동량 ≈ v * LOOKAHEAD_TIME * sin(w_raw * LOOKAHEAD_TIME / 2)
    lateral_pred = current_v * LOOKAHEAD_TIME * np.sin(
        np.clip(w_raw * LOOKAHEAD_TIME * 0.5, -1.0, 1.0)
    )
    # 예측 이동이 거리 오차를 상쇄하는 방향이면 조향을 비례해서 줄임
    if abs(dist_error) > 0.5:
        correction = float(np.clip(lateral_pred / (dist_error + 1e-6), 0.0, 0.8))
        w_raw = w_raw * (1.0 - correction * 0.5)

    # ── 전방 장애물 접근 시 속도/조향 블렌딩 ────────────────────────
    if fm < THRESH_SLOW:
        blend = float(np.clip(
            (THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0
        ))
        w_raw = (1.0 - blend) * w_raw + blend * adir * 0.5
        v     = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V

    w = float(np.clip(w_raw, -MAX_W, MAX_W))
    return (v, w), (wall_dist, wall_angle, valid)   # 디버그용 튜플 함께 반환

def wall_follow_cmd(scan, fm, adir, follow_side, current_v=None):
    """wall_follow 래퍼 — (v, w) 만 반환, 디버그 정보는 버림"""
    result = wall_follow(scan, fm, adir, follow_side, current_v)
    if isinstance(result[0], tuple):
        return result[0]
    return result

def wall_follow_debug(scan, fm, adir, follow_side, current_v=None):
    """wall_follow 래퍼 — (v, w), (wall_dist, wall_angle, valid) 반환"""
    result = wall_follow(scan, fm, adir, follow_side, current_v)
    if isinstance(result[0], tuple):
        return result[0], result[1]
    return result, (0.0, 0.0, False)


# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]), "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]),    "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 95], [138, 207, 246]),   "hsv2": None,
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
APPROACH_V     = 0.13
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8

WALL_SCAN_DIST      = 150.0
MISSION_TIMEOUT_SEC = 10.0

# ── STATE ─────────────────────────────────────────────────────────────
mode            = "LIDAR"
mission_idx     = 0
detect_count    = 0
arrive_count    = 0
follow_side     = "L"
lidar_state     = "WALL_SEARCH"

park_state      = "TRACK"
last_seen_x     = 160
last_bottom_y   = 0
park_t          = None
search_t        = None
mission_start_t = time.time()
hop_start_t     = None
last_cmd        = (0.0, 0.0)

# 디버그용
dbg_wall_dist  = 0.0
dbg_wall_angle = 0.0
dbg_valid      = False

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

        # ── 10초 타임아웃 감시 ────────────────────────────────────────
        is_searching = (mode == "LIDAR") or (
            mode == "PARK" and park_state in
            ["WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW", "SEARCH"])

        if is_searching:
            if time.time() - mission_start_t > MISSION_TIMEOUT_SEC:
                print(f"🚨 [{target}] {MISSION_TIMEOUT_SEC}초 경과! 도약합니다.")
                mode        = "PARK"
                park_state  = "SAFE_HOP"
                hop_start_t = time.time()
                detect_count = 0
                continue
        elif park_state not in ["SAFE_HOP"]:
            mission_start_t = time.time()

        # ── 색상 검출 ─────────────────────────────────────────────────
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

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (arrive_x1 <= cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # ══ LIDAR 모드 ════════════════════════════════════════════════
        if mode == "LIDAR":
            if found: detect_count += 1
            else:     detect_count  = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode       = "PARK"
                park_state = "TRACK"
                continue

            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    follow_side = "L"
                    lidar_state = "WALL_APPROACH"
                else:
                    send_cmd(0.0, WALL_SEARCH_W)

            elif lidar_state == "WALL_APPROACH":
                wall_dist, _, valid = estimate_wall(scan, follow_side)
                if valid and wall_dist <= WALL_TARGET * 1.4:
                    lidar_state = "WALL_FOLLOW"
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP:        send_cmd(0.08, adir * 1.0)
                    elif fm < THRESH_TURN:      send_cmd(WALL_APPROACH_V * 0.6, adir * 0.7)
                    else:                       send_cmd(WALL_APPROACH_V, sign * 0.3)

            elif lidar_state == "WALL_FOLLOW":
                (v, w), (dbg_wall_dist, dbg_wall_angle, dbg_valid) = \
                    wall_follow_debug(scan, fm, adir, follow_side, WALL_V)
                send_cmd(v, w)

        # ══ PARK 모드 ═════════════════════════════════════════════════
        elif mode == "PARK":

            # ── SAFE_HOP ─────────────────────────────────────────────
            if park_state == "SAFE_HOP":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        print(f"[{target}] 도약 중 타겟 발견!")
                        detect_count    = 0
                        park_state      = "TRACK"
                        mission_start_t = time.time()
                        continue
                else:
                    detect_count = 0

                elapsed_hop = time.time() - hop_start_t
                if elapsed_hop < 2.0:
                    send_cmd(0.0, 1.2)
                    cv2.putText(frame, "HOP: TURNING", (10, 45), 0, 0.5, (0, 0, 255), 2)
                else:
                    if fm > 130.0:
                        send_cmd(0.0, 1.0)
                        cv2.putText(frame, "HOP: SCANNING", (10, 45), 0, 0.5, (0, 150, 255), 2)
                    elif fm > 50.0:
                        send_cmd(WALL_V, 0.0)
                        cv2.putText(frame, f"HOP: MOVING ({fm:.0f}cm)", (10, 45), 0, 0.5, (0, 255, 0), 2)
                    else:
                        print("   → 새 장애물 도착! 벽 탐색 재시작")
                        park_state      = "WALL_APPROACH"
                        mission_start_t = time.time()
                        continue

                cv2.imshow("f", frame); cv2.waitKey(1); continue

            # ── FORWARD ──────────────────────────────────────────────
            if park_state == "FORWARD":
                if time.time() - park_t >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t     = time.time()
                else:
                    send_cmd(*last_cmd)

            # ── PARKING ──────────────────────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                if time.time() - park_t >= PARK_SEC:
                    mission_idx  += 1
                    arrive_count  = 0
                    detect_count  = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"
                    continue

            # ── WALL_SEARCH ──────────────────────────────────────────
            elif park_state == "WALL_SEARCH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else:
                    detect_count = 0

                if fm < WALL_SCAN_DIST:
                    park_state = "WALL_APPROACH"; continue
                send_cmd(0.0, WALL_SEARCH_W)

            # ── WALL_APPROACH ─────────────────────────────────────────
            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else:
                    detect_count = 0

                wall_dist, _, valid = estimate_wall(scan, follow_side)
                if valid and wall_dist <= WALL_TARGET * 1.4:
                    park_state = "WALL_FOLLOW"; continue

                sign = 1 if follow_side == "L" else -1
                if fm < THRESH_STOP:        v, w = 0.08, adir * 1.0
                elif fm < THRESH_TURN:      v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else:                       v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)

            # ── WALL_FOLLOW ───────────────────────────────────────────
            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else:
                    detect_count = 0

                (v, w), (dbg_wall_dist, dbg_wall_angle, dbg_valid) = \
                    wall_follow_debug(scan, fm, adir, follow_side, WALL_V)
                send_cmd(v, w)

            # ── TRACK ─────────────────────────────────────────────────
            elif park_state == "TRACK":
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state   = "FORWARD"
                    park_t       = time.time()
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
                    v, w = reduced_v, cam_w(err_x)
                else:
                    w_cam = cam_w(err_x); w_lid = adir * 0.7
                    if fm < THRESH_STOP:        v, w = 0.09, w_lid
                    elif fm < THRESH_TURN:      v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                    else:                       v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                last_cmd = (v, w)
                send_cmd(v, w)

            # ── SEARCH ───────────────────────────────────────────────
            elif park_state == "SEARCH":
                if search_t is None:
                    search_t = time.time(); arrive_count = 0
                if time.time() - search_t > 5.0:
                    park_state = "WALL_SEARCH"; search_t = None
                else:
                    send_cmd(0.0, -1.0 if last_seen_x > cx_mid else 1.0)

        # ── 디버그 오버레이 ───────────────────────────────────────────
        search_time_left = MISSION_TIMEOUT_SEC - (time.time() - mission_start_t)
        if is_searching and search_time_left > 0:
            cv2.putText(frame, f"Timeout: {search_time_left:.1f}s | Side:{follow_side}",
                        (10, 20), 0, 0.50, (255, 150, 0), 2)

        in_wall_follow = (
            (mode == "LIDAR" and lidar_state == "WALL_FOLLOW") or
            (mode == "PARK"  and park_state  == "WALL_FOLLOW")
        )
        if in_wall_follow:
            color_v = (0, 255, 100) if dbg_valid else (0, 80, 255)
            cv2.putText(frame,
                f"WF dist={dbg_wall_dist:.1f}cm "
                f"ang={np.rad2deg(dbg_wall_angle):.1f}deg "
                f"{'OK' if dbg_valid else 'NO'}",
                (10, H - 10), 0, 0.42, color_v, 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
