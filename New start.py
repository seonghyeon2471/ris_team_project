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

# ── LIDAR ─────────────────────────────────────────────────────────────
EMA_ALPHA    = 0.35
MEDIAN_K     = 2
FRONT_RANGE  = 90   # 정면 감지 범위 (±90도 = 총 180도)
THRESH_SLOW  = 55.0
THRESH_TURN  = 35.0
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

def wall_follow(scan, fm, adir, follow_side):
    sd          = side_dist(scan, follow_side)
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1

    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

    if sd > WALL_TARGET * 2.0:
        return (WALL_V * 0.7, sign * WALL_LOST_W)

    err = sd - WALL_TARGET
    w   = sign * WALL_KP * err
    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V
    w = float(np.clip(w, -0.9, 0.9))
    return (v, w)

# ── LIDAR 우선순위 판단 함수 ──────────────────────────────────────────
# 반환값: (override, v, w, label)
#   override=True  → LIDAR 위험, 즉시 이 v/w 사용 (카메라 무시)
#   override=False → 안전, 카메라 로직 진행 가능
def lidar_priority(scan, fm, adir):
    """
    LIDAR를 최우선으로 체크하여 장애물 회피가 필요하면 True + 회피 명령 반환.
    THRESH_STOP  이하 → 긴급 정지 + 강한 회전
    THRESH_TURN  이하 → 감속 + 회전 (카메라 완전 무시)
    THRESH_SLOW  이하 → 감속 + 카메라/LIDAR 블렌딩 신호 (override=False, 호출자가 블렌딩)
    """
    if fm < THRESH_STOP:
        # 긴급: 거의 정지, 강한 회전 회피
        v = 0.06
        w = adir * 1.2
        return True, v, w, f"LIDAR-EMRG fm:{fm:.0f}"

    if fm < THRESH_TURN:
        # 위험: 저속 + 확실한 LIDAR 회피 방향
        v = 0.10
        w = adir * 0.9
        return True, v, w, f"LIDAR-AVOID fm:{fm:.0f}"

    # THRESH_SLOW 이하는 override 없이 호출자가 블렌딩 처리
    return False, 0.0, 0.0, ""

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
KP_ROT         = 0.040
W_MIN          = 0.35   # 0.25  → 0.35  (최소 회전속도 ↑)
APPROACH_V     = 0.22   # 0.17  → 0.22  (색상 추적 전진속도 ↑)
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.10
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8

# wall-following
WALL_TARGET    = 30.0
WALL_SCAN_DIST = 130.0
WALL_APPROACH_V = 0.26  # 0.20 → 0.26 (벽 접근속도 ↑)
WALL_KP        = 0.016  # 0.012 → 0.016 (벽 추종 보정 감도 ↑)
WALL_V         = 0.28   # 0.22 → 0.28  (wall-following 전진속도 ↑)
WALL_TURN_V    = 0.13   # 0.10 → 0.13  (장애물 회피 중 전진속도 ↑)
WALL_LOST_W    = 0.55   # 0.4  → 0.55  (벽 놓쳤을 때 회전속도 ↑)
WALL_SEARCH_W  = 1.6    # 1.4  → 1.6   (제자리 탐색 회전속도 최대)
WALL_CONFIRM_SEC = 0.15 # fm 감지 후 정지하고 L/R 재측정하는 대기시간

# 동일 장애물 주위로 계속 도는 것을 방지
WALL_LOOP_LIMIT = 2                          # 같은 장애물 기준 허용 최대 바퀴 수
WALL_LOOP_RAD   = WALL_LOOP_LIMIT * 2 * math.pi  # 누적 회전각 임계값(rad) = 2바퀴
ESCAPE_SEC      = 0.6                        # 탈출 기동 지속 시간
ESCAPE_V        = 0.22                       # 탈출 기동 전진속도

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
wall_confirm_t = None
wall_heading_accum = 0.0
escape_t       = None
last_cmd      = (0.0, 0.0)

print(f"START | MISSION: {MISSION}")
_prev_t = time.time()

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret: continue

        now_t = time.time()
        dt = min(now_t - _prev_t, 0.5)   # 멈춤/지연 시 과도한 누적 방지
        _prev_t = now_t

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

        # ══════════════════════════════════════════════════════════════
        # ★ LIDAR 우선순위 체크 — 아래 상태는 override 제외
        #   FORWARD/PARKING : 목표 도달 후 처리
        #   WALL_SEARCH     : 제자리 회전이 목적 — fm이 낮아도 방해하면 안 됨
        #   WALL_SEARCH_CONFIRM : 정지 후 L/R 재측정 중 — 방해하면 안 됨
        # ══════════════════════════════════════════════════════════════
        _skip_states = {"FORWARD", "PARKING", "WALL_SEARCH", "WALL_SEARCH_CONFIRM"}
        _cur_state   = lidar_state if mode == "LIDAR" else park_state
        lidar_override, lo_v, lo_w, lo_label = lidar_priority(scan, fm, adir)

        if lidar_override and _cur_state not in _skip_states:
            send_cmd(lo_v, lo_w)
            cv2.putText(frame, lo_label, (10, 65), 0, 0.55, (0, 0, 255), 2)
            # detect_count는 초기화하지 않음 (장애물 회피 후 다시 이어서 감지)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27: break
            continue   # ← 이 아래 카메라/wall 로직 전부 건너뜀

        # ══ LIDAR 모드 ════════════════════════════════════════════════
        if mode == "LIDAR":
            # 색상 감지 확인 (LIDAR 장애물 없을 때만 여기 도달)
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

            # ── A. 벽 탐색 중 제자리 회전 (WALL_SEARCH) ──────────
            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    # 회전 중 순간 측정값은 진입 방향(edge)에 편향되므로
                    # 바로 판단하지 않고 일단 정지 → CONFIRM 상태에서 재측정
                    stop_robot()
                    wall_confirm_t = time.time()
                    lidar_state = "WALL_SEARCH_CONFIRM"
                    print(f"[LIDAR] 벽 감지 fm:{fm:.0f}cm → 정지 후 L/R 재측정")
                else:
                    send_cmd(0.0, WALL_SEARCH_W)
                    cv2.putText(frame, f"LIDAR-WALL-SEARCH fm:{fm:.0f}",
                                (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── A-2. 정지 후 L/R 재측정 (WALL_SEARCH_CONFIRM) ────
            elif lidar_state == "WALL_SEARCH_CONFIRM":
                stop_robot()
                if fm >= WALL_SCAN_DIST:
                    # 정지하는 사이 벽이 더 멀어짐(오감지) → 다시 탐색 회전
                    lidar_state = "WALL_SEARCH"
                elif time.time() - wall_confirm_t >= WALL_CONFIRM_SEC:
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    lidar_state = "WALL_APPROACH"
                    print(f"[LIDAR] 재측정 완료 L:{l:.0f} R:{r:.0f} → follow_side={follow_side}")
                else:
                    cv2.putText(frame, f"LIDAR-WALL-CONFIRM fm:{fm:.0f}",
                                (10, 25), 0, 0.5, (0, 255, 255), 1)

            # ── B. 벽으로 접근 중 (WALL_APPROACH) ─────────────────
            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    lidar_state = "WALL_FOLLOW"
                    wall_heading_accum = 0.0
                    print(f"[LIDAR] 벽 도달 {follow_side}:{sd:.0f}cm → wall-following 시작")
                else:
                    sign = 1 if follow_side == "L" else -1
                    # THRESH_STOP/TURN은 위에서 override로 처리됨
                    # 여기선 THRESH_SLOW 구간만 블렌딩
                    if fm < THRESH_SLOW:
                        blend = float(np.clip(
                            (THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
                        v = WALL_APPROACH_V * (1.0 - 0.5 * blend)
                        w = (1 - blend) * (sign * 0.3) + blend * (adir * 0.6)
                    else:
                        v, w = WALL_APPROACH_V, sign * 0.3
                    send_cmd(v, w)
                    cv2.putText(frame, f"LIDAR-WALL-APPROACH {follow_side}:{sd:.0f}cm",
                                (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── C. wall-following으로 탐색 (WALL_FOLLOW) ─────────
            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                wall_heading_accum += w * dt
                if abs(wall_heading_accum) >= WALL_LOOP_RAD:
                    print(f"[LIDAR] 동일 장애물 {WALL_LOOP_LIMIT}바퀴 회전 감지 "
                          f"({math.degrees(wall_heading_accum):.0f}°) → 탈출 후 다른 장애물 탐색")
                    lidar_state = "WALL_ESCAPE"
                    escape_t = time.time()
                    wall_heading_accum = 0.0
                    continue
                send_cmd(v, w)
                sd_disp = side_dist(scan, follow_side)
                cv2.putText(frame, f"LIDAR-WALL-FOLLOW {follow_side}:{sd_disp:.0f}cm "
                            f"loop:{math.degrees(wall_heading_accum):.0f}deg",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── D. 동일 장애물 탈출 기동 (WALL_ESCAPE) ────────────
            elif lidar_state == "WALL_ESCAPE":
                sign = 1 if follow_side == "L" else -1
                send_cmd(ESCAPE_V, -sign * 0.5)   # 추종 방향과 반대로 틀며 전진 → 거리 확보
                if time.time() - escape_t >= ESCAPE_SEC:
                    lidar_state = "WALL_SEARCH"
                    print("[LIDAR] 탈출 기동 완료 → 새 장애물 탐색 재시작")
                cv2.putText(frame, "LIDAR-WALL-ESCAPE", (10, 25), 0, 0.5, (255, 0, 255), 1)

            cv2.putText(frame, "MODE: LIDAR", (10, 45), 0, 0.5, (255, 255, 255), 1)

        # ══ PARK 모드 ════════════════════════════════════════════════
        elif mode == "PARK":

            # ── 1. 도착 후 전진 중 (FORWARD) ──────────────────────
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

            # ── 2. 정차 중 (PARKING) ───────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"
                        print(f"다음 미션 [{MISSION[mission_idx]}] wall-following 탐색 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 3-A. 벽 탐색 중 제자리 회전 (WALL_SEARCH) ────────
            elif park_state == "WALL_SEARCH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] 탐색 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                if fm < WALL_SCAN_DIST:
                    # 회전 중 순간 측정값은 진입 방향(edge)에 편향되므로
                    # 바로 판단하지 않고 일단 정지 → CONFIRM 상태에서 재측정
                    stop_robot()
                    wall_confirm_t = time.time()
                    park_state = "WALL_SEARCH_CONFIRM"
                    print(f"벽 감지 fm:{fm:.0f}cm → 정지 후 L/R 재측정")
                    continue

                send_cmd(0.0, WALL_SEARCH_W)
                cv2.putText(frame, f"WALL-SEARCH [{target}] 회전 중 fm:{fm:.0f}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── 3-A-2. 정지 후 L/R 재측정 (WALL_SEARCH_CONFIRM) ──
            elif park_state == "WALL_SEARCH_CONFIRM":
                stop_robot()
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] 재측정 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                if fm >= WALL_SCAN_DIST:
                    park_state = "WALL_SEARCH"
                elif time.time() - wall_confirm_t >= WALL_CONFIRM_SEC:
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    park_state = "WALL_APPROACH"
                    print(f"재측정 완료 L:{l:.0f} R:{r:.0f} → follow_side={follow_side}")
                else:
                    cv2.putText(frame, f"WALL-CONFIRM [{target}] fm:{fm:.0f}",
                                (10, 25), 0, 0.5, (0, 255, 255), 1)

            # ── 3-B. 벽으로 접근 중 (WALL_APPROACH) ───────────────
            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] 접근 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    park_state = "WALL_FOLLOW"
                    wall_heading_accum = 0.0
                    print(f"벽 도달 {follow_side}:{sd:.0f}cm → wall-following 시작")
                    continue

                sign = 1 if follow_side == "L" else -1
                # THRESH_SLOW 구간 블렌딩 (STOP/TURN은 위에서 override)
                if fm < THRESH_SLOW:
                    blend = float(np.clip(
                        (THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
                    v = WALL_APPROACH_V * (1.0 - 0.5 * blend)
                    w = (1 - blend) * (sign * 0.3) + blend * (adir * 0.6)
                else:
                    v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)
                cv2.putText(frame, f"WALL-APPROACH [{target}] {follow_side}:{sd:.0f}cm",
                            (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── 3-C. wall-following으로 탐색 (WALL_FOLLOW) ─────────
            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] wall-following 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side)
                wall_heading_accum += w * dt
                if abs(wall_heading_accum) >= WALL_LOOP_RAD:
                    print(f"[{target}] 동일 장애물 {WALL_LOOP_LIMIT}바퀴 회전 감지 "
                          f"({math.degrees(wall_heading_accum):.0f}°) → 탈출 후 다른 장애물 탐색")
                    park_state = "WALL_ESCAPE"
                    escape_t = time.time()
                    wall_heading_accum = 0.0
                    continue
                send_cmd(v, w)
                sd_disp = side_dist(scan, follow_side)
                cv2.putText(frame, f"WALL-FOLLOW [{target}] {follow_side}:{sd_disp:.0f}cm "
                            f"loop:{math.degrees(wall_heading_accum):.0f}deg",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── 3-D. 동일 장애물 탈출 기동 (WALL_ESCAPE) ───────────
            elif park_state == "WALL_ESCAPE":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] 탈출 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                sign = 1 if follow_side == "L" else -1
                send_cmd(ESCAPE_V, -sign * 0.5)   # 추종 방향과 반대로 틀며 전진 → 거리 확보
                if time.time() - escape_t >= ESCAPE_SEC:
                    park_state = "WALL_SEARCH"
                    print(f"[{target}] 탈출 기동 완료 → 새 장애물 탐색 재시작")
                cv2.putText(frame, f"WALL-ESCAPE [{target}]", (10, 25), 0, 0.5, (255, 0, 255), 1)

            # ── 4. 객체 추적 중 (TRACK) ────────────────────────────
            elif found:
                park_state = "TRACK"
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    print(f"[{target}] centroid {ARRIVE_CONFIRM}프레임 확정 → {ARRIVE_FORWARD_SEC}초 전진")
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

                    # ★ THRESH_SLOW 구간: 카메라와 LIDAR 블렌딩
                    #   (THRESH_STOP/TURN은 루프 상단 override에서 이미 처리)
                    if fm < THRESH_SLOW:
                        blend = float(np.clip(
                            (THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
                        w_cam = cam_w(err_x)
                        w_lid = adir * 0.7
                        # blend 높을수록 LIDAR 비중 증가
                        w = (1 - blend) * w_cam + blend * w_lid
                        v = reduced_v * (1.0 - 0.5 * blend)
                    else:
                        v = reduced_v
                        w = cam_w(err_x)

                    last_cmd = (v, w)
                    send_cmd(v, w)

                cv2.putText(frame, f"TRACKING: {target} fm:{fm:.0f}", (10, 25), 0, 0.6, draw, 1)

            # ── 5. 객체 놓침 (SEARCH) ──────────────────────────────
            else:
                park_state = "SEARCH"
                arrive_count = 0
                w = (-1.0 if last_seen_x > cx_mid else 1.0)
                send_cmd(0.0, w)
                cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        # fm 표시
        color_fm = (0, 255, 0) if fm > THRESH_SLOW else (0, 165, 255) if fm > THRESH_TURN else (0, 0, 255)
        cv2.putText(frame, f"fm:{fm:.0f}cm", (W - 90, 20), 0, 0.5, color_fm, 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
