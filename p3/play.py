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
FRONT_RANGE  = 90
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

# 어느 쪽 벽을 추종할지 결정 (타겟이 보이면 타겟 쪽, 안 보이면 라이다 기준 더 넓게 트인 쪽)
# WALL_SEARCH -> WALL_APPROACH로 넘어가는 순간에만 호출, 추종 중에는 절대 다시 호출하지 않음
def decide_follow_side(adir, found, cx_obj, cx_mid):
    if found and cx_obj >= 0:
        return "L" if cx_obj < cx_mid else "R"
    return "R" if adir == 1 else "L"

def side_dist(scan, side):
    if side == "L":
        idx = np.arange(265, 296) % 360
    else:
        idx = np.arange(65, 96) % 360
    return float(np.min(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# 320x240 카메라 기준 cx_obj(색지 중심 x좌표) -> LiDAR 각도(정면=0 기준 오프셋)로 변환
def cx_to_lidar_angle(cx_obj, cx_mid, frame_w=320):
    if cx_obj < 0:
        return 0.0
    ratio = (cx_obj - cx_mid) / (frame_w / 2.0)
    ratio = float(np.clip(ratio, -1.0, 1.0))
    return CAM_TO_LIDAR_SIGN * ratio * (CAMERA_HFOV / 2.0)

# 색지 방향(target_angle, 정면 기준 오프셋) ±TARGET_CHECK_HALFWIDTH 범위의 LiDAR 최소 거리
def obstacle_on_target(scan, target_angle):
    center = int(round(target_angle)) % 360
    idx = np.arange(center - TARGET_CHECK_HALFWIDTH,
                     center + TARGET_CHECK_HALFWIDTH + 1) % 360
    return float(np.min(scan[idx]))

def wall_follow(scan, fm, adir, follow_side):
    sd          = side_dist(scan, follow_side)
    left_close  = side_min(scan, 240, 300)
    right_close = side_min(scan, 60, 120)
    sign = -1 if follow_side == "L" else 1

    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    if left_close < SIDE_STOP:
        return (WALL_V * 0.7,  0.7)
    if right_close < SIDE_STOP:
        return (WALL_V * 0.7, -0.7)

    if left_close < BOTTLENECK_ENTER and right_close < BOTTLENECK_ENTER:
        err_center = left_close - right_close
        w_center = float(np.clip(-CENTER_KP * err_center, -0.9, 0.9))
        return (BOTTLENECK_V, w_center)

    if sd > WALL_TARGET * 2.0:
        return (0.05, sign * WALL_LOST_W)

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

# ── MOTOR ─────────────────────────────────────────────────────────────
# 마지막으로 실제 전송된 (v, w)를 기록 — Watchdog(P1)이 "최근 얼마나 회전했는지"를
#   판단하는 데 사용. send_cmd를 거치는 모든 상태에서 동일하게 누적되므로 상태별로 따로
#   계측 코드를 넣지 않아도 된다.
last_sent_v, last_sent_w = 0.0, 0.0

def send_cmd(v, w):
    global last_sent_v, last_sent_w
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    last_sent_v, last_sent_w = float(v), float(w)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]), "hsv2": None, "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]),    "hsv2": None, "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 95], [138, 207, 246]),   "hsv2": None, "bgr":  ([40,  0,   0], [255, 220, 220]), "draw": (255, 80, 0)},
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

ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8

WALL_TARGET    = 10.0
SIDE_STOP      = 6.0
BOTTLENECK_ENTER = 25.0
CENTER_KP      = 0.030
BOTTLENECK_V   = 0.15
WALL_SCAN_DIST = 150.0
WALL_APPROACH_V = 0.20
WALL_KP        = 0.024
WALL_V         = 0.22
WALL_TURN_V    = 0.10
WALL_LOST_W    = 1.3
WALL_SEARCH_W  = 1.1

# P1: 같은 색을 이 시간 넘게 못 찾으면 LOOP_ESCAPE (시간 기반 트리거)
# ★ 변경: 10.0 -> 6.0 (더 빨리 "맴돈다"고 판단하도록 단축)
MISSION_TIMEOUT_SEC = 6.0

TARGET_CLEAR_DIST      = 40.0
TARGET_CHECK_HALFWIDTH = 15
CAMERA_HFOV            = 60.0
CAM_TO_LIDAR_SIGN      = 1

# P1: 같은 자리를 맴도는 것(회전 누적)을 감지하기 위한 임계값/탈출 동작
# ★ 변경: 400도 -> 260도 (더 적게 돌아도 맴돈다고 판단 -> 더 빠른 인식)
FULL_LOOP_THRESH   = math.radians(260)
LOOP_ESCAPE_BACK_V = -0.12
LOOP_ESCAPE_SEC    = 0.6
# ★ 신규: 탈출 시 회전 단계의 길이/속도를 별도 상수로 분리 (기존엔 1.5초, WALL_SEARCH_W*1.2로 하드코딩)
LOOP_ESCAPE_TURN_SEC = 2.6                  # 1.5 -> 2.6초로 연장 (더 오래 회전)
LOOP_ESCAPE_TURN_W   = WALL_SEARCH_W * 1.6  # 1.2배 -> 1.6배로 증가 (더 빠르게/많이 회전)

# ── 우선순위(P1 > P2 > P3) ──────────────────────────────────────────────
# P0 비상정지   : 전방 < THRESH_STOP → wall_follow()/각 상태 내부에서 즉시 처리. 상태와 무관하게 항상 최우선.
# P1 Watchdog   : (회전 누적 > FULL_LOOP_THRESH) OR (시간 초과 > MISSION_TIMEOUT_SEC) → LOOP_ESCAPE로 강제 전환.
#                 탐색 상태(WALL_SEARCH/APPROACH/FOLLOW)에서만 감시하고, TRACK/FORWARD/PARKING처럼
#                 "진행 중"인 상태에서는 누적을 멈춘다(=watchdog 일시정지) → 목표를 쫓는 중에 오작동 개입 방지.
# P2 Target     : 색지가 카메라에서 DETECT_CONFIRM 프레임 연속 검출되면, 탐색/탈출 상태를 즉시 덮고 TRACK으로 전환.
# P3 Explore    : WALL_SEARCH → WALL_APPROACH → WALL_FOLLOW (기본 벽 추종 탐색, 항상 실행 가능한 fallback).
EXPLORE_STATES = ("WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW")
TARGET_INTERRUPTIBLE_STATES = EXPLORE_STATES + ("LOOP_ESCAPE",)

# ── STATE ─────────────────────────────────────────────────────────────
state         = "WALL_SEARCH"   # mode + lidar_state + park_state를 단일 상태로 통합 (중복 로직 제거)
mission_idx   = 0
detect_count  = 0
arrive_count  = 0
follow_side   = "L"   # 초기값일 뿐, WALL_SEARCH -> WALL_APPROACH 전환 시 decide_follow_side()로 항상 재결정됨

park_t        = None
mission_start_t = time.time()
last_cmd      = (0.0, 0.0)

escape_t      = None   # LOOP_ESCAPE 진입 시각
loop_accum    = 0.0    # P1 Watchdog: 탐색 상태에서 누적된 |w|*dt (대략적인 "누적 회전각", 단위: rad)
loop_t_prev   = time.time()

was_found     = False  # 색지가 "보이다가 사라지는 순간"을 감지하기 위한 직전 프레임 found 상태

print(f"START | MISSION: {MISSION}")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        now_t = time.time()
        dt = now_t - loop_t_prev
        loop_t_prev = now_t

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
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        cv2.putText(frame, f"TARGET: {target.upper()}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
        cv2.putText(frame, f"STATE:{state} LOOP:{math.degrees(loop_accum):.0f}/{math.degrees(FULL_LOOP_THRESH):.0f}deg",
                    (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # ══ P1: Watchdog (회전 누적 / 타임아웃) ════════════════════════════
        # 탐색 상태에서만 누적하고, TRACK처럼 목표를 향해 "진행 중"일 때는 누적을 멈춘다.
        if state in EXPLORE_STATES:
            loop_accum += abs(last_sent_w) * dt
            if (now_t - mission_start_t > MISSION_TIMEOUT_SEC) or (loop_accum > FULL_LOOP_THRESH):
                state = "LOOP_ESCAPE"
                escape_t = now_t
                loop_accum = 0.0
                detect_count = 0
                cv2.imshow("f", frame); cv2.waitKey(1); continue
        elif state != "LOOP_ESCAPE":
            # TRACK/FORWARD/PARKING 등 "진행 중" 상태에서는 watchdog 시계/누적을 계속 리셋해 둔다.
            mission_start_t = now_t
            loop_accum = 0.0

        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big    = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        cx_obj, cy_obj = -1, -1
        if found:
            M_mom = cv2.moments(big)
            if M_mom["m00"] > 0:
                cx_obj = int(M_mom["m10"] / M_mom["m00"])
                cy_obj = int(M_mom["m01"] / M_mom["m00"])
            bx, by_top, bw, bh = cv2.boundingRect(big)
            last_seen_x = cx_obj
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # 색지가 "보이다가 사라지는 순간" 감지 -> 그 직전 위치 기준으로 follow_side 즉시 갱신
        if was_found and not found and 'last_seen_x' in dir():
            follow_side = "L" if last_seen_x < cx_mid else "R"
        was_found = found

        # ══ P2: Target — 색지 발견 시 탐색/탈출 상태를 즉시 덮고 TRACK으로 ═══════════
        if state in TARGET_INTERRUPTIBLE_STATES:
            if found:
                detect_count += 1
                if detect_count >= DETECT_CONFIRM:
                    detect_count = 0
                    state = "TRACK"
            else:
                detect_count = 0

        # ══ P3 + 상태별 디스패치 ═══════════════════════════════════════════
        if state == "WALL_SEARCH":
            if fm < WALL_SCAN_DIST:
                follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                state = "WALL_APPROACH"
            else:
                send_cmd(0.0, WALL_SEARCH_W)

        elif state == "WALL_APPROACH":
            sd = side_dist(scan, follow_side)
            if sd <= WALL_TARGET * 1.3:
                state = "WALL_FOLLOW"
            else:
                sign = -1 if follow_side == "L" else 1
                if fm < THRESH_STOP: send_cmd(0.08, adir * 1.0)
                elif fm < THRESH_TURN: send_cmd(WALL_APPROACH_V * 0.6, adir * 0.7)
                else: send_cmd(WALL_APPROACH_V, sign * 0.3)

        elif state == "WALL_FOLLOW":
            v, w = wall_follow(scan, fm, adir, follow_side)
            send_cmd(v, w)

        elif state == "LOOP_ESCAPE":
            # 1) 잠시 후진 → 2) 추가 회전으로 새 방향 모색(연장됨) → 3) WALL_SEARCH로 복귀
            elapsed_esc = now_t - escape_t
            if elapsed_esc < LOOP_ESCAPE_SEC:
                send_cmd(LOOP_ESCAPE_BACK_V, 0.0)
            elif elapsed_esc < LOOP_ESCAPE_SEC + LOOP_ESCAPE_TURN_SEC:
                send_cmd(0.0, LOOP_ESCAPE_TURN_W)
            else:
                state = "WALL_SEARCH"
                mission_start_t = now_t
                loop_accum = 0.0

        elif state == "TRACK":
            if found:
                target_angle = cx_to_lidar_angle(cx_obj, cx_mid, W)
                target_clear = obstacle_on_target(scan, target_angle)
                if target_clear < TARGET_CLEAR_DIST:
                    # 색지 방향이 막혀 있음 -> TRACK 중단, 장애물 우회로 전환
                    arrive_count = 0
                    follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                    state = "WALL_APPROACH"
                    cv2.imshow("f", frame); cv2.waitKey(1); continue

            arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

            if arrive_count >= ARRIVE_CONFIRM:
                arrive_count = 0
                state = "FORWARD"
                park_t = now_t
                send_cmd(ARRIVE_FORWARD_V, 0.0)
            else:
                err_x = cx_obj - cx_mid
                err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                reduced_v = APPROACH_V * (1.0 - err_ratio)

                def cam_w(ex):
                    raw = -KP_ROT * ex
                    if abs(raw) < W_MIN and ex != 0: return -W_MIN if ex > 0 else W_MIN
                    return raw

                if fm >= THRESH_SLOW: v, w = reduced_v, cam_w(err_x)
                else:
                    w_cam = cam_w(err_x); w_lid = adir * 0.7
                    if fm < THRESH_STOP: v, w = 0.09, w_lid
                    elif fm < THRESH_TURN: v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                    else: v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                last_cmd = (v, w); send_cmd(v, w)

        elif state == "FORWARD":
            elapsed = now_t - park_t
            if elapsed >= ARRIVE_FORWARD_SEC:
                stop_robot()
                state = "PARKING"
                park_t = now_t
            else:
                send_cmd(*last_cmd)

        elif state == "PARKING":
            stop_robot()
            if now_t - park_t >= PARK_SEC:
                mission_idx += 1
                arrive_count = 0
                detect_count = 0
                if mission_idx < len(MISSION):
                    state = "WALL_SEARCH"
                    mission_start_t = now_t
                    loop_accum = 0.0

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
