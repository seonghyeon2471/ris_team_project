import cv2
import serial
import numpy as np
import time
import threading

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# ── CAMERA ────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FPS, 60)        # [PATCH] FPS 상향 시도 (카메라가 지원하는 한도까지)
time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR 스캔 처리 ───────────────────────────────────────────────────
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 90
THRESH_SLOW = 55.0
THRESH_TURN = 32.0
THRESH_STOP = 19.0

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
        if len(raw) != 5:
            continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 70:
            _ema(angle, dist_cm)
        if sf == 1:
            _median()
            with scan_lock:
                _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock:
        return _scan_pub.copy()

def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# ── 벽 직선 추정 (왼쪽 벽 고정) ───────────────────────────────────────
WALL_SCAN_START = 55
WALL_SCAN_END   = 125
WALL_MAX_DIST   = 80.0
WALL_MIN_POINTS = 5

def estimate_wall(scan):
    angles_deg = np.arange(WALL_SCAN_START, WALL_SCAN_END + 1)
    angles_rad = np.deg2rad(angles_deg)
    dists = scan[angles_deg % 360].astype(np.float32)

    valid_mask = (dists > 2.0) & (dists < WALL_MAX_DIST)
    if np.sum(valid_mask) < WALL_MIN_POINTS:
        return 0.0, 0.0, False

    a = angles_rad[valid_mask]
    d = dists[valid_mask]
    px = d * np.cos(a)
    py = d * np.sin(a)

    A = np.vstack([px, np.ones(len(px))]).T
    try:
        m, b = np.linalg.lstsq(A, py, rcond=None)[0]
    except Exception:
        return 0.0, 0.0, False

    return abs(float(b)), float(np.arctan(m)), True

# ── 벽 추종 제어기 (왼쪽 벽 고정) ─────────────────────────────────────
WALL_TARGET     = 15.0
WALL_KH         = 0.60
WALL_KD         = 0.022
HEADING_CLAMP   = 0.45
DIST_CLAMP      = 0.45
WALL_V          = 0.22
WALL_TURN_V     = 0.10
WALL_APPROACH_V = 0.20
WALL_LOST_W     = 0.45
WALL_SEARCH_W   = 1.1
MAX_W           = 0.90
WALL_SCAN_DIST  = 150.0

# [PATCH] 벽 탐색을 제자리 회전 대신 직진 + 약한 회전(나선형)으로 변경
FIND_WALL_V     = 0.16   # 벽을 찾는 동안 전진 속도
FIND_WALL_W     = 0.45   # 벽을 찾는 동안 함께 줄 회전 속도 (방향 탐색용)

def wall_follow(scan, fm, adir):
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)

    if fm < THRESH_STOP:
        return 0.08, adir * 1.1
    if fm < THRESH_TURN:
        return WALL_TURN_V, adir * 0.85
    if left_close < THRESH_STOP:
        return WALL_V * 0.7, -0.7
    if right_close < THRESH_STOP:
        return WALL_V * 0.7, 0.7

    wall_dist, wall_angle, valid = estimate_wall(scan)
    if not valid:
        return WALL_V * 0.6, WALL_LOST_W

    heading_term = float(np.clip(-WALL_KH * wall_angle, -HEADING_CLAMP, HEADING_CLAMP))
    dist_term    = float(np.clip(WALL_KD * (wall_dist - WALL_TARGET), -DIST_CLAMP, DIST_CLAMP))
    w = float(np.clip(heading_term + dist_term, -MAX_W, MAX_W))
    return WALL_V, w

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv": ([169, 136, 114], [179, 220, 255]), "bgr": ([20, 20, 80], [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv": ([25, 60, 160],   [32, 161, 255]),  "bgr": ([0, 80, 80], [255, 255, 255]),  "draw": (0, 200, 255)},
    "blue":   {"hsv": ([96, 100, 95],   [138, 207, 246]), "bgr": ([40, 0, 0], [255, 220, 220]),   "draw": (255, 80, 0)},
}
MISSION = ["red", "yellow", "blue"]

def make_mask(frame, hsv, name):
    cfg = COLOR_CFG[name]
    m  = cv2.inRange(hsv, np.array(cfg["hsv"][0]), np.array(cfg["hsv"][1]))
    bm = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    return cv2.bitwise_and(m, bm)

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA           = 250
KP_ROT             = 0.012
W_MIN              = 0.12
DEADBAND_PX        = 8
CENTROID_EMA_ALPHA = 0.4
APPROACH_V         = 0.13
PARK_SEC           = 1.2
DETECT_CONFIRM     = 6
LOST_CONFIRM       = 8

ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 30
ARRIVE_FORWARD_SEC = 0.88
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8

# [PATCH] 상태 정체(stuck) 탈출 로직 파라미터
STUCK_TIMEOUT   = 6.0   # 한 상태에 이 시간(초) 이상 머무르면 탈출 시도
ESCAPE_W        = 1.2   # 탈출 시 각속도 크기
ESCAPE_TURN_SEC = None  # 아래에서 180도 기준으로 자동 계산

# [PATCH] 측면으로 색을 놓쳤을 때 마지막 방향으로 재탐색하는 파라미터
SIDE_LOST_V        = 0.10   # 재탐색 중 전진 속도 (살짝 전진하며 회전)
SIDE_LOST_W        = 1.0   # 재탐색 중 회전 각속도 크기
SIDE_LOST_TIMEOUT  = 2.0    # 이 시간(초) 동안 재탐색해도 못 찾으면 FIND_WALL로 폴백
SIDE_EDGE_MARGIN   = 40     # 화면 가장자리로부터 이 픽셀 이내에서 놓쳤을 때만 "측면 이탈"로 판단

def cam_w(ex):
    if abs(ex) < DEADBAND_PX:
        return 0.0
    raw = -KP_ROT * ex
    if abs(raw) < W_MIN:
        return -W_MIN if ex > 0 else W_MIN
    return raw

# [PATCH] 180도 회전에 필요한 시간 추정
# send_cmd에서 w는 np.clip(-1.6, 1.6) 범위지만 실제 각속도(rad/s) 단위가
# 시스템마다 다를 수 있어, 우선 "약 180도 = pi(rad)" 기준으로 ESCAPE_W를 각속도(rad/s)로 가정.
# 실제 로봇에서 각속도 단위가 다르면 ESCAPE_TURN_SEC만 직접 보정하면 됨.
ESCAPE_TURN_SEC = float(np.pi / ESCAPE_W)  # ≈ 2.62초 (필요시 실측 후 조정)

# ── STATE ─────────────────────────────────────────────────────────────
# FIND_WALL → APPROACH_WALL → FOLLOW_WALL → (색 검출 시) TRACK → FORWARD → PARKING
state        = "FIND_WALL"
mission_idx  = 0
detect_count = 0
arrive_count = 0
lost_count   = 0
park_t       = None
last_cmd     = (0.0, 0.0)
filtered_cx  = None

# [PATCH] 상태별 정체 감지용 타이머 / 탈출 상태
state_enter_t   = time.time()
escaping        = False
escape_t        = None
escape_dir      = 1
state_before_escape = None

# [PATCH] 측면 이탈 재탐색용 상태
last_seen_side  = 0     # -1: 화면 왼쪽으로 사라짐, +1: 오른쪽으로 사라짐, 0: 정보 없음
side_lost_t     = None  # 측면 재탐색 시작 시각

def enter_state(new_state):
    """상태 전환 시 정체 타이머 리셋을 위한 헬퍼"""
    global state, state_enter_t
    state = new_state
    state_enter_t = time.time()

print(f"START | MISSION: {MISSION}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame  = cv2.flip(frame, 1)
        H, W   = frame.shape[:2]
        cx_mid = W // 2
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan   = get_scan()
        fm     = front_min(scan)
        adir   = avoid_dir(scan)

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        # ── 색상 검출 ──────────────────────────────────────────────
        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big   = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        cx_obj, cy_obj = -1, -1
        if found:
            M = cv2.moments(big)
            if M["m00"] > 0:
                cx_obj = int(M["m10"] / M["m00"])
                cy_obj = int(M["m01"] / M["m00"])
            filtered_cx = float(cx_obj) if filtered_cx is None else \
                (1 - CENTROID_EMA_ALPHA) * filtered_cx + CENTROID_EMA_ALPHA * cx_obj
            cx_obj = int(filtered_cx)
            bx, by, bw, bh = cv2.boundingRect(big)
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), draw, 2)
        else:
            filtered_cx = None

        arrive_x1, arrive_x2 = cx_mid - ARRIVE_X_MARGIN, cx_mid + ARRIVE_X_MARGIN

        def in_arrive_zone():
            return arrive_x1 <= cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP

        # [PATCH] ── 정체(stuck) 탈출 체크 ───────────────────────────
        # FORWARD/PARKING은 원래 시간 기반으로 짧게 끝나는 상태라 탈출 로직에서 제외.
        # 그 외 상태(FIND_WALL/APPROACH_WALL/FOLLOW_WALL/TRACK)에서만 정체를 감시.
        if not escaping and state in ("FIND_WALL", "APPROACH_WALL", "FOLLOW_WALL", "TRACK"):
            if time.time() - state_enter_t >= STUCK_TIMEOUT:
                escaping = True
                escape_t = time.time()
                state_before_escape = state
                escape_dir = -adir if adir != 0 else 1  # 현재 빈 공간의 반대쪽(=막힌 방향 반대)으로 회전
                print(f"[ESCAPE] '{state_before_escape}' 상태 {STUCK_TIMEOUT}s 초과 → 반대방향 180도 회전 탈출")

        if escaping:
            if time.time() - escape_t < ESCAPE_TURN_SEC:
                send_cmd(0.0, escape_dir * ESCAPE_W)
                cv2.imshow("f", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue
            else:
                escaping = False
                detect_count = 0
                lost_count   = 0
                arrive_count = 0
                enter_state("FIND_WALL")
                cv2.imshow("f", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue

        # ── 상태 머신 ─────────────────────────────────────────────
        if state in ("FIND_WALL", "APPROACH_WALL", "FOLLOW_WALL"):
            if found:
                detect_count += 1
                if detect_count >= DETECT_CONFIRM:
                    detect_count = 0
                    enter_state("TRACK")
                    continue
            else:
                detect_count = 0

            if state == "FIND_WALL":
                if fm < WALL_SCAN_DIST:
                    enter_state("APPROACH_WALL")
                else:
                    # [PATCH] 제자리 회전 대신 전진하며 살짝 회전(나선형 탐색)
                    send_cmd(FIND_WALL_V, adir * FIND_WALL_W)

            elif state == "APPROACH_WALL":
                wall_dist, _, valid = estimate_wall(scan)
                if valid and wall_dist <= WALL_TARGET * 1.4:
                    enter_state("FOLLOW_WALL")
                elif fm < THRESH_STOP:
                    send_cmd(0.08, adir * 1.0)
                elif fm < THRESH_TURN:
                    send_cmd(WALL_APPROACH_V * 0.6, adir * 0.7)
                else:
                    send_cmd(WALL_APPROACH_V, 0.3)

            elif state == "FOLLOW_WALL":
                v, w = wall_follow(scan, fm, adir)
                send_cmd(v, w)

        elif state == "TRACK":
            if found:
                lost_count = 0
                side_lost_t = None  # [PATCH] 다시 찾았으니 재탐색 타이머 리셋

                # [PATCH] 화면 가장자리 근처에서 보이고 있다면 "이쪽으로 사라질 가능성"을 미리 기록
                if cx_obj <= SIDE_EDGE_MARGIN:
                    last_seen_side = -1   # 왼쪽 가장자리 쪽
                elif cx_obj >= (W - SIDE_EDGE_MARGIN):
                    last_seen_side = 1    # 오른쪽 가장자리 쪽
                # 중앙 근처에 있으면 last_seen_side는 갱신하지 않고 이전 값 유지

                arrive_count = arrive_count + 1 if in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    enter_state("FORWARD")
                    park_t = time.time()
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                    continue

                err_x = cx_obj - cx_mid
                err_ratio = min(abs(err_x) / cx_mid, 1.0)
                reduced_v = APPROACH_V * (1.0 - err_ratio)

                if fm >= THRESH_SLOW:
                    v, w = reduced_v, cam_w(err_x)
                else:
                    w_cam, w_lid = cam_w(err_x), adir * 0.7
                    if fm < THRESH_STOP:
                        v, w = 0.09, w_lid
                    elif fm < THRESH_TURN:
                        v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                    else:
                        v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                last_cmd = (v, w)
                send_cmd(v, w)
            else:
                lost_count += 1

                # [PATCH] 측면으로 사라진 적이 있으면(last_seen_side != 0) 그 방향으로 회전하며 재탐색
                if last_seen_side != 0:
                    if side_lost_t is None:
                        side_lost_t = time.time()

                    if time.time() - side_lost_t < SIDE_LOST_TIMEOUT:
                        # 오른쪽(+1)으로 사라졌으면 오른쪽으로 회전(+w)하며 따라가 다시 센트로이드 포착
                        # 왼쪽(-1)으로 사라졌으면 왼쪽으로 회전(-w)
                        send_cmd(SIDE_LOST_V, last_seen_side * SIDE_LOST_W)
                    else:
                        # 재탐색 시간 초과 → 포기하고 벽 추종으로 폴백
                        lost_count   = 0
                        arrive_count = 0
                        last_seen_side = 0
                        side_lost_t  = None
                        enter_state("FIND_WALL")
                elif lost_count >= LOST_CONFIRM:
                    # 측면 이탈 정보가 없는 일반적인 분실(예: 가까이서 갑자기 가림 등)은 기존처럼 처리
                    lost_count   = 0
                    arrive_count = 0
                    enter_state("FIND_WALL")
                else:
                    send_cmd(*last_cmd)

        elif state == "FORWARD":
            if time.time() - park_t >= ARRIVE_FORWARD_SEC:
                stop_robot()
                state  = "PARKING"
                park_t = time.time()
            else:
                send_cmd(*last_cmd)

        elif state == "PARKING":
            stop_robot()
            if time.time() - park_t >= PARK_SEC:
                mission_idx += 1
                detect_count = 0
                arrive_count = 0
                last_seen_side = 0   # [PATCH] 다음 미션을 위해 이전 색의 측면 기억 초기화
                side_lost_t = None
                enter_state("FIND_WALL")

        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)
        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
