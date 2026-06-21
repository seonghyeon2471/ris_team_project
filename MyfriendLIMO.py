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

# ── LIDAR ─────────────────────────────────────────────────────────────
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

# ── 벽 직선 추정 ───────────────────────────────────────────────────────
WALL_SCAN_START_L = 55
WALL_SCAN_END_L   = 125
WALL_SCAN_START_R = 235
WALL_SCAN_END_R   = 305
WALL_MAX_DIST     = 80.0
WALL_MIN_POINTS   = 5

def estimate_wall(scan, side):
    if side == "L":
        angles_deg = np.arange(WALL_SCAN_START_L, WALL_SCAN_END_L + 1)
    else:
        angles_deg = np.arange(WALL_SCAN_START_R, WALL_SCAN_END_R + 1)

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

    wall_angle = float(np.arctan(m))
    wall_dist = abs(float(b))
    return wall_dist, wall_angle, True

# ── 벽 추종 제어기 ────────────────────────────────────────────────────
WALL_TARGET      = 20.0
WALL_KH          = 1.20
WALL_KD          = 0.06
WALL_V           = 0.22
WALL_TURN_V      = 0.10
WALL_APPROACH_V  = 0.20
WALL_LOST_W      = 0.45
WALL_SEARCH_W    = 1.1
LOOKAHEAD_TIME   = 0.35
MAX_W            = 0.90

# ── 코너 회전 파라미터 ────────────────────────────────────────────────
# 직전까지 벽을 잘 따라가던 상태(valid=True, wall_dist가 목표치 근처)에서
# 갑자기 valid=False가 되는 순간을 "코너 진입"으로 명시적으로 판단해
# 더 빠르고 확실하게 꺾어 들어가기 위한 전용 모드.
CORNER_TURN_W       = 0.9     # 코너 전용 회전속도 (WALL_LOST_W=0.45보다 빠름)
CORNER_TURN_V       = 0.10    # 코너 도는 동안 전진속도 낮춤 (오버슈트 방지)
CORNER_MAX_SEC      = 2.2     # 약 90~100도 분량 최대 회전 시간 (timeout)
CORNER_ENTRY_DIST   = WALL_TARGET * 1.6  # 코너 진입 직전 "정상 추종 중"으로 볼 거리 한계

# wall_follow는 순수 함수라 프레임 간 상태를 갖지 않으므로,
# "직전까지 정상 추종 중이었는가"를 외부(호출부)에서 관리해야 함.
# 이 클래스가 그 상태를 캡슐화한다.
class CornerTracker:
    def __init__(self):
        self.was_following = False   # 직전 프레임에 valid=True였는지
        self.in_corner      = False  # 현재 코너 회전 모드인지
        self.corner_start_t = None
        self.corner_sign    = 1.0
        self.last_side       = None  # follow_side가 바뀌면 상태를 새로 시작

    def reset(self):
        self.was_following = False
        self.in_corner      = False
        self.corner_start_t = None

corner_tracker = CornerTracker()

def wall_follow(scan, fm, adir, follow_side, current_v=None):
    if current_v is None:
        current_v = WALL_V

    sign = 1.0 if follow_side == "L" else -1.0

    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)

    if fm < THRESH_STOP:
        return (0.08, adir * 1.1), (0.0, 0.0, False)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85), (0.0, 0.0, False)
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7), (0.0, 0.0, False)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7, 0.7), (0.0, 0.0, False)

    wall_dist, wall_angle, valid = estimate_wall(scan, follow_side)

    if not valid:
        return (WALL_V * 0.6, sign * WALL_LOST_W), (wall_dist, wall_angle, False)

    heading_term = -sign * WALL_KH * wall_angle
    dist_error   = wall_dist - WALL_TARGET
    v_safe       = max(abs(current_v), 0.05)
    dist_term    = sign * float(np.arctan(WALL_KD * dist_error / v_safe))
    w_raw        = heading_term + dist_term

    lateral_pred = current_v * LOOKAHEAD_TIME * np.sin(
        np.clip(w_raw * LOOKAHEAD_TIME * 0.5, -1.0, 1.0)
    )
    if abs(dist_error) > 0.5:
        correction = float(np.clip(lateral_pred / (dist_error + 1e-6), 0.0, 0.8))
        w_raw = w_raw * (1.0 - correction * 0.5)

    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w_raw = (1.0 - blend) * w_raw + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V

    w = float(np.clip(w_raw, -MAX_W, MAX_W))
    return (v, w), (wall_dist, wall_angle, valid)

def wall_follow_cmd(scan, fm, adir, follow_side, current_v=None):
    result = wall_follow(scan, fm, adir, follow_side, current_v)
    return result[0]

def wall_follow_debug(scan, fm, adir, follow_side, current_v=None):
    return wall_follow(scan, fm, adir, follow_side, current_v)

# ── 코너 인지 벽 추종 ─────────────────────────────────────────────────
def wall_follow_with_corner(scan, fm, adir, follow_side, current_v=None, tracker=None):
    """
    wall_follow를 감싸서 '코너 진입' 전이를 감지하고,
    감지되면 CORNER_TURN_W로 빠르게 꺾어 새 벽을 재포착할 때까지
    전용 회전 모드를 유지한다.

    반환: (v, w), (wall_dist, wall_angle, valid), in_corner
    """
    if tracker is None:
        tracker = corner_tracker
    if current_v is None:
        current_v = WALL_V

    # WALL_FOLLOW 상태에 새로 진입했거나(follow_side가 바뀜) 추종 대상이
    # 바뀌었으면 묵은 코너 상태가 남아있지 않도록 새로 시작한다.
    if tracker.last_side != follow_side:
        tracker.reset()
        tracker.last_side = follow_side

    sign = 1.0 if follow_side == "L" else -1.0
    wall_dist, wall_angle, valid = estimate_wall(scan, follow_side)

    # ── 이미 코너 회전 중인 경우 ────────────────────────────────────
    if tracker.in_corner:
        elapsed = time.time() - tracker.corner_start_t

        # 새 벽을 다시 잡았으면(가까운 거리로 valid) 코너 모드 종료
        if valid and wall_dist <= CORNER_ENTRY_DIST:
            tracker.in_corner = False
            tracker.was_following = True
            tracker.corner_start_t = None
            # 코너 종료 직후엔 일반 wall_follow로 바로 넘긴다
            (v, w), dbg = wall_follow(scan, fm, adir, follow_side, current_v)
            return (v, w), dbg, False

        # 타임아웃: 90~100도 분량을 다 돌았는데도 못 찾으면
        # 일반 LOST 폴백(약한 회전)으로 넘겨서 무한 회전 방지
        if elapsed > CORNER_MAX_SEC:
            tracker.in_corner = False
            tracker.corner_start_t = None
            return (WALL_V * 0.6, tracker.corner_sign * WALL_LOST_W), \
                   (wall_dist, wall_angle, valid), False

        # 코너 전용 회전 유지: 속도 낮추고 빠르게 꺾기
        return (CORNER_TURN_V, tracker.corner_sign * CORNER_TURN_W), \
               (wall_dist, wall_angle, valid), True

    # ── 코너 회전 중이 아닌 경우: 정상 추종 / 코너 진입 판단 ────────
    (v, w), (wd, wa, wvalid) = wall_follow(scan, fm, adir, follow_side, current_v)

    if wvalid:
        # 목표치 근처에서 정상 추종 중이면 "코너 진입 가능" 상태로 기록
        tracker.was_following = wd <= CORNER_ENTRY_DIST
        return (v, w), (wd, wa, wvalid), False

    # valid=False가 떴다 — 직전까지 정상 추종 중이었다면 코너 진입으로 확정
    if tracker.was_following:
        tracker.in_corner = True
        tracker.corner_start_t = time.time()
        tracker.corner_sign = sign
        tracker.was_following = False
        return (CORNER_TURN_V, sign * CORNER_TURN_W), (wd, wa, wvalid), True

    # 정상 추종 중이 아니었는데 valid=False (예: 시작부터 벽을 못 찾음)
    # → 기존 동작(약한 LOST 회전) 그대로 유지
    return (v, w), (wd, wa, wvalid), False

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red": {
        "hsv1": ([169, 136, 114], [179, 220, 255]),
        "hsv2": None,
        "bgr": ([20, 20, 80], [255, 255, 255]),
        "draw": (0, 0, 255)
    },
    "yellow": {
        "hsv1": ([25, 60, 160], [32, 161, 255]),
        "hsv2": None,
        "bgr": ([0, 80, 80], [255, 255, 255]),
        "draw": (0, 200, 255)
    },
    "blue": {
        "hsv1": ([96, 100, 95], [138, 207, 246]),
        "hsv2": None,
        "bgr": ([40, 0, 0], [255, 220, 220]),
        "draw": (255, 80, 0)
    },
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

# [PATCH] 센트로이드 추적 P제어 재조정
# [PATCH-2] 각속도 전체 30% 증가 (KP_ROT, W_MIN 동시 스케일링하여
#           데드밴드 직후 점프값과 비례 기울기의 비율을 유지)
KP_ROT         = 0.0156   # 0.012 * 1.3
W_MIN          = 0.156    # 0.12  * 1.3
DEADBAND_PX    = 8

APPROACH_V     = 0.13
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

# [PATCH] 센트로이드 노이즈 완화용 EMA 필터
CENTROID_EMA_ALPHA = 0.4

# [PATCH] TRACK 중 타겟을 연속 몇 프레임 못 보면 "놓쳤다"고 판단할지
LOST_CONFIRM = 8

ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 30
ARRIVE_FORWARD_SEC = 0.88
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8

WALL_SCAN_DIST      = 150.0
MISSION_TIMEOUT_SEC = 10.0

# SEARCH 서브스테이트: 타겟을 놓쳤을 때 마지막으로 보인 방향으로 360도 회전 탐색
SEARCH_W            = 0.85
SEARCH_FULL_ROT_SEC = float(2 * np.pi / SEARCH_W)  # ≈ 7.39초

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

# [PATCH] 추가 상태 변수
filtered_cx  = None   # 센트로이드 EMA 필터 값
lost_count   = 0       # TRACK 중 연속 미검출 프레임 카운터
search_dir   = 1.0     # SEARCH 진입 시 고정되는 회전 방향

dbg_wall_dist  = 0.0
dbg_wall_angle = 0.0
dbg_valid      = False
dbg_in_corner  = False

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
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        is_searching = (mode == "LIDAR") or (
            mode == "PARK" and park_state in ["WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW", "SEARCH"]
        )

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

            # [PATCH] 센트로이드 EMA 필터링 (노이즈로 인한 회전 떨림 완화)
            if cx_obj >= 0:
                filtered_cx = float(cx_obj) if filtered_cx is None else \
                    (1 - CENTROID_EMA_ALPHA) * filtered_cx + CENTROID_EMA_ALPHA * cx_obj
                cx_obj = int(filtered_cx)

            bx, by_top, bw, bh = cv2.boundingRect(big)
            last_seen_x   = cx_obj
            last_bottom_y = min(by_top + bh, 239)
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)
        else:
            # 검출 실패 시 필터 상태 리셋 (다음에 다시 잡히면 새로 시작)
            filtered_cx = None

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (arrive_x1 <= cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # [PATCH] 데드존 적용된 카메라 회전 제어
        def cam_w(ex):
            if abs(ex) < DEADBAND_PX:
                return 0.0
            raw = -KP_ROT * ex
            if abs(raw) < W_MIN:
                return -W_MIN if ex > 0 else W_MIN
            return raw

        # ── LIDAR 모드 ───────────────────────────────────────────────
        if mode == "LIDAR":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
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
                    corner_tracker.reset()
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP:
                        send_cmd(0.08, adir * 1.0)
                    elif fm < THRESH_TURN:
                        send_cmd(WALL_APPROACH_V * 0.6, adir * 0.7)
                    else:
                        send_cmd(WALL_APPROACH_V, sign * 0.3)

            elif lidar_state == "WALL_FOLLOW":
                (v, w), (dbg_wall_dist, dbg_wall_angle, dbg_valid), dbg_in_corner = \
                    wall_follow_with_corner(scan, fm, adir, follow_side, WALL_V)
                send_cmd(v, w)

        # ── PARK 모드 ───────────────────────────────────────────────
        elif mode == "PARK":

            if park_state == "SAFE_HOP":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        print(f"[{target}] 도약 중 타겟 발견!")
                        detect_count = 0
                        park_state = "TRACK"
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
                        park_state = "WALL_APPROACH"
                        mission_start_t = time.time()
                        continue

                cv2.imshow("f", frame)
                cv2.waitKey(1)
                continue

            if park_state == "FORWARD":
                if time.time() - park_t >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t = time.time()
                else:
                    send_cmd(*last_cmd)

            elif park_state == "PARKING":
                stop_robot()
                if time.time() - park_t >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"
                    continue

            elif park_state == "WALL_SEARCH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        continue
                else:
                    detect_count = 0

                if fm < WALL_SCAN_DIST:
                    park_state = "WALL_APPROACH"
                    continue
                send_cmd(0.0, WALL_SEARCH_W)

            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        continue
                else:
                    detect_count = 0

                wall_dist, _, valid = estimate_wall(scan, follow_side)
                if valid and wall_dist <= WALL_TARGET * 1.4:
                    park_state = "WALL_FOLLOW"
                    corner_tracker.reset()
                    continue

                sign = 1 if follow_side == "L" else -1
                if fm < THRESH_STOP:
                    v, w = 0.08, adir * 1.0
                elif fm < THRESH_TURN:
                    v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else:
                    v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)

            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        continue
                else:
                    detect_count = 0

                (v, w), (dbg_wall_dist, dbg_wall_angle, dbg_valid), dbg_in_corner = \
                    wall_follow_with_corner(scan, fm, adir, follow_side, WALL_V)
                send_cmd(v, w)

            elif park_state == "TRACK":
                if found:
                    # [PATCH] 타겟이 다시 보이면 미검출 카운터 리셋
                    lost_count = 0

                    arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                    if arrive_count >= ARRIVE_CONFIRM:
                        arrive_count = 0
                        park_state = "FORWARD"
                        park_t = time.time()
                        send_cmd(ARRIVE_FORWARD_V, 0.0)
                        continue

                    err_x = cx_obj - cx_mid

                    # [PATCH-2] 정렬이 덜 된 상태에서 속도가 너무 빨리
                    # 회복되는 문제 완화: 감쇠 곡선을 선형 → 제곱으로 변경.
                    # (새 파라미터 추가 없이 기존 cx_mid 스케일 그대로 사용)
                    err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                    reduced_v = APPROACH_V * (1.0 - err_ratio) ** 2

                    if fm >= THRESH_SLOW:
                        v, w = reduced_v, cam_w(err_x)
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

                else:
                    # [PATCH] 타겟을 놓친 경우 처리
                    lost_count += 1
                    if lost_count >= LOST_CONFIRM:
                        lost_count = 0
                        arrive_count = 0
                        search_dir = -1.0 if last_seen_x > cx_mid else 1.0
                        park_state = "SEARCH"
                        search_t = None
                        continue
                    else:
                        send_cmd(*last_cmd)

            elif park_state == "SEARCH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        search_t = None
                        park_state = "TRACK"
                        continue
                else:
                    detect_count = 0

                if search_t is None:
                    search_t = time.time()

                if time.time() - search_t > SEARCH_FULL_ROT_SEC:
                    park_state = "WALL_SEARCH"
                    search_t = None
                else:
                    send_cmd(0.0, search_dir * SEARCH_W)

        # ── 디버그 오버레이 ───────────────────────────────────────────
        search_time_left = MISSION_TIMEOUT_SEC - (time.time() - mission_start_t)
        if is_searching and search_time_left > 0:
            cv2.putText(frame, f"Timeout: {search_time_left:.1f}s | Side:{follow_side}",
                        (10, 20), 0, 0.50, (255, 150, 0), 2)

        if mode == "PARK" and park_state == "SEARCH":
            cv2.putText(frame, f"SEARCH dir={'R' if search_dir < 0 else 'L'}",
                        (10, 35), 0, 0.50, (0, 200, 255), 2)

        in_wall_follow = (
            (mode == "LIDAR" and lidar_state == "WALL_FOLLOW") or
            (mode == "PARK" and park_state == "WALL_FOLLOW")
        )
        if in_wall_follow:
            color_v = (0, 255, 100) if dbg_valid else (0, 80, 255)
            cv2.putText(frame,
                f"WF dist={dbg_wall_dist:.1f}cm ang={np.rad2deg(dbg_wall_angle):.1f}deg {'OK' if dbg_valid else 'NO'}",
                (10, H - 10), 0, 0.42, color_v, 1)
            if dbg_in_corner:
                cv2.putText(frame, "CORNER TURN", (10, H - 28), 0, 0.5, (0, 255, 255), 2)

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
