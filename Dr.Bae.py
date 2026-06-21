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

# ── 좁은 길(끼임 위험) 감지 ───────────────────────────────────────────
NARROW_SQUEEZE_DIST = 30.0   # 정면/좌/우 모두 이 거리(cm) 이내면 좁은 길에 낀 것으로 판단
NARROW_FORWARD_V    = 0.10   # 좁은 길에서 회전 대신 조심스럽게 직진하는 속도
NARROW_MAX_SEC      = 3.0    # 이 시간 이상 끼임 상태로 직진했는데도 안 빠져나가면 정지 (막다른 길로 추정)
NARROW_HARD_STOP    = 12.0   # 정면이 이 거리(cm)보다 가까워지면 직진 중에도 즉시 정지

def narrow_squeeze(scan, fm):
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    return fm < NARROW_SQUEEZE_DIST and left_close < NARROW_SQUEEZE_DIST and right_close < NARROW_SQUEEZE_DIST

# ── 벽 직선 추정 ───────────────────────────────────────────────────────
WALL_SCAN_START_L = 55
WALL_SCAN_END_L   = 125
WALL_SCAN_START_R = 235
WALL_SCAN_END_R   = 305
WALL_MAX_DIST      = 80.0
WALL_MIN_POINTS    = 5

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

def wall_follow(scan, fm, adir, follow_side, current_v=None):
    if current_v is None:
        current_v = WALL_V

    sign = 1.0 if follow_side == "L" else -1.0

    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)

    # ── [수정] 안전 분기(전방/측면 근접)와 무관하게 실제 벽 추정은 항상 먼저 계산.
    #     기존에는 안전 분기에 걸리면 무조건 valid=False를 강제 반환했기 때문에,
    #     모서리에서 fm이 작아진 것 뿐인데도 "벽을 잃어버렸다"고 오인해
    #     CORNER_SEARCH ↔ WALL_APPROACH ↔ WALL_FOLLOW 사이를 무한히 왕복하며
    #     L/R 메시지가 번갈아 출력되고 로봇이 거의 정지하는 문제가 있었음.
    wall_dist, wall_angle, valid = estimate_wall(scan, follow_side)

    # [추가] 정면+좌+우 모두 30cm 이내 → 좁은 길에 낀 상황.
    #         이때 회전하면 양쪽 벽에 부딫힐 위험이 있으므로 회전 대신 조심히 직진 시도.
    if fm < NARROW_SQUEEZE_DIST and left_close < NARROW_SQUEEZE_DIST and right_close < NARROW_SQUEEZE_DIST:
        return (NARROW_FORWARD_V, 0.0), (wall_dist, wall_angle, valid)

    if fm < THRESH_STOP:
        return (0.08, adir * 1.1), (wall_dist, wall_angle, valid)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85), (wall_dist, wall_angle, valid)
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7), (wall_dist, wall_angle, valid)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7, 0.7), (wall_dist, wall_angle, valid)

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
KP_ROT          = 0.030
W_MIN           = 0.20
APPROACH_V      = 0.13
PARK_SEC        = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 30
ARRIVE_FORWARD_SEC = 0.88
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8

MISSION_TIMEOUT_SEC = 10.0

SEARCH_W            = 0.85
SEARCH_FULL_ROT_SEC = float(2 * np.pi / SEARCH_W)

# ========== 센트로이드 추적 파라미터 ==========
CENTROID_TRACK_V     = 0.12      # 센트로이드 추적 속도
CENTROID_KP          = 0.035     # 센트로이드 회전 게인
CENTROID_MIN_DIST    = 8.0       # 색지 근접 정지 거리 (cm)
CENTROID_ALIGN_TOL   = 15        # 중심 정렬 허용 오차 (pixel)
CENTROID_STABLE_CNT  = 10        # 안정화 카운트

# ========== 직진 및 스캔 파라미터 ==========
SECTOR_SCAN_W    = 1.2           # 스캔 회전 속도
SECTOR_SCAN_SEC  = 0.65           # 1.2 rad/s * 1.0s = 약 70도 회전
STEP_FORWARD_V   = 0.18          # 직진 속도
STEP_FORWARD_SEC = 1.5           # 좌우 살피기 전 직진 시간
WALL_START_DIST  = 30.0          # 전방 30cm 이내 장애물 감지 시 강제 전환 거리

# ========== 개활지(개방 공간) 복귀 파라미터 ==========
OPEN_FIELD_DIST      = 50.0      # 전방 이 거리(cm) 이내에 아무것도 없으면 "개활지"로 판단
OPEN_FIELD_LIMIT_SEC = 5.0       # 개활지 상태가 이 시간(초) 이상 지속되면 180도 돌아서 복귀
RETURN_TURN_SEC      = float(np.pi / SECTOR_SCAN_W)  # 180도 회전에 걸리는 시간

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

# 스캔 상태 관리용 변수
scan_phase      = 0   
scan_t          = 0.0

# 개활지 복귀 관리용 변수
open_field_since = None   # 전방 OPEN_FIELD_DIST 밖이 비어있기 시작한 시각
return_turn_t    = 0.0    # RETURN_TURN 상태 시작 시각

# 좁은 길 끼임 관리용 변수
squeeze_since    = None   # 정면+좌+우가 동시에 NARROW_SQUEEZE_DIST 이내로 들어온 시각

dbg_wall_dist  = 0.0
dbg_wall_angle = 0.0
dbg_valid      = False

print(f"START | MISSION: {MISSION}")
print("========== 탐색: 1.5초 직진 중 실시간 장애물 감지 적용 완료 ==========")

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

        # 좁은 길 끼임 지속시간 추적 (어떤 상태에서든 매 프레임 갱신)
        if narrow_squeeze(scan, fm):
            if squeeze_since is None:
                squeeze_since = time.time()
        else:
            squeeze_since = None

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
            mode == "PARK" and park_state in ["WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW", "SEARCH", "CENTROID_TRACK", "RETURN_TURN"]
        )

        if is_searching:
            if time.time() - mission_start_t > MISSION_TIMEOUT_SEC:
                print(f"🚨 [{target}] {MISSION_TIMEOUT_SEC}초 경과! 도약합니다.")
                mode        = "PARK"
                park_state  = "SAFE_HOP"
                hop_start_t = time.time()
                detect_count = 0
                open_field_since = None
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

        def centroid_track_control(cx_obj, cx_mid, last_bottom_y, ARRIVE_Y_TOP):
            err_x = cx_obj - cx_mid
            err_y = last_bottom_y - ARRIVE_Y_TOP
            
            if err_y < CENTROID_MIN_DIST * 3:
                v = 0.05
            elif err_y < CENTROID_MIN_DIST * 6:
                v = 0.08
            else:
                v = CENTROID_TRACK_V
            
            w = -CENTROID_KP * err_x
            if abs(w) < W_MIN and err_x != 0:
                w = -W_MIN if err_x > 0 else W_MIN
            
            return v, w, err_x, err_y

        # ── LIDAR 모드 ───────────────────────────────────────────────
        if mode == "LIDAR":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "CENTROID"
                arrive_count = 0
                print(f"[{target}] LIDAR → CENTROID 모드 전환!")
                continue

            if lidar_state == "WALL_SEARCH":
                # 1순위: 전방 30cm 이내 막힘 → 기존 동작(벽 추종 진입) 그대로 유지
                if fm < WALL_START_DIST:
                    left_dist = side_min(scan, 0, 90)
                    right_dist = side_min(scan, 270, 360)
                    follow_side = "L" if left_dist < right_dist else "R"
                    print(f"🚧 [LIDAR] 직진/탐색 중 전방 {fm:.1f}cm 막힘! {follow_side} 벽 추종 진입")
                    lidar_state = "WALL_APPROACH"
                    scan_phase = 0
                    open_field_since = None
                    continue

                # [추가] 2순위: 전방 50cm 이내에도 아무것도 없는 "개활지" 상태가
                #        OPEN_FIELD_LIMIT_SEC 이상 지속되면 180도 돌아서 복귀
                if fm > OPEN_FIELD_DIST:
                    if open_field_since is None:
                        open_field_since = time.time()
                    elif time.time() - open_field_since > OPEN_FIELD_LIMIT_SEC:
                        print(f"🔄 [LIDAR] {OPEN_FIELD_LIMIT_SEC:.0f}초간 {OPEN_FIELD_DIST:.0f}cm 이내 아무것도 없음! 180도 회전 후 복귀")
                        lidar_state = "RETURN_TURN"
                        open_field_since = None
                        return_turn_t = time.time()
                        continue
                else:
                    open_field_since = None  # 50cm 이내에 뭔가 다시 보이면 타이머 리셋

                # 3순위: 시간 기반 직진 후 스캔 패턴
                if scan_phase == 0:
                    scan_t = time.time()
                    scan_phase = 1
                elif scan_phase == 1:
                    # [수정됨] 1.5초 직진 실행 (이제 루프가 돌며 위의 fm < WALL_START_DIST 조건에 걸리게 됨)
                    send_cmd(STEP_FORWARD_V, 0.0)
                    if time.time() - scan_t > STEP_FORWARD_SEC:
                        scan_t = time.time()
                        scan_phase = 2
                elif scan_phase == 2:
                    send_cmd(0.0, SECTOR_SCAN_W)
                    if time.time() - scan_t > SECTOR_SCAN_SEC:
                        scan_t = time.time()
                        scan_phase = 3
                elif scan_phase == 3:
                    send_cmd(0.0, -SECTOR_SCAN_W)
                    if time.time() - scan_t > SECTOR_SCAN_SEC * 2:
                        scan_t = time.time()
                        scan_phase = 4
                elif scan_phase == 4:
                    send_cmd(0.0, SECTOR_SCAN_W)
                    if time.time() - scan_t > SECTOR_SCAN_SEC:
                        scan_phase = 0 

            elif lidar_state == "WALL_APPROACH":
                wall_dist, _, valid = estimate_wall(scan, follow_side)
                if valid and wall_dist <= WALL_TARGET * 1.4:
                    lidar_state = "WALL_FOLLOW"
                elif narrow_squeeze(scan, fm):
                    print(f"⚠️ [LIDAR] 좁은 길 끼임 위험! 정면 {fm:.1f}cm 조심히 직진")
                    send_cmd(NARROW_FORWARD_V, 0.0)
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP:
                        send_cmd(0.08, adir * 1.0)
                    elif fm < THRESH_TURN:
                        send_cmd(WALL_APPROACH_V * 0.6, adir * 0.7)
                    else:
                        send_cmd(WALL_APPROACH_V, sign * 0.3)

            elif lidar_state == "WALL_FOLLOW":
                (v, w), (dbg_wall_dist, dbg_wall_angle, dbg_valid) = \
                    wall_follow_debug(scan, fm, adir, follow_side, WALL_V)
                
                if not dbg_valid:
                    print("⚠️ [LIDAR] 모서리 감지! 좌우 스캔 시작")
                    lidar_state = "CORNER_SEARCH"
                    continue
                send_cmd(v, w)
            
            elif lidar_state == "CORNER_SEARCH":
                left_dist = side_min(scan, 0, 70)
                right_dist = side_min(scan, 290, 360)
                
                if min(left_dist, right_dist) < 30.0:
                    follow_side = "L" if left_dist < right_dist else "R"
                    print(f"[LIDAR] 새 장애물 포착! 추종 방향: {follow_side}")
                    lidar_state = "WALL_APPROACH"
                else:
                    send_cmd(0.0, WALL_SEARCH_W)

            elif lidar_state == "RETURN_TURN":
                # 개활지에서 180도 회전하여 진행 반대 방향으로 복귀
                send_cmd(0.0, SECTOR_SCAN_W)
                if time.time() - return_turn_t > RETURN_TURN_SEC:
                    lidar_state = "WALL_SEARCH"
                    scan_phase = 0
                    open_field_since = None

        # ── CENTROID 모드 (독립) ─────────────────────────────────────────
        elif mode == "CENTROID":
            if found:
                v, w, err_x, err_y = centroid_track_control(cx_obj, cx_mid, last_bottom_y, ARRIVE_Y_TOP)
                send_cmd(v, w)
                
                if abs(err_x) < CENTROID_ALIGN_TOL and err_y < CENTROID_MIN_DIST * 2:
                    arrive_count += 1
                else:
                    arrive_count = 0
                
                if arrive_count >= CENTROID_STABLE_CNT:
                    print(f"✅ [{target}] 센트로이드 주차 완료! (count={arrive_count})")
                    stop_robot()
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    mode = "LIDAR"
                    lidar_state = "WALL_SEARCH"
                    scan_phase = 0 
                    open_field_since = None
                    continue
            else:
                print(f"👀 [{target}] 타겟이 카메라 밑으로 사라짐! 0.8초 직진 마무리 돌입!")
                mode = "FINISH_FORWARD"
                park_t = time.time()
                continue
        
        # ── 마무리 직진 상태 ──
        elif mode == "FINISH_FORWARD":
            if time.time() - park_t < 0.8:
                send_cmd(ARRIVE_FORWARD_V, 0.0) 
            else:
                print(f"🎉 [{target}] 0.8초 직진 완료! 주차 완벽 성공!")
                stop_robot()
                mission_idx += 1
                arrive_count = 0
                detect_count = 0
                if mission_idx < len(MISSION):
                    mode = "LIDAR"
                    lidar_state = "WALL_SEARCH"
                    scan_phase = 0 
                    open_field_since = None
                else:
                    mode = "DONE"
                continue

        # ── PARK 모드 ───────────────────────────────────────────────
        elif mode == "PARK":
            if park_state not in ["SAFE_HOP", "FORWARD", "FINISH_FORWARD", "PARKING"]:
                if fm < 10.0:
                    print("⚠️ [긴급] 충돌 직전 물리적 장애물 감지! 우회 시작")
                    left_dist = side_min(scan, 0, 90)
                    right_dist = side_min(scan, 270, 360)
                    follow_side = "L" if left_dist < right_dist else "R"
                    park_state = "WALL_APPROACH"
                    detect_count = 0 

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
            
            elif park_state == "FORWARD":
                if time.time() - park_t >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t = time.time()
                else:
                    send_cmd(*last_cmd)
            
            elif park_state == "FINISH_FORWARD":
                if time.time() - park_t < 0.8:  
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                else:
                    print(f"🎉 [{target}] 0.8초 직진 완료! 주차 완벽 성공!")
                    stop_robot()
                    park_state = "PARKING"
                    park_t = time.time()

            elif park_state == "PARKING":
                stop_robot()
                if time.time() - park_t >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"
                        scan_phase = 0 
                        open_field_since = None
                    continue

            # PARK 모드의 WALL_SEARCH 상태
            elif park_state == "WALL_SEARCH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "CENTROID_TRACK"
                        arrive_count = 0
                        print(f"[{target}] PARK → CENTROID_TRACK 상태 전환!")
                        continue
                else:
                    detect_count = 0

                # [수정] PARK 모드의 스캔 루프 내에서도 전방 차단 상시 체크 (30cm 이내 → 기존 동작 유지)
                if fm < WALL_START_DIST:
                    left_dist = side_min(scan, 0, 90)
                    right_dist = side_min(scan, 270, 360)
                    follow_side = "L" if left_dist < right_dist else "R"
                    print(f"🚧 [PARK] 직진/탐색 중 전방 {fm:.1f}cm 막힘! {follow_side} 벽 추종 진입")
                    park_state = "WALL_APPROACH"
                    scan_phase = 0
                    open_field_since = None
                    continue

                # [추가] 전방 50cm 이내에도 아무것도 없는 "개활지" 상태가
                #        OPEN_FIELD_LIMIT_SEC 이상 지속되면 180도 돌아서 복귀
                if fm > OPEN_FIELD_DIST:
                    if open_field_since is None:
                        open_field_since = time.time()
                    elif time.time() - open_field_since > OPEN_FIELD_LIMIT_SEC:
                        print(f"🔄 [PARK] {OPEN_FIELD_LIMIT_SEC:.0f}초간 {OPEN_FIELD_DIST:.0f}cm 이내 아무것도 없음! 180도 회전 후 복귀")
                        park_state = "RETURN_TURN"
                        open_field_since = None
                        return_turn_t = time.time()
                        continue
                else:
                    open_field_since = None

                # 2순위: 직진 후 스캔 패턴
                if scan_phase == 0:
                    scan_t = time.time()
                    scan_phase = 1
                elif scan_phase == 1:
                    # [수정됨] 직진하면서 매 프레임 감시를 이어감
                    send_cmd(STEP_FORWARD_V, 0.0)
                    if time.time() - scan_t > STEP_FORWARD_SEC:
                        scan_t = time.time()
                        scan_phase = 2
                elif scan_phase == 2:
                    send_cmd(0.0, SECTOR_SCAN_W)
                    if time.time() - scan_t > SECTOR_SCAN_SEC:
                        scan_t = time.time()
                        scan_phase = 3
                elif scan_phase == 3:
                    send_cmd(0.0, -SECTOR_SCAN_W)
                    if time.time() - scan_t > SECTOR_SCAN_SEC * 2:
                        scan_t = time.time()
                        scan_phase = 4
                elif scan_phase == 4:
                    send_cmd(0.0, SECTOR_SCAN_W)
                    if time.time() - scan_t > SECTOR_SCAN_SEC:
                        scan_phase = 0

            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "CENTROID_TRACK"
                        arrive_count = 0
                        continue
                else:
                    detect_count = 0

                wall_dist, _, valid = estimate_wall(scan, follow_side)
                if valid and wall_dist <= WALL_TARGET * 1.4:
                    park_state = "WALL_FOLLOW"
                    continue

                if narrow_squeeze(scan, fm):
                    print(f"⚠️ [PARK] 좁은 길 끼임 위험! 정면 {fm:.1f}cm 조심히 직진")
                    v, w = NARROW_FORWARD_V, 0.0
                else:
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
                        park_state = "CENTROID_TRACK"
                        arrive_count = 0
                        continue
                else:
                    detect_count = 0

                (v, w), (dbg_wall_dist, dbg_wall_angle, dbg_valid) = \
                    wall_follow_debug(scan, fm, adir, follow_side, WALL_V)
                
                if not dbg_valid:
                    print("⚠️ [PARK] 모서리 감지! 좌우 스캔 시작")
                    park_state = "CORNER_SEARCH"
                    continue
                send_cmd(v, w)
                
            elif park_state == "CORNER_SEARCH":
                left_dist = side_min(scan, 0, 70)
                right_close = side_min(scan, 290, 360)
                
                if min(left_dist, right_close) < 30.0:
                    follow_side = "L" if left_dist < right_close else "R"
                    print(f"[PARK] 새 장애물 포착! 추종 방향: {follow_side}")
                    park_state = "WALL_APPROACH"
                else:
                    send_cmd(0.0, WALL_SEARCH_W)

            elif park_state == "RETURN_TURN":
                # 개활지에서 180도 회전하여 진행 반대 방향으로 복귀
                send_cmd(0.0, SECTOR_SCAN_W)
                if time.time() - return_turn_t > RETURN_TURN_SEC:
                    park_state = "WALL_SEARCH"
                    scan_phase = 0
                    open_field_since = None

            elif park_state == "CENTROID_TRACK":
                if found:
                    v, w, err_x, err_y = centroid_track_control(cx_obj, cx_mid, last_bottom_y, ARRIVE_Y_TOP)
                    send_cmd(v, w)
                    
                    if abs(err_x) < CENTROID_ALIGN_TOL and err_y < CENTROID_MIN_DIST * 2:
                        arrive_count += 1
                    else:
                        arrive_count = 0
                    
                    if arrive_count >= CENTROID_STABLE_CNT:
                        print(f"✅ [{target}] CENTROID_TRACK 주차 완료! (count={arrive_count})")
                        stop_robot()
                        mission_idx += 1
                        arrive_count = 0
                        detect_count = 0
                        continue
                else:
                    detect_count = 0
                    print(f"👀 [{target}] 타겟이 카메라 밑으로 사라짐! 0.8초 직진 마무리 돌입!")
                    park_state = "FINISH_FORWARD"
                    park_t = time.time()
                    continue

            elif park_state == "TRACK":
                if not found:
                    print(f"👀 [{target}] TRACK 중 시야에서 사라짐! 0.8초 직진 마무리 돌입!")
                    park_state = "FINISH_FORWARD"
                    park_t = time.time()
                    continue

                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                    continue

                err_x = cx_obj - cx_mid
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

            elif park_state == "SEARCH":
                if search_t is None:
                    search_t = time.time()
                    arrive_count = 0
                if time.time() - search_t > SEARCH_FULL_ROT_SEC:
                    park_state = "WALL_SEARCH"
                    search_t = None
                    scan_phase = 0
                else:
                    send_cmd(0.0, -SEARCH_W if last_seen_x > cx_mid else SEARCH_W)

        # ── 디버그 오버레이 ───────────────────────────────────────────
        search_time_left = MISSION_TIMEOUT_SEC - (time.time() - mission_start_t)
        if is_searching and search_time_left > 0:
            cv2.putText(frame, f"Timeout: {search_time_left:.1f}s | Side:{follow_side}",
                        (10, 20), 0, 0.50, (255, 150, 0), 2)

        if mode in ["CENTROID", "FINISH_FORWARD"] or park_state in ["CENTROID_TRACK", "FINISH_FORWARD"]:
            err_x = cx_obj - cx_mid if found else 0
            err_y = last_bottom_y - ARRIVE_Y_TOP if found else 0
            
            color = (0, 255, 255) if found else (255, 0, 0)
            cv2.putText(frame,
                f"CENTROID: dx={err_x:.1f} dy={err_y:.1f} cnt={arrive_count}",
                (10, 40), 0, 0.45, color, 1)
            
            if found:
                cv2.line(frame, (cx_mid, ARRIVE_Y_TOP), (cx_obj, last_bottom_y), (0, 255, 255), 2)
                cv2.circle(frame, (cx_obj, last_bottom_y), 8, (255, 255, 0), 2)

        in_wall_follow = (
            (mode == "LIDAR" and lidar_state == "WALL_FOLLOW") or
            (mode == "PARK" and park_state in ["WALL_FOLLOW", "CENTROID_TRACK"])
        )
        if in_wall_follow:
            color_v = (0, 255, 100) if dbg_valid else (0, 80, 255)
            cv2.putText(frame,
                f"WF dist={dbg_wall_dist:.1f}cm ang={np.rad2deg(dbg_wall_angle):.1f}deg {'OK' if dbg_valid else 'NO'}",
                (10, H - 10), 0, 0.42, color_v, 1)

        if (mode == "LIDAR" and lidar_state == "WALL_SEARCH") or (mode == "PARK" and park_state == "WALL_SEARCH"):
            status_text = ["", "FORWARD(1.5s)", "SCAN_LEFT", "SCAN_RIGHT", "SCAN_CENTER"][scan_phase] if scan_phase < 5 else ""
            cv2.putText(frame, f"PHASE: {status_text}", (10, 60), 0, 0.5, (255, 0, 255), 2)
        elif (mode == "LIDAR" and lidar_state == "RETURN_TURN") or (mode == "PARK" and park_state == "RETURN_TURN"):
            cv2.putText(frame, "PHASE: RETURN_TURN (180deg)", (10, 60), 0, 0.5, (0, 255, 255), 2)

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
