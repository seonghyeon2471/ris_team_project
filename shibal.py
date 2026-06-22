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
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR PARAMS ──────────────────────────────────────────────────────
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
        if len(raw) != 5: 
            continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150: 
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

def side_dist(scan, side):
    idx = np.arange(65, 96) % 360 if side == "L" else np.arange(265, 296) % 360
    return float(np.min(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

def wall_follow(scan, fm, adir, follow_side):
    global last_w
    sd          = side_dist(scan, follow_side)
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1

    if fm < THRESH_STOP:
        w_bounce = -1.2 if last_w > 0 else 1.2
        if abs(last_w) < 0.02: 
            w_bounce = adir * 1.1
        return (0.0, w_bounce)
        
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

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
def send_cmd(v, w, current_fm=None):
    global last_w
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    
    # 인자로 받은 실시간 fm 값을 기준으로 업데이트 여부 결정
    if current_fm is not None and current_fm >= THRESH_STOP:
        if abs(w) > 0.01:
            last_w = w
            
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): 
    send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]), "hsv2": None, "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([25, 60, 160], [32, 161, 255]),    "hsv2": None, "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([96, 100, 95], [138, 207, 246]),   "hsv2": None, "bgr":  ([40,  0,    0], [255, 220, 220]), "draw": (255, 80, 0)},
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
ARRIVE_FORWARD_SEC = 1.0
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8

WALL_TARGET     = 20.0
WALL_SCAN_DIST  = 40.0  
WALL_APPROACH_V = 0.20  
WALL_KP         = 0.012
WALL_V          = 0.22
WALL_TURN_V     = 0.10
WALL_LOST_W     = 0.9     
WALL_SEARCH_W   = 0.9     
MISSION_TIMEOUT_SEC = 5.0  

# ── 개활지(open field) 판단 파라미터 ──────────────────────────────────
OPEN_FIELD_INIT_SPIN_SEC  = 2.0   # SAFE_HOP 진입 시 고정 스캔 회전 시간
OPEN_FIELD_WINDOW_SEC     = 5.0   # 개활지 판단 윈도우(연속 관찰 시간)
OPEN_FIELD_START_DIST     = 60.0  # 윈도우 시작 시점의 임계 거리
OPEN_FIELD_SHRINK_PER_SEC = 8.0   # 1초 지날 때마다 줄어드는 임계 거리 (60→52→44→36→28→20)
OPEN_FIELD_CLOSE_DIST     = 40.0  # 이 이하로 가까우면 시간 무관하게 즉시 WALL_APPROACH

# 개활지 확정 시 180도 회전 파라미터
SAFE_HOP_W        = 1.2           # 회전 각속도 (rad/s 가정 — 실측치로 교체 가능)
SAFE_HOP_TURN_DEG = 180.0
SAFE_HOP_TURN_SEC = math.radians(SAFE_HOP_TURN_DEG) / SAFE_HOP_W

# ── STATE ─────────────────────────────────────────────────────────────
mode            = "LIDAR"   
mission_idx     = 0
detect_count    = 0
arrive_count    = 0
follow_side     = "L"   
lidar_state     = "WALL_SEARCH"
park_state      = "TRACK"   

last_seen_x   = 160
last_bottom_y = 0
park_t        = None
search_t      = None      
mission_start_t = time.time() 
hop_start_t     = None        
turn_start_t    = None        # [추가] 개활지 확정 후 180도 턴 시작 시각
last_cmd      = (0.0, 0.0)
last_w        = 0.0  

print(f"START | MISSION: {MISSION}")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
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
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]
        cv2.putText(frame, f"TARGET: {target.upper()}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)

        is_searching = (mode == "LIDAR") or (mode == "PARK" and park_state in ["WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW", "SEARCH"])
        
        if is_searching:
            if time.time() - mission_start_t > MISSION_TIMEOUT_SEC:
                mode = "PARK"
                park_state = "SAFE_HOP"
                hop_start_t = time.time()
                detect_count = 0
                continue
        elif park_state not in ["SAFE_HOP"]:
            mission_start_t = time.time()

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
            last_seen_x   = cx_obj
            last_bottom_y = min(by_top + bh, 239)
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (arrive_x1 <= cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # ── LIDAR 모드 ═════════════════════════════════════════════════════
        if mode == "LIDAR":
            detect_count = detect_count + 1 if found else 0

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
                    send_cmd(0.0, WALL_SEARCH_W, fm)

            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3: 
                    lidar_state = "WALL_FOLLOW"
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP: 
                        w_bounce = -1.0 if last_w > 0 else 1.0
                        if abs(last_w) < 0.02: 
                            w_bounce = adir * 1.0
                        send_cmd(0.0, w_bounce, fm)
                    elif fm < THRESH_TURN: 
                        send_cmd(WALL_APPROACH_V * 0.6, adir * 0.7, fm)
                    else: 
                        send_cmd(WALL_APPROACH_V, sign * 0.3, fm)

            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w, fm)

        # ── PARK 모드 ═════════════════════════════════════════════════════
        elif mode == "PARK":
            if park_state == "SAFE_HOP":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        mission_start_t = time.time()
                        continue
                else: 
                    detect_count = 0

                elapsed_hop = time.time() - hop_start_t
                if elapsed_hop < OPEN_FIELD_INIT_SPIN_SEC:
                    send_cmd(0.0, 1.2, fm)
                else:
                    open_t       = elapsed_hop - OPEN_FIELD_INIT_SPIN_SEC
                    shrink_steps = int(open_t // 1.0)
                    dyn_thresh   = max(OPEN_FIELD_START_DIST - shrink_steps * OPEN_FIELD_SHRINK_PER_SEC, 0.0)

                    if fm <= OPEN_FIELD_CLOSE_DIST:
                        # 가까운 건 시간 무관하게 곧장 접근
                        park_state = "WALL_APPROACH"
                        mission_start_t = time.time()
                        continue
                    elif fm <= dyn_thresh:
                        # 현재 임계거리 안으로 뭔가 들어옴 -> 그 방향으로 직진
                        send_cmd(WALL_V, 0.0, fm)
                    elif open_t >= OPEN_FIELD_WINDOW_SEC:
                        # 5초 내내 임계거리 안으로 아무것도 안 들어옴 -> 개활지 확정, 180도 턴
                        park_state = "OPEN_FIELD_TURN"
                        turn_start_t = time.time()
                        continue
                    else:
                        # 아직 판단 윈도우 진행 중, 계속 회전 스캔
                        send_cmd(0.0, 1.0, fm)
                
                cv2.imshow("f", frame)
                cv2.waitKey(1)
                continue

            if park_state == "OPEN_FIELD_TURN":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        mission_start_t = time.time()
                        continue
                else: 
                    detect_count = 0

                elapsed_turn = time.time() - turn_start_t
                if elapsed_turn < SAFE_HOP_TURN_SEC:
                    send_cmd(0.0, SAFE_HOP_W, fm)
                else:
                    park_state = "WALL_SEARCH"
                    mission_start_t = time.time()
                    continue

                cv2.imshow("f", frame)
                cv2.waitKey(1)
                continue

            if park_state == "FORWARD":
                elapsed = time.time() - park_t
                if elapsed >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t = time.time()
                else: 
                    send_cmd(last_cmd[0], last_cmd[1], fm)

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
                send_cmd(0.0, WALL_SEARCH_W, fm)

            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        continue
                else: 
                    detect_count = 0

                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3: 
                    park_state = "WALL_FOLLOW"
                    continue

                sign = 1 if follow_side == "L" else -1
                if fm < THRESH_STOP: 
                    w_bounce = -1.0 if last_w > 0 else 1.0
                    if abs(last_w) < 0.02: 
                        w_bounce = adir * 1.0
                    v, w = 0.0, w_bounce
                elif fm < THRESH_TURN: 
                    v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else: 
                    v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w, fm)

            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        continue
                else: 
                    detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w, fm)

            elif park_state == "TRACK":
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    send_cmd(ARRIVE_FORWARD_V, 0.0, fm)
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
                        v, w = reduced_v, cam_w(err_x)
                    else:
                        w_cam = cam_w(err_x)
                        w_lid = adir * 0.7
                        if fm < THRESH_STOP: 
                            w_bounce = -1.2 if last_w > 0 else 1.2
                            if abs(last_w) < 0.02: 
                                w_bounce = w_lid
                            v, w = 0.0, w_bounce
                        elif fm < THRESH_TURN: 
                            v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                        else: 
                            v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                    last_cmd = (v, w)
                    send_cmd(v, w, fm)

            elif park_state == "SEARCH":
                elapsed_search = time.time() - search_t
                if elapsed_search > 5.0:
                    park_state = "WALL_SEARCH"
                else:
                    v = 0.0
                    w = -1.0 if last_seen_x > cx_mid else 1.0
                    send_cmd(v, w, fm)

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
