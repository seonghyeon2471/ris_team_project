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

# ── 회피 히스테리시스 (TRACK 상태 전용) ──────────────────────────────────
AVOID_ENTER   = THRESH_TURN        # 30cm 이하로 들어오면 회피 시작
AVOID_EXIT    = THRESH_SLOW * 1.3  # 약 71.5cm 이상 멀어져야 회피 해제
AVOID_MIN_SEC = 0.5                # 회피 시작 후 최소 이 시간 동안은 유지 (방향 흔들림 방지)

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

print("WAITING FOR FIRST SCAN DATA...")
time.sleep(1.5)

def get_scan():
    with scan_lock: return _scan_pub.copy()

def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

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

def wall_follow(scan, fm, adir, follow_side, found, last_seen_x, cx_mid):
    sd          = side_dist(scan, follow_side)
    left_close  = side_min(scan, 240, 300)   
    right_close = side_min(scan, 60, 120)    
    sign = 1 if follow_side == "L" else -1

    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    if left_close < SIDE_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < SIDE_STOP:
        return (WALL_V * 0.7,  0.7)

    if left_close < BOTTLENECK_ENTER and right_close < BOTTLENECK_ENTER:
        err_center = left_close - right_close   
        w_center = float(np.clip(CENTER_KP * err_center, -0.9, 0.9))
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

    if not found:
        obj_dir = 1.0 if last_seen_x < cx_mid else -1.0
        is_safe = True
        if obj_dir > 0 and left_close < (WALL_TARGET * 1.3):
            is_safe = False
        if obj_dir < 0 and right_close < (WALL_TARGET * 1.3):
            is_safe = False
            
        if is_safe:
            w += (obj_dir * 0.25)

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
MISSION_TIMEOUT_SEC = 10.0  

FULL_LOOP_THRESH   = math.radians(400)   
LOOP_ESCAPE_BACK_V = -0.12               
LOOP_ESCAPE_SEC    = 0.6                 

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
search_t      = None      
mission_start_t = time.time() 
hop_start_t     = None        
last_cmd      = (0.0, 0.0)

# ── TRACK 회피 상태 ───────────────────────────────────────────────────
avoiding       = False
avoid_start_t  = 0.0
avoid_dir_lock = 1

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
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

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
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # ══ LIDAR 모드 ═════════════════════════════════════════════════════
        if mode == "LIDAR":
            if found: detect_count += 1
            else: detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                continue

            if lidar_state == "WALL_SEARCH":
                # 전방 외에 좌/우 측면 근접 거리 데이터도 함께 산출
                left_close  = side_min(scan, 240, 300)
                right_close = side_min(scan, 60, 120)

                # 정면(fm)뿐만 아니라 좌/우 측면에라도 벽이 감지되면 즉시 주행 모드로 진입
                if fm < WALL_SCAN_DIST or left_close < WALL_SCAN_DIST or right_close < WALL_SCAN_DIST:
                    follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                    lidar_state = "WALL_APPROACH"
                else: 
                    send_cmd(0.0, -adir * WALL_SEARCH_W)

            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3: lidar_state = "WALL_FOLLOW"
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP: send_cmd(0.08, adir * 1.0)
                    elif fm < THRESH_TURN: send_cmd(WALL_APPROACH_V * 0.6, adir * 0.7)
                    else: send_cmd(WALL_APPROACH_V, sign * 0.3)

            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side, found, last_seen_x, cx_mid)
                send_cmd(v, w)

        # ══ PARK 모드 ═════════════════════════════════════════════════════
        elif mode == "PARK":

            if park_state == "SAFE_HOP":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        mission_start_t = time.time()
                        continue
                else: detect_count = 0

                elapsed_hop = time.time() - hop_start_t
                
                if elapsed_hop < 2.0:
                    send_cmd(0.0, 1.2)
                else:
                    if fm > 130.0:
                        send_cmd(0.0, 1.0)
                    elif fm > 50.0:
                        send_cmd(WALL_V, 0.0)
                    else:
                        follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                        park_state = "WALL_APPROACH"
                        mission_start_t = time.time()
                        continue
                
                cv2.imshow("f", frame); cv2.waitKey(1); continue


            if park_state == "FORWARD":
                elapsed = time.time() - park_t
                if elapsed >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t = time.time()
                else: send_cmd(*last_cmd)

            elif park_state == "PARKING":
                stop_robot()
                if time.time() - park_t >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    avoiding = False
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
                else: detect_count = 0

                # 주차 후 탐색 시에도 측면 근접 데이터를 판별식에 포함
                left_close  = side_min(scan, 240, 300)
                right_close = side_min(scan, 60, 120)

                # 전방, 좌측, 우측 중 어느 한 곳이라도 벽이 보이면 바로 APPROACH 모드로 전환하여 주행 시작
                if fm < WALL_SCAN_DIST or left_close < WALL_SCAN_DIST or right_close < WALL_SCAN_DIST:
                    follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                    park_state = "WALL_APPROACH"
                    continue
                send_cmd(0.0, -adir * WALL_SEARCH_W)

            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else: detect_count = 0

                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3: park_state = "WALL_FOLLOW"; continue

                sign = 1 if follow_side == "L" else -1
                if fm < THRESH_STOP: v, w = 0.08, adir * 1.0
                elif fm < THRESH_TURN: v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else: v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)

            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else: detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side, found, last_seen_x, cx_mid)
                send_cmd(v, w)

            elif park_state == "TRACK":
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                # ── 회피 진입 판정 (히스테리시스) ──
                if not avoiding and fm < AVOID_ENTER:
                    avoiding = True
                    avoid_start_t = time.time()
                    avoid_dir_lock = adir   # 진입 시점 방향을 고정 (매 프레임 재계산 안 함)

                # ── 회피 중이면 카메라 신호 완전히 무시하고 회피 동작만 수행 ──
                if avoiding:
                    elapsed_avoid = time.time() - avoid_start_t
                    if fm > AVOID_EXIT and elapsed_avoid > AVOID_MIN_SEC:
                        avoiding = False   # 충분히 멀어지고 최소 유지시간도 지나야 해제
                    else:
                        if fm < THRESH_STOP:
                            v, w = 0.09, avoid_dir_lock * 0.95
                        elif fm < THRESH_TURN:
                            v, w = 0.13, avoid_dir_lock * 0.85
                        else:
                            v, w = 0.18, avoid_dir_lock * 0.65
                        last_cmd = (v, w)
                        send_cmd(v, w)
                        cv2.imshow("f", frame)
                        if cv2.waitKey(1) & 0xFF == 27: break
                        continue

                # ── 회피 중이 아닐 때만 도착 판정 + 카메라 추적 ──
                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0; park_state = "FORWARD"; park_t = time.time()
                    send_cmd(ARRIVE_FORWARD_V, 0.0); continue
                else:
                    err_x = cx_obj - cx_mid
                    err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                    reduced_v = APPROACH_V * (1.0 - err_ratio)

                    def cam_w(ex):
                        raw = -KP_ROT * ex
                        if abs(raw) < W_MIN and ex != 0: return -W_MIN if ex > 0 else W_MIN
                        return raw

                    v, w = reduced_v, cam_w(err_x)
                    last_cmd = (v, w); send_cmd(v, w)

            elif park_state == "SEARCH":
                if park_state != "SEARCH":
                    park_state = "SEARCH"; search_t = time.time(); arrive_count = 0
                elapsed_search = time.time() - search_t
                if elapsed_search > 5.0:
                    park_state = "WALL_SEARCH"
                else:
                    v = 0.0; w = (-1.0 if last_seen_x > cx_mid else 1.0)
                    send_cmd(v, w)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
