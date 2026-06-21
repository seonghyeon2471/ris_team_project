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

# ── LIDAR PARAMS ──────────────────────────────────────────────────────
EMA_ALPHA    = 0.35
MEDIAN_K     = 2
FRONT_RANGE  = 90    

THRESH_SLOW  = 25.0   
THRESH_TURN  = 15.0   
THRESH_STOP  = 7.0    

_scan     = np.full(360, 50.0, dtype=np.float32)
_scan_pub = np.full(360, 50.0, dtype=np.float32)
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
        
        if 3 < dist_cm < 50: _ema(angle, dist_cm)
        
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

# [수정] 무조건 왼쪽(L)만 반환하도록 단순화
def decide_follow_side(found, cx_obj, cx_mid, last_seen_x):
    return "L"

def side_dist(scan, side):
    if side == "L":
        idx = np.arange(265, 296) % 360   
    else:
        idx = np.arange(65, 96) % 360     
    return float(np.min(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# [수정] 무조건 왼쪽 벽 추종 + 우회전 회피로 강제 고정
def wall_follow(scan, fm, found, last_seen_x, cx_mid):
    sd          = side_dist(scan, "L")
    left_close  = side_min(scan, 240, 300)   
    right_close = side_min(scan, 60, 120)    
    
    sign = 1          # 왼쪽 벽 추종 조향 방향
    turn_dir = -1.0   # 정면 회피 시 우회전 방향

    if fm < THRESH_STOP:
        return (0.01, turn_dir * 1.2)          
    if fm < THRESH_TURN:
        return (WALL_TURN_V, turn_dir * 1.0)  

    if left_close < SIDE_STOP:
        return (WALL_V * 0.5, -0.8)  
    if right_close < SIDE_STOP:
        return (WALL_V * 0.5,  0.8)  

    if left_close < BOTTLENECK_ENTER and right_close < BOTTLENECK_ENTER:
        err_center = left_close - right_close   
        w_center = float(np.clip(0.035 * err_center, -0.6, 0.6))
        return (BOTTLENECK_V, w_center)

    if sd > WALL_TARGET * 1.8:
        return (0.07, sign * WALL_LOST_W * 0.7)

    err = sd - WALL_TARGET
    w   = sign * WALL_KP * err
    
    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * turn_dir * 0.7  
        v = WALL_V * (1.0 - 0.50 * blend)          
    else:
        v = WALL_V

    if not found and fm >= THRESH_SLOW:
        is_safe = True
        if turn_dir > 0 and left_close < (WALL_TARGET * 1.2): is_safe = False
        if turn_dir < 0 and right_close < (WALL_TARGET * 1.2): is_safe = False
        if is_safe:
            w += (turn_dir * 0.15)

    w = float(np.clip(w, -1.2, 1.2))
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
    "yellow": {"hsv1": ([25, 60, 160], [32, 161, 255]),    "hsv2": None, "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([96, 100, 95], [138, 207, 246]),   "hsv2": None, "bgr":  ([40,  0,   0], [255, 220, 220]), "draw": (255, 80, 0)},
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

# ── 논문 알고리즘 적용 (복합 알고리즘 기반 장애물 회피) ────────────────────
def complex_vision_obstacle_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 1. 3x3 Sobel 연산자 윤곽선 추출
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    sobel = cv2.magnitude(sobelx, sobely)
    sobel = np.uint8(np.clip(sobel, 0, 255))
    
    # 2. NOR Convert 및 경계치 설정 (229)
    inv_sobel = cv2.bitwise_not(sobel)
    _, binary = cv2.threshold(inv_sobel, 229, 255, cv2.THRESH_BINARY)
    
    # 3. Labeling (Segmentation) 및 루프의 픽셀 면적 연산
    cnts, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    max_area = 0
    obs_cx = -1
    
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                obs_cx = int(M["m10"] / M["m00"])
                
    # 장애물이 화면의 상당부분(10^4 기준) 차지하는지 확인
    is_obstacle = max_area > 10000 
    
    return is_obstacle, obs_cx, max_area

# ── WALL FOLLOW / PARK 세부 파라미터 ───────────────────────────────────────────
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

WALL_TARGET      = 14.0    
SIDE_STOP        = 9.5     
BOTTLENECK_ENTER = 14.5    
WALL_SCAN_DIST   = 40.0    
SIDE_SCAN_DIST   = 22.0    

WALL_APPROACH_V  = 0.14    
WALL_KP          = 0.035   
WALL_V           = 0.17    

WALL_TURN_V      = 0.04    
WALL_LOST_W      = 1.2     
WALL_SEARCH_W    = 1.1     
MISSION_TIMEOUT_SEC = 10.0  

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"   
mission_idx   = 0
detect_count  = 0
arrive_count  = 0
lidar_state   = "WALL_SEARCH"

park_state    = "TRACK"   
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
escape_t      = None      
mission_start_t = time.time() 
hop_start_t      = None        
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

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        cv2.putText(frame, f"TARGET: {target.upper()}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)

        is_searching = (mode == "LIDAR") or (mode == "PARK" and park_state in ["WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW", "CORNER_ESCAPE"])
        
        # 주차 대상이 아닐 때만 비전 기반 전방 장애물 회피 검사 수행
        vision_obs = False
        v_cx = -1
        if is_searching:
            vision_obs, v_cx, v_area = complex_vision_obstacle_detection(frame)
            if vision_obs:
                cv2.putText(frame, f"VISION OBS: {int(v_area)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.circle(frame, (v_cx, H//2), 5, (0, 0, 255), -1)

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

        # ── 논문 복합 알고리즘 기반 강제 회피 동작 ──
        # 라이다로 잡히지 않는 특수 형태의 장애물을 비전으로 먼저 인식하여 회피
        if vision_obs and not found:
            # 장애물이 화면의 우측에 있으면 좌회전, 좌측에 있으면 우회전
            turn_speed = -1.0 if v_cx > cx_mid else 1.0
            send_cmd(0.04, turn_speed) # 속도를 줄이고 회피 회전
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        # ══ LIDAR 모드 ═════════════════════════════════════════════════════
        if mode == "LIDAR":
            if found and fm > THRESH_SLOW: detect_count += 1
            else: detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                continue

            if lidar_state == "WALL_SEARCH":
                left_close  = side_min(scan, 240, 300)
                right_close = side_min(scan, 60, 120)

                if fm < WALL_SCAN_DIST or left_close < SIDE_SCAN_DIST or right_close < SIDE_SCAN_DIST:
                    lidar_state = "WALL_APPROACH"
                else: 
                    send_cmd(0.0, -WALL_SEARCH_W)

            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, "L")
                if sd <= WALL_TARGET * 1.3: lidar_state = "WALL_FOLLOW"
                else:
                    sign = 1
                    turn_dir = -1.0
                    if fm < THRESH_STOP: send_cmd(0.01, turn_dir * 1.2)
                    elif fm < THRESH_TURN: send_cmd(WALL_APPROACH_V * 0.4, turn_dir * 0.9)
                    else: send_cmd(WALL_APPROACH_V, sign * 0.3)

            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, found, last_seen_x, cx_mid)
                send_cmd(v, w)

        # ══ PARK 모드 ═════════════════════════════════════════════════════
        elif mode == "PARK":

            if park_state == "SAFE_HOP":
                if found and fm > THRESH_SLOW:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; mission_start_t = time.time(); continue
                else: detect_count = 0

                elapsed_hop = time.time() - hop_start_t
                if elapsed_hop < 2.0: send_cmd(0.0, 1.2)
                else:
                    if fm > 100.0: send_cmd(0.0, 1.0)
                    elif fm > 40.0: send_cmd(WALL_V, 0.0)
                    else:
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
                    arrive_count = 0; detect_count = 0
                    if mission_idx < len(MISSION): park_state = "WALL_SEARCH"
                    continue

            elif park_state == "WALL_SEARCH":
                if found and fm > THRESH_SLOW:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else: detect_count = 0

                left_close  = side_min(scan, 240, 300)
                right_close = side_min(scan, 60, 120)

                if fm < WALL_SCAN_DIST or left_close < SIDE_SCAN_DIST or right_close < SIDE_SCAN_DIST:
                    park_state = "WALL_APPROACH"
                    continue
                send_cmd(0.0, -WALL_SEARCH_W)

            elif park_state == "WALL_APPROACH":
                if found and fm > THRESH_SLOW:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else: detect_count = 0

                sd = side_dist(scan, "L")
                if sd <= WALL_TARGET * 1.3: park_state = "WALL_FOLLOW"; continue

                sign = 1
                turn_dir = -1.0
                if fm < THRESH_STOP: v, w = 0.01, turn_dir * 1.2
                elif fm < THRESH_TURN: v, w = WALL_APPROACH_V * 0.4, turn_dir * 0.9
                else: v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)

            elif park_state == "WALL_FOLLOW":
                if found and fm > THRESH_SLOW:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else: 
                    detect_count = 0
                    sd = side_dist(scan, "L")
                    if fm > WALL_SCAN_DIST and sd > SIDE_SCAN_DIST:
                        park_state = "CORNER_ESCAPE"
                        escape_t = time.time()
                        continue
                v, w = wall_follow(scan, fm, found, last_seen_x, cx_mid)
                send_cmd(v, w)

            elif park_state == "CORNER_ESCAPE":
                if found and fm > THRESH_SLOW:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else: detect_count = 0

                elapsed_escape = time.time() - escape_t
                
                if elapsed_escape < 2.5:
                    sign = 1
                    v = 0.06                      
                    w = sign * WALL_LOST_W * 0.8  
                    send_cmd(v, w)
                else:
                    park_state = "WALL_SEARCH"
                    continue

            elif park_state == "TRACK":
                if not found:
                    park_state = "WALL_SEARCH"  
                    arrive_count = 0
                    continue

                if centroid_in_arrive_zone():
                    arrive_count += 1
                    if arrive_count >= ARRIVE_CONFIRM:
                        arrive_count = 0; park_state = "FORWARD"; park_t = time.time()
                        send_cmd(ARRIVE_FORWARD_V, 0.0); continue
                else:
                    arrive_count = 0
                    if fm < THRESH_TURN:
                        park_state = "WALL_APPROACH"
                        continue

                err_x = cx_obj - cx_mid
                err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                reduced_v = APPROACH_V * (1.0 - err_ratio)

                def cam_w(ex):
                    raw = -KP_ROT * ex
                    if abs(raw) < W_MIN and ex != 0: return -W_MIN if ex > 0 else W_MIN
                    return raw

                v, w = reduced_v, cam_w(err_x)
                last_cmd = (v, w)
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
