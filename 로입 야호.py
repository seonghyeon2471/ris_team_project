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
    if d > 0: _scan[a] = (1 - EMA_ALPHA) * _scan[a] + EMA_ALPHA * d

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
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3: continue
        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
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
    idx = np.arange(85, 96) % 360 if side == "L" else np.arange(265, 276) % 360
    return float(np.mean(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

def wall_follow(scan, fm, adir, follow_side):
    sd = side_dist(scan, follow_side)
    left_close = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1
    if fm < THRESH_STOP: return (0.08, adir * 1.1)
    if fm < THRESH_TURN: return (WALL_TURN_V, adir * 0.85)
    if left_close < THRESH_STOP: return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP: return (WALL_V * 0.7, 0.7)
    if sd > WALL_TARGET * 2.0: return (WALL_V * 0.7, sign * WALL_LOST_W)
    err = sd - WALL_TARGET
    w = sign * WALL_KP * err
    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else: v = WALL_V
    w = float(np.clip(w, -0.9, 0.9))
    return (v, w)

# ── 탈출 로직 관련 변수 ────────────────────────────────────────────────
WALL_FOLLOW_STUCK_TURNS = 1.0
ESCAPE_SEC = 1.5
ESCAPE_V = 0.18
ESCAPE_W = 0.8
wall_follow_heading_accum = 0.0
wall_follow_last_t = None
escape_t = None

def wall_follow_track_heading(w, now):
    global wall_follow_heading_accum, wall_follow_last_t
    if wall_follow_last_t is None: wall_follow_last_t = now; return False
    dt = now - wall_follow_last_t; wall_follow_last_t = now
    wall_follow_heading_accum += w * dt
    return abs(wall_follow_heading_accum) >= WALL_FOLLOW_STUCK_TURNS * 2 * math.pi

def wall_follow_heading_reset():
    global wall_follow_heading_accum, wall_follow_last_t
    wall_follow_heading_accum = 0.0; wall_follow_last_t = None

def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4); w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR / PARAMS ────────────────────────────────────────────────────
COLOR_CFG = {
    "red": {"hsv1": ([169, 136, 114], [179, 220, 255]), "hsv2": None, "bgr": ([20, 20, 80], [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]), "hsv2": None, "bgr": ([0, 80, 80], [255, 255, 255]), "draw": (0, 200, 255)},
    "blue": {"hsv1": ([98, 100, 95], [138, 207, 246]), "hsv2": None, "bgr": ([40, 0, 0], [255, 220, 220]), "draw": (255, 80, 0)},
}
MISSION = ["red", "yellow", "blue"]
MIN_AREA, KP_ROT, W_MIN, APPROACH_V, PARK_SEC, DETECT_CONFIRM = 400, 0.030, 0.20, 0.17, 1.2, 6
ARRIVE_Y_TOP, ARRIVE_X_MARGIN, ARRIVE_FORWARD_SEC, ARRIVE_FORWARD_V, ARRIVE_CONFIRM = int(240 * 0.85), 30, 0.8, 0.15, 8
WALL_TARGET, WALL_SCAN_DIST, WALL_APPROACH_V, WALL_KP, WALL_V, WALL_TURN_V, WALL_LOST_W, WALL_SEARCH_W = 20.0, 150.0, 0.20, 0.012, 0.22, 0.10, 0.5, 1.1

def make_mask(frame, hsv, name):
    cfg = COLOR_CFG[name]
    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    m = cv2.inRange(hsv, lo1, hi1)
    if cfg["hsv2"]:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        m = cv2.bitwise_or(m, cv2.inRange(hsv, lo2, hi2))
    bm = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    return cv2.bitwise_and(m, bm)

# ── STATE ─────────────────────────────────────────────────────────────
mode, mission_idx, detect_count, arrive_count, follow_side = "LIDAR", 0, 0, 0, "L"
lidar_state, park_state = "WALL_SEARCH", "TRACK"
last_seen_x, last_cmd = 160, (0.0, 0.0)

try:
    while True:
        ret, frame = cap.read()
        if not ret: continue
        frame = cv2.flip(frame, 1); H, W = frame.shape[:2]; cx_mid = W // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV); scan = get_scan(); fm = front_min(scan); adir = avoid_dir(scan)
        
        if mission_idx >= len(MISSION): stop_robot(); continue
        target = MISSION[mission_idx]; mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts else None; found = big is not None and cv2.contourArea(big) > MIN_AREA
        if found:
            M_mom = cv2.moments(big)
            if M_mom["m00"] > 0: cx_obj, cy_obj = int(M_mom["m10"] / M_mom["m00"]), int(M_mom["m01"] / M_mom["m00"])
            last_seen_x = cx_obj
        
        # ── LIDAR 모드 ────────────────────────────────────────────────
        if mode == "LIDAR":
            if found: detect_count += 1
            else: detect_count = 0
            if detect_count >= DETECT_CONFIRM: mode = "PARK"; park_state = "TRACK"; continue
            
            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    follow_side = "L" if side_dist(scan, "L") <= side_dist(scan, "R") else "R"
                    lidar_state = "WALL_APPROACH"
                else: send_cmd(0.0, WALL_SEARCH_W)
            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3: lidar_state = "WALL_FOLLOW"; wall_follow_heading_reset()
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP: v, w = 0.08, adir * 1.0
                    else: v, w = WALL_APPROACH_V, sign * 0.3
                    send_cmd(v, w)
            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                if wall_follow_track_heading(w, time.time()): lidar_state = "WALL_ESCAPE"; escape_t = time.time(); wall_follow_heading_reset()
                else: send_cmd(v, w)
            elif lidar_state == "WALL_ESCAPE":
                if time.time() - escape_t >= ESCAPE_SEC or fm < THRESH_STOP: lidar_state = "WALL_SEARCH"
                else: send_cmd(ESCAPE_V, -ESCAPE_W if follow_side == "L" else ESCAPE_W)

        # ── PARK 모드 ────────────────────────────────────────────────
        elif mode == "PARK":
            if park_state == "FORWARD":
                if time.time() - park_t >= ARRIVE_FORWARD_SEC: stop_robot(); park_state = "PARKING"; park_t = time.time()
                else: send_cmd(*last_cmd)
            elif park_state == "PARKING":
                stop_robot()
                if time.time() - park_t >= PARK_SEC: mission_idx += 1; park_state = "WALL_SEARCH"
            elif park_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST: follow_side = "L" if side_dist(scan, "L") <= side_dist(scan, "R") else "R"; park_state = "WALL_APPROACH"
                else: send_cmd(0.0, WALL_SEARCH_W)
            elif park_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3: park_state = "WALL_FOLLOW"; wall_follow_heading_reset()
                else:
                    sign = 1 if follow_side == "L" else -1
                    v, w = WALL_APPROACH_V, sign * 0.3
                    send_cmd(v, w)
            elif park_state == "WALL_FOLLOW":
                if found: detect_count += 1
                else: detect_count = 0
                if detect_count >= DETECT_CONFIRM: park_state = "TRACK"
                else:
                    v, w = wall_follow(scan, fm, adir, follow_side)
                    if wall_follow_track_heading(w, time.time()): park_state = "WALL_ESCAPE"; escape_t = time.time(); wall_follow_heading_reset()
                    else: send_cmd(v, w)
            elif park_state == "WALL_ESCAPE":
                if time.time() - escape_t >= ESCAPE_SEC or fm < THRESH_STOP: park_state = "WALL_SEARCH"
                else: send_cmd(ESCAPE_V, -ESCAPE_W if follow_side == "L" else ESCAPE_W)
            elif park_state == "TRACK":
                if found:
                    if (cx_obj >= cx_mid-ARRIVE_X_MARGIN and cx_obj <= cx_mid+ARRIVE_X_MARGIN and cy_obj >= ARRIVE_Y_TOP):
                        arrive_count += 1
                    else: arrive_count = 0
                    if arrive_count >= ARRIVE_CONFIRM: park_state = "FORWARD"; park_t = time.time(); send_cmd(ARRIVE_FORWARD_V, 0.0)
                    else:
                        err_x = cx_obj - cx_mid
                        w = np.clip(-KP_ROT * err_x, -0.9, 0.9)
                        v = APPROACH_V * (1.0 - min(abs(err_x)/cx_mid, 1.0))
                        last_cmd = (v, w); send_cmd(v, w)
                else: send_cmd(0.0, (-1.0 if last_seen_x > cx_mid else 1.0))

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break
except KeyboardInterrupt: pass
finally: stop_robot(); cap.release(); lidar_ser.write(bytes([0xA5, 0x25])); cv2.destroyAllWindows()
    
