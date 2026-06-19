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
    idx = np.arange(85, 96) % 360 if side == "L" else np.arange(265, 276) % 360
    return float(np.mean(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.median(scan[idx])) 

def wall_follow(scan, fm, adir, follow_side):
    sd          = side_dist(scan, follow_side)
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1

    if fm < THRESH_STOP: return (0.08, adir * 1.1)
    if fm < THRESH_TURN: return (WALL_TURN_V, adir * 0.85)
    if left_close < THRESH_STOP: return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP: return (WALL_V * 0.7, 0.7)
    if sd > WALL_TARGET * 2.0: return (WALL_V * 0.7, sign * WALL_LOST_W)

    err = sd - WALL_TARGET
    w   = sign * WALL_KP * err
    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else: v = WALL_V
    return (v, np.clip(w, -0.9, 0.9))

def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]), "bgr": ([20, 20, 80], [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]), "bgr": ([0, 80, 80], [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 95], [138, 207, 246]), "bgr": ([40, 0, 0], [255, 220, 220]), "draw": (255, 80, 0)},
}
MISSION = ["red", "yellow", "blue"]

def make_mask(frame, hsv, name):
    cfg = COLOR_CFG[name]
    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    m = cv2.inRange(hsv, lo1, hi1)
    bm = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    return cv2.bitwise_and(m, bm)

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA           = 400
KP_ROT             = 0.030
W_MIN              = 0.20
APPROACH_V         = 0.17
PARK_SEC           = 1.2
DETECT_CONFIRM     = 6
ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8
WALL_TARGET        = 20.0
WALL_SCAN_DIST     = 150.0
WALL_APPROACH_V    = 0.20
WALL_KP            = 0.012
WALL_V             = 0.22
WALL_TURN_V        = 0.10
WALL_LOST_W        = 0.5
WALL_SEARCH_W      = 1.1

# ── STATE ─────────────────────────────────────────────────────────────
mode, mission_idx, detect_count, arrive_count = "PARK", 0, 0, 0
follow_side = "L"
park_state = "WALL_SEARCH"
last_seen_x, park_t = 160, None

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret: continue
        frame = cv2.flip(frame, 1)
        H, W = frame.shape[:2]
        
        roi_start = int(H * 0.3)
        frame_roi = frame[roi_start:H, :]
        hsv_roi = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2HSV)
        
        scan = get_scan()
        fm = front_min(scan)
        adir = avoid_dir(scan)

        if mission_idx >= len(MISSION):
            stop_robot(); cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        mask = make_mask(frame_roi, hsv_roi, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts and cv2.contourArea(max(cnts, key=cv2.contourArea)) > MIN_AREA else None
        found = big is not None
        
        if found:
            M = cv2.moments(big)
            cx_obj = int(M["m10"] / M["m00"])
            last_seen_x = cx_obj

        if park_state == "PARKING":
            stop_robot()
            if time.time() - park_t >= PARK_SEC:
                follow_side = "R" if follow_side == "L" else "L"
                mission_idx += 1
                park_state = "WALL_SEARCH"
            continue

        elif park_state == "WALL_SEARCH":
            if found: detect_count += 1
            else: detect_count = 0
            if detect_count >= DETECT_CONFIRM: park_state = "TRACK"
            
            sd_l = np.mean(scan[60:120])
            sd_r = np.mean(scan[240:300])
            if fm < WALL_SCAN_DIST and (sd_l < 100 or sd_r < 100):
                park_state = "WALL_APPROACH"
            elif (sd_l + sd_r) > 200:
                send_cmd(0.1, 0.5)
            else:
                send_cmd(0.0, WALL_SEARCH_W)

        elif park_state == "WALL_APPROACH":
            if found: park_state = "TRACK"
            sd = side_dist(scan, follow_side)
            if sd <= WALL_TARGET * 1.3: park_state = "WALL_FOLLOW"
            else:
                sign = 1 if follow_side == "L" else -1
                send_cmd(WALL_APPROACH_V, sign * 0.3)

        elif park_state == "WALL_FOLLOW":
            if found: park_state = "TRACK"
            v, w = wall_follow(scan, fm, adir, follow_side)
            send_cmd(v, w)

        elif park_state == "TRACK":
            if found:
                err_x = cx_obj - (W // 2)
                v = APPROACH_V * (1.0 - min(abs(err_x)/(W/2), 1.0))
                w = -KP_ROT * err_x
                send_cmd(v, w)
                if cv2.contourArea(big) > 3000: park_state = "FORWARD"; park_t = time.time()
            else: park_state = "SEARCH"

        elif park_state == "FORWARD":
            send_cmd(ARRIVE_FORWARD_V, 0.0)
            if time.time() - park_t >= ARRIVE_FORWARD_SEC: park_state = "PARKING"; park_t = time.time()

        elif park_state == "SEARCH":
            if found: park_state = "TRACK"
            else:
                send_cmd(0.0, (-1.0 if last_seen_x > W//2 else 1.0))

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

finally:
    stop_robot()
    cap.release()
    cv2.destroyAllWindows()
