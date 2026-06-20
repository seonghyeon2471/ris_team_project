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
EMA_ALPHA, MEDIAN_K, FRONT_RANGE = 0.35, 2, 90
THRESH_SLOW, THRESH_TURN, THRESH_STOP = 55.0, 30.0, 18.0
_scan = np.full(360, 150.0, dtype=np.float32)
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

def front_min(scan): return float(np.min(scan[np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360]))
def avoid_dir(scan): return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1
def side_dist(scan, side): return float(np.mean(scan[np.arange(85, 96) % 360 if side == "L" else np.arange(265, 276) % 360]))
def side_min(scan, start, end): return float(np.min(scan[np.arange(start, end) % 360]))

def wall_follow(scan, fm, adir, follow_side):
    sd, sign = side_dist(scan, follow_side), (1 if follow_side == "L" else -1)
    if fm < THRESH_STOP: return (0.08, adir * 1.1)
    if fm < THRESH_TURN: return (WALL_TURN_V, adir * 0.85)
    if side_min(scan, 60, 120) < THRESH_STOP: return (WALL_V * 0.7, -0.7)
    if side_min(scan, 240, 300) < THRESH_STOP: return (WALL_V * 0.7, 0.7)
    if sd > WALL_TARGET * 2.0: return (WALL_V * 0.7, sign * WALL_LOST_W)
    err = sd - WALL_TARGET
    w = np.clip(sign * WALL_KP * err, -0.9, 0.9)
    if fm < THRESH_SLOW:
        blend = np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0)
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else: v = WALL_V
    return (v, w)

# ── 탈출 로직 관련 함수 ────────────────────────────────────────────────
WALL_FOLLOW_STUCK_TURNS, ESCAPE_SEC, ESCAPE_V, ESCAPE_W = 1.0, 1.5, 0.18, 0.8
wall_follow_heading_accum, wall_follow_last_t, escape_t = 0.0, None, None

def wall_follow_track_heading(w, now):
    global wall_follow_heading_accum, wall_follow_last_t
    if wall_follow_last_t is None: wall_follow_last_t = now; return False
    dt = now - wall_follow_last_t; wall_follow_last_t = now
    wall_follow_heading_accum += w * dt
    return abs(wall_follow_heading_accum) >= WALL_FOLLOW_STUCK_TURNS * 2 * math.pi

def wall_follow_heading_reset():
    global wall_follow_heading_accum, wall_follow_last_t
    wall_follow_heading_accum, wall_follow_last_t = 0.0, None

def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4); w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── CONFIG / PARAMS ──────────────────────────────────────────────────
COLOR_CFG = {
    "red": {"hsv1": ([169, 136, 114], [179, 220, 255]), "bgr": ([20, 20, 80], [255, 255, 255])},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]), "bgr": ([0, 80, 80], [255, 255, 255])},
    "blue": {"hsv1": ([98, 100, 95], [138, 207, 246]), "bgr": ([40, 0, 0], [255, 220, 220])}
}
MISSION = ["red", "yellow", "blue"]
MIN_AREA, KP_ROT, APPROACH_V, PARK_SEC, DETECT_CONFIRM = 400, 0.030, 0.17, 2.0, 6
ARRIVE_Y_TOP, ARRIVE_X_MARGIN, ARRIVE_FORWARD_SEC, ARRIVE_FORWARD_V, ARRIVE_CONFIRM = 204, 30, 0.4, 0.12, 8
WALL_TARGET, WALL_SCAN_DIST, WALL_APPROACH_V, WALL_KP, WALL_V, WALL_TURN_V, WALL_LOST_W, WALL_SEARCH_W = 20.0, 150.0, 0.20, 0.012, 0.22, 0.10, 0.5, 1.1

mode, mission_idx, detect_count, arrive_count, follow_side = "LIDAR", 0, 0, 0, "L"
lidar_state, park_state, last_seen_x, park_t = "WALL_SEARCH", "TRACK", 160, None

try:
    while True:
        ret, frame = cap.read()
        if not ret: continue
        frame = cv2.flip(frame, 1); hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV); scan = get_scan(); fm = front_min(scan); adir = avoid_dir(scan)
        
        if mission_idx >= len(MISSION): stop_robot(); continue
        target = MISSION[mission_idx]; cfg = COLOR_CFG[target]
        mask = cv2.bitwise_and(cv2.inRange(hsv, np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])), cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1])))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts else None; found = big is not None and cv2.contourArea(big) > MIN_AREA
        if found:
            M = cv2.moments(big)
            if M["m00"] > 0: cx_obj, cy_obj = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]); last_seen_x = cx_obj

        # ── 주행 로직 ──────────────────────────────────────────────────
        if mode == "LIDAR":
            if found: detect_count += 1
            else: detect_count = 0
            if detect_count >= DETECT_CONFIRM: mode, park_state = "PARK", "TRACK"; continue
            
            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST: follow_side = "L" if side_dist(scan, "L") <= side_dist(scan, "R") else "R"; lidar_state = "WALL_APPROACH"
                else: send_cmd(0.0, WALL_SEARCH_W)
            elif lidar_state == "WALL_APPROACH":
                if side_dist(scan, follow_side) <= WALL_TARGET * 1.3: lidar_state = "WALL_FOLLOW"; wall_follow_heading_reset()
                else: v, w = WALL_APPROACH_V, (1 if follow_side == "L" else -1) * 0.3; send_cmd(v, w)
            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                if wall_follow_track_heading(w, time.time()): lidar_state = "WALL_ESCAPE"; escape_t = time.time(); wall_follow_heading_reset()
                else: send_cmd(v, w)
            elif lidar_state == "WALL_ESCAPE":
                if time.time() - escape_t >= ESCAPE_SEC or fm < THRESH_STOP: lidar_state = "WALL_SEARCH"
                else: send_cmd(ESCAPE_V, -ESCAPE_W if follow_side == "L" else ESCAPE_W)

        elif mode == "PARK":
            if park_state == "TRACK":
                if found:
                    err_x = cx_obj - 160
                    if abs(err_x) < ARRIVE_X_MARGIN and cy_obj > ARRIVE_Y_TOP: arrive_count += 1
                    else: arrive_count = 0
                    if arrive_count >= ARRIVE_CONFIRM: park_state, park_t = "FORWARD", time.time()
                    else:
                        w_cam = np.clip(-KP_ROT * err_x, -0.9, 0.9)
                        v, w = (0.1, 0.6 * w_cam + 0.4 * adir) if fm < THRESH_SLOW else (APPROACH_V * (1.0 - min(abs(err_x)/160, 1.0)), w_cam)
                        send_cmd(v, w)
                else: send_cmd(0.0, 1.0)
            elif park_state == "FORWARD":
                send_cmd(ARRIVE_FORWARD_V, 0.0)
                if time.time() - park_t >= ARRIVE_FORWARD_SEC: stop_robot(); park_state, park_t = "PARKING", time.time()
            elif park_state == "PARKING":
                stop_robot()
                if time.time() - park_t >= PARK_SEC: mission_idx += 1; park_state = "WALL_SEARCH"; arrive_count = 0; detect_count = 0
            elif park_state in ["WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW", "WALL_ESCAPE"]:
                if found: detect_count += 1
                else: detect_count = 0
                if detect_count >= DETECT_CONFIRM: park_state = "TRACK"
                else:
                    if park_state == "WALL_SEARCH":
                        if fm < WALL_SCAN_DIST: follow_side = "L" if side_dist(scan, "L") <= side_dist(scan, "R") else "R"; park_state = "WALL_APPROACH"
                        else: send_cmd(0.0, WALL_SEARCH_W)
                    elif park_state == "WALL_APPROACH":
                        if side_dist(scan, follow_side) <= WALL_TARGET * 1.3: park_state = "WALL_FOLLOW"; wall_follow_heading_reset()
                        else: send_cmd(WALL_APPROACH_V, (1 if follow_side == "L" else -1) * 0.3)
                    elif park_state == "WALL_FOLLOW":
                        v, w = wall_follow(scan, fm, adir, follow_side)
                        if wall_follow_track_heading(w, time.time()): park_state = "WALL_ESCAPE"; escape_t = time.time(); wall_follow_heading_reset()
                        else: send_cmd(v, w)
                    elif park_state == "WALL_ESCAPE":
                        if time.time() - escape_t >= ESCAPE_SEC or fm < THRESH_STOP: park_state = "WALL_SEARCH"
                        else: send_cmd(ESCAPE_V, -ESCAPE_W if follow_side == "L" else ESCAPE_W)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break
except KeyboardInterrupt: pass
finally: stop_robot(); cap.release(); lidar_ser.write(bytes([0xA5, 0x25])); cv2.destroyAllWindows()
