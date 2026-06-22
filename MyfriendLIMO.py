#준수
import cv2
import serial
import numpy as np
import time
import math
import threading

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

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
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR ─────────────────────────────────────────────────────────────
EMA_ALPHA = 0.35
MEDIAN_K = 2
FRONT_RANGE = 90   
THRESH_SLOW = 55.0
THRESH_TURN = 30.0
THRESH_STOP = 18.0

_scan = np.full(360, 150.0, dtype=np.float32)
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
    if side == "L":
        idx = np.arange(65, 96) % 360
    else:
        idx = np.arange(265, 296) % 360
    return float(np.min(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

def wall_follow(scan, fm, adir, follow_side):
    sd = side_dist(scan, follow_side)
    left_close = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1

    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

    if sd > WALL_TARGET * 2.0:
        return (0.05, sign * WALL_LOST_W)

    err = sd - WALL_TARGET
    w = sign * WALL_KP * err
    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V
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

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA = 400
KP_ROT = 0.030
W_MIN = 0.10
APPROACH_V = 0.13
PARK_SEC = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V = 0.15
ARRIVE_CONFIRM = 8

WALL_TARGET = 15.0
WALL_SCAN_DIST = 60.0  
WALL_APPROACH_V = 0.20  
WALL_KP = 0.012
WALL_V = 0.22
WALL_TURN_V = 0.10
WALL_LOST_W = 0.9     
WALL_SEARCH_W = 0.9     
MISSION_TIMEOUT_SEC = 10.0  

# ── STATE ─────────────────────────────────────────────────────────────
mode = "LIDAR"   
mission_idx = 0
detect_count = 0
arrive_count = 0
follow_side = "L"   
lidar_state = "WALL_SEARCH"

park_state = "TRACK"   
last_seen_x = 160
last_bottom_y = 0
park_t = None
search_t = None      
mission_start_t = time.time() 
hop_start_t = None        
last_cmd = (0.0, 0.0)

# ★ 정체 상태 탈출을 위한 변수
STUCK_TIMEOUT = 6.0          # 같은 상태 지속 제한 시간 (6초)
ESCAPE_DURATION = 1.5        # 강제 회전 및 후진 탈출 시간 (1.5초)
last_state_t = time.time()   # 상태가 마지막으로 바뀐 시간 저장
prev_mode = mode
prev_lidar_state = lidar_state
prev_park_state = park_state
escape_t = None              # 탈출 시작 시간 저장
escape_dir = 1               # 탈출 회전 방향

print(f"START | MISSION: {MISSION}")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame = cv2.flip(frame, 1)
        H, W = frame.shape[:2]
        cx_mid = W // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan = get_scan()
        fm = front_min(scan)
        adir = avoid_dir(scan)

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw = COLOR_CFG[target]["draw"]

        cv2.putText(frame, f"TARGET: {target.upper()}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)

        # ── 타겟 오브젝트 검출 ──
        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        # ★ [수정 및 통합] 정체 탈출 판단 로직 (중복 블록 제거 및 50cm 조건 최적화)
        if not found and park_state != "ESCAPE":
            if (mode != prev_mode) or (mode == "LIDAR" and lidar_state != prev_lidar_state) or (mode == "PARK" and park_state != prev_park_state):
                last_state_t = time.time()  
                prev_mode = mode
                prev_lidar_state = lidar_state
                prev_park_state = park_state
            
            current_stuck_state = (lidar_state in ["WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW"]) if mode == "L
