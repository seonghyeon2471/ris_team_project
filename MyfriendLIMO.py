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
MIN_AREA           = 400
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

def cam_w(ex):
    if abs(ex) < DEADBAND_PX:
        return 0.0
    raw = -KP_ROT * ex
    if abs(raw) < W_MIN:
        return -W_MIN if ex > 0 else W_MIN
    return raw

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

        # ── 상태 머신 ─────────────────────────────────────────────
        if state in ("FIND_WALL", "APPROACH_WALL", "FOLLOW_WALL"):
            if found:
                detect_count += 1
                if detect_count >= DETECT_CONFIRM:
                    detect_count = 0
                    state = "TRACK"
                    continue
            else:
                detect_count = 0

            if state == "FIND_WALL":
                if fm < WALL_SCAN_DIST:
                    state = "APPROACH_WALL"
                else:
                    send_cmd(0.0, WALL_SEARCH_W)

            elif state == "APPROACH_WALL":
                wall_dist, _, valid = estimate_wall(scan)
                if valid and wall_dist <= WALL_TARGET * 1.4:
                    state = "FOLLOW_WALL"
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
                arrive_count = arrive_count + 1 if in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    state = "FORWARD"
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
                if lost_count >= LOST_CONFIRM:
                    lost_count   = 0
                    arrive_count = 0
                    state = "FIND_WALL"
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
                state = "FIND_WALL"

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
