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
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR FILTER ──────────────────────────────────────────────────────
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 45

THRESH_SLOW = 55.0
THRESH_TURN = 35.0
THRESH_STOP = 18.0
THRESH_CLEAR = 60.0

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
        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
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
    left = np.mean(scan[1:90])
    right = np.mean(scan[271:360])
    return 1 if left >= right else -1

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# ── COLOR CONFIG ───────────────────────────────────────────────────────
COLOR_CFG = {
    "red": {
        "hsv1": ([169, 168, 96], [179, 222, 157]),
        "hsv2": None,
        "bgr":  ([20, 20, 80], [255, 255, 255]),
        "draw": (0, 0, 255)
    },
    "yellow": {
        "hsv1": ([16, 137, 142], [30, 214, 195]),
        "hsv2": None,
        "bgr":  ([0, 80, 80], [255, 255, 255]),
        "draw": (0, 200, 255)
    },
    "blue": {
        "hsv1": ([106, 168, 54], [131, 210, 82]),
        "hsv2": None,
        "bgr":  ([40, 0, 0], [255, 220, 220]),
        "draw": (255, 80, 0)
    }
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
KP_ROT         = 0.003
APPROACH_V     = 0.22
PARK_SEC       = 1.2
DETECT_CONFIRM = 6
BOTTOM_10PCT   = int(240 * 0.90)

MEM_TIMEOUT    = 8.0
RELOCATE_ANGLE = 40
RELOCATE_SEC   = 3.0

# ── STATE ─────────────────────────────────────────────────────────────
mode = "SEARCH"   # SEARCH, TRACK, AVOID, RELOCATE, PARKING
mission_idx = 0
detect_count = 0

mem_valid = False
mem_x = 160
mem_y = 0
mem_t = 0

park_t = None
avoid_dir_last = 1

print(f"START | MISSION: {MISSION}")

def remember_target(ox, by_bot):
    global mem_valid, mem_x, mem_y, mem_t
    mem_valid = True
    mem_x = ox
    mem_y = by_bot
    mem_t = time.time()

def memory_alive():
    return mem_valid and (time.time() - mem_t) < MEM_TIMEOUT

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.flip(frame, 1)
        H, W = frame.shape[:2]
        cx_mid = W // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan = get_scan()
        fm = front_min(scan)
        adir = avoid_dir(scan)
        avoid_dir_last = adir

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        target = MISSION[mission_idx]
        draw = COLOR_CFG[target]["draw"]

        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        ox, by_bot, err_x = None, None, None
        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            ox = bx + bw // 2
            by_bot = min(by_top + bh, 239)
            err_x = ox - cx_mid
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)
            cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)

        # ── 장애물 우선 ───────────────────────────────────────────────
        if fm < THRESH_STOP:
            mode = "AVOID"

        if mode == "AVOID":
            v = 0.06
            w = adir * 1.15
            send_cmd(v, w)
            cv2.putText(frame, "MODE: AVOID", (10, 25), 0, 0.6, (0, 0, 255), 2)

            if fm >= THRESH_CLEAR:
                if memory_alive():
                    mode = "RELOCATE"
                else:
                    mode = "SEARCH"

        # ── SEARCH ───────────────────────────────────────────────────
        elif mode == "SEARCH":
            if found:
                remember_target(ox, by_bot)
                detect_count += 1
                if detect_count >= DETECT_CONFIRM:
                    detect_count = 0
                    mode = "TRACK"
            else:
                detect_count = 0
                if fm < THRESH_TURN:
                    v, w = 0.12, adir * 0.6
                else:
                    v, w = 0.22, 0.0
                send_cmd(v, w)

            cv2.putText(frame, "MODE: SEARCH", (10, 25), 0, 0.6, (255, 255, 0), 1)

        # ── TRACK ────────────────────────────────────────────────────
        elif mode == "TRACK":
            if found:
                remember_target(ox, by_bot)

                if by_bot >= BOTTOM_10PCT and abs(err_x) < 30:
                    stop_robot()
                    mode = "PARKING"
                    park_t = time.time()
                else:
                    if fm >= THRESH_SLOW:
                        v = APPROACH_V
                        w = -KP_ROT * err_x
                    else:
                        w_cam = -KP_ROT * err_x
                        w_lid = adir * 0.7
                        if fm < THRESH_STOP:
                            v, w = 0.09, w_lid
                        elif fm < THRESH_TURN:
                            v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                        else:
                            v, w = 0.18, 0.3 * w_lid + 0.7 * w_cam
                    send_cmd(v, w)
            else:
                mode = "RELOCATE"

            cv2.putText(frame, f"MODE: TRACK {target}", (10, 25), 0, 0.6, draw, 1)

        # ── RELOCATE ────────────────────────────────────────────────
        elif mode == "RELOCATE":
            if not memory_alive():
                mode = "SEARCH"
            else:
                err_mem = mem_x - cx_mid
                if abs(err_mem) < RELOCATE_ANGLE:
                    if found:
                        mode = "TRACK"
                        detect_count = 0
                    else:
                        v, w = 0.14, 0.0
                        send_cmd(v, w)
                else:
                    v = 0.12
                    w = -KP_ROT * err_mem
                    send_cmd(v, w)

                if time.time() - mem_t > RELOCATE_SEC and found:
                    mode = "TRACK"

            cv2.putText(frame, f"MODE: RELOCATE {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        # ── PARKING ────────────────────────────────────────────────
        elif mode == "PARKING":
            stop_robot()
            cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)
            if time.time() - park_t >= PARK_SEC:
                mission_idx += 1
                mem_valid = False
                detect_count = 0
                mode = "SEARCH"

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
