import cv2
import serial
import numpy as np
import time
import math
import threading

# SERIAL
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# CAMERA
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(1.0)

# LIDAR BOOT
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR START")

# PARAMETERS
MAX_SPEED = 0.22
MIN_SPEED = 0.09
MAX_W = 0.9
THRESH_30 = 32.0
THRESH_20 = 22.0
THRESH_10 = 12.0
FRONT_CHECK_RANGE = 45
WALL_SEARCH_V = 0.15
WALL_SEARCH_W = 0.45
WALL_SEARCH_DIST = 80.0

# LIDAR FILTER
EMA_ALPHA = 0.35
MEDIAN_K = 2
scan_data = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

def apply_ema(angle, new_dist_cm):
    if not isinstance(new_dist_cm, (int, float)) or new_dist_cm <= 0:
        return
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def apply_median_filter():
    k = MEDIAN_K
    window = 2*k + 1
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i+d)%360 for d in range(-k, k+1)]
        values = np.sort(scan_data[idx])
        filtered[i] = values[window//2]
    scan_data[:] = filtered

def get_front_min():
    idx = np.arange(-FRONT_CHECK_RANGE, FRONT_CHECK_RANGE+1) % 360
    return float(np.min(scan_data[idx]))

def choose_avoid_direction():
    left_avg = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    return 1 if left_avg >= right_avg else -1

def left_dist(scan):
    idx = np.arange(85, 96) % 360
    return float(np.mean(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

def nearest_obstacle_angle(scan):
    return int(np.argmin(scan))

def nearest_obstacle_dist(scan):
    return float(np.min(scan))

# LIDAR LOOP
def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue
        s_flag = raw[0] & 0x01
        if ((raw[0]&0x02)>>1) != (1-s_flag) or (raw[1]&0x01)!=1 or (raw[0]>>2)<3:
            continue
        angle = int(((raw[1]>>1) | (raw[2]<<7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4]<<8)) / 40.0
        if 3 < dist_cm < 150:
            apply_ema(angle, dist_cm)
        if s_flag != 1:
            continue
        apply_median_filter()
        with scan_lock:
            scan_data[:] = scan_data

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock:
        return scan_data.copy()

# MOTOR
def send_cmd(v, w):
    v = np.clip(v, -MAX_SPEED, MAX_SPEED)
    w = np.clip(w, -MAX_W, MAX_W)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# WALL-FOLLOW
WALL_TARGET = 30.0
WALL_SCAN_DIST = 45.0
WALL_APPROACH_V = 0.18
WALL_KP = 0.015
WALL_V = 0.20
WALL_LOST_W = 0.8

def wall_follow(scan, fm, adir):
    ld = left_dist(scan)
    left_close = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    if fm < THRESH_10:
        return (MIN_SPEED, adir * MAX_W)
    if fm < THRESH_20:
        return (0.12, adir * 0.8)
    if left_close < THRESH_10:
        return (WALL_V * 0.7, -0.9)
    if right_close < THRESH_10:
        return (WALL_V * 0.7, 0.9)
    if ld > WALL_TARGET * 2.0:
        nearest = nearest_obstacle_angle(scan)
        err_a = nearest if nearest <= 180 else nearest - 360
        w_recover = float(np.clip(-err_a / 90.0 * WALL_LOST_W, -WALL_LOST_W, WALL_LOST_W))
        return (WALL_V * 0.6, w_recover)
    err = ld - WALL_TARGET
    w = WALL_KP * err
    if fm < THRESH_30:
        blend = float(np.clip((THRESH_30 - fm) / (THRESH_30 - THRESH_20 + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.7
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V
    w = float(np.clip(w, -1.4, 1.4))
    return (v, w)

# COLOR CONFIG
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 175], [179, 207, 255]), "hsv2": None, "bgr": ([20, 20, 80], [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 48, 193], [45, 170, 255]), "hsv2": None, "bgr": ([0, 80, 80], [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 123], [138, 207, 246]), "hsv2": None, "bgr": ([40, 0, 0], [255, 220, 220]), "draw": (255, 80, 0)},
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

# PARAMS
MIN_AREA = 400
KP_ROT = 0.035
W_MIN = 0.30
APPROACH_V = 0.17
PARK_SEC = 1.2
DETECT_CONFIRM = 6
ARRIVE_Y_TOP = int(240 * 0.85)
ARRIVE_X_MARGIN = 40
ARRIVE_FORWARD_SEC = 0.7
ARRIVE_FORWARD_V = 0.15
ARRIVE_CONFIRM = 8
FULL_CIRCLE_RAD = 2 * math.pi * 0.85
ESCAPE_RAD = math.pi * 0.6
ESCAPE_V = 0.13
ESCAPE_W = 1.70

# STATE
mode = "LIDAR"
mission_idx = 0
detect_count = 0
arrive_count = 0
park_state = "TRACK"
last_seen_x = 160
park_t = None
last_cmd = (0.0, 0.0)
wf_angle_accum = 0.0
wf_last_t = None
esc_angle_accum = 0.0
ws_start_t = None

print(f"START | MISSION: {MISSION}")  # ← 241 줄이 여기일 수 있음

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
        fm = get_front_min()
        adir = choose_avoid_direction()

        if mode == "LIDAR":
            nearest = nearest_obstacle_angle(scan)
            nd = nearest_obstacle_dist(scan)
            
            if nd < WALL_SEARCH_DIST:
                err_a = nearest if nearest <= 180 else nearest - 360
                if abs(err_a) > 10:
                    w_s = float(np.clip(-err_a / 60.0 * WALL_SEARCH_W, -WALL_SEARCH_W, WALL_SEARCH_W))
                    v_s = WALL_SEARCH_V
                else:
                    v_s = WALL_SEARCH_V * 1.2
                    w_s = 0.0
                
                if fm < THRESH_10:
                    v_s, w_s = MIN_SPEED, adir * MAX_W
                elif fm < THRESH_20:
                    v_s, w_s = 0.12, adir * 0.8
                elif fm < THRESH_30:
                    v_s, w_s = 0.15, adir * 0.7
                
                send_cmd(v_s, w_s)
            else:
                v, w = WALL_SEARCH_V, adir * 0.2
                if fm < THRESH_10:
                    v, w = MIN_SPEED, adir * MAX_W
                elif fm < THRESH_20:
                    v, w = 0.12, adir * 0.8
                elif fm < THRESH_30:
                    v, w = 0.15, adir * 0.7
                send_cmd(v, w)

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    lidar_ser.close()
    cv2.destroyAllWindows()
