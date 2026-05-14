import serial
import math
import time
import numpy as np

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# =========================================
# LIDAR START
# =========================================
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()

lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)

print("LIDAR START")

# =========================================
# PARAMETER
# =========================================
SCAN_LIMIT = 150

MAX_SPEED = 0.20
MIN_SPEED = 0.07
MAX_W     = 1.5
TURN_GAIN = 1.8

FRONT_RANGE = 60

# filter
EMA_ALPHA = 0.3
MEDIAN_K  = 2

# obstacle
SAFE_DIST = 17
INFLATION_MAX_DIST = 25

FRONT_CLEAR_DIST  = 23
FRONT_CLEAR_RANGE = 15

# emergency
EMERGENCY_DIST = 10   # cm (7cm → 너무 타이트해서 보정)
REVERSE_SPEED  = -0.10
REVERSE_TIME   = 0.2
ROTATE_TIME    = 1.0
ROTATE_W       = 0.9

# footprint
ROBOT_HALF_WIDTH = 0.10
SAFE_MARGIN = 0.05
SAFE_WIDTH = ROBOT_HALF_WIDTH + SAFE_MARGIN

# smoothing
SMOOTHING = 0.55

# =========================================
# STATE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state = STATE_NORMAL
t_end = 0
rotate_dir = 1

scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0

# =========================================
# UTIL
# =========================================
def normalize(a):
    return int(a) % 360

# =========================================
# LIDAR PARSER (RAW FIXED)
# =========================================
def read_lidar_packet():
    raw = lidar_ser.read(5)
    if len(raw) != 5:
        return None

    s_flag = raw[0] & 0x01
    s_inv  = (raw[0] & 0x02) >> 1

    if s_inv != (1 - s_flag):
        return None

    if (raw[1] & 0x01) != 1:
        return None

    quality = raw[0] >> 2
    if quality < 3:
        return None

    angle_raw = (raw[1] >> 1) | (raw[2] << 7)
    angle = int(angle_raw / 64.0) % 360

    dist_raw = raw[3] | (raw[4] << 8)
    dist_cm = (dist_raw / 4.0) / 10.0

    if dist_cm < 3 or dist_cm > SCAN_LIMIT:
        return None

    return angle, dist_cm, s_flag

# =========================================
# SCAN UPDATE (EMA)
# =========================================
def update_scan():
    pkt = read_lidar_packet()
    if pkt is None:
        return False

    angle, dist, s_flag = pkt

    scan_data[angle] = (
        (1 - EMA_ALPHA) * scan_data[angle]
        + EMA_ALPHA * dist
    )

    return s_flag == 1

# =========================================
# FILTER
# =========================================
def median_filter():
    k = MEDIAN_K
    tmp = np.copy(scan_data)

    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k+1)]
        tmp[i] = np.median(scan_data[idx])

    scan_data[:] = tmp

# =========================================
# INFLATION
# =========================================
def inflate(dists):
    out = dists.copy()

    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d > INFLATION_MAX_DIST:
            continue

        alpha = math.degrees(math.asin(min(ROBOT_HALF_WIDTH / d, 1.0)))

        s = max(0, int(i - alpha))
        e = min(len(dists)-1, int(i + alpha))

        out[s:e+1] = 0

    return out

# =========================================
# GAP SEARCH
# =========================================
def find_gaps(dists):
    gaps = []
    start = None

    for i, d in enumerate(dists):
        if d > SAFE_DIST:
            if start is None:
                start = i
        else:
            if start is not None:
                gaps.append((start, i-1))
                start = None

    if start is not None:
        gaps.append((start, len(dists)-1))

    return gaps

# =========================================
# GAP SCORE
# =========================================
def score(gap, dists):
    s, e = gap
    w = e - s
    center = (s + e) / 2
    center_d = dists[int(center)]
    avg = np.mean(dists[s:e+1])
    mn  = np.min(dists[s:e+1])

    return w*1.5 + avg*2.0 + mn - abs(center-180)*0.15

# =========================================
# SELECT GAP
# =========================================
def select_gaps(gaps, dists):
    best = None
    best_s = -1e9

    for g in gaps:
        s = score(g, dists)
        if s > best_s:
            best_s = s
            best = g

    return best

# =========================================
# CONTROL
# =========================================
def control(target):
    global prev_angle

    w = math.radians(target) * TURN_GAIN
    w = np.clip(w, -MAX_W, MAX_W)

    if abs(target) > 15:
        return 0.0, w

    front = np.min(scan_data[170:190])

    v = MAX_SPEED if front > 40 else MIN_SPEED

    return v, w

# =========================================
# SEND
# =========================================
def send(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

# =========================================
# ESCAPE DIR
# =========================================
def escape():
    left = np.mean(scan_data[60:120])
    right = np.mean(scan_data[240:300])
    return 1 if left > right else -1

# =========================================
# MAIN LOOP
# =========================================
print("START")

while True:

    ok = update_scan()
    if not ok:
        continue

    median_filter()

    front_min = np.min(scan_data[170:190])

    now = time.time()

    # =========================
    # EMERGENCY
    # =========================
    if state == STATE_NORMAL and front_min < EMERGENCY_DIST:

        rotate_dir = escape()
        state = STATE_REVERSE
        t_end = now + REVERSE_TIME

    # =========================
    # REVERSE
    # =========================
    if state == STATE_REVERSE:
        if now < t_end:
            send(REVERSE_SPEED, 0)
            continue
        else:
            state = STATE_ROTATE
            t_end = now + ROTATE_TIME
            continue

    # =========================
    # ROTATE
    # =========================
    if state == STATE_ROTATE:
        if now < t_end:
            send(0, ROTATE_W * rotate_dir)
            continue
        else:
            state = STATE_NORMAL
            continue

    # =========================
    # NORMAL
    # =========================
    d = inflate(scan_data)
    gaps = find_gaps(d)

    if not gaps:
        send(REVERSE_SPEED, 0)
        continue

    best = select_gaps(gaps, d)

    s, e = best
    target = (s + e) / 2

    target = (target - 180) * SMOOTHING

    v, w = control(target)

    send(v, w)

    print("v:", v, "w:", w, "front:", front_min)
