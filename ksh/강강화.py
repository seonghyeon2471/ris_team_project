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
# ROBOT PARAM
# =========================================
ROBOT_RADIUS = 17.0
WHEEL_BASE   = 17.0
LIDAR_TO_REAR_AXLE = 13.5   # ★ 핵심 추가

# =========================================
# DRIVE PARAM
# =========================================
MAX_SPEED = 0.14
MIN_SPEED = 0.05
MAX_W     = 1.5
TURN_GAIN = 1.8

SCAN_LIMIT  = 150
FRONT_RANGE = 60

# =========================================
# FILTER
# =========================================
EMA_ALPHA = 0.3
MEDIAN_K  = 2

# =========================================
# SAFETY PARAM
# =========================================
SAFE_DIST          = 15
INFLATION_MAX_DIST = 20
DANGER_DIST        = 10
EMERGENCY_DIST     = 7

FRONT_CLEAR_DIST  = 23
FRONT_CLEAR_RANGE = 15

# =========================================
# STATE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state = STATE_NORMAL
maneuver_end_time = 0
rotate_dir = 1

scan_data  = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0
v_cmd = 0.0
w_cmd = 0.0

# =========================================
# UTIL
# =========================================
def apply_ema(angle, dist):
    scan_data[angle] = (1-EMA_ALPHA)*scan_data[angle] + EMA_ALPHA*dist

def apply_median():
    k = MEDIAN_K
    out = np.empty(360)
    for i in range(360):
        idx = [(i+d)%360 for d in range(-k,k+1)]
        out[i] = np.median(scan_data[idx])
    scan_data[:] = out

# =========================================
# ★ 핵심: 동적 팽창 반경
# =========================================
def get_turn_inflation_radius(v, w):
    if abs(w) < 0.05:
        return ROBOT_RADIUS

    R = abs(v / w)
    rear_orbit = math.sqrt(R**2 + LIDAR_TO_REAR_AXLE**2)
    return ROBOT_RADIUS + (rear_orbit - R)

# =========================================
# INFLATION
# =========================================
def inflate(dists, radius):
    out = dists.copy()
    for i, d in enumerate(dists):
        if d < 5 or d > INFLATION_MAX_DIST:
            continue
        alpha = math.degrees(math.asin(min(radius/d, 1.0)))
        s = max(0, int(i-alpha))
        e = min(len(dists)-1, int(i+alpha))
        out[s:e+1] = 0
    return out

# =========================================
# GAP
# =========================================
def find_gaps(dists):
    gaps = []
    start = None
    for i,d in enumerate(dists):
        if d > SAFE_DIST:
            if start is None:
                start = i
        else:
            if start is not None:
                gaps.append((start,i-1))
                start = None
    if start is not None:
        gaps.append((start,len(dists)-1))
    return gaps

def score(gap, dists):
    s,e = gap
    width = e-s
    center = (s+e)//2
    return width + np.mean(dists[s:e+1]) - abs(center*0.1)

def best_gap(gaps, dists):
    return max(gaps, key=lambda g: score(g,dists))

# =========================================
# PLANNING
# =========================================
def find_best(v_prev, w_prev, smoothing=0.55):
    global prev_angle

    angles = np.arange(-FRONT_RANGE, FRONT_RANGE+1)
    dists  = np.array([scan_data[a%360] for a in angles])

    radius = get_turn_inflation_radius(v_prev, w_prev)
    proc   = inflate(dists, radius)

    gaps = find_gaps(proc)
    if not gaps:
        return None

    s,e = best_gap(gaps, proc)
    gap_angle = float(angles[(s+e)//2])

    front = np.min(scan_data[np.arange(-10,11)%360])

    if front > FRONT_CLEAR_DIST:
        target = gap_angle * 0.3
    else:
        target = gap_angle

    target = prev_angle*smoothing + target*(1-smoothing)
    prev_angle = target

    return target, front

# =========================================
# CONTROL
# =========================================
def compute(v_target):
    w = math.radians(v_target) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    front_min = np.min(scan_data[np.arange(-10,11)%360])

    if abs(v_target) > 10:
        v = 0.02 if front_min > 20 else 0
    else:
        v = max(MIN_SPEED, min(MAX_SPEED, front_min/40))

    return v, w

def send(v,w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

# =========================================
# MAIN
# =========================================
print("START")

try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

        if (raw[0] >> 2) < 3:
            continue

        angle = int(((raw[1]>>1)|(raw[2]<<7))/64)%360
        dist  = (raw[3]|(raw[4]<<8))/40.0

        if 3 < dist < SCAN_LIMIT:
            apply_ema(angle, dist)

        if raw[0] & 0x01 != 1:
            continue

        apply_median()

        now = time.time()

        front_min = np.min(scan_data[np.arange(-10,11)%360])

        # emergency
        if front_min < EMERGENCY_DIST:
            send(-0.1, 0)
            continue

        result = find_best(v_cmd, w_cmd)

        if result is None:
            send(-0.1, rotate_dir*0.9)
            continue

        target, front = result
        v_cmd, w_cmd = compute(target)
        send(v_cmd, w_cmd)

        print(f"v:{v_cmd:.2f} w:{w_cmd:.2f} front:{front:.1f}")

except KeyboardInterrupt:
    send(0,0)
    lidar_ser.write(bytes([0xA5,0x25]))
