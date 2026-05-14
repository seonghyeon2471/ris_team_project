import serial
import math
import time
import numpy as np

# =========================================
# SERIAL
# =========================================
arduino = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# LiDAR start
lidar.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar.reset_input_buffer()
lidar.write(bytes([0xA5, 0x20]))
lidar.read(7)

print("SYSTEM READY")

# =========================================
# PARAMETER
# =========================================
MAX_V = 0.18
MIN_V = 0.06
MAX_W = 1.25

TURN_GAIN = 1.6
SMOOTHING = 0.6

SAFE_DIST = 18
EMERGENCY_DIST = 10

ROBOT_HALF = 0.10

EMA_ALPHA = 0.25
MEDIAN_K = 2

SPRING_GAIN = 0.012

# =========================================
# STATE
# =========================================
STATE_NORMAL = 0
STATE_REVERSE = 1
STATE_ROTATE = 2

state = STATE_NORMAL
t_end = 0
rot_dir = 1

scan = np.full(360, 999.0, dtype=np.float32)

last_centers = []

yaw_i = 0
yaw_prev = 0

cen_i = 0
cen_prev = 0

# =========================================
# FRONT / ZONES
# =========================================
def front_idx():
    return list(range(330, 360)) + list(range(0, 30))

def left_zone():
    return scan[60:150]

def right_zone():
    return scan[210:300]

# =========================================
# LIDAR PARSER (FIXED ORIENTATION)
# =========================================
def read_pkt():
    raw = lidar.read(5)
    if len(raw) != 5:
        return None

    if (raw[1] & 0x01) != 1:
        return None

    q = raw[0] >> 2
    if q < 3:
        return None

    angle = ((raw[1] >> 1) | (raw[2] << 7)) // 64
    angle = (360 - angle) % 360   # 🔥 FIX

    dist = (raw[3] | (raw[4] << 8)) / 40.0

    if dist < 3 or dist > 150:
        return None

    return angle, dist, raw[0] & 1

# =========================================
# SCAN UPDATE
# =========================================
def update():
    pkt = read_pkt()
    if pkt is None:
        return False

    a, d, s = pkt
    scan[a] = (1-EMA_ALPHA)*scan[a] + EMA_ALPHA*d
    return s == 1

# =========================================
# FILTER
# =========================================
def median():
    tmp = scan.copy()

    for i in range(360):
        idx = [(i+d)%360 for d in range(-MEDIAN_K, MEDIAN_K+1)]
        tmp[i] = np.median(scan[idx])

    scan[:] = tmp

# =========================================
# BASIC FEATURES
# =========================================
def front():
    return np.min(scan[front_idx()])

def left():
    return np.mean(left_zone())

def right():
    return np.mean(right_zone())

def center_error():
    return right() - left()

# =========================================
# PID
# =========================================
def pid_y(e):
    global yaw_i, yaw_prev
    yaw_i += e
    d = e - yaw_prev
    yaw_prev = e
    return 0.018*e + 0.01*d

def pid_c(e):
    global cen_i, cen_prev
    cen_i += e
    d = e - cen_prev
    cen_prev = e
    return 0.008*e + 0.004*d

def spring():
    return SPRING_GAIN * (right() - left())

# =========================================
# INFLATION
# =========================================
def inflate(d):
    out = d.copy()

    for i in range(360):
        if d[i] < 5 or d[i] > 60:
            continue

        a = math.degrees(math.asin(min(ROBOT_HALF/d[i], 1.0)))

        s = max(0, i-int(a))
        e = min(359, i+int(a))

        out[s:e+1] = 0

    return out

# =========================================
# GAP
# =========================================
def find_gaps(d):
    g = []
    s = None

    for i in range(360):
        if d[i] > SAFE_DIST:
            if s is None:
                s = i
        else:
            if s is not None:
                g.append((s,i))
                s=None

    if s is not None:
        g.append((s,359))

    return g

def best_gap(glist, d):
    best=None
    bs=-1e9

    for s,e in glist:
        w=e-s
        avg=np.mean(d[s:e+1])
        mn=np.min(d[s:e+1])
        c=(s+e)/2

        score = w*1.5 + avg*2 + mn - abs(c-180)*0.15

        if score>bs:
            bs=score
            best=(s,e)

    return best

# =========================================
# LOOP DETECT
# =========================================
def loop_check(center):
    last_centers.append(center)
    if len(last_centers) > 10:
        last_centers.pop(0)

    if len(last_centers) < 10:
        return False

    return np.std(last_centers) < 2.0

# =========================================
# CONTROL
# =========================================
def control(target):

    w = pid_y(target) + pid_c(center_error()) + spring()
    w = np.clip(w, -MAX_W, MAX_W)

    if abs(target) > 15:
        return 0.0, w

    v = MAX_V if front() > 40 else MIN_V

    return v, w

# =========================================
# SEND
# =========================================
def send(v,w):
    arduino.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop():
    send(0,0)
    lidar.write(bytes([0xA5,0x25]))

# =========================================
# MAIN LOOP
# =========================================
print("RUNNING")

try:
    while True:

        if not update():
            continue

        median()

        f = front()

        # =========================
        # EMERGENCY
        # =========================
        if state == STATE_NORMAL and f < EMERGENCY_DIST:
            send(-0.1, 0)
            state = STATE_REVERSE
            t_end = time.time() + 0.25

        if state == STATE_REVERSE:
            if time.time() < t_end:
                send(-0.1, 0)
                continue
            state = STATE_ROTATE
            t_end = time.time() + 1.2

        if state == STATE_ROTATE:
            if time.time() < t_end:
                send(0, rot_dir * 0.85)
                continue
            state = STATE_NORMAL

        # =========================
        # NORMAL
        # =========================
        d = inflate(scan)
        g = find_gaps(d)

        if not g:
            send(-0.1, 0)
            continue

        g = best_gap(g, d)

        if g is None:
            send(-0.1, 0)
            continue

        s,e = g
        center = (s+e)/2

        if loop_check(center):
            send(-0.1, 0)
            rot_dir = 1
            state = STATE_ROTATE
            t_end = time.time() + 1.5
            continue

        target = (center - 180) * SMOOTHING

        v,w = control(target)

        send(v,w)

        print("v:",v,"w:",w,"front:",f)

except KeyboardInterrupt:
    print("STOPPED")

finally:
    stop()
    arduino.close()
    lidar.close()
