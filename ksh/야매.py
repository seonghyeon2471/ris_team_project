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
# LIDAR START / STOP
# =========================================
def lidar_start():
    lidar_ser.write(bytes([0xA5, 0x40]))
    time.sleep(2)
    lidar_ser.reset_input_buffer()

    lidar_ser.write(bytes([0xA5, 0x20]))
    lidar_ser.read(7)

    print("LIDAR START")

def lidar_stop():
    arduino_ser.write(b"0,0\n")
    lidar_ser.write(bytes([0xA5, 0x25]))
    print("LIDAR STOP + ROBOT STOP")

lidar_start()

# =========================================
# PARAM
# =========================================
SCAN_LIMIT = 150

SAFE_FRONT = 25
DANGER     = 15
REVERSE_D  = 10

MAX_SPEED  = 0.12
TURN_GAIN  = 1.6

# =========================================
# SCAN BUFFER
# =========================================
scan = np.full(360, SCAN_LIMIT, dtype=np.float32)

def send(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

# =========================================
# SIMPLE FILTER
# =========================================
EMA = 0.35

def update(angle, dist):
    scan[angle] = (1-EMA)*scan[angle] + EMA*dist

def front_dist():
    return np.min(scan[np.arange(-10, 11) % 360])

# =========================================
# MOTION STATE
# =========================================
STATE_LEFT  = 0
STATE_RIGHT = 1
STATE_BACK  = 2

state = STATE_LEFT
t0 = time.time()

# =========================================
# CONTROL LAW
# =========================================
def parabola_left(t):
    # 포물선: 시간이 갈수록 더 왼쪽으로
    return - (t * t) * 18   # 각도

def parabola_right(t):
    return (t * t) * 20

# =========================================
# MAIN LOOP
# =========================================
print("START")

try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

        if (raw[0] >> 2) < 3:
            continue

        angle = int(((raw[1]>>1)|(raw[2]<<7))/64) % 360
        dist  = (raw[3] | (raw[4]<<8)) / 40.0

        if 3 < dist < SCAN_LIMIT:
            update(angle, dist)

        if raw[0] & 0x01 != 1:
            continue

        f = front_dist()

        # =====================================
        # EMERGENCY STOP
        # =====================================
        if f < DANGER:
            send(-0.08, 0)
            continue

        # =====================================
        # STATE CHANGE
        # =====================================
        if state == STATE_LEFT and f < SAFE_FRONT:
            state = STATE_RIGHT
            t0 = time.time()

        elif state == STATE_RIGHT and f < SAFE_FRONT:
            state = STATE_BACK
            t0 = time.time()

        elif state == STATE_BACK and f > SAFE_FRONT:
            state = STATE_LEFT
            t0 = time.time()

        # =====================================
        # CONTROL
        # =====================================
        t = time.time() - t0

        if state == STATE_LEFT:
            target = parabola_left(t)
            v = MAX_SPEED
            w = math.radians(target) * TURN_GAIN

        elif state == STATE_RIGHT:
            target = parabola_right(t)
            v = MAX_SPEED * 0.8
            w = math.radians(target) * TURN_GAIN

        else:  # BACK
            v = -0.06
            w = 0

        send(v, w)

        print(f"state:{state} f:{f:.1f} v:{v:.2f} w:{w:.2f}")

except KeyboardInterrupt:
    send(0, 0)
    lidar_stop()
