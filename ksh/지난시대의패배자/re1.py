import serial
import numpy as np
import time
import math

# =========================
# SERIAL
# =========================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.01)

time.sleep(2)

# =========================
# MAP
# =========================
scan = np.full(360, 100.0)
MAX_DIST = 150.0

# =========================
# CONTROL PARAM
# =========================
BASE_V = 0.12
KP_W = 0.02
STOP_DIST = 5.0

# =========================
# MOTOR
# =========================
def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

# =========================
# LIDAR INIT
# =========================
def start_lidar():
    lidar_ser.write(bytes([0xA5, 0x40]))  # reset
    time.sleep(2)
    lidar_ser.reset_input_buffer()

    lidar_ser.write(bytes([0xA5, 0x20]))  # scan start
    time.sleep(1)

start_lidar()

# =========================
# RAW PARSER (SYNC SAFE)
# =========================
def read_lidar():
    while True:
        b = lidar_ser.read(1)
        if len(b) == 0:
            return None

        # sync byte detect
        if (b[0] & 0x01) != 1:
            continue

        raw = b + lidar_ser.read(4)
        if len(raw) != 5:
            return None

        try:
            angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
            dist = (raw[3] | (raw[4] << 8)) / 40.0

            if 3 < dist < MAX_DIST:
                return angle, dist

        except:
            return None

        return None

# =========================
# CONTROL
# =========================
def control():

    front = np.min(np.r_[scan[350:360], scan[0:10]])
    left = np.mean(scan[80:100])
    right = np.mean(scan[260:280])

    # ---------------------------------
    # FRONT BLOCK
    # ---------------------------------
    if front < STOP_DIST:
        return 0.0, 1.2   # rotate

    # ---------------------------------
    # WALL FOLLOW (5cm 유지)
    # ---------------------------------
    if left < 30 or right < 30:

        if left < right:
            error = 5.0 - left
            w = -KP_W * error
        else:
            error = right - 5.0
            w = KP_W * error

        return BASE_V, w

    # ---------------------------------
    # FREE RUN
    # ---------------------------------
    return BASE_V, 0.0

# =========================
# MAIN LOOP
# =========================
print("ROBOT START")

last = time.time()

try:
    while True:

        data = read_lidar()

        # update scan
        if data is not None:
            angle, dist = data
            scan[angle] = dist

        # control loop (20Hz)
        if time.time() - last > 0.05:

            v, w = control()
            send_cmd(v, w)

            front = np.min(np.r_[scan[350:360], scan[0:10]])
            print(f"front:{front:.1f} v:{v:.2f} w:{w:.2f}")

            last = time.time()

except KeyboardInterrupt:
    send_cmd(0, 0)
    print("STOP")
