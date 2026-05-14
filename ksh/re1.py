import serial
import numpy as np
import time
import math

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.01)

# =========================================
# MAP
# =========================================
scan = np.full(360, 100.0)

MAX_DIST = 150.0

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

# =========================================
# LIDAR RESET + START
# =========================================
def start_lidar():
    lidar_ser.write(bytes([0xA5, 0x40]))  # reset
    time.sleep(2)
    lidar_ser.reset_input_buffer()

    lidar_ser.write(bytes([0xA5, 0x20]))  # scan start
    time.sleep(0.5)

# =========================================
# PARSE RAW PACKET (5 bytes)
# =========================================
def read_lidar():
    raw = lidar_ser.read(5)
    if len(raw) != 5:
        return None

    try:
        # sync check
        if (raw[0] & 0x01) != 1:
            return None

        if (raw[1] & 0x01) != 1:
            return None

        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist = (raw[3] | (raw[4] << 8)) / 40.0

        if 3 < dist < MAX_DIST:
            return angle, dist

    except:
        return None

    return None

# =========================================
# CONTROL
# =========================================
BASE_V = 0.12
KP_W = 0.02

def control():
    front = min(np.r_[scan[350:360], scan[0:10]])
    left = np.mean(scan[80:100])
    right = np.mean(scan[260:280])

    # front obstacle
    if front < 5:
        return 0.0, 1.2

    # wall follow
    if left < 30 or right < 30:

        if left < right:
            error = 5.0 - left
            w = -KP_W * error
        else:
            error = right - 5.0
            w = KP_W * error

        return BASE_V, w

    return BASE_V, 0.0

# =========================================
# MAIN
# =========================================
print("START")

start_lidar()

last_time = time.time()

try:
    while True:

        # -------- LIDAR UPDATE --------
        data = read_lidar()
        if data is not None:
            angle, dist = data
            scan[angle] = dist

        # slow down update
        if time.time() - last_time > 0.05:

            v, w = control()
            send_cmd(v, w)

            print(f"front:{np.min(np.r_[scan[350:360], scan[0:10]]):.1f} v:{v:.2f} w:{w:.2f}")

            last_time = time.time()

except KeyboardInterrupt:
    send_cmd(0, 0)
    print("STOP")
