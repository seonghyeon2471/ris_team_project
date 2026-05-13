import serial
import time
import math

# =========================
# PORT
# =========================
LIDAR_PORT = "/dev/ttyUSB0"
MOTOR_PORT = "/dev/serial0"
BAUD = 115200

lidar = serial.Serial(LIDAR_PORT, 115200, timeout=1)
motor = serial.Serial(MOTOR_PORT, BAUD, timeout=1)

# =========================
# PARAMETER
# =========================
BASE_RPM = 80

K_STEER = 1.0
SMOOTH = 0.6

MAX_DIST = 2000  # mm
MIN_FRONT = 250  # mm

prev_l = BASE_RPM
prev_r = BASE_RPM


# =========================
# LIDAR START
# =========================
def start_lidar():
    lidar.write(b'\xA5\x60')  # start scan
    time.sleep(1)


# =========================
# PARSE 5-BYTE PACKET
# =========================
def read_measurement():
    b = lidar.read(5)
    if len(b) != 5:
        return None

    if (b[0] & 0x01) != 1:
        return None

    quality = b[0] >> 2
    if quality < 10:
        return None

    angle_q6 = ((b[1] >> 1) | (b[2] << 7))
    angle = angle_q6 / 64.0

    dist = (b[3] | (b[4] << 8)) / 4.0  # mm

    if dist <= 0 or dist > MAX_DIST:
        return None

    angle = (angle + 180) % 360 - 180

    return angle, dist


# =========================
# BUILD SCAN FRAME
# =========================
def get_scan():
    scan = []

    start_time = time.time()

    while time.time() - start_time < 0.05:  # 20Hz frame
        m = read_measurement()
        if m:
            scan.append(m)

    return scan


# =========================
# SPLIT
# =========================
def split(scan):
    left, right, front = [], [], []

    for a, d in scan:
        if -30 <= a <= 30:
            front.append(d)
        elif 0 < a < 180:
            right.append(d)
        else:
            left.append(d)

    return left, right, front


# =========================
# SCORE
# =========================
def score(group):
    if not group:
        return 0
    return sum((MAX_DIST - d) / MAX_DIST for d in group) / len(group)


# =========================
# FRONT RISK
# =========================
def front_risk(front):
    if not front:
        return 0

    m = min(front)
    if m > MIN_FRONT:
        return 0

    return (MIN_FRONT - m) / MIN_FRONT


# =========================
# CONTROL
# =========================
def compute(scan):

    L, R, F = split(scan)

    left_s = score(L)
    right_s = score(R)

    turn = (right_s - left_s)

    turn += front_risk(F)

    left_rpm = BASE_RPM - K_STEER * turn * 60
    right_rpm = BASE_RPM + K_STEER * turn * 60

    left_rpm = max(0, min(150, left_rpm))
    right_rpm = max(0, min(150, right_rpm))

    return left_rpm, right_rpm


# =========================
# SMOOTH
# =========================
def smooth(prev, new):
    return SMOOTH * prev + (1 - SMOOTH) * new


# =========================
# SEND
# =========================
def send(l, r):
    motor.write(f"{int(l)},{int(r)}\n".encode())


# =========================
# MAIN
# =========================
print("RAW LIDAR DRIVE START")

start_lidar()

while True:
    try:
        scan = get_scan()

        if len(scan) < 5:
            send(0, 0)
            continue

        l, r = compute(scan)

        l = smooth(prev_l, l)
        r = smooth(prev_r, r)

        send(l, r)

        prev_l, prev_r = l, r

        print(f"L:{l:.1f} R:{r:.1f} pts:{len(scan)}")

        time.sleep(0.03)

    except KeyboardInterrupt:
        break

    except Exception as e:
        print("ERROR:", e)

send(0, 0)
lidar.close()
motor.close()
print("STOP")
