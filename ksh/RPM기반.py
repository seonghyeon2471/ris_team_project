import serial
import time
import math
from rplidar import RPLidar

# =========================
# PORT
# =========================
LIDAR_PORT = "/dev/ttyUSB0"
MOTOR_PORT = "/dev/serial0"
BAUD = 115200

lidar = RPLidar(LIDAR_PORT)
motor = serial.Serial(MOTOR_PORT, BAUD, timeout=1)

# =========================
# PARAMETER
# =========================
BASE_RPM = 80

K_STEER = 0.9          # steering gain
K_FRONT = 1.5          # front avoidance gain

MAX_DIST = 2000        # mm
MIN_SAFE = 250         # mm (정면 안전거리)

SMOOTH_ALPHA = 0.6     # smoothing

prev_left = BASE_RPM
prev_right = BASE_RPM


# =========================
def weight(dist):
    if dist <= 0 or dist > MAX_DIST:
        return 0
    return (MAX_DIST - dist) / MAX_DIST


# =========================
def split_scan(scan):
    left, right, front = [], [], []

    for angle, dist in scan:
        if dist == 0:
            continue

        if -30 <= angle <= 30:
            front.append(dist)
        elif 0 < angle < 180:
            right.append(dist)
        else:
            left.append(dist)

    return left, right, front


# =========================
def score(side):
    if len(side) == 0:
        return 0
    return sum(weight(d) for d in side) / len(side)


# =========================
def front_risk(front):
    if len(front) == 0:
        return 0

    min_d = min(front)
    if min_d > MIN_DIST:
        return 0

    return (MIN_DIST - min_d) / MIN_DIST


# =========================
def compute_control(scan):
    left_s, right_s, front_s = split_scan(scan)

    L = score(left_s)
    R = score(right_s)

    turn = (R - L)

    # front emergency bias
    f_risk = front_risk(front_s)
    turn += f_risk

    left_rpm = BASE_RPM - K_STEER * turn * 50
    right_rpm = BASE_RPM + K_STEER * turn * 50

    # clamp
    left_rpm = max(0, min(150, left_rpm))
    right_rpm = max(0, min(150, right_rpm))

    return left_rpm, right_rpm, L, R, f_risk


# =========================
def smooth(prev, new):
    return SMOOTH_ALPHA * prev + (1 - SMOOTH_ALPHA) * new


# =========================
def send(l, r):
    motor.write(f"{int(l)},{int(r)}\n".encode())


# =========================
try:
    print("Stable steering start")

    for scan in lidar.iter_scans():

        data = [(a, d) for _, a, d in scan]

        left, right, L, R, f = compute_control(data)

        # smoothing
        left = smooth(prev_left, left)
        right = smooth(prev_right, right)

        send(left, right)

        prev_left, prev_right = left, right

        print(f"L:{left:.1f} R:{right:.1f} | Ls:{L:.2f} Rs:{R:.2f} | front:{f:.2f}")

        time.sleep(0.03)

except KeyboardInterrupt:
    print("STOP")

finally:
    send(0, 0)
    lidar.stop()
    lidar.disconnect()
    motor.close()
