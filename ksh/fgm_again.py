import serial
import math
import time
import numpy as np

# =========================
# LiDAR
# =========================
lidar = serial.Serial("/dev/ttyUSB0", 460800, timeout=1)

# =========================
# Arduino
# =========================
motor = serial.Serial("/dev/serial0", 115200, timeout=1)

# =========================
# PARAM
# =========================
FOV = 120              # deg
MAX_DIST = 2.0         # m
THRESHOLD = 0.7        # obstacle 판단
CENTER = 180           # LiDAR 기준 (0~360 가정)

BASE_V = 0.15

# =========================
# parse dummy (너 코드에 맞게 수정 필요)
# =========================
def get_lidar_points():
    # 실제 rplidar lib 쓰면 scan으로 교체
    # (angle, distance)
    return []

# =========================
# filter front 120deg
# =========================
def filter_front(points):
    filtered = []

    for angle, dist in points:

        if dist == 0 or dist > MAX_DIST:
            continue

        # front 120 deg
        if -60 <= angle <= 60:
            filtered.append((angle, dist))

    return filtered

# =========================
# gap detection
# =========================
def extract_gaps(data):
    gaps = []
    current = []

    for angle, dist in data:

        free = dist > THRESHOLD

        if free:
            current.append((angle, dist))
        else:
            if len(current) > 0:
                gaps.append(current)
                current = []

    if len(current) > 0:
        gaps.append(current)

    return gaps

# =========================
# choose best gap
# =========================
def select_gap(gaps):

    best_score = -999
    best = None

    for g in gaps:

        angles = [a for a, d in g]
        dists  = [d for a, d in g]

        center = (min(angles) + max(angles)) / 2
        width  = abs(max(angles) - min(angles))
        dmin   = min(dists)

        # GRP style weighting
        score = width * (dmin + 0.1)

        if score > best_score:
            best_score = score
            best = (center, dmin, width)

    return best

# =========================
# control law (GRP simplified)
# =========================
def control(gap):

    if gap is None:
        return 0.15, 0

    center, dmin, width = gap

    # GRP weighting
    alpha = 1.2
    beta  = 0.6

    phi_gap = center
    phi_ref = 0

    w = alpha * phi_gap + beta * (1.0 / (dmin + 0.01))

    # normalize
    w = w / 100.0

    v = BASE_V

    return v, w

# =========================
# send to arduino
# =========================
def send(v, w):
    msg = f"{v},{w}\n"
    motor.write(msg.encode())

# =========================
# MAIN LOOP
# =========================
while True:

    points = get_lidar_points()
    front = filter_front(points)
    gaps = extract_gaps(front)
    best = select_gap(gaps)

    v, w = control(best)

    send(v, w)

    print("v:", v, "w:", w)

    time.sleep(0.1)
