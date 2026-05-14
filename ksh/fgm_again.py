import serial
import time
import math

# =========================
# SERIAL
# =========================
lidar = serial.Serial("/dev/ttyUSB0", 460800, timeout=1)
motor = serial.Serial("/dev/serial0", 115200, timeout=1)

# =========================
# PARAMETER
# =========================
FOV = 120
MAX_DIST = 2.0
THRESHOLD = 0.7

EMERGENCY_DIST = 0.07   # 7cm
BASE_V = 0.15

# =========================
# -------------------------
# LiDAR DATA PARSER (placeholder)
# -------------------------
def get_lidar_points():
    """
    TODO: RPLIDAR C1 library 붙이면 scan 데이터 넣기
    return [(angle, dist), ...]
    angle: -180 ~ 180 기준 변환 필요
    """
    return []

# =========================
# FILTER FRONT 120 DEG
# =========================
def filter_front(points):
    front = []

    for angle, dist in points:

        if dist <= 0 or dist > MAX_DIST:
            continue

        if -60 <= angle <= 60:
            front.append((angle, dist))

    return front

# =========================
# EMERGENCY CHECK (핵심)
# =========================
def emergency_check(front):

    for angle, dist in front:
        if -30 <= angle <= 30 and dist <= EMERGENCY_DIST:
            return True

    return False

# =========================
# ESCAPE DIRECTION (좌/우 비교)
# =========================
def escape_direction(points):

    left = []
    right = []

    for angle, dist in points:

        if 60 <= angle <= 120:
            left.append(dist)

        if -120 <= angle <= -60:
            right.append(dist)

    left_avg = sum(left)/len(left) if left else 0.01
    right_avg = sum(right)/len(right) if right else 0.01

    return 1 if left_avg > right_avg else -1

# =========================
# GAP EXTRACTION
# =========================
def extract_gaps(front):

    gaps = []
    current = []

    for angle, dist in front:

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
# GAP SELECTION (GRP STYLE)
# =========================
def select_best_gap(gaps):

    best_score = -999
    best = None

    for g in gaps:

        angles = [a for a, d in g]
        dists  = [d for a, d in g]

        center = (min(angles) + max(angles)) / 2
        width  = abs(max(angles) - min(angles))
        dmin   = min(dists)

        score = width * (dmin + 0.1)

        if score > best_score:
            best_score = score
            best = (center, dmin, width)

    return best

# =========================
# CONTROL (GRP simplified)
# =========================
def control(gap):

    if gap is None:
        return 0.12, 0

    center, dmin, width = gap

    # GRP weighting (simplified)
    alpha = 1.0
    beta  = 0.6

    w = alpha * center + beta * (1.0 / (dmin + 0.01))

    w = w / 120.0   # normalize

    v = BASE_V

    return v, w

# =========================
# SEND TO ARDUINO
# =========================
def send(v, w):
    motor.write(f"{v},{w}\n".encode())

# =========================
# MAIN LOOP
# =========================
while True:

    points = get_lidar_points()
    front = filter_front(points)

    # =========================
    # 🚨 EMERGENCY STOP
    # =========================
    if emergency_check(front):

        print("EMERGENCY!")

        send(0, 0)
        time.sleep(0.3)

        direction = escape_direction(points)

        # 회전 탈출
        for _ in range(6):
            send(0, 0.6 * direction)
            time.sleep(0.1)

        continue

    # =========================
    # GAP LOGIC
    # =========================
    gaps = extract_gaps(front)
    best_gap = select_best_gap(gaps)

    v, w = control(best_gap)

    send(v, w)

    print("v:", v, " w:", w)

    time.sleep(0.1)
