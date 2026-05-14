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

EMERGENCY_DIST = 0.10   # ⚠ 7cm → 실제는 10cm 추천
BASE_V = 0.15

# robot footprint
ROBOT_HALF_WIDTH = 0.10   # 20cm / 2
SAFE_MARGIN = 0.05
SAFE_WIDTH = ROBOT_HALF_WIDTH + SAFE_MARGIN

# =========================
# LiDAR PARSER (여기만 실제 라이브러리 연결)
# =========================
def get_lidar_points():
    """
    return: [(angle, dist), ...]
    angle: -180 ~ 180 기준으로 정규화 필요
    """
    return []

# =========================
# FILTER FRONT
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
# EMERGENCY CHECK
# =========================
def emergency_check(front):

    for angle, dist in front:
        if -30 <= angle <= 30 and dist <= EMERGENCY_DIST:
            return True

    return False

# =========================
# ESCAPE DIRECTION
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
# FOOTPRINT CHECK (핵심)
# =========================
def is_gap_safe(gap):

    angles = [a for a, d in gap]
    dists  = [d for a, d in gap]

    center_dist = sum(dists) / len(dists)

    required_angle = math.degrees(
        math.atan(SAFE_WIDTH / center_dist)
    )

    actual_width = abs(max(angles) - min(angles))

    return actual_width > (2 * required_angle)

# =========================
# GAP SELECTION (GRP + FOOTPRINT)
# =========================
def select_best_gap(gaps):

    best_score = -999
    best = None

    for g in gaps:

        if not is_gap_safe(g):
            continue

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

    # GRP weighting
    alpha = 1.0
    beta  = 0.6

    w = alpha * center + beta * (1.0 / (dmin + 0.01))

    w = w / 120.0

    v = BASE_V

    # slowdown in tight space
    if dmin < 0.5:
        v *= 0.7

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
    # EMERGENCY STOP
    # =========================
    if emergency_check(front):

        print("EMERGENCY STOP")

        send(0, 0)
        time.sleep(0.3)

        direction = escape_direction(points)

        # escape rotation
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
