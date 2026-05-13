import serial
import math
import time
from collections import deque

# =========================================
# SERIAL
# =========================================

MOTOR_PORT = "/dev/serial0"
MOTOR_BAUD = 115200

motor_ser = serial.Serial(
    MOTOR_PORT,
    MOTOR_BAUD,
    timeout=1
)

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

lidar_ser = serial.Serial(
    LIDAR_PORT,
    LIDAR_BAUD,
    timeout=1
)

# =========================================
# LIDAR START
# =========================================

# RESET
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)

# SCAN START
lidar_ser.write(bytes([0xA5, 0x20]))

print("LIDAR START")

# =========================================
# PARAMETERS
# =========================================

SAFE_RADIUS = 0.05

EMERGENCY_DIST = 0.03
DEAD_END_DIST = 0.10

BASE_SPEED = 0.18

MAX_W = 1.3
TURN_GAIN = 1.5

SEARCH_MIN = -85
SEARCH_MAX = 85

SMOOTHING = 0.75

# =========================================
# RECOVERY PARAMETERS
# =========================================

RECOVERY_BACK_TIME = 0.28
RECOVERY_ROTATE_TIME = 0.55
RECOVERY_FORWARD_TIME = 0.35

RECOVERY_BACK_SPEED = -0.07
RECOVERY_ROTATE_W = 1.2
RECOVERY_FORWARD_SPEED = 0.08

GAP_THRESHOLD = 0.22

# =========================================
# MEMORY
# =========================================

path_history = deque(maxlen=300)

is_backtracking = False

# =========================================
# MAP
# =========================================

scan_data = [10.0] * 360
cost_map = [0.0] * 360

prev_angle = 0

# =========================================
# FUNCTIONS
# =========================================

def normalize_angle(angle):

    while angle < 0:
        angle += 360

    while angle >= 360:
        angle -= 360

    return int(angle)

# =========================================

def is_front_angle(angle):

    angle = normalize_angle(angle)

    return (
        (0 <= angle <= 90)
        or
        (270 <= angle <= 359)
    )

# =========================================

def clear_costmap():

    global cost_map

    for i in range(360):
        cost_map[i] = 0.0

# =========================================

def inflate_obstacle(angle, dist):

    global cost_map

    if dist < 0.05:

        for a in range(
            angle - 90,
            angle + 91
        ):

            idx = normalize_angle(a)

            cost_map[idx] += 999

        return

    spread = math.degrees(
        math.atan(SAFE_RADIUS / dist)
    )

    spread = int(spread)

    for a in range(
        angle - spread,
        angle + spread + 1
    ):

        idx = normalize_angle(a)

        if not is_front_angle(idx):
            continue

        cost = (
            1.0 /
            max(dist, 0.05)
        ) * 8.0

        center_weight = (
            1.0 -
            abs(a - angle)
            / max(spread, 1)
        )

        cost *= (1.0 + center_weight)

        cost_map[idx] += cost

# =========================================

def build_costmap():

    clear_costmap()

    for angle in range(360):

        if not is_front_angle(angle):
            continue

        dist = scan_data[angle]

        if dist < 0.03:
            continue

        if dist > 4.0:
            continue

        inflate_obstacle(angle, dist)

# =========================================

def find_best_direction():

    global prev_angle

    best_angle = None
    best_score = -999999

    for angle in range(
        SEARCH_MIN,
        SEARCH_MAX + 1
    ):

        idx = normalize_angle(angle)

        if not is_front_angle(idx):
            continue

        cost = cost_map[idx]

        front_dist = scan_data[idx]

        straight_penalty = (
            abs(angle) * 0.05
        )

        score = (
            front_dist * 5.0
            - cost * 2.5
            - straight_penalty
        )

        if score > best_score:

            best_score = score
            best_angle = angle

    if best_angle is None:
        return None

    best_angle = (
        prev_angle * SMOOTHING
        +
        best_angle * (1.0 - SMOOTHING)
    )

    prev_angle = best_angle

    return best_angle

# =========================================

def compute_cmd(angle):

    w = math.radians(angle) * TURN_GAIN

    w = max(
        min(w, MAX_W),
        -MAX_W
    )

    speed_scale = (
        1.0 -
        min(abs(angle) / 90.0, 0.7)
    )

    front_dist = scan_data[0]

    obstacle_scale = min(
        front_dist / 0.8,
        1.0
    )

    speed_scale *= obstacle_scale

    speed_scale = max(
        speed_scale,
        0.35
    )

    v = BASE_SPEED * speed_scale

    return v, w

# =========================================

def send_cmd(v, w, record=True):

    global is_backtracking

    msg = f"{v:.3f},{w:.3f}\n"

    motor_ser.write(msg.encode())

    if (
        record
        and
        not is_backtracking
    ):

        path_history.append(
            (v, w, time.time())
        )

# =========================================

def stop_robot():

    send_cmd(0.0, 0.0)

# =========================================

def compute_gap_score(start_angle, end_angle):

    score = 0.0

    continuous = 0
    max_continuous = 0

    for a in range(start_angle, end_angle + 1):

        idx = normalize_angle(a)

        d = scan_data[idx]

        d = min(d, 2.0)

        if d > GAP_THRESHOLD:

            continuous += 1

            score += d

        else:

            max_continuous = max(
                max_continuous,
                continuous
            )

            continuous = 0

    max_continuous = max(
        max_continuous,
        continuous
    )

    score += max_continuous * 0.35

    return score

# =========================================

def emergency_escape():

    global is_backtracking

    print("================================")
    print(" SMART RECOVERY ")
    print("================================")

    is_backtracking = True

    # BACKWARD

    send_cmd(
        RECOVERY_BACK_SPEED,
        0.0,
        record=False
    )

    time.sleep(RECOVERY_BACK_TIME)

    stop_robot()

    time.sleep(0.08)

    # SPACE ANALYSIS

    left_score = compute_gap_score(
        25,
        90
    )

    right_score = compute_gap_score(
        -90,
        -25
    )

    print(
        f"LEFT:{left_score:.2f} | "
        f"RIGHT:{right_score:.2f}"
    )

    # SELECT DIRECTION

    if left_score > right_score:

        rotate_w = RECOVERY_ROTATE_W

        print("ESCAPE LEFT")

    else:

        rotate_w = -RECOVERY_ROTATE_W

        print("ESCAPE RIGHT")

    # ROTATE

    send_cmd(
        0.02,
        rotate_w,
        record=False
    )

    time.sleep(RECOVERY_ROTATE_TIME)

    stop_robot()

    time.sleep(0.05)

    # FORWARD ESCAPE

    send_cmd(
        RECOVERY_FORWARD_SPEED,
        0.0,
        record=False
    )

    time.sleep(RECOVERY_FORWARD_TIME)

    stop_robot()

    time.sleep(0.15)

    is_backtracking = False

    print("RECOVERY END")

# =========================================
# MAIN
# =========================================

print("====================================")
print(" FOOTPRINT NAVIGATION START ")
print("====================================")

try:

    while True:

        data = lidar_ser.read(5)

        if len(data) != 5:
            continue

        s_flag = data[0] & 0x01

        s_inv_flag = (
            (data[0] & 0x02) >> 1
        )

        if s_inv_flag != (1 - s_flag):
            continue

        check_bit = data[1] & 0x01

        if check_bit != 1:
            continue

        quality = data[0] >> 2

        if quality < 10:
            continue

        angle_q6 = (
            (data[1] >> 1)
            |
            (data[2] << 7)
        )

        angle = angle_q6 / 64.0
        angle = int(angle)

        if not is_front_angle(angle):
            continue

        distance_q2 = (
            data[3]
            |
            (data[4] << 8)
        )

        distance = distance_q2 / 4.0

        dist = distance / 1000.0

        if dist < 0.03:
            continue

        if dist > 6.0:
            continue

        scan_data[angle] = dist

        build_costmap()

        # FRONT CHECK

        front_min = 10.0

        for a in range(-10, 11):

            idx = normalize_angle(a)

            d = scan_data[idx]

            if d < front_min:
                front_min = d

        # EMERGENCY

        if front_min < EMERGENCY_DIST:

            emergency_escape()

            continue

        # NORMAL NAVIGATION

        best_angle = find_best_direction()

        if best_angle is None:

            send_cmd(0.0, 1.0)

            time.sleep(0.25)

            continue

        v, w = compute_cmd(best_angle)

        send_cmd(v, w)

        print(
            f"DIR:{best_angle:6.1f} | "
            f"v:{v:.2f} | "
            f"w:{w:.2f} | "
            f"front:{front_min:.2f}"
        )

except KeyboardInterrupt:

    print("\nSTOP")

finally:

    stop_robot()

    lidar_ser.write(
        bytes([0xA5, 0x25])
    )

    lidar_ser.close()

    motor_ser.close()

    print("END")
