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

lidar_ser.reset_input_buffer()

# SCAN START
lidar_ser.write(bytes([0xA5, 0x20]))

# descriptor 제거
lidar_ser.read(7)

print("LIDAR START")

# =========================================
# ROBOT PARAMETER
# =========================================

# 로봇 크기
# 앞뒤 20cm
# 좌우 16cm
# LiDAR는 앞에서 2.5cm 뒤

FRONT_OFFSET = 0.025
SIDE_HALF = 0.08

# =========================================
# PARAMETERS
# =========================================

EMERGENCY_DIST = 0.08
DEAD_END_DIST = 0.18

BASE_SPEED = 0.18

MAX_W = 1.2
TURN_GAIN = 1.4

SEARCH_MIN = -85
SEARCH_MAX = 85

SMOOTHING = 0.72

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

def get_robot_radius(angle):

    angle = normalize_angle(angle)

    if angle > 180:
        angle -= 360

    a = abs(angle)

    # =========================
    # FRONT
    # =========================
    if a <= 20:
        return 0.03

    # =========================
    # FRONT DIAGONAL
    # =========================
    elif a <= 55:
        return 0.06

    # =========================
    # SIDE
    # =========================
    else:
        return 0.08

# =========================================

def clear_costmap():

    global cost_map

    for i in range(360):
        cost_map[i] = 0.0

# =========================================

def inflate_obstacle(angle, dist):

    global cost_map

    robot_radius = get_robot_radius(angle)

    # 매우 가까우면 전방 전체 위험 처리
    if dist < robot_radius:

        for a in range(-90, 91):

            idx = normalize_angle(a)

            cost_map[idx] += 999

        return

    spread = math.degrees(
        math.atan(robot_radius / dist)
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
        ) * 10.0

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

        # 뒤쪽 금지
        if abs(angle) > 80:
            continue

        idx = normalize_angle(angle)

        if not is_front_angle(idx):
            continue

        cost = cost_map[idx]

        front_dist = scan_data[idx]

        straight_penalty = (
            abs(angle) * 0.06
        )

        score = (
            front_dist * 5.0
            - cost * 2.8
            - straight_penalty
        )

        if score > best_score:

            best_score = score
            best_angle = angle

    if best_angle is None:
        return None

    # steering smoothing
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

    # 회전 클수록 감속
    speed_scale = (
        1.0 -
        min(abs(angle) / 90.0, 0.75)
    )

    # 정면 최소 거리
    front_dist = min(
        scan_data[normalize_angle(a)]
        for a in range(-8, 9)
    )

    obstacle_scale = min(
        front_dist / 0.8,
        1.0
    )

    speed_scale *= obstacle_scale

    speed_scale = max(
        speed_scale,
        0.30
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

def emergency_escape():

    print("EMERGENCY")

    send_cmd(-0.06, 0.0)

    time.sleep(0.25)

    send_cmd(0.02, 1.1)

    time.sleep(0.45)

# =========================================

def is_dead_end():

    front = min(
        scan_data[normalize_angle(a)]
        for a in range(-18, 19)
    )

    left = min(
        scan_data[normalize_angle(a)]
        for a in range(55, 86)
    )

    right = min(
        scan_data[normalize_angle(a)]
        for a in range(-85, -54)
    )

    return (
        front < DEAD_END_DIST
        and
        left < DEAD_END_DIST
        and
        right < DEAD_END_DIST
    )

# =========================================

def backtrack():

    global is_backtracking

    print("BACKTRACK START")

    is_backtracking = True

    if len(path_history) < 10:

        emergency_escape()

        is_backtracking = False

        return

    history = list(path_history)

    for i in range(
        len(history) - 1,
        0,
        -1
    ):

        v, w, t2 = history[i]
        _, _, t1 = history[i - 1]

        dt = t2 - t1

        dt = min(dt, 0.25)

        send_cmd(
            -v,
            -w,
            record=False
        )

        time.sleep(dt)

    stop_robot()

    time.sleep(0.3)

    # 새 방향 탐색
    send_cmd(
        0.0,
        1.0,
        record=False
    )

    time.sleep(0.7)

    stop_robot()

    path_history.clear()

    is_backtracking = False

    print("BACKTRACK END")

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

        if quality < 3:
            continue

        angle_q6 = (
            (data[1] >> 1)
            |
            (data[2] << 7)
        )

        angle = angle_q6 / 64.0
        angle = int(angle)

        if angle < 0 or angle >= 360:
            continue

        distance_q2 = (
            data[3]
            |
            (data[4] << 8)
        )

        distance = distance_q2 / 4.0

        # mm -> m
        dist = distance / 1000.0

        # filtering
        if dist < 0.03:
            continue

        if dist > 6.0:
            continue

        scan_data[angle] = dist

        # 한 바퀴 완료 시 계산
        if s_flag == 1:

            build_costmap()

            # =========================
            # FRONT CHECK
            # =========================

            front_min = min(
                scan_data[normalize_angle(a)]
                for a in range(-10, 11)
            )

            # =========================
            # EMERGENCY
            # =========================

            if front_min < EMERGENCY_DIST:

                emergency_escape()

                continue

            # =========================
            # DEAD END
            # =========================

            if is_dead_end():

                print("DEAD END")

                backtrack()

                continue

            # =========================
            # NAVIGATION
            # =========================

            best_angle = find_best_direction()

            if best_angle is None:

                send_cmd(0.0, 1.0)

                time.sleep(0.2)

                continue

            v, w = compute_cmd(best_angle)

            send_cmd(v, w)

            print(
                f"DIR:{best_angle:6.1f} | "
                f"v:{v:.2f} | "
                f"w:{w:.2f} | "
                f"front:{front_min:.2f}"
            )

# =========================================

except KeyboardInterrupt:

    print("\nSTOP")

# =========================================

finally:

    stop_robot()

    lidar_ser.write(
        bytes([0xA5, 0x25])
    )

    lidar_ser.close()

    motor_ser.close()

    print("END")
