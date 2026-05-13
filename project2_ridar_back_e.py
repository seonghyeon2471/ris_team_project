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

lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)

print("LIDAR START")

# =========================================
# ROBOT PARAMETER
# =========================================

EMERGENCY_DIST = 0.10
DEAD_END_DIST = 0.18

BASE_SPEED = 0.18

MAX_W = 1.2
TURN_GAIN = 1.5

SEARCH_MIN = -85
SEARCH_MAX = 85

SMOOTHING = 0.75

# =========================================
# MEMORY
# =========================================

path_history = deque(maxlen=300)
is_backtracking = False

# =========================================
# STATE CONTROL (핵심 추가)
# =========================================
emergency_cooldown = 0

# =========================================
# MAP
# =========================================

scan_data = [10.0] * 360
cost_map = [0.0] * 360

prev_angle = 0

# =========================================
# UTIL
# =========================================

def normalize_angle(angle):
    return angle % 360


def is_front_angle(angle):
    angle = normalize_angle(angle)
    return angle <= 90 or angle >= 270


def get_robot_radius(angle):
    angle = normalize_angle(angle)
    if angle > 180:
        angle -= 360

    a = abs(angle)

    if a <= 20:
        return 0.03
    elif a <= 55:
        return 0.06
    else:
        return 0.08


# =========================================
# COSTMAP
# =========================================

def clear_costmap():
    for i in range(360):
        cost_map[i] = 0.0


def inflate_obstacle(angle, dist):
    robot_radius = get_robot_radius(angle)

    if dist < robot_radius:
        for a in range(-90, 91):
            cost_map[normalize_angle(a)] += 999
        return

    spread = int(math.degrees(math.atan(robot_radius / max(dist, 0.01))))

    for a in range(angle - spread, angle + spread + 1):
        idx = normalize_angle(a)

        if not is_front_angle(idx):
            continue

        # 🔥 개선된 cost (거리 기반 강화)
        cost = max(0.0, (1.2 - dist)) * 5.0

        center_weight = 1.0 - abs(a - angle) / max(spread, 1)
        cost *= (1.0 + center_weight)

        cost_map[idx] += cost


def build_costmap():
    clear_costmap()

    for angle in range(360):

        if not is_front_angle(angle):
            continue

        dist = scan_data[angle]

        if dist < 0.03 or dist > 4.0:
            continue

        inflate_obstacle(angle, dist)


# =========================================
# PATH PLANNING
# =========================================

def find_best_direction():
    global prev_angle

    best_angle = None
    best_score = -1e9

    for angle in range(SEARCH_MIN, SEARCH_MAX + 1):

        idx = normalize_angle(angle)

        if not is_front_angle(idx):
            continue

        front_dist = scan_data[idx]
        cost = cost_map[idx]

        # 🔥 넓은 공간 우선 + 직진 강화
        straight_penalty = abs(angle) * 0.18

        score = (
            front_dist * 6.0
            - cost * 3.0
            - straight_penalty
        )

        if score > best_score:
            best_score = score
            best_angle = angle

    if best_angle is None:
        return None

    best_angle = prev_angle * SMOOTHING + best_angle * (1 - SMOOTHING)
    prev_angle = best_angle

    return best_angle


# =========================================
# CONTROL
# =========================================

def compute_cmd(angle):

    w = math.radians(angle) * TURN_GAIN
    w = max(min(w, MAX_W), -MAX_W)

    speed_scale = 1.0 - min(abs(angle) / 90.0, 0.8)

    front_dist = min(
        scan_data[normalize_angle(a)]
        for a in range(-8, 9)
    )

    obstacle_scale = min(front_dist / 0.8, 1.0)

    speed_scale *= obstacle_scale
    speed_scale = max(speed_scale, 0.25)

    v = BASE_SPEED * speed_scale

    return v, w


def send_cmd(v, w, record=True):
    global is_backtracking

    motor_ser.write(f"{v:.3f},{w:.3f}\n".encode())

    if record and not is_backtracking:
        path_history.append((v, w, time.time()))


def stop_robot():
    send_cmd(0.0, 0.0)


# =========================================
# EMERGENCY (핵심 수정)
# =========================================

def emergency_escape():
    global emergency_cooldown

    print("EMERGENCY")

    send_cmd(-0.08, 0.0)
    time.sleep(0.4)

    send_cmd(0.0, 0.0)
    time.sleep(0.1)

    send_cmd(0.0, 1.2)
    time.sleep(0.5)

    emergency_cooldown = 20


# =========================================
# DEAD END
# =========================================

def is_dead_end():

    front = min(scan_data[normalize_angle(a)] for a in range(-18, 19))
    left = min(scan_data[normalize_angle(a)] for a in range(55, 86))
    right = min(scan_data[normalize_angle(a)] for a in range(-85, -54))

    return front < DEAD_END_DIST and left < DEAD_END_DIST and right < DEAD_END_DIST


def backtrack():
    global is_backtracking

    print("BACKTRACK START")

    is_backtracking = True

    if len(path_history) < 10:
        emergency_escape()
        is_backtracking = False
        return

    history = list(path_history)

    for i in range(len(history) - 1, 0, -1):
        v, w, t2 = history[i]
        _, _, t1 = history[i - 1]

        dt = min(t2 - t1, 0.25)

        send_cmd(-v, -w, record=False)
        time.sleep(dt)

    stop_robot()
    time.sleep(0.3)

    send_cmd(0.0, 1.0, record=False)
    time.sleep(0.7)

    stop_robot()

    path_history.clear()
    is_backtracking = False

    print("BACKTRACK END")


# =========================================
# MAIN LOOP
# =========================================

print("FOOTPRINT NAVIGATION START")

try:
    while True:

        # cooldown 처리
        if emergency_cooldown > 0:
            emergency_cooldown -= 1
            continue

        data = lidar_ser.read(5)
        if len(data) != 5:
            continue

        s_flag = data[0] & 0x01
        s_inv_flag = (data[0] & 0x02) >> 1

        if s_inv_flag != (1 - s_flag):
            continue

        if (data[1] & 0x01) != 1:
            continue

        quality = data[0] >> 2
        if quality < 3:
            continue

        angle_q6 = (data[1] >> 1) | (data[2] << 7)
        angle = int(angle_q6 / 64.0)

        if angle < 0 or angle >= 360:
            continue

        distance_q2 = data[3] | (data[4] << 8)
        dist = (distance_q2 / 4.0) / 1000.0

        if dist < 0.03 or dist > 6.0:
            continue

        scan_data[angle] = dist

        if s_flag == 1:

            build_costmap()

            front_min = min(
                scan_data[normalize_angle(a)]
                for a in range(-10, 11)
            )

            if front_min < EMERGENCY_DIST:
                emergency_escape()
                continue

            if is_dead_end():
                print("DEAD END")
                backtrack()
                continue

            best_angle = find_best_direction()

            if best_angle is None:
                send_cmd(0.0, 1.0)
                time.sleep(0.2)
                continue

            v, w = compute_cmd(best_angle)
            send_cmd(v, w)

            print(
                f"DIR:{best_angle:6.1f} | v:{v:.2f} | w:{w:.2f} | front:{front_min:.2f}"
            )

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
    lidar_ser.close()
    motor_ser.close()
    print("END")
