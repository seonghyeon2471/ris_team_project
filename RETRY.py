# =========================================
# 필요한 라이브러리
# =========================================

import serial
import math
import time

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

lidar_ser.write(bytes([0xA5, 0x20]))

print("LIDAR START")

# =========================================
# PARAMETERS
# =========================================

# 장애물 안전 반경
SAFE_RADIUS = 0.035

# 기본 전진 속도
BASE_SPEED = 0.15

# 최대 회전 속도
MAX_W = 1.5

# 회전 민감도
TURN_GAIN = 1.8

# 탐색 범위
SEARCH_MIN = -85
SEARCH_MAX = 85

# 조향 부드럽게
SMOOTHING = 0.75

# 통과 가능 거리
GAP_THRESHOLD = 0.14

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

    # 조향 부드럽게
    best_angle = (
        prev_angle * SMOOTHING
        +
        best_angle * (1.0 - SMOOTHING)
    )

    prev_angle = best_angle

    return best_angle

# =========================================

def compute_cmd(angle):

    # 회전 계산
    w = math.radians(angle) * TURN_GAIN

    w = max(
        min(w, MAX_W),
        -MAX_W
    )

    # 회전 많으면 감속
    speed_scale = (
        1.0 -
        min(abs(angle) / 90.0, 0.7)
    )

    # 전방 거리 기반 감속
    front_dist = scan_data[0]

    obstacle_scale = min(
        front_dist / 0.8,
        1.0
    )

    speed_scale *= obstacle_scale

    # 최소 속도 보장
    speed_scale = max(
        speed_scale,
        0.25
    )

    v = BASE_SPEED * speed_scale

    return v, w

# =========================================

def send_cmd(v, w):

    msg = f"{v:.3f},{w:.3f}\n"

    motor_ser.write(msg.encode())

# =========================================

def stop_robot():

    send_cmd(0.0, 0.0)

# =========================================
# MAIN
# =========================================

print("====================================")
print(" FTG NAVIGATION START ")
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

        # 라이다 데이터 저장
        scan_data[angle] = dist

        # 장애물 맵 생성
        build_costmap()

        # 최적 방향 탐색
        best_angle = find_best_direction()

        # 방향 못찾으면 회전
        if best_angle is None:

            send_cmd(0.0, 1.0)

            time.sleep(0.03)

            continue

        # 속도 계산
        v, w = compute_cmd(best_angle)

        # 모터 전송
        send_cmd(v, w)

        print(
            f"DIR:{best_angle:6.1f} | "
            f"v:{v:.2f} | "
            f"w:{w:.2f}"
        )

# =========================================
# 종료
# =========================================

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
