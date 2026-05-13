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
# ROBOT SIZE
# =========================================

ROBOT_LENGTH = 0.20
ROBOT_WIDTH = 0.15

LIDAR_OFFSET = 0.015

# =========================================
# PARAMETERS
# =========================================

SAFE_RADIUS = 0.05

BASE_SPEED = 0.18

MAX_W = 1.8

TURN_GAIN = 2.0

SEARCH_MIN = -85
SEARCH_MAX = 85

SMOOTHING = 0.65

# 최소 통과 공간
GAP_THRESHOLD = 0.11

# =========================================
# MAP
# =========================================

scan_data = [3.0] * 360

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
        0 <= angle <= 90
        or
        270 <= angle <= 359
    )

# =========================================

def get_distance(angle):

    idx = normalize_angle(angle)

    return scan_data[idx]

# =========================================

def find_best_direction():

    global prev_angle

    best_angle = 0

    best_score = -999999

    # 전방 탐색
    for angle in range(
        SEARCH_MIN,
        SEARCH_MAX + 1
    ):

        dist = get_distance(angle)

        # 라이다 위치 보정
        dist -= LIDAR_OFFSET

        if dist < GAP_THRESHOLD:
            continue

        # 직진 선호
        straight_weight = (
            1.0 -
            abs(angle) / 100.0
        )

        # 거리 기반 점수
        score = (
            dist * 5.0
            +
            straight_weight * 1.5
        )

        # 가장 좋은 방향 선택
        if score > best_score:

            best_score = score

            best_angle = angle

    # 방향 smoothing
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

    # 회전 많을수록 감속
    turn_scale = (
        1.0 -
        min(abs(angle) / 90.0, 0.65)
    )

    # 전방 거리 기반 감속
    front_dist = get_distance(0)

    obstacle_scale = min(
        front_dist / 0.7,
        1.0
    )

    speed_scale = (
        turn_scale *
        obstacle_scale
    )

    # 절대 멈추지 않게 최소 속도 유지
    speed_scale = max(
        speed_scale,
        0.45
    )

    v = BASE_SPEED * speed_scale

    return v, w

# =========================================

def send_cmd(v, w):

    msg = f"{v:.3f},{w:.3f}\n"

    motor_ser.write(
        msg.encode()
    )

# =========================================

def stop_robot():

    send_cmd(0.0, 0.0)

# =========================================
# MAIN
# =========================================

print("================================")
print(" FTG START ")
print("================================")

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

        angle = int(angle_q6 / 64.0)

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

        if dist > 3.0:
            continue

        # 라이다 저장
        scan_data[angle] = dist

        # 방향 탐색
        best_angle = find_best_direction()

        # 속도 계산
        v, w = compute_cmd(best_angle)

        # 전송
        send_cmd(v, w)

        print(
            f"DIR:{best_angle:6.1f} | "
            f"v:{v:.2f} | "
            f"w:{w:.2f} | "
            f"front:{scan_data[0]:.2f}"
        )

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(
        bytes([0xA5, 0x25])
    )

    lidar_ser.close()

    motor_ser.close()

    print("END")
