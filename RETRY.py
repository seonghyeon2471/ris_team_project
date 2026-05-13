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
# ROBOT
# =========================================

ROBOT_WIDTH = 0.15
ROBOT_LENGTH = 0.20

LIDAR_OFFSET = 0.015

# =========================================
# PARAMETERS
# =========================================

BASE_SPEED = 0.17

MIN_SPEED = 0.05

MAX_W = 1.6

TURN_GAIN = 1.9

FRONT_DANGER = 0.28

FRONT_CRITICAL = 0.16

SMOOTHING = 0.7

# =========================================
# MAP
# =========================================

scan_data = [3.0] * 360

prev_w = 0.0

# =========================================
# FUNCTIONS
# =========================================

def normalize_angle(angle):

    return int(angle % 360)

# =========================================

def is_front(angle):

    angle = normalize_angle(angle)

    return (
        angle <= 90
        or
        angle >= 270
    )

# =========================================

def get_distance(angle):

    idx = normalize_angle(angle)

    return scan_data[idx]

# =========================================

def average_distance(start, end):

    values = []

    for a in range(start, end + 1):

        d = get_distance(a)

        if 0.03 < d < 3.0:
            values.append(d)

    if len(values) == 0:
        return 0.03

    return sum(values) / len(values)

# =========================================

def compute_control():

    global prev_w

    # =====================================
    # 영역별 거리 계산
    # =====================================

    front = average_distance(-15, 15)

    left = average_distance(35, 90)

    right = average_distance(-90, -35)

    left_front = average_distance(10, 40)

    right_front = average_distance(-40, -10)

    # =====================================
    # 중앙 유지 steering
    # =====================================

    error = left - right

    w = error * TURN_GAIN

    # =====================================
    # 전방 장애물 회피 강화
    # =====================================

    if front < FRONT_DANGER:

        avoid = (
            left_front
            -
            right_front
        )

        w += avoid * 3.2

    # =====================================
    # 매우 가까우면 강회전
    # =====================================

    if front < FRONT_CRITICAL:

        if left > right:

            w += 1.2

        else:

            w -= 1.2

    # =====================================
    # smoothing
    # =====================================

    w = (
        prev_w * SMOOTHING
        +
        w * (1.0 - SMOOTHING)
    )

    prev_w = w

    # =====================================
    # 회전 제한
    # =====================================

    w = max(
        min(w, MAX_W),
        -MAX_W
    )

    # =====================================
    # 속도 계산
    # =====================================

    turn_scale = (
        1.0 -
        min(abs(w) / MAX_W, 0.75)
    )

    front_scale = min(
        front / 0.8,
        1.0
    )

    speed_scale = (
        turn_scale *
        front_scale
    )

    speed_scale = max(
        speed_scale,
        0.35
    )

    v = BASE_SPEED * speed_scale

    # 매우 가까우면 저속 회전
    if front < FRONT_CRITICAL:

        v = MIN_SPEED

    return v, w, front

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

print("================================")
print(" SMOOTH NAVIGATION START ")
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

        # 전방 180도만 사용
        if not is_front(angle):
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

        scan_data[angle] = dist

        # 제어 계산
        v, w, front = compute_control()

        # 모터 전송
        send_cmd(v, w)

        print(
            f"v:{v:.2f} | "
            f"w:{w:.2f} | "
            f"front:{front:.2f}"
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
