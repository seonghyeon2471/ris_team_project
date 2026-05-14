import serial
import math
import time

# =========================
# SERIAL
# =========================
MOTOR_PORT = "/dev/serial0"
MOTOR_BAUD = 115200

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

motor_ser = serial.Serial(MOTOR_PORT, MOTOR_BAUD, timeout=1)
lidar_ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=1)

# =========================
# PARAMETER
# =========================
TARGET_WALL = 5.0   # cm 유지 거리
STOP_FRONT = 5.0    # 정면 장애물 기준
KP = 0.8            # 벽 추종 gain

BASE_V = 0.15       # 기본 전진 속도
MAX_W = 1.5         # 회전 제한

# =========================
# LIDAR PARSE (단순 버전)
# =========================
def read_lidar():
    """
    return:
    front, left, right (cm)
    """
    try:
        line = lidar_ser.readline().decode(errors="ignore").strip()

        # 예시 포맷 가정: "angle,distance"
        angle, dist = line.split(",")

        angle = float(angle)
        dist = float(dist)

        # 방향 필터링
        if -10 <= angle <= 10:
            return dist, None, None

        elif 80 <= angle <= 100:
            return None, dist, None

        elif 260 <= angle <= 280:
            return None, None, dist

    except:
        pass

    return None, None, None


# =========================
# MOTOR SEND
# =========================
def send_motor(v, w):
    msg = f"{v},{w}\n"
    motor_ser.write(msg.encode())


# =========================
# MAIN LOOP
# =========================
front_d = 100
left_d = 100
right_d = 100

while True:

    f, l, r = read_lidar()

    if f is not None:
        front_d = f
    if l is not None:
        left_d = l
    if r is not None:
        right_d = r

    # =========================
    # CASE 1: FRONT OBSTACLE
    # =========================
    if front_d < STOP_FRONT:

        v = 0.0
        w = 1.2   # 제자리 회전

    else:

        # =========================
        # CASE 2: BOTH WALL
        # =========================
        if left_d < 30 and right_d < 30:

            error = left_d - right_d
            w = -KP * error
            v = BASE_V

        # =========================
        # CASE 3: LEFT WALL ONLY
        # =========================
        elif left_d < 30:

            error = TARGET_WALL - left_d
            w = -KP * error
            v = BASE_V

        # =========================
        # CASE 4: RIGHT WALL ONLY
        # =========================
        elif right_d < 30:

            error = right_d - TARGET_WALL
            w = KP * error
            v = BASE_V

        # =========================
        # CASE 5: NO WALL → 직진
        # =========================
        else:
            v = BASE_V
            w = 0.0

    # clamp
    w = max(-MAX_W, min(MAX_W, w))

    send_motor(v, w)

    print(f"F:{front_d:.1f} L:{left_d:.1f} R:{right_d:.1f} | v:{v:.2f} w:{w:.2f}")

    time.sleep(0.05)
