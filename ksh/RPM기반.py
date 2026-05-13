import serial
import math
import time
from collections import deque

# =========================
# SERIAL SETUP
# =========================
MOTOR_PORT = "/dev/serial0"
MOTOR_BAUD = 115200

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

motor_ser = serial.Serial(MOTOR_PORT, MOTOR_BAUD, timeout=1)
lidar_ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=1)

# =========================
# ROBOT PARAMETER
# =========================
BASE_RPM = 80
MAX_BOOST = 60

ANGLE_FRONT = 30  # 앞쪽 기준 ±30도만 사용

# =========================
# UTIL
# =========================
def send_motor(left_rpm, right_rpm):
    """
    아두이노로 UART 전송 (예: L100R120)
    """
    cmd = f"L{int(left_rpm)}R{int(right_rpm)}\n"
    motor_ser.write(cmd.encode())


def parse_lidar_frame(frame):
    """
    ⚠️ RP LiDAR raw frame parser (간단 버전 placeholder)
    실제 과제 코드에 맞게 SDK 쓰는 경우 이 부분만 교체하면 됨
    """
    # 여기서는 가정: (angle, distance)
    # 실제 구현에서는 rplidar SDK or packet parsing 필요
    return []


def get_obstacle_segments(points, threshold=800):
    """
    장애물 구간 추출:
    - distance < threshold 인 구간을 angle segment로 묶음
    """
    segments = []

    start = None

    for angle, dist in points:
        if dist < threshold:
            if start is None:
                start = angle
        else:
            if start is not None:
                segments.append((start, angle))
                start = None

    if start is not None:
        segments.append((start, 360))

    return segments


def compute_weight(segments):
    """
    좌/우 영향도 계산
    """
    left = 0
    right = 0

    for s, e in segments:
        width = abs(e - s)

        mid = (s + e) / 2

        # normalize angle
        if 0 <= mid <= 180:
            right += width
        else:
            left += width

    return left, right


def clamp(x, min_v, max_v):
    return max(min_v, min(max_v, x))


# =========================
# CONTROL LOOP
# =========================
print("start control...")

while True:

    # 1. LiDAR read (placeholder)
    raw_points = parse_lidar_frame(lidar_ser.read(1024))

    # 없으면 직진
    if len(raw_points) == 0:
        send_motor(BASE_RPM, BASE_RPM)
        continue

    # 2. 장애물 구간 추출
    segments = get_obstacle_segments(raw_points)

    # 3. 좌/우 영향도
    left_w, right_w = compute_weight(segments)

    # 4. 방향 결정 (핵심)
    diff = right_w - left_w

    # scaling
    boost = clamp(abs(diff) * 0.5, 0, MAX_BOOST)

    left_rpm = BASE_RPM
    right_rpm = BASE_RPM

    # 오른쪽 장애물이 많다 → 왼쪽으로 회전 (right speed 증가)
    if diff > 0:
        right_rpm += boost
        left_rpm -= boost * 0.3

    # 왼쪽 장애물이 많다 → 오른쪽 회전
    elif diff < 0:
        left_rpm += boost
        right_rpm -= boost * 0.3

    # clamp
    left_rpm = clamp(left_rpm, 0, 200)
    right_rpm = clamp(right_rpm, 0, 200)

    # 5. motor send
    send_motor(left_rpm, right_rpm)

    time.sleep(0.03)
