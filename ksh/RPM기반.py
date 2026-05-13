import serial
import time
import math
from rplidar import RPLidar

# =========================
# PORT
# =========================
LIDAR_PORT = "/dev/ttyUSB0"
MOTOR_PORT = "/dev/serial0"
BAUD = 115200

lidar = RPLidar(LIDAR_PORT)
motor_ser = serial.Serial(MOTOR_PORT, BAUD, timeout=1)

# =========================
# BASE SPEED
# =========================
BASE_RPM = 80

# steering sensitivity
K = 0.8

# obstacle influence range
MAX_DIST = 2000  # mm
MIN_DIST = 150    # 너무 가까우면 강하게 반응

# =========================
def weight(dist):
    """가까울수록 영향 크게"""
    if dist < 1:
        return 0
    if dist > MAX_DIST:
        return 0
    return (MAX_DIST - dist) / MAX_DIST

# =========================
def compute_steering(scan):
    """
    좌/우 장애물 비대 차이 계산
    """

    left = 0
    right = 0

    for angle, dist in scan:
        if dist == 0:
            continue

        w = weight(dist)

        # 오른쪽 (0~180)
        if 0 <= angle < 180:
            right += w
        else:
            left += w

    return left, right

# =========================
def safe_mode(left, right):
    """
    너무 가까우면 급회전 강화
    """
    return (right - left)

# =========================
def send_rpm(left_rpm, right_rpm):
    cmd = f"{int(left_rpm)},{int(right_rpm)}\n"
    motor_ser.write(cmd.encode())

# =========================
try:
    print("LiDAR steering started")

    for scan in lidar.iter_scans():

        data = [(angle, dist) for _, angle, dist in scan]

        left_score, right_score = compute_steering(data)

        # =========================
        # steering logic
        # =========================
        turn = safe_mode(left_score, right_score)

        left_rpm = BASE_RPM - K * turn
        right_rpm = BASE_RPM + K * turn

        # =========================
        # clamp
        # =========================
        left_rpm = max(0, min(150, left_rpm))
        right_rpm = max(0, min(150, right_rpm))

        send_rpm(left_rpm, right_rpm)

        print(f"L:{left_rpm:.1f} R:{right_rpm:.1f} | L:{left_score:.2f} R:{right_score:.2f}")

except KeyboardInterrupt:
    print("Stopping")

finally:
    lidar.stop()
    lidar.disconnect()
    motor_ser.close()
