import serial
import math
import time
from collections import deque

# =========================
# SERIAL SETUP
# =========================

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

MOTOR_PORT = "/dev/serial0"
MOTOR_BAUD = 115200

lidar_ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=1)
motor_ser = serial.Serial(MOTOR_PORT, MOTOR_BAUD, timeout=1)

# =========================
# ROBOT PARAM
# =========================

SAFE_DISTANCE = 0.35   # 35cm 이하 장애물
FRONT_ANGLE_RANGE = 180

MAX_SPEED = 120
BASE_SPEED = 80

# =========================
# LIDAR PARSE (RP Lidar A1/A2 style)
# =========================
def read_lidar_scan():
    """
    return: list of (angle, distance)
    angle: 0~359
    distance: meters
    """
    scan = []

    while True:
        b = lidar_ser.read(1)
        if not b:
            continue

        # RP Lidar 패킷 시작 감지 (간단 버전)
        if b == b'\xA5':
            header = lidar_ser.read(4)
            if len(header) < 4:
                continue

            data_count = 12  # simplified assumption

            for _ in range(data_count):
                raw = lidar_ser.read(3)
                if len(raw) < 3:
                    continue

                # 거리 계산 (단순화)
                distance = raw[1] << 8 | raw[0]
                distance = distance / 1000.0  # mm -> m

                angle = (raw[2]) * 3  # simplified mapping

                if distance > 0:
                    scan.append((angle % 360, distance))

            break

    return scan


# =========================
# GAP FINDING
# =========================
def find_best_gap(scan):
    """
    returns best center angle of free space
    """

    # front filtering (±90도)
    front = []

    for angle, dist in scan:
        # front 180 degree window
        if angle <= 90 or angle >= 270:
            # normalize to -180~180
            a = angle
            if a > 180:
                a -= 360
            front.append((a, dist))

    # sort by angle
    front.sort(key=lambda x: x[0])

    # mark free space
    free_angles = []

    for a, d in front:
        if d > SAFE_DISTANCE:
            free_angles.append(a)

    if len(free_angles) < 5:
        return 0  # fallback straight

    # find largest continuous gap
    gaps = []
    start = free_angles[0]
    prev = free_angles[0]

    for a in free_angles[1:]:
        if abs(a - prev) > 15:  # gap break
            gaps.append((start, prev))
            start = a
        prev = a

    gaps.append((start, prev))

    # pick largest gap
    best_gap = max(gaps, key=lambda g: g[1] - g[0])

    center_angle = (best_gap[0] + best_gap[1]) / 2

    return center_angle


# =========================
# MOTOR CONTROL
# =========================
def set_motor(left, right):
    cmd = f"{int(left)},{int(right)}\n"
    motor_ser.write(cmd.encode())


# =========================
# STEERING CONTROL
# =========================
def control_robot(angle):
    """
    angle: -90 ~ 90
    """

    k = 1.2  # steering gain

    turn = k * angle

    left_speed = BASE_SPEED + turn
    right_speed = BASE_SPEED - turn

    # clamp
    left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))

    set_motor(left_speed, right_speed)


# =========================
# MAIN LOOP
# =========================
def main():
    while True:
        scan = read_lidar_scan()

        target_angle = find_best_gap(scan)

        print("TARGET ANGLE:", target_angle)

        control_robot(target_angle)

        time.sleep(0.05)


if __name__ == "__main__":
    main()
