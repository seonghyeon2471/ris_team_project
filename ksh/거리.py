import serial

import math

import time

import numpy as np

# =========================================

# SERIAL

# =========================================

arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

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

# PARAMETERS (당신이 수정한 값 유지 + 약간 정리)

# =========================================

MAX_SPEED = 0.40 # ← 빠르게 가고 싶으면 그대로, 불안하면 0.28~0.32로 낮추세요

MIN_SPEED = 0.09

MAX_W = 1.6

THRESH_30 = 25.0 # 25cm부터 중간 조향

THRESH_20 = 20.0

THRESH_10 = 10.0

FRONT_CHECK_RANGE = 45 # 전방 체크 범위

# FILTER

EMA_ALPHA = 0.35

MEDIAN_K = 2

scan_data = np.full(360, 150.0, dtype=np.float32)

# =========================================

# UTIL

# =========================================

def apply_ema(angle, new_dist_cm):

    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def apply_median_filter():

    k = MEDIAN_K

    window = 2 * k + 1

    filtered = np.empty(360, dtype=np.float32)

    for i in range(360):

        indices = [(i + d) % 360 for d in range(-k, k + 1)]

        values = np.sort(scan_data[indices])

        filtered[i] = values[window // 2]

    scan_data[:] = filtered

def get_front_min():

    indices = np.arange(-FRONT_CHECK_RANGE, FRONT_CHECK_RANGE + 1) % 360

    return float(np.min(scan_data[indices]))

def choose_avoid_direction():

    """좌/우 평균 비교해서 장애물 반대 방향 선택"""

    left_avg = float(np.mean(scan_data[1:90]))

    right_avg = float(np.mean(scan_data[271:360]))

    return 1 if left_avg >= right_avg else -1 # 1: 오른쪽 회전, -1: 왼쪽 회전

# =========================================

# MOTOR

# =========================================

def send_cmd(v, w):

    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():

    send_cmd(0.0, 0.0)

# =========================================

# MAIN LOOP

# =========================================

print("PURE FORWARD OBSTACLE AVOIDANCE START (후진 없음)")

try:

    while True:

        raw = lidar_ser.read(5)

        if len(raw) != 5:

            continue

        # LiDAR packet parsing

        s_flag = raw[0] & 0x01

        if ((raw[0] & 0x02) >> 1) != (1 - s_flag) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:

            continue

        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360

        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0

        if 3 < dist_cm < 150:

            apply_ema(angle, dist_cm)

        if s_flag != 1:

            continue

        apply_median_filter()

        # =============== 회피 로직 ===============

        front_min = get_front_min()

        if front_min < THRESH_10:

            direction = choose_avoid_direction()

            v = MIN_SPEED

            w = direction * MAX_W

            print(f"🚨 VERY CLOSE! front={front_min:.1f}cm → STRONG TURN (dir={direction})")

        elif front_min < THRESH_20:

            direction = choose_avoid_direction()

            v = 0.12

            w = direction * 1.4

            print(f"⚠️ CRITICAL front={front_min:.1f}cm → STRONG TURN (dir={direction})")

        elif front_min < THRESH_30:

            direction = choose_avoid_direction()

            v = 0.15

            w = direction * 1.2

            print(f"⚡ WARNING front={front_min:.1f}cm → MEDIUM TURN (dir={direction})")

        else:

            v = MAX_SPEED

            w = 0.0

            # print("→ STRAIGHT")

        send_cmd(v, w)

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(bytes([0xA5, 0x25]))
