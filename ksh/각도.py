import serial
import time
import math
import numpy as np

# =========================================
# SERIAL
# =========================================
ARDUINO_PORT = "/dev/serial0"
LIDAR_PORT   = "/dev/ttyUSB0"

arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar   = serial.Serial(LIDAR_PORT,   460800, timeout=0.05)

# =========================================
# LIDAR START
# =========================================
print("LiDAR 시작 중...")

lidar.write(bytes([0xA5, 0x40]))
time.sleep(1)

lidar.write(bytes([0xA5, 0x20]))
time.sleep(2)

lidar.reset_input_buffer()

print("✅ 시작 완료")

# =========================================
# PARAMETERS
# =========================================
REACTION_DIST = 0.75

BASE_V = 0.22

MAX_W = 7.0

# smoothing
smoothed_w = 0.0
ALPHA = 0.55

# =========================================
# LIDAR PARSE
# =========================================
def get_scan_points():

    chunk = lidar.read(1400)

    points = []

    i = 0

    while i + 4 < len(chunk):

        q  = chunk[i]
        a1 = chunk[i+1]
        a2 = chunk[i+2]
        d1 = chunk[i+3]
        d2 = chunk[i+4]

        angle_raw = (a2 << 8) | a1
        angle = (angle_raw / 64.0) % 360.0

        dist_raw = (d2 << 8) | d1
        dist = dist_raw / 4.0 / 1000.0

        if 0.08 < dist < 5.0 and (q & 0x01):

            if angle > 180:
                angle -= 360

            points.append((angle, dist))

        i += 5

    return points

# =========================================
# MAIN
# =========================================
try:

    while True:

        points = get_scan_points()

        v = BASE_V
        w = 0.0

        avoidance_sum = 0.0
        weight_sum = 0.0

        front_min = 999

        # =========================================
        # 장애물 회피 계산
        # =========================================
        for theta, dist in points:

            # 앞쪽만 사용
            if abs(theta) > 120:
                continue

            if dist < front_min:
                front_min = dist

            # 반응 거리 안만 사용
            if dist > REACTION_DIST:
                continue

            abs_theta = abs(theta)

            # =====================================
            # 각도별 회피 강도
            # 정면 강함 / 측면 약함
            # =====================================

            if abs_theta < 15:
                angle_gain = 2.6

            elif abs_theta < 35:
                angle_gain = 2.0

            elif abs_theta < 60:
                angle_gain = 1.3

            elif abs_theta < 90:
                angle_gain = 0.8

            else:
                angle_gain = 0.4

            # 가까울수록 강하게
            dist_gain = (REACTION_DIST - dist) / REACTION_DIST

            # 회피 벡터
            steer = -(theta / 60.0)

            score = steer * angle_gain * dist_gain

            avoidance_sum += score
            weight_sum += dist_gain

        # =========================================
        # 평균 steering
        # =========================================
        if weight_sum > 0:

            target_w = (avoidance_sum / weight_sum) * 6.5

            # 초근접 회피
            if front_min < 0.18:
                target_w *= 1.8

            # smoothing
            smoothed_w = (
                ALPHA * target_w
                + (1 - ALPHA) * smoothed_w
            )

            w = smoothed_w

        else:
            smoothed_w *= 0.75
            w = smoothed_w

        # 제한
        w = max(-MAX_W, min(MAX_W, w))

        # =========================================
        # 속도 자동 감소
        # =========================================
        if front_min < 0.30:
            v = 0.10

        elif front_min < 0.50:
            v = 0.16

        # =========================================
        # SEND
        # =========================================
        cmd = f"{v:.2f},{w:.3f}\n"

        arduino.write(cmd.encode())

        print(
            f"front:{front_min:.2f}m  "
            f"v:{v:.2f}  "
            f"w:{w:+.3f}"
        )

        time.sleep(0.03)

# =========================================
# EXIT
# =========================================
except KeyboardInterrupt:

    print("\n종료")

    arduino.write(b"0.0,0.0\n")

finally:

    lidar.write(bytes([0xA5, 0x25]))

    arduino.close()
    lidar.close()
