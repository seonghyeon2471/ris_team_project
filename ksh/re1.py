import serial
import math
import time
import numpy as np
from rplidar import RPLidar

# =========================================
# SERIAL (Arduino)
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

# =========================================
# LIDAR (RPLIDAR SDK 방식)
# =========================================
lidar = RPLidar('/dev/ttyUSB0', baudrate=460800)

# =========================================
# ROBOT PARAMETER
# =========================================
SCAN_SIZE = 360
scan_data = np.full(SCAN_SIZE, 100.0)

# 거리 제한
MAX_DIST = 150

# control parameter
BASE_V = 0.12
KP_W = 1.2
STOP_DIST = 5.0   # cm

# =========================================
# SEND MOTOR
# =========================================
def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

# =========================================
# BUILD MAP FROM LIDAR
# =========================================
def update_scan():
    global scan_data

    scan_data[:] = MAX_DIST

    # 하나의 full scan 받기
    for scan in lidar.iter_scans():
        for (_, angle, dist) in scan:

            if dist < 3 or dist > MAX_DIST:
                continue

            idx = int(angle) % 360
            scan_data[idx] = min(scan_data[idx], dist)

        break   # 한 프레임만 사용 (중요: 실시간 안정성)

# =========================================
# CONTROL LOGIC
# =========================================
def control():

    # front (±10도)
    front = np.min(scan_data[350:360].tolist() + scan_data[0:10].tolist())

    # left / right
    left = np.mean(scan_data[80:100])
    right = np.mean(scan_data[260:280])

    # =====================================
    # 1. FRONT OBSTACLE → TURN
    # =====================================
    if front < STOP_DIST:
        v = 0.0
        w = 1.2   # 제자리 회전
        return v, w

    # =====================================
    # 2. WALL FOLLOW (5cm 유지)
    # =====================================
    if min(left, right) < 30:

        # 더 가까운 벽 따라가기
        if left < right:
            error = 5.0 - left
            w = -KP_W * error
        else:
            error = right - 5.0
            w = KP_W * error

        v = BASE_V
        return v, w

    # =====================================
    # 3. NO WALL → GO STRAIGHT
    # =====================================
    v = BASE_V
    w = 0.0
    return v, w

# =========================================
# MAIN LOOP
# =========================================
print("ROBOT START")

try:
    # LiDAR start
    lidar.start_motor()
    time.sleep(1)

    for scan in lidar.iter_scans():

        # scan update
        scan_data[:] = MAX_DIST

        for (_, angle, dist) in scan:

            if 3 < dist < MAX_DIST:
                idx = int(angle) % 360
                scan_data[idx] = min(scan_data[idx], dist)

        # control
        v, w = control()
        send_cmd(v, w)

        print(f"v:{v:.2f} w:{w:.2f} front:{np.min(scan_data[350:360].tolist()+scan_data[0:10].tolist()):.1f}")

except KeyboardInterrupt:
    print("STOP")

finally:
    send_cmd(0, 0)
    lidar.stop()
    lidar.disconnect()
