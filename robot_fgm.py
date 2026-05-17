import time
import math
import numpy as np
from rplidar import RPLidar  # pip install rplidar-roboticia

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

# =========================================
# RPLIDAR C1
# =========================================
lidar = RPLidar('/dev/ttyUSB0', baudrate=460800, timeout=1)

# =========================================
# ROBOT PARAMETERS (단위: meter)
# =========================================
ROBOT_WIDTH = 0.16
ROBOT_LENGTH = 0.21
LIDAR_X_OFFSET = -0.025      # 라이다가 앞에서 2.5cm 뒤 → 앞쪽이 negative
WHEEL_BASE = 0.17            # Arduino와 동일

SAFE_DIST = 0.30             # 30cm
MIN_PATH_WIDTH = 0.45        # 통로 최소 폭

# PID for angular control
KP = 1.2
KI = 0.05
KD = 0.4

integral = 0.0
last_error = 0.0
last_time = time.time()

print("🚀 RPLIDAR + Differential Drive Navigation Start")

lidar.start_motor()
time.sleep(2)
lidar.clear_input_buffer()

def send_vw(v: float, w: float):
    """Arduino로 v, w 전송 (m/s, rad/s)"""
    cmd = f"{v:.3f},{w:.3f}\n"
    arduino_ser.write(cmd.encode())
    arduino_ser.flush()

def get_lidar_points():
    """한 스캔 데이터 가져오기 (미터 단위)"""
    try:
        scan = next(lidar.iter_scans(max_buf_meas=1500))
        points = []
        for _, angle, dist in scan:
            if 150 < dist < 12000:          # 0.15m ~ 12m
                rad = math.radians(angle)
                x = (dist / 1000.0) * math.cos(rad)   # meter
                y = (dist / 1000.0) * math.sin(rad)
                points.append((angle, dist/1000.0, x, y))
        return points
    except:
        return []

def find_center_error(points):
    """전방 좌/우 gap을 이용한 중앙 편차 계산"""
    left_dist = 999.0
    right_dist = 999.0

    for angle, dist, x, y in points:
        if dist < 0.15: 
            continue

        # 전방 60° ~ 120° 영역 (90° = 정면)
        if 45 < angle < 135:
            if angle < 90:          # 오른쪽 전방
                right_dist = min(right_dist, dist)
            else:                   # 왼쪽 전방
                left_dist = min(left_dist, dist)

    # 너무 가까우면 위험
    if left_dist < SAFE_DIST or right_dist < SAFE_DIST:
        return None

    # 중앙 편차 (양수 = 왼쪽이 더 넓음 → 오른쪽으로 돌려야 함)
    error = left_dist - right_dist
    return error

try:
    while True:
        points = get_lidar_points()
        if len(points) < 60:
            send_vw(0.0, 0.0)
            time.sleep(0.1)
            continue

        error = find_center_error(points)

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        if error is None:                     # 장애물 감지
            send_vw(-0.15, 0.0)               # 후진
            time.sleep(0.6)
            send_vw(0.0, 1.2)                 # 강하게 좌/우 회전 (필요시 번갈아)
            time.sleep(0.8)
            continue

        # PID
        integral += error * dt
        derivative = (error - last_error) / dt if dt > 0 else 0
        last_error = error

        w = KP * error + KI * integral + KD * derivative
        w = max(min(w, 2.5), -2.5)            # angular velocity limit (rad/s)

        v = 0.25                              # 기본 전진 속도 (m/s) → 상황에 따라 0.15~0.35 조정

        # 좁은 통로에서는 속도 줄임
        if min(left_dist if 'left_dist' in locals() else 999, 
               right_dist if 'right_dist' in locals() else 999) < 0.5:
            v = 0.15

        send_vw(v, w)
        time.sleep(0.05)   # 20Hz 제어 루프

except KeyboardInterrupt:
    print("\n🛑 Stopping...")
    send_vw(0.0, 0.0)
    lidar.stop_motor()
    lidar.disconnect()
    arduino_ser.close()
