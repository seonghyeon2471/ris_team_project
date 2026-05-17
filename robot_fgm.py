import serial
import time
import math
import numpy as np
from rplidar import RPLidar  # 또는 rplidarc1

# =========================================
# SERIAL & HARDWARE
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

# RPLIDAR C1
lidar = RPLidar('/dev/ttyUSB0', baudrate=460800, timeout=1)  # C1은 460800이 표준

# 로봇 치수 (cm)
ROBOT_LENGTH = 21
ROBOT_WIDTH = 16
LIDAR_OFFSET = 2.5          # 앞쪽에서 LIDAR까지 거리
WHEEL_AXIS_FROM_REAR = 3.5
WHEEL_PROTRUDE = 2          # 양쪽 바퀴 돌출

# 안전 거리 (cm)
SAFE_DISTANCE = 25          # 정면/측면 최소 거리
MIN_PATH_WIDTH = 40         # 통로 최소 폭 (로봇 폭 + 여유)

# PID 게인 (중앙 추종용)
KP = 0.8
KI = 0.01
KD = 0.3

integral = 0
last_error = 0

def send_motor_command(left_speed: int, right_speed: int):
    """Arduino로 PWM 명령 전송 (예: L150 R-120 형식)"""
    cmd = f"L{left_speed} R{right_speed}\n"
    arduino_ser.write(cmd.encode())
    arduino_ser.flush()

def get_lidar_scan():
    """한 번의 scan 데이터 가져오기 (cm 단위)"""
    try:
        scan = next(lidar.iter_scans(max_buf_meas=2000))
        points = []
        for quality, angle, distance in scan:
            if distance > 0 and distance < 1200:  # 12m 이내
                rad = math.radians(angle)
                x = distance * math.cos(rad)   # cm
                y = distance * math.sin(rad)
                points.append((angle, distance, x, y))
        return points
    except:
        return []

def find_path_center(points):
    """
    LiDAR로 '통로 중앙' 찾기
    - 앞쪽 60° ~ 120° (로봇 기준 전방)에서 빈 공간(거리 큰 영역) 탐색
    - 좌/우 벽까지 거리 차이 → error
    """
    front_left_dist = 999
    front_right_dist = 999
    left_wall_dist = 999
    right_wall_dist = 999

    for angle, dist, x, y in points:
        if dist < 10: continue  # 노이즈

        # 전방 영역 (로봇 기준)
        if 30 < angle < 150:   # 0°=정면 오른쪽, 90°=정면, 180°=뒤
            if 30 < angle < 90:   # 오른쪽 전방
                front_right_dist = min(front_right_dist, dist)
            else:                 # 왼쪽 전방
                front_left_dist = min(front_left_dist, dist)

        # 측면 벽 (로봇 폭 고려)
        if 0 < angle < 30:    # 오른쪽
            right_wall_dist = min(right_wall_dist, dist)
        elif 150 < angle < 180:  # 왼쪽
            left_wall_dist = min(left_wall_dist, dist)

    # 중앙 편차 계산 (cm)
    left_open = front_left_dist
    right_open = front_right_dist
    error = (left_open - right_open) / 2.0   # 양수면 왼쪽이 더 넓음 → 오른쪽으로 correction 필요

    # 너무 좁으면 후진/정지
    path_width = left_open + right_open
    if path_width < MIN_PATH_WIDTH or min(left_open, right_open) < SAFE_DISTANCE:
        return None, error  # 위험

    return error, error   # (center_error, wall_error)

def pid_control(error):
    global integral, last_error
    integral += error
    derivative = error - last_error
    last_error = error

    steer = KP * error + KI * integral + KD * derivative
    steer = max(min(steer, 80), -80)  # 최대 steer 제한
    return steer

print("LIDAR + Navigation Start!")

lidar.start_motor()   # 모터 ON (C1은 start_motor 필요)

try:
    while True:
        points = get_lidar_scan()
        if len(points) < 50:
            send_motor_command(0, 0)
            time.sleep(0.1)
            continue

        error, _ = find_path_center(points)

        if error is None:  # 장애물/막다른 길
            send_motor_command(-80, -80)  # 후진
            time.sleep(0.8)
            send_motor_command(100, -100)  # 제자리 회전 (방향 전환)
            time.sleep(1.0)
            continue

        steer = pid_control(error)

        base_speed = 120
        left_speed = int(base_speed + steer)
        right_speed = int(base_speed - steer)

        # 속도 제한
        left_speed = max(min(left_speed, 255), -150)
        right_speed = max(min(right_speed, 255), -150)

        send_motor_command(left_speed, right_speed)

        time.sleep(0.05)  # 20Hz 제어

except KeyboardInterrupt:
    print("Stop")
    send_motor_command(0, 0)
    lidar.stop_motor()
    lidar.disconnect()
    arduino_ser.close()
