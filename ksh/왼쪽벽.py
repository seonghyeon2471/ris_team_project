# =========================================
# ★ 왼쪽 벽 따라가기 전용 (Left Wall Following)
# =========================================
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
print("LIDAR START - Left Wall Following Mode")

# =========================================
# PARAM
# =========================================
ROBOT_RADIUS = 14
MAX_SPEED = 0.18
MIN_SPEED = 0.08
MAX_W = 0.85
TURN_GAIN = 0.75

SCAN_LIMIT = 150
DANGER_DIST = 10

# Wall Following Parameters
TARGET_WALL_DIST = 18.0      # 목표 벽까지 거리 (cm)
WALL_FOLLOW_GAIN = 0.028     # P 게인 (조정 가능)
MIN_WALL_DIST = 8.0          # 너무 가까우면 강하게 벗어나기

# FILTER
EMA_ALPHA = 0.35
MEDIAN_K = 2

# DATA
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)

# =========================================
# FILTER
# =========================================
def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def apply_median_filter():
    k = MEDIAN_K
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        indices = [(i + d) % 360 for d in range(-k, k + 1)]
        filtered[i] = np.median(scan_data[indices])
    scan_data[:] = filtered

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
print("Left Wall Following Start")

try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

        s_flag = raw[0] & 0x01
        s_inv_flag = (raw[0] & 0x02) >> 1
        if s_inv_flag != (1 - s_flag):
            continue
        if (raw[1] & 0x01) != 1:
            continue

        quality = raw[0] >> 2
        if quality < 3:
            continue

        angle_raw = (raw[1] >> 1) | (raw[2] << 7)
        angle = int(angle_raw / 64.0) % 360
        dist_raw = raw[3] | (raw[4] << 8)
        dist_cm = (dist_raw / 4.0) / 10.0

        if dist_cm < 3 or dist_cm > SCAN_LIMIT:
            continue

        apply_ema(angle, dist_cm)

        if s_flag != 1:
            continue

        apply_median_filter()

        # ==================== Left Wall Following ====================
        left_wall_dist = np.median(scan_data[50:100])     # 왼쪽 벽 영역
        front_left = np.min(scan_data[20:60])             # 왼쪽 앞
        front = np.min(scan_data[np.arange(-25, 26) % 360])  # 정면

        # 안전 거리 이하일 때 강제 회피 (우회전)
        if front < DANGER_DIST:
            send_cmd(0.0, -0.9)   # 급우회전
            continue

        # PD 제어 (P만 사용)
        error = TARGET_WALL_DIST - left_wall_dist   # 왼쪽은 부호가 반대
        w = error * WALL_FOLLOW_GAIN

        # 너무 가까우면 강하게 오른쪽으로
        if left_wall_dist < MIN_WALL_DIST:
            w = -0.7

        w = np.clip(w, -MAX_W, MAX_W)

        # 속도 결정
        if front < 35:
            speed = MIN_SPEED
        else:
            speed = MAX_SPEED * (1.0 - abs(w) * 0.6)   # 회전할수록 천천히
            speed = max(speed, MIN_SPEED)

        send_cmd(speed, w)

        print(f"Left :{left_wall_dist:5.1f}  Err:{error:5.1f}  W:{w:6.3f}  V:{speed:.2f}")

except KeyboardInterrupt:
    print("\nSTOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
