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
# PARAMETERS
# =========================================
MAX_SPEED = 0.40
MIN_SPEED = 0.09
MAX_W = 1.65               # ← 필요하면 1.7~1.8로 올려보세요

THRESH_30 = 25.0
THRESH_20 = 20.0
THRESH_10 = 10.0

FRONT_CHECK_RANGE = 45

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

# =========================================
# ★★★ 핵심 개선: 실시간 좌/우 clearance 비교 ★★★
# =========================================
def get_avoid_direction():
    # 좌측 측면 (35° ~ 125°): 전방과 너무 겹치지 않게
    left_sector = scan_data[35:126]
    # 우측 측면 (235° ~ 325°)
    right_sector = scan_data[235:326]
    
    left_min = float(np.min(left_sector))
    right_min = float(np.min(right_sector))
    
    # 양쪽 거리가 비슷하면 직진 (약간의 hysteresis)
    if abs(left_min - right_min) < 8.0:      # ← 8cm 이내 차이면 직진
        return 0.0
    
    # 한쪽이 더 가까우면 반대쪽으로 꺾기
    if left_min < right_min:
        return 1.0      # 왼쪽이 가까움 → 오른쪽으로 꺾기
    else:
        return -1.0     # 오른쪽이 가까움 → 왼쪽으로 꺾기

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
print("IMPROVED SIDE-CLEARANCE OBSTACLE AVOIDANCE START")

try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

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

        # =============== 개선된 회피 로직 ===============
        front_min = get_front_min()
        
        # 실시간 좌/우 방향 결정
        direction = get_avoid_direction()

        if front_min < THRESH_10:
            v = MIN_SPEED
            w = direction * MAX_W
            print(f"🚨 VERY CLOSE! front={front_min:.1f}cm | dir={direction:+.1f}")
        elif front_min < THRESH_20:
            v = 0.12
            w = direction * 1.45
            print(f"⚠️ CRITICAL front={front_min:.1f}cm | dir={direction:+.1f}")
        elif front_min < THRESH_30:
            v = 0.15
            w = direction * 1.15
            print(f"⚡ WARNING front={front_min:.1f}cm | dir={direction:+.1f}")
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
