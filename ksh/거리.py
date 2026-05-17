import serial
import math
import time
import numpy as np

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

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

# 최대 직진 속도
MAX_SPEED = 1.0

# 최소 속도
MIN_SPEED = 0.12

# 최대 회전
MAX_W = 1.8

# 거리 기준
THRESH_30 = 20.0
THRESH_20 = 14.0
THRESH_10 = 8.0

# 전방 체크 범위
# 기존 45 -> 너무 넓어서 벽도 장애물로 인식
FRONT_CHECK_RANGE = 15

# EMA 필터
EMA_ALPHA = 0.55

# 라이다 저장
scan_data = np.full(360, 150.0, dtype=np.float32)

# =========================================
# FILTER
# =========================================
def apply_ema(angle, new_dist_cm):

    scan_data[angle] = (
        (1.0 - EMA_ALPHA) * scan_data[angle]
        + EMA_ALPHA * new_dist_cm
    )

# =========================================
# FRONT DISTANCE
# =========================================
def get_front_min():

    indices = np.arange(
        -FRONT_CHECK_RANGE,
        FRONT_CHECK_RANGE + 1
    ) % 360

    return float(np.min(scan_data[indices]))

# =========================================
# CHOOSE DIRECTION
# =========================================
def choose_avoid_direction():

    left_avg = float(np.mean(scan_data[10:90]))
    right_avg = float(np.mean(scan_data[270:350]))

    return 1 if left_avg >= right_avg else -1

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():

    send_cmd(0.0, 0.0)

# =========================================
# MAIN LOOP
# =========================================
print("HIGH SPEED OBSTACLE AVOIDANCE START")

try:

    while True:

        raw = lidar_ser.read(5)

        if len(raw) != 5:
            continue

        s_flag = raw[0] & 0x01

        if (
            ((raw[0] & 0x02) >> 1) != (1 - s_flag)
            or (raw[1] & 0x01) != 1
            or (raw[0] >> 2) < 3
        ):
            continue

        angle = int(
            ((raw[1] >> 1) | (raw[2] << 7)) / 64.0
        ) % 360

        dist_cm = (
            (raw[3] | (raw[4] << 8)) / 40.0
        )

        # 거리 유효 범위
        if 3 < dist_cm < 150:

            apply_ema(angle, dist_cm)

        # 한 바퀴 완료 시만 판단
        if s_flag != 1:
            continue

        # =====================================
        # FRONT DISTANCE
        # =====================================
        front_min = get_front_min()

        # =====================================
        # DEFAULT
        # =====================================
        v = MAX_SPEED
        w = 0.0

        # =====================================
        # VERY CLOSE
        # =====================================
        if front_min < THRESH_10:

            direction = choose_avoid_direction()

            v = MIN_SPEED
            w = direction * MAX_W

            print(
                f"🚨 VERY CLOSE "
                f"front={front_min:.1f} "
                f"v={v:.2f} "
                f"w={w:.2f}"
            )

        # =====================================
        # CLOSE
        # =====================================
        elif front_min < THRESH_20:

            direction = choose_avoid_direction()

            v = 0.20
            w = direction * 1.5

            print(
                f"⚠️ CLOSE "
                f"front={front_min:.1f} "
                f"v={v:.2f} "
                f"w={w:.2f}"
            )

        # =====================================
        # WARNING
        # =====================================
        elif front_min < THRESH_30:

            direction = choose_avoid_direction()

            v = 0.45
            w = direction * 0.9

            print(
                f"⚡ WARNING "
                f"front={front_min:.1f} "
                f"v={v:.2f} "
                f"w={w:.2f}"
            )

        # =====================================
        # FREE ROAD
        # =====================================
        else:

            print(
                f"✅ FREE "
                f"front={front_min:.1f} "
                f"v={v:.2f}"
            )

        # =====================================
        # SEND
        # =====================================
        send_cmd(v, w)

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(bytes([0xA5, 0x25]))
