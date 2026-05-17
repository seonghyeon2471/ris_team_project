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

print("✅ NAVIGATION START")

# =========================================
# PARAMETER
# =========================================
SCAN_LIMIT = 500.0

BASE_V = 0.22
MIN_V  = 0.08

MAX_W = 6.5

REACTION_DIST = 75.0      # cm

FRONT_RANGE = 60

EMA_ALPHA = 0.35

# smoothing
smoothed_w = 0.0
W_ALPHA = 0.55

# =========================================
# SCAN DATA
# =========================================
scan_data = np.full(360, SCAN_LIMIT, dtype=np.float32)

# =========================================
# FILTER
# =========================================
def apply_ema(angle, dist):

    scan_data[angle] = (
        (1.0 - EMA_ALPHA) * scan_data[angle]
        + EMA_ALPHA * dist
    )

# =========================================
# LIDAR READ
# =========================================
def read_lidar():

    while True:

        raw = lidar.read(5)

        if len(raw) != 5:
            return False

        s_flag = raw[0] & 0x01

        if (raw[0] & 0x02) >> 1 != (1 - s_flag):
            return False

        if (raw[1] & 0x01) != 1:
            return False

        if (raw[0] >> 2) < 3:
            return False

        angle = int(
            (
                ((raw[1] >> 1) | (raw[2] << 7))
                / 64.0
            )
        ) % 360

        dist_cm = (
            (raw[3] | (raw[4] << 8))
            / 40.0
        )

        if 3 < dist_cm < SCAN_LIMIT:
            apply_ema(angle, dist_cm)

        return s_flag

# =========================================
# FRONT MIN
# =========================================
def get_front_min():

    front_angles = (
        list(range(330, 360))
        + list(range(0, 31))
    )

    return float(np.min(scan_data[front_angles]))

# =========================================
# CONTROL
# =========================================
def compute_control():

    global smoothed_w

    # =========================
    # FRONT SECTOR
    # =========================
    front_sector = (
        list(range(300, 360))
        + list(range(0, 61))
    )

    left_score = 0.0
    right_score = 0.0

    front_min = 999.0

    for a in front_sector:

        dist = scan_data[a]

        if dist < front_min:
            front_min = dist

        if dist > REACTION_DIST:
            continue

        # =====================
        # 정면 기준 각도 변환
        # =====================
        if a <= 180:
            theta = a
        else:
            theta = a - 360

        abs_theta = abs(theta)

        # =====================
        # 정면일수록 강한 회피
        # =====================
        if abs_theta < 10:
            angle_gain = 2.8

        elif abs_theta < 20:
            angle_gain = 2.2

        elif abs_theta < 35:
            angle_gain = 1.6

        elif abs_theta < 50:
            angle_gain = 1.0

        else:
            angle_gain = 0.5

        # 가까울수록 위험
        danger_gain = (
            (REACTION_DIST - dist)
            / REACTION_DIST
        )

        score = angle_gain * danger_gain

        # =====================
        # 방향 누적
        # =====================
        if theta > 0:
            left_score += score

        else:
            right_score += score

    # =========================
    # 회전 계산
    # =========================
    target_w = (
        (right_score - left_score)
        * 3.8
    )

    # smoothing
    smoothed_w = (
        W_ALPHA * target_w
        + (1.0 - W_ALPHA) * smoothed_w
    )

    w = smoothed_w

    # limit
    w = max(-MAX_W, min(MAX_W, w))

    # =========================
    # 속도 조절
    # =========================
    v = BASE_V

    if front_min < 20:
        v = MIN_V

    elif front_min < 35:
        v = 0.12

    elif front_min < 50:
        v = 0.16

    return v, w, front_min, left_score, right_score

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):

    cmd = f"{v:.3f},{-w:.3f}\n"

    arduino.write(cmd.encode())

# =========================================
# MAIN LOOP
# =========================================
try:

    while True:

        s_flag = read_lidar()

        # 한 바퀴 완료 시만 제어
        if s_flag != 1:
            continue

        v, w, front_min, ls, rs = compute_control()

        send_cmd(v, w)

        print(
            f"front:{front_min:5.1f}cm | "
            f"L:{ls:5.2f} | "
            f"R:{rs:5.2f} | "
            f"v:{v:.2f} | "
            f"w:{w:+.2f}"
        )

        time.sleep(0.02)

# =========================================
# EXIT
# =========================================
except KeyboardInterrupt:

    print("\nSTOP")

    send_cmd(0.0, 0.0)

finally:

    lidar.write(bytes([0xA5, 0x25]))

    arduino.close()
    lidar.close()
