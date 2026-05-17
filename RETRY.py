import serial
import math
import time
import numpy as np

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.1
)

lidar_ser = serial.Serial(
    "/dev/ttyUSB0",
    460800,
    timeout=0.1
)

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
# PARAMETER
# =========================================
SCAN_LIMIT = 150

BASE_SPEED = 0.20
MIN_SPEED  = 0.06

MAX_W      = 0.85
TURN_GAIN  = 0.020

WHEEL_BASE = 0.17

# =========================================
# OBSTACLE PARAMETER
# =========================================
SAFE_DIST  = 45
FRONT_DIST = 55

# =========================================
# FILTER
# =========================================
EMA_ALPHA = 0.35
MEDIAN_K  = 2

# =========================================
# SMOOTHING
# =========================================
STEERING_ALPHA = 0.18

current_w = 0.0

# =========================================
# DATA
# =========================================
scan_data = np.full(
    360,
    float(SCAN_LIMIT),
    dtype=np.float32
)

# =========================================
# FILTER
# =========================================
def apply_ema(angle, dist):

    scan_data[angle] = (
        (1.0 - EMA_ALPHA)
        * scan_data[angle]
        + EMA_ALPHA * dist
    )

def apply_median_filter():

    filtered = np.empty(
        360,
        dtype=np.float32
    )

    for i in range(360):

        idx = [
            (i + d) % 360
            for d in range(-MEDIAN_K, MEDIAN_K + 1)
        ]

        values = np.sort(scan_data[idx])

        filtered[i] = values[len(values)//2]

    scan_data[:] = filtered

# =========================================
# REGION
# =========================================
def get_region_mean(start_deg, end_deg):

    if start_deg <= end_deg:

        idx = np.arange(
            start_deg,
            end_deg + 1
        )

    else:

        idx = np.concatenate((
            np.arange(start_deg, 360),
            np.arange(0, end_deg + 1)
        ))

    return float(np.mean(scan_data[idx]))

def get_region_min(start_deg, end_deg):

    if start_deg <= end_deg:

        idx = np.arange(
            start_deg,
            end_deg + 1
        )

    else:

        idx = np.concatenate((
            np.arange(start_deg, 360),
            np.arange(0, end_deg + 1)
        ))

    return float(np.min(scan_data[idx]))

# =========================================
# SENSOR ANALYSIS
# =========================================
def analyze_obstacle():

    # =========================
    # 정면
    # =========================
    front_region = scan_data[
        np.arange(-8, 9) % 360
    ]

    front_mean = float(
        np.mean(front_region)
    )

    front_min = float(
        np.min(front_region)
    )

    front = (
        front_mean * 0.7
        + front_min * 0.3
    )

    # =========================
    # 좌측
    # =========================
    left_mean = get_region_mean(15, 70)
    left_min  = get_region_min(15, 70)

    left = (
        left_mean * 0.7
        + left_min * 0.3
    )

    # =========================
    # 우측
    # =========================
    right_mean = get_region_mean(290, 345)
    right_min  = get_region_min(290, 345)

    right = (
        right_mean * 0.7
        + right_min * 0.3
    )

    return front, left, right

# =========================================
# CURVATURE AVOIDANCE
# =========================================
def compute_control():

    global current_w

    front, left, right = analyze_obstacle()

    # =====================================
    # 좌우 obstacle force
    # =====================================
    left_force = max(
        0.0,
        SAFE_DIST - left
    )

    right_force = max(
        0.0,
        SAFE_DIST - right
    )

    # =====================================
    # steering 생성
    # =====================================
    target_w = (
        right_force
        - left_force
    ) * TURN_GAIN

    # =====================================
    # 정면 가까우면 회전 강화
    # =====================================
    if front < FRONT_DIST:

        boost = (
            (FRONT_DIST - front)
            / FRONT_DIST
        )

        target_w *= (
            1.0 + boost * 2.2
        )

    # =====================================
    # steering smoothing
    # =====================================
    current_w = (
        current_w * (1.0 - STEERING_ALPHA)
        + target_w * STEERING_ALPHA
    )

    # =====================================
    # 회전 제한
    # =====================================
    current_w = np.clip(
        current_w,
        -MAX_W,
        MAX_W
    )

    # =====================================
    # 속도 계산
    # =====================================
    front_scale = np.clip(
        front / 80.0,
        0.35,
        1.0
    )

    v = BASE_SPEED * front_scale

    # =====================================
    # 회전 클수록 감속
    # =====================================
    turn_scale = max(
        0.55,
        1.0 - abs(current_w) * 0.7
    )

    v *= turn_scale

    v = max(v, MIN_SPEED)

    # =====================================
    # Differential Drive
    # =====================================
    left_wheel = (
        v - (WHEEL_BASE / 2.0) * current_w
    )

    right_wheel = (
        v + (WHEEL_BASE / 2.0) * current_w
    )

    return (
        left_wheel,
        right_wheel,
        v,
        current_w,
        front,
        left,
        right
    )

# =========================================
# MOTOR
# =========================================
def send_motor(left_wheel, right_wheel):

    cmd = (
        f"{left_wheel:.3f},"
        f"{right_wheel:.3f}\n"
    )

    arduino_ser.write(cmd.encode())

def stop_robot():

    send_motor(0.0, 0.0)

# =========================================
# MAIN LOOP
# =========================================
print("START NAVIGATION")

try:

    while True:

        raw = lidar_ser.read(5)

        if len(raw) != 5:
            continue

        s_flag = raw[0] & 0x01

        if (
            (raw[0] & 0x02) >> 1
            != (1 - s_flag)
        ):
            continue

        if (raw[1] & 0x01) != 1:
            continue

        if (raw[0] >> 2) < 3:
            continue

        angle = int(
            (
                ((raw[1] >> 1)
                | (raw[2] << 7))
                / 64.0
            )
        ) % 360

        dist = (
            (raw[3]
            | (raw[4] << 8))
            / 40.0
        )

        if 3 < dist < SCAN_LIMIT:

            apply_ema(angle, dist)

        # 한 바퀴 완료
        if s_flag != 1:
            continue

        apply_median_filter()

        (
            left_wheel,
            right_wheel,
            v,
            w,
            front,
            left,
            right
        ) = compute_control()

        send_motor(
            left_wheel,
            right_wheel
        )

        print(
            f"v:{v:.2f} | "
            f"w:{w:.2f} | "
            f"LW:{left_wheel:.2f} | "
            f"RW:{right_wheel:.2f} | "
            f"F:{front:.1f} | "
            f"L:{left:.1f} | "
            f"R:{right:.1f}"
        )

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(bytes([0xA5, 0x25]))
