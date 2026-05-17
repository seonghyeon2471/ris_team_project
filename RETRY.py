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
# DRIVE PARAMETER
# =========================================
BASE_SPEED = 0.18
MIN_SPEED  = 0.05

TURN_GAIN = 0.020
MAX_W     = 1.0

WHEEL_BASE = 0.17

# =========================================
# DETECTION PARAMETER
# =========================================
SCAN_LIMIT      = 150

SIDE_DIST       = 40
FRONT_DIST      = 45
EMERGENCY_DIST  = 10

# =========================================
# FILTER PARAMETER
# =========================================
EMA_ALPHA = 0.35
MEDIAN_K  = 2

# =========================================
# HEADING HOLD
# =========================================
STEERING_DECAY = 0.92

prev_w = 0.0

# =========================================
# STATE
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

    k = MEDIAN_K

    filtered = np.empty(
        360,
        dtype=np.float32
    )

    for i in range(360):

        idx = [
            (i + d) % 360
            for d in range(-k, k + 1)
        ]

        values = np.sort(scan_data[idx])

        filtered[i] = values[len(values)//2]

    scan_data[:] = filtered

# =========================================
# REGION DISTANCE
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
# OBSTACLE ANALYSIS
# =========================================
def analyze_obstacle():

    # =========================
    # 정면 최소거리
    # =========================
    front = min(
        get_region_min(350, 359),
        get_region_min(0, 10)
    )

    # =========================
    # 좌우 평균거리
    # =========================
    left = get_region_mean(15, 60)

    right = get_region_mean(300, 345)

    return front, left, right

# =========================================
# CONTROL
# =========================================
def compute_control():

    global prev_w

    front, left, right = analyze_obstacle()

    # =====================================
    # 기본 직진
    # =====================================
    target_w = 0.0

    # =====================================
    # 좌측 장애물 회피
    # =====================================
    if left < SIDE_DIST:

        error = SIDE_DIST - left

        target_w -= (
            error * TURN_GAIN
        )

    # =====================================
    # 우측 장애물 회피
    # =====================================
    if right < SIDE_DIST:

        error = SIDE_DIST - right

        target_w += (
            error * TURN_GAIN
        )

    # =====================================
    # 정면 장애물 강제 회피
    # =====================================
    if front < FRONT_DIST:

        if left > right:

            target_w += (
                (FRONT_DIST - front)
                * TURN_GAIN * 2.5
            )

        else:

            target_w -= (
                (FRONT_DIST - front)
                * TURN_GAIN * 2.5
            )

    # =====================================
    # 전방 충분히 비면
    # 회전 성분 자동 감소
    # =====================================
    front_weight = (
        1.0
        - min(front / 100.0, 1.0)
    )

    target_w *= (
        0.25 + front_weight
    )

    # =====================================
    # Heading Hold
    # 갑작스러운 방향 변화 방지
    # =====================================
    target_w = (
        prev_w * STEERING_DECAY
        + target_w * (1.0 - STEERING_DECAY)
    )

    prev_w = target_w

    # =====================================
    # 각속도 제한
    # =====================================
    target_w = np.clip(
        target_w,
        -MAX_W,
        MAX_W
    )

    # =====================================
    # 속도 계산
    # =====================================
    if front > 80:

        v = BASE_SPEED

    elif front > 50:

        v = 0.15

    elif front > 30:

        v = 0.11

    elif front > EMERGENCY_DIST:

        v = 0.07

    else:

        v = 0.03

    # =====================================
    # 회전 클수록 감속
    # =====================================
    turn_scale = max(
        0.45,
        1.0 - abs(target_w)
    )

    v *= turn_scale

    v = max(v, MIN_SPEED)

    # =====================================
    # Differential Drive
    # =====================================
    left_wheel = (
        v - (WHEEL_BASE / 2.0) * target_w
    )

    right_wheel = (
        v + (WHEEL_BASE / 2.0) * target_w
    )

    return (
        left_wheel,
        right_wheel,
        v,
        target_w,
        front,
        left,
        right
    )

# =========================================
# MOTOR SEND
# =========================================
def send_motor(left_wheel, right_wheel):

    cmd = (
        f"{left_wheel:.3f},"
        f"{right_wheel:.3f}\n"
    )

    arduino_ser.write(cmd.encode())

# =========================================
# STOP
# =========================================
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
