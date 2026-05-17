import serial
import math
import time
import numpy as np

# =========================================================
# SERIAL
# =========================================================
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

# =========================================================
# LIDAR START
# =========================================================
lidar_ser.write(bytes([0xA5, 0x40]))

time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)

print("LIDAR START")

# =========================================================
# PARAMETER
# =========================================================

SCAN_LIMIT = 150

# 기본 속도
BASE_SPEED = 0.22

# 최소 속도
# 너무 낮으면 장애물 앞에서 멈춤
MIN_SPEED = 0.11

# 최대 회전 속도
MAX_W = 1.2

# 조향 게인
TURN_GAIN = 0.022

# 바퀴 간 거리
WHEEL_BASE = 0.17

# =========================================================
# SOFT AVOIDANCE
# =========================================================

# 일반 회피 거리
SAFE_DIST = 45

# 정면 회피 시작 거리
FRONT_DIST = 60

# =========================================================
# HARD SAFETY
# =========================================================

# 진짜 위험 거리
# 이 안으로 들어오면 강제 회피
CRITICAL_DIST = 18

# 강제 회피 회전 속도
CRITICAL_W = 0.95

# =========================================================
# FILTER
# =========================================================

EMA_ALPHA = 0.35
MEDIAN_K  = 2

# =========================================================
# SMOOTHING
# =========================================================

STEERING_ALPHA = 0.22

current_w = 0.0

# =========================================================
# ESCAPE MEMORY
# =========================================================

# 정면 막혔을 때
# 회피 방향 기억

escape_dir = 1

# =========================================================
# DATA
# =========================================================
scan_data = np.full(
    360,
    float(SCAN_LIMIT),
    dtype=np.float32
)

# =========================================================
# EMA FILTER
# =========================================================
def apply_ema(angle, dist):

    scan_data[angle] = (
        (1.0 - EMA_ALPHA)
        * scan_data[angle]
        + EMA_ALPHA * dist
    )

# =========================================================
# MEDIAN FILTER
# =========================================================
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

# =========================================================
# REGION
# =========================================================
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

# =========================================================
# SENSOR ANALYSIS
# =========================================================
def analyze_obstacle():

    # =====================================================
    # FRONT
    # =====================================================

    # 전방 범위를 넓게 사용
    # 늦은 회피 방지

    front_region = scan_data[
        np.arange(-15, 16) % 360
    ]

    front_mean = float(
        np.mean(front_region)
    )

    front_min = float(
        np.min(front_region)
    )

    # 평균 + 최소 혼합
    # min 비중 증가
    # 충돌 방지 강화

    front = (
        front_mean * 0.5
        + front_min * 0.5
    )

    # =====================================================
    # LEFT
    # =====================================================

    left_mean = get_region_mean(15, 70)
    left_min  = get_region_min(15, 70)

    left = (
        left_mean * 0.7
        + left_min * 0.3
    )

    # =====================================================
    # RIGHT
    # =====================================================

    right_mean = get_region_mean(290, 345)
    right_min  = get_region_min(290, 345)

    right = (
        right_mean * 0.7
        + right_min * 0.3
    )

    return (
        front,
        front_min,
        left,
        right
    )

# =========================================================
# CONTROL
# =========================================================
def compute_control():

    global current_w
    global escape_dir

    (
        front,
        front_min,
        left,
        right
    ) = analyze_obstacle()

    # =====================================================
    # obstacle force
    # =====================================================

    left_force = max(
        0.0,
        SAFE_DIST - left
    )

    right_force = max(
        0.0,
        SAFE_DIST - right
    )

    # =====================================================
    # SOFT AVOIDANCE
    # =====================================================

    # 부드러운 회피

    target_w = (
        left_force
        - right_force
    ) * TURN_GAIN

    # =====================================================
    # FRONT BOOST
    # =====================================================

    if front < FRONT_DIST:

        # 더 넓은 방향 선택

        if left > right:

            escape_dir = 1

        else:

            escape_dir = -1

        # 가까울수록 steering 강화

        boost = (
            (FRONT_DIST - front)
            / FRONT_DIST
        )

        target_w *= (
            1.0 + boost * 4.0
        )

        # steering 0 방지

        target_w += (
            0.35 * escape_dir
        )

    # =====================================================
    # HARD SAFETY OVERRIDE
    # =====================================================

    # 진짜 가까우면
    # 부드러운 회피 중단
    # 강제 회피

    if front_min < CRITICAL_DIST:

        target_w = (
            CRITICAL_W
            * escape_dir
        )

    # =====================================================
    # SMOOTHING
    # =====================================================

    current_w = (
        current_w * (1.0 - STEERING_ALPHA)
        + target_w * STEERING_ALPHA
    )

    # =====================================================
    # LIMIT
    # =====================================================

    current_w = np.clip(
        current_w,
        -MAX_W,
        MAX_W
    )

    # =====================================================
    # SPEED
    # =====================================================

    # 가까울수록 감속

    front_scale = np.clip(
        front / 80.0,
        0.45,
        1.0
    )

    v = BASE_SPEED * front_scale

    # =====================================================
    # TURN SLOWDOWN
    # =====================================================

    # 회전 중 너무 감속하면
    # 장애물 앞에서 멈춤

    turn_scale = max(
        0.70,
        1.0 - abs(current_w) * 0.35
    )

    v *= turn_scale

    # =====================================================
    # MIN SPEED
    # =====================================================

    v = max(v, MIN_SPEED)

    # =====================================================
    # DIFFERENTIAL DRIVE
    # =====================================================

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
        front_min,
        left,
        right,
        left_force,
        right_force,
        escape_dir
    )

# =========================================================
# MOTOR
# =========================================================
def send_motor(left_wheel, right_wheel):

    cmd = (
        f"{left_wheel:.3f},"
        f"{right_wheel:.3f}\n"
    )

    arduino_ser.write(cmd.encode())

# =========================================================
# STOP
# =========================================================
def stop_robot():

    send_motor(0.0, 0.0)

# =========================================================
# MAIN LOOP
# =========================================================
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

        # =================================================
        # 한 바퀴 스캔 완료
        # =================================================

        if s_flag != 1:
            continue

        apply_median_filter()

        (
            left_wheel,
            right_wheel,
            v,
            w,
            front,
            front_min,
            left,
            right,
            left_force,
            right_force,
            escape_dir
        ) = compute_control()

        # =================================================
        # MOTOR SEND
        # =================================================

        send_motor(
            left_wheel,
            right_wheel
        )

        # =================================================
        # DEBUG
        # =================================================

        print(
            f"v:{v:.2f} | "
            f"w:{w:.2f} | "
            f"F:{front:.1f} | "
            f"Fmin:{front_min:.1f} | "
            f"L:{left:.1f} | "
            f"R:{right:.1f} | "
            f"LF:{left_force:.1f} | "
            f"RF:{right_force:.1f} | "
            f"DIR:{escape_dir}"
        )

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(
        bytes([0xA5, 0x25])
    )
