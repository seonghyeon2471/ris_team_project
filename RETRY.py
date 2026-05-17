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

BASE_SPEED = 0.18
MIN_SPEED  = 0.05

MAX_W      = 0.9

WHEEL_BASE = 0.17

# =========================================
# DISTANCE PARAMETER
# =========================================
FRONT_WARN_DIST   = 60
FRONT_DANGER_DIST = 35
EMERGENCY_DIST    = 12

SIDE_DIST = 35

# =========================================
# FILTER
# =========================================
EMA_ALPHA = 0.35
MEDIAN_K  = 2

# =========================================
# STATE MACHINE
# =========================================
STATE_FORWARD     = 0
STATE_AVOID_LEFT  = 1
STATE_AVOID_RIGHT = 2

state = STATE_FORWARD

AVOID_HOLD_TIME = 0.65
state_end_time  = 0.0

# =========================================
# STEERING
# =========================================
TARGET_W_FORWARD = 0.0
TARGET_W_AVOID   = 0.55

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

# =========================================
# SENSOR ANALYSIS
# =========================================
def analyze():

    # 정면
    front = min(
        get_region_min(350, 359),
        get_region_min(0, 10)
    )

    # 좌우
    left = (
        get_region_mean(15, 60) * 0.7
        + get_region_min(15, 60) * 0.3
    )

    right = (
        get_region_mean(300, 345) * 0.7
        + get_region_min(300, 345) * 0.3
    )

    return front, left, right

# =========================================
# STATE UPDATE
# =========================================
def update_state(front, left, right):

    global state
    global state_end_time

    now = time.time()

    # =====================================
    # 회피 상태 유지
    # =====================================
    if now < state_end_time:
        return

    # =====================================
    # 전방 충분히 비어있음
    # =====================================
    if front > FRONT_WARN_DIST:

        state = STATE_FORWARD
        return

    # =====================================
    # 오른쪽이 더 막힘
    # → 왼쪽 회피
    # =====================================
    if right < left:

        state = STATE_AVOID_LEFT

    # =====================================
    # 왼쪽이 더 막힘
    # → 오른쪽 회피
    # =====================================
    else:

        state = STATE_AVOID_RIGHT

    state_end_time = (
        now + AVOID_HOLD_TIME
    )

# =========================================
# CONTROL
# =========================================
def compute_control(front):

    global current_w

    # =====================================
    # STATE별 목표 회전
    # =====================================
    if state == STATE_FORWARD:

        target_w = TARGET_W_FORWARD

    elif state == STATE_AVOID_LEFT:

        target_w = TARGET_W_AVOID

    else:

        target_w = -TARGET_W_AVOID

    # =====================================
    # 매우 가까우면 회전 강화
    # =====================================
    if front < FRONT_DANGER_DIST:

        scale = (
            (FRONT_DANGER_DIST - front)
            / FRONT_DANGER_DIST
        )

        target_w *= (
            1.0 + scale * 1.2
        )

    # =====================================
    # Steering Smoothing
    # =====================================
    current_w = (
        current_w * (1.0 - STEERING_ALPHA)
        + target_w * STEERING_ALPHA
    )

    current_w = np.clip(
        current_w,
        -MAX_W,
        MAX_W
    )

    # =====================================
    # 속도 계산
    # =====================================
    if front > 90:

        v = BASE_SPEED

    elif front > 60:

        v = 0.15

    elif front > 40:

        v = 0.11

    elif front > EMERGENCY_DIST:

        v = 0.07

    else:

        v = 0.04

    # =====================================
    # 회전 클수록 감속
    # =====================================
    turn_scale = max(
        0.45,
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
        current_w
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

        # =================================
        # SENSOR
        # =================================
        front, left, right = analyze()

        # =================================
        # STATE UPDATE
        # =================================
        update_state(
            front,
            left,
            right
        )

        # =================================
        # CONTROL
        # =================================
        (
            left_wheel,
            right_wheel,
            v,
            w
        ) = compute_control(front)

        # =================================
        # SEND
        # =================================
        send_motor(
            left_wheel,
            right_wheel
        )

        # =================================
        # DEBUG
        # =================================
        state_name = {
            0: "FORWARD",
            1: "LEFT",
            2: "RIGHT"
        }[state]

        print(
            f"{state_name} | "
            f"v:{v:.2f} | "
            f"w:{w:.2f} | "
            f"F:{front:.1f} | "
            f"L:{left:.1f} | "
            f"R:{right:.1f}"
        )

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(bytes([0xA5, 0x25]))
