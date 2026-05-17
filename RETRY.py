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
# ROBOT PARAMETER
# =========================================
ROBOT_RADIUS = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED = 0.18
MIN_SPEED = 0.10

TURN_GAIN = 1.4
MAX_W      = 1.0

SCAN_LIMIT = 150

# =========================================
# AVOIDANCE PARAMETER
# =========================================
SIDE_DIST        = 35      # 측면 회피 시작 거리
FRONT_DANGER     = 18      # 정면 위험 거리
EMERGENCY_DIST   = 8       # 긴급 회피 거리

AVOID_GAIN       = 1.6
FRONT_AVOID_GAIN = 2.5

# =========================================
# FILTER PARAMETER
# =========================================
EMA_ALPHA = 0.35
MEDIAN_K  = 2

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state = STATE_NORMAL

maneuver_end_time = 0.0
rotate_dir        = 1

REVERSE_DURATION = 0.18
ROTATE_DURATION  = 0.65

REVERSE_SPEED = -0.08
ROTATE_W      = 0.9

# =========================================
# DATA
# =========================================
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)

# =========================================
# FILTER
# =========================================
def apply_ema(angle, new_dist_cm):

    scan_data[angle] = (
        (1.0 - EMA_ALPHA) * scan_data[angle]
        + EMA_ALPHA * new_dist_cm
    )

def apply_median_filter():

    k = MEDIAN_K
    window = 2 * k + 1

    filtered = np.empty(360, dtype=np.float32)

    for i in range(360):

        indices = [(i + d) % 360 for d in range(-k, k + 1)]

        values = np.sort(scan_data[indices])

        filtered[i] = values[window // 2]

    scan_data[:] = filtered

# =========================================
# OBSTACLE AVOIDANCE
# =========================================
def find_target_angle():

    # =========================
    # 전방 거리
    # =========================
    front = float(
        np.min(
            scan_data[np.arange(-8, 9) % 360]
        )
    )

    # =========================
    # 좌측 거리
    # =========================
    left = float(
        np.min(
            scan_data[np.arange(15, 55) % 360]
        )
    )

    # =========================
    # 우측 거리
    # =========================
    right = float(
        np.min(
            scan_data[np.arange(305, 346) % 360]
        )
    )

    # =========================
    # 기본 목표 = 직진
    # =========================
    target_angle = 0.0

    # =====================================
    # 왼쪽 장애물 가까우면
    # 오른쪽으로 조금 밀기
    # =====================================
    if left < SIDE_DIST:

        target_angle -= (
            (SIDE_DIST - left)
            * AVOID_GAIN
        )

    # =====================================
    # 오른쪽 장애물 가까우면
    # 왼쪽으로 조금 밀기
    # =====================================
    if right < SIDE_DIST:

        target_angle += (
            (SIDE_DIST - right)
            * AVOID_GAIN
        )

    # =====================================
    # 정면 장애물 매우 가까우면
    # 강제 회피
    # =====================================
    if front < FRONT_DANGER:

        if left > right:

            target_angle += (
                (FRONT_DANGER - front)
                * FRONT_AVOID_GAIN
            )

        else:

            target_angle -= (
                (FRONT_DANGER - front)
                * FRONT_AVOID_GAIN
            )

    # 최대 조향 제한
    target_angle = np.clip(
        target_angle,
        -45,
        45
    )

    return target_angle, front, left, right

# =========================================
# CONTROL
# =========================================
def compute_cmd(target_angle, front):

    # 조향 계산
    w = math.radians(target_angle) * TURN_GAIN

    w = float(
        np.clip(w, -MAX_W, MAX_W)
    )

    # =====================================
    # 속도 계산
    # =====================================

    # 장애물 멀면 최고속
    if front > 50:

        v = MAX_SPEED

    # 일반 회피 구간
    elif front > 25:

        obstacle_scale = front / 50.0

        v = max(
            MAX_SPEED * obstacle_scale,
            MIN_SPEED
        )

    # 위험 구간
    elif front > EMERGENCY_DIST:

        v = 0.05

    # 매우 위험
    else:

        v = 0.02

    # =====================================
    # 회전 클수록 감속
    # =====================================
    if abs(target_angle) > 30:

        v *= 0.6

    elif abs(target_angle) > 20:

        v *= 0.8

    return v, w

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():

    send_cmd(0.0, 0.0)

def choose_avoid_direction():

    left_avg = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))

    return 1 if left_avg >= right_avg else -1

# =========================================
# MAIN LOOP
# =========================================
print("NAVIGATION START")

try:

    while True:

        raw = lidar_ser.read(5)

        if len(raw) != 5:
            continue

        s_flag = raw[0] & 0x01

        if (raw[0] & 0x02) >> 1 != (1 - s_flag):
            continue

        if (raw[1] & 0x01) != 1:
            continue

        if (raw[0] >> 2) < 3:
            continue

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

        if s_flag != 1:
            continue

        apply_median_filter()

        now = time.time()

        # =================================
        # STATE MACHINE
        # =================================
        if state == STATE_REVERSE:

            if now < maneuver_end_time:

                send_cmd(REVERSE_SPEED, 0.0)

            else:

                state = STATE_ROTATE

                maneuver_end_time = (
                    now + ROTATE_DURATION
                )

            continue

        if state == STATE_ROTATE:

            if now < maneuver_end_time:

                send_cmd(
                    0.0,
                    ROTATE_W * rotate_dir
                )

            else:

                rotate_dir *= -1

                state = STATE_NORMAL

            continue

        # =================================
        # NORMAL
        # =================================
        target_angle, front, left, right = (
            find_target_angle()
        )

        # 긴급 충돌 방지
        if front < EMERGENCY_DIST:

            rotate_dir = choose_avoid_direction()

            state = STATE_REVERSE

            maneuver_end_time = (
                now + REVERSE_DURATION
            )

            send_cmd(REVERSE_SPEED, 0.0)

            continue

        # 속도 계산
        v, w = compute_cmd(
            target_angle,
            front
        )

        send_cmd(v, w)

        print(
            f"TRG:{target_angle:5.1f}° | "
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
