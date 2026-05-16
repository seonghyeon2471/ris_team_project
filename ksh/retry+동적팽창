# =========================================
# ★ 동적 팽창 추가된 버전
# 기존 로직 최대한 유지
# =========================================

import serial
import math
import time
import numpy as np

# =========================================
# SERIAL
# =========================================

arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# IMU (MPU-6050 등) I2C — smbus2 사용
USE_IMU = True

try:
    import smbus2
    imu_bus  = smbus2.SMBus(1)
    IMU_ADDR = 0x68
    imu_bus.write_byte_data(IMU_ADDR, 0x6B, 0)

except Exception:
    USE_IMU = False
    print("[WARN] IMU 초기화 실패 → IMU 비활성화")

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
# ROBOT PHYSICAL PARAMETER
# =========================================

ROBOT_RADIUS = 14.0
WHEEL_BASE   = 17.0

# ★ 추가
# 라이다 ~ 뒷바퀴축 거리
LIDAR_TO_REAR_AXLE = 13.5

# =========================================
# DRIVE PARAMETER
# =========================================

MAX_SPEED = 0.20
MIN_SPEED = 0.07

MAX_W     = 1.1
TURN_GAIN = 1.1

SCAN_LIMIT  = 150
FRONT_RANGE = 72

# =========================================
# FILTER PARAMETER
# =========================================

EMA_ALPHA = 0.3
MEDIAN_K  = 2

# =========================================
# SMOOTHING PARAMETER
# =========================================

SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.20

DANGER_DIST = 18

# =========================================
# GAP PARAMETER
# =========================================

SAFE_DIST          = 17
INFLATION_MAX_DIST = 25

FRONT_CLEAR_DIST  = 23
FRONT_CLEAR_RANGE = 15

# =========================================
# STATE MACHINE
# =========================================

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2
STATE_RAMP    = 3

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST   = 6

REVERSE_DURATION = 0.18
ROTATE_DURATION  = 1.00

REVERSE_SPEED = -0.10
ROTATE_W      = 0.7

# =========================================
# RAMP PARAMETER
# =========================================

RAMP_PITCH_THRESH  = 8.0
RAMP_EXIT_PITCH    = 3.0

LIDAR_DROP_THRESH  = 30.0
RAMP_LIDAR_TIMEOUT = 3.0

RAMP_SPEED         = 0.14
RAMP_INFLATION_MAX = 10
RAMP_SAFE_DIST     = 8

# =========================================
# STATE
# =========================================

scan_data = np.full(
    360,
    float(SCAN_LIMIT),
    dtype=np.float32
)

prev_angle = 0.0

prev_front_avg  = float(SCAN_LIMIT)
ramp_start_time = 0.0

# ★ 추가
v_cmd = 0.0
w_cmd = 0.0

# =========================================
# UTIL
# =========================================

def normalize_angle(angle):
    return int(angle) % 360

# =========================================
# IMU
# =========================================

def read_imu_pitch():

    if not USE_IMU:
        return 0.0

    try:

        def read_word(reg):

            h = imu_bus.read_byte_data(IMU_ADDR, reg)
            l = imu_bus.read_byte_data(IMU_ADDR, reg + 1)

            val = (h << 8) | l

            return val - 65536 if val >= 0x8000 else val

        ax = read_word(0x3B) / 16384.0
        ay = read_word(0x3D) / 16384.0
        az = read_word(0x3F) / 16384.0

        pitch = math.degrees(
            math.atan2(ax, math.sqrt(ay**2 + az**2))
        )

        return abs(pitch)

    except Exception:
        return 0.0

# =========================================
# RAMP DETECT
# =========================================

def detect_ramp(front_avg):

    global prev_front_avg

    pitch = read_imu_pitch()

    prev_front_avg = front_avg

    if USE_IMU:
        return pitch >= RAMP_PITCH_THRESH

    drop = front_avg - prev_front_avg

    return drop >= LIDAR_DROP_THRESH

def ramp_exited():

    if USE_IMU:
        return read_imu_pitch() < RAMP_EXIT_PITCH

    return (time.time() - ramp_start_time) > RAMP_LIDAR_TIMEOUT

# =========================================
# FILTER
# =========================================

def apply_ema(angle, new_dist_cm):

    scan_data[angle] = (
        (1.0 - EMA_ALPHA) * scan_data[angle]
        + EMA_ALPHA * new_dist_cm
    )

def apply_median_filter():

    k      = MEDIAN_K
    window = 2 * k + 1

    filtered = np.empty(360, dtype=np.float32)

    for i in range(360):

        indices = [
            (i + d) % 360
            for d in range(-k, k + 1)
        ]

        values = np.sort(scan_data[indices])

        filtered[i] = values[window // 2]

    scan_data[:] = filtered

# =========================================
# ★ 동적 팽창 반경
# =========================================

def get_dynamic_radius(v, w):

    if abs(w) < 0.05:
        return ROBOT_RADIUS

    R = abs(v / w)

    rear_orbit = math.sqrt(
        R**2 + LIDAR_TO_REAR_AXLE**2
    )

    inner_cut = rear_orbit - R

    return ROBOT_RADIUS + inner_cut

# =========================================
# OBSTACLE INFLATION
# =========================================

def inflate_obstacles(
    dists,
    inflation_max=None,
    dynamic_radius=None
):

    if inflation_max is None:
        inflation_max = INFLATION_MAX_DIST

    # ★ 핵심
    radius = (
        dynamic_radius
        if dynamic_radius is not None
        else ROBOT_RADIUS
    )

    proc = dists.copy()

    for i in range(len(dists)):

        d = dists[i]

        if d < 5 or d >= inflation_max:
            continue

        alpha = math.degrees(
            math.asin(min(radius / d, 1.0))
        )

        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists)-1, int(i + alpha))

        proc[start_idx:end_idx+1] = 0.0

    return proc

# =========================================
# GAP SEARCH
# =========================================

def find_gaps(proc_dists, angles, safe_dist=None):

    if safe_dist is None:
        safe_dist = SAFE_DIST

    gaps      = []
    gap_start = None

    for i, d in enumerate(proc_dists):

        if d > safe_dist:

            if gap_start is None:
                gap_start = i

        else:

            if gap_start is not None:

                gaps.append((gap_start, i - 1))
                gap_start = None

    if gap_start is not None:
        gaps.append((gap_start, len(proc_dists)-1))

    return gaps

def score_gap(gap, proc_dists, angles):

    start, end = gap

    width = end - start

    center_i     = (start + end) / 2.0
    center_angle = angles[int(center_i)]

    avg_dist = np.mean(proc_dists[start:end+1])
    min_dist = np.min(proc_dists[start:end+1])

    score = (
        width               * 1.5
        + avg_dist          * 2.0
        + min_dist          * 1.0
        - abs(center_angle) * 0.15
    )

    return score

def select_best_gap(gaps, proc_dists, angles):

    best_gap   = None
    best_score = -1e9

    for gap in gaps:

        s = score_gap(gap, proc_dists, angles)

        if s > best_score:
            best_score = s
            best_gap   = gap

    return best_gap

# =========================================
# PLANNING
# =========================================

def find_best_direction(
    smoothing,
    on_ramp=False
):

    global prev_angle
    global v_cmd
    global w_cmd

    angles = np.arange(
        -FRONT_RANGE,
        FRONT_RANGE + 1
    )

    dists = np.array(
        [scan_data[a % 360] for a in angles],
        dtype=np.float32
    )

    infl_max  = (
        RAMP_INFLATION_MAX
        if on_ramp
        else INFLATION_MAX_DIST
    )

    safe_dist = (
        RAMP_SAFE_DIST
        if on_ramp
        else SAFE_DIST
    )

    # =====================================
    # ★ 핵심 추가
    # =====================================

    dynamic_radius = get_dynamic_radius(
        v_cmd,
        w_cmd
    )

    proc_dists = inflate_obstacles(
        dists,
        inflation_max=infl_max,
        dynamic_radius=dynamic_radius
    )

    gaps = find_gaps(
        proc_dists,
        angles,
        safe_dist=safe_dist
    )

    if not gaps:
        return None

    best_gap = select_best_gap(
        gaps,
        proc_dists,
        angles
    )

    start, end = best_gap

    center_i  = (start + end) / 2.0
    gap_angle = float(angles[int(center_i)])

    front_clear = float(np.min(
        scan_data[
            np.arange(
                -FRONT_CLEAR_RANGE,
                FRONT_CLEAR_RANGE + 1
            ) % 360
        ]
    ))

    if front_clear > FRONT_CLEAR_DIST:

        target     = gap_angle * 0.2
        bias_label = "STRAIGHT"

    else:

        target     = gap_angle
        bias_label = "GAP"

    target = (
        prev_angle * smoothing
        + target * (1.0 - smoothing)
    )

    prev_angle = target

    return (
        target,
        bias_label,
        front_clear,
        dynamic_radius
    )

# =========================================
# CONTROL
# =========================================

ALIGN_THRESHOLD = 15

def compute_cmd(target_angle, on_ramp=False):

    w = math.radians(target_angle) * TURN_GAIN

    w = float(np.clip(w, -MAX_W, MAX_W))

    if abs(target_angle) > ALIGN_THRESHOLD:
        return 0.0, w

    front_min = float(np.min(
        scan_data[np.arange(-10, 11) % 360]
    ))

    if on_ramp:
        return RAMP_SPEED, w

    obstacle_scale = min(front_min / 40.0, 1.0)

    speed = max(
        MAX_SPEED * obstacle_scale,
        MIN_SPEED
    )

    return speed, w

def send_cmd(v, w):

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# AVOID DIRECTION
# =========================================

def choose_avoid_direction():

    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))

    if left_avg >= right_avg:

        print(
            f"  [AVOID DIR] LEFT "
            f"(L:{left_avg:.1f} "
            f"R:{right_avg:.1f})"
        )

        return 1

    else:

        print(
            f"  [AVOID DIR] RIGHT "
            f"(L:{left_avg:.1f} "
            f"R:{right_avg:.1f})"
        )

        return -1

# =========================================
# MAIN LOOP
# =========================================

print("NAVIGATION START")

try:

    while True:

        raw = lidar_ser.read(5)

        if len(raw) != 5:
            continue

        s_flag     = raw[0] & 0x01
        s_inv_flag = (raw[0] & 0x02) >> 1

        if s_inv_flag != (1 - s_flag):
            continue

        if (raw[1] & 0x01) != 1:
            continue

        quality = raw[0] >> 2

        if quality < 3:
            continue

        angle_raw = (
            (raw[1] >> 1)
            | (raw[2] << 7)
        )

        angle = int(angle_raw / 64.0) % 360

        dist_raw = raw[3] | (raw[4] << 8)

        dist_cm = (dist_raw / 4.0) / 10.0

        if dist_cm < 3 or dist_cm > SCAN_LIMIT:
            continue

        apply_ema(angle, dist_cm)

        if s_flag != 1:
            continue

        apply_median_filter()

        now = time.time()

        front_avg = float(np.mean(
            scan_data[np.arange(-10, 11) % 360]
        ))

        front_min = float(np.min(
            scan_data[np.arange(-10, 11) % 360]
        ))

        # =================================
        # REVERSE
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

        # =================================
        # ROTATE
        # =================================

        if state == STATE_ROTATE:

            if now < maneuver_end_time:

                send_cmd(
                    0.0,
                    ROTATE_W * rotate_dir
                )

            else:

                rotate_dir *= -1

                state      = STATE_NORMAL
                prev_angle = 0.0

            continue

        # =================================
        # RAMP
        # =================================

        if state == STATE_RAMP:

            if ramp_exited():

                state = STATE_NORMAL

                print("[RAMP→NORMAL]")

            else:

                result = find_best_direction(
                    SMOOTHING_NORMAL,
                    on_ramp=True
                )

                if result is not None:

                    (
                        target_angle,
                        bias_label,
                        front_clear,
                        dynamic_radius
                    ) = result

                    v_cmd, w_cmd = compute_cmd(
                        target_angle,
                        on_ramp=True
                    )

                    send_cmd(v_cmd, w_cmd)

                    print(
                        f"[RAMP] "
                        f"target:{target_angle:.1f} "
                        f"dynR:{dynamic_radius:.1f}"
                    )

            continue

        # =================================
        # NORMAL
        # =================================

        if front_min < EMERGENCY_DIST:

            rotate_dir = choose_avoid_direction()

            state = STATE_REVERSE

            maneuver_end_time = (
                now + REVERSE_DURATION
            )

            send_cmd(REVERSE_SPEED, 0.0)

            continue

        if detect_ramp(front_avg):

            state           = STATE_RAMP
            ramp_start_time = now

            continue

        smoothing = (
            SMOOTHING_DANGER
            if front_min < DANGER_DIST
            else SMOOTHING_NORMAL
        )

        result = find_best_direction(
            smoothing
        )

        if result is None:

            rotate_dir = choose_avoid_direction()

            state = STATE_REVERSE

            maneuver_end_time = (
                now + REVERSE_DURATION
            )

            send_cmd(REVERSE_SPEED, 0.0)

            continue

        (
            target_angle,
            bias_label,
            front_clear,
            dynamic_radius
        ) = result

        v_cmd, w_cmd = compute_cmd(target_angle)

        send_cmd(v_cmd, w_cmd)

        print(
            f"TARGET:{target_angle:6.1f}° | "
            f"v:{v_cmd:.2f} | "
            f"w:{w_cmd:.2f} | "
            f"front:{front_min:.1f} | "
            f"dynR:{dynamic_radius:.1f} | "
            f"{bias_label}"
        )

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(bytes([0xA5, 0x25]))
