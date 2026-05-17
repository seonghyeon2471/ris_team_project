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
# ROBOT PARAMETER
# =========================================
ROBOT_RADIUS = 17.0
WHEEL_BASE   = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED = 0.14
MIN_SPEED = 0.11

MAX_W     = 0.8
TURN_GAIN = 1.8

SCAN_LIMIT  = 150
FRONT_RANGE = 65

# =========================================
# FILTER PARAMETER
# =========================================
EMA_ALPHA = 0.3
MEDIAN_K  = 2

# =========================================
# SMOOTHING
# =========================================
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.20

DANGER_DIST = 12

# =========================================
# GAP PARAMETER
# =========================================
SAFE_DIST          = 14
INFLATION_MAX_DIST = 25

FRONT_CLEAR_DIST  = 23
FRONT_CLEAR_RANGE = 15

# =========================================
# GOAL
# =========================================
GOAL_ANGLE  = 0
GOAL_WEIGHT = 1.5

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL   = 0
STATE_REVERSE  = 1
STATE_ROTATE   = 2
STATE_REORIENT = 3

state = STATE_NORMAL

maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST   = 8
REVERSE_DURATION = 0.18
ROTATE_DURATION  = 1.00

REVERSE_SPEED = -0.10
ROTATE_W      = 0.9

# =========================================
# REORIENT
# =========================================
REORIENT_W          = 0.6
REORIENT_CLEAR_DIST = 25
REORIENT_TIMEOUT    = 4.0

reorient_dir        = 1
reorient_start_time = 0.0

# =========================================
# GAP LOCK
# =========================================
GAP_LOCK_TIME = 0.55

gap_lock_angle    = 0.0
gap_lock_end_time = 0.0

# =========================================
# EMERGENCY DEBOUNCE
# =========================================
EMERGENCY_COUNT_LIMIT = 3
emergency_count = 0

# =========================================
# DATA
# =========================================
scan_data  = np.full(
    360,
    float(SCAN_LIMIT),
    dtype=np.float32
)

prev_angle = 0.0

# =========================================
# FILTER
# =========================================
def apply_ema(angle, new_dist_cm):

    scan_data[angle] = (
        (1.0 - EMA_ALPHA)
        * scan_data[angle]
        + EMA_ALPHA * new_dist_cm
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
# INFLATION
# =========================================
def inflate_obstacles(dists):

    proc = dists.copy()

    for i in range(len(dists)):

        d = dists[i]

        if d < 5 or d >= INFLATION_MAX_DIST:
            continue

        alpha = math.degrees(
            math.asin(
                min(ROBOT_RADIUS / d, 1.0)
            )
        )

        start_idx = max(
            0,
            int(i - alpha)
        )

        end_idx = min(
            len(dists) - 1,
            int(i + alpha)
        )

        proc[start_idx:end_idx+1] = 0.0

    return proc

# =========================================
# GAP SEARCH
# =========================================
def find_gaps(proc_dists):

    gaps = []

    gap_start = None

    for i, d in enumerate(proc_dists):

        if d > SAFE_DIST:

            if gap_start is None:
                gap_start = i

        else:

            if gap_start is not None:

                gaps.append(
                    (gap_start, i - 1)
                )

                gap_start = None

    if gap_start is not None:

        gaps.append(
            (gap_start, len(proc_dists)-1)
        )

    return gaps

# =========================================
# GAP SCORE
# =========================================
def score_gap(gap, proc_dists, angles):

    start, end = gap

    width = end - start

    center_i = int(
        (start + end) / 2
    )

    center_angle = angles[center_i]

    avg_dist = np.mean(
        proc_dists[start:end+1]
    )

    goal_diff = abs(
        center_angle - GOAL_ANGLE
    )

    goal_bonus = (
        max(
            0.0,
            (FRONT_RANGE - goal_diff)
            / FRONT_RANGE
        )
        * GOAL_WEIGHT
    )

    score = (
        width * 0.5
        + avg_dist * 1.2
        - abs(center_angle) * 1.6
        + goal_bonus
    )

    return score

# =========================================
# BEST GAP
# =========================================
def select_best_gap(gaps, proc_dists, angles):

    best_gap   = None
    best_score = -1e9

    for gap in gaps:

        s = score_gap(
            gap,
            proc_dists,
            angles
        )

        if s > best_score:

            best_score = s
            best_gap   = gap

    return best_gap

# =========================================
# BEST INDEX
# =========================================
def find_best_index_in_gap(
    best_gap,
    proc_dists,
    angles
):

    start, end = best_gap

    best_idx   = int((start + end) / 2)
    best_score = -1e9

    for i in range(start, end + 1):

        dist   = proc_dists[i]
        angle  = angles[i]

        margin = min(
            i - start,
            end - i
        )

        score = (
            dist * 1.0
            + margin * 0.8
            - abs(angle) * 1.5
        )

        if score > best_score:

            best_score = score
            best_idx   = i

    return best_idx

# =========================================
# REORIENT DIRECTION
# =========================================
def choose_reorient_direction():

    left_avg = float(
        np.mean(scan_data[1:91])
    )

    right_avg = float(
        np.mean(scan_data[270:360])
    )

    if left_avg >= right_avg:
        return 1
    else:
        return -1

# =========================================
# AVOID DIRECTION
# =========================================
def choose_avoid_direction():

    left_avg = float(
        np.mean(scan_data[1:90])
    )

    right_avg = float(
        np.mean(scan_data[271:360])
    )

    if left_avg >= right_avg:
        return 1
    else:
        return -1

# =========================================
# FIND BEST DIRECTION
# =========================================
def find_best_direction(smoothing):

    global prev_angle
    global gap_lock_angle
    global gap_lock_end_time

    now = time.time()

    angles = np.arange(
        -FRONT_RANGE,
        FRONT_RANGE + 1
    )

    dists = np.array([
        scan_data[a % 360]
        for a in angles
    ], dtype=np.float32)

    proc_dists = inflate_obstacles(dists)

    gaps = find_gaps(proc_dists)

    if not gaps:
        return None

    # =====================================
    # GAP LOCK
    # =====================================
    if now < gap_lock_end_time:

        gap_angle = gap_lock_angle

    else:

        best_gap = select_best_gap(
            gaps,
            proc_dists,
            angles
        )

        best_idx = find_best_index_in_gap(
            best_gap,
            proc_dists,
            angles
        )

        gap_angle = float(
            angles[best_idx]
        )

        gap_lock_angle = gap_angle

        gap_lock_end_time = (
            now + GAP_LOCK_TIME
        )

    # =====================================
    # FRONT CLEAR
    # =====================================
    front_clear = float(
        np.min(
            scan_data[
                np.arange(
                    -FRONT_CLEAR_RANGE,
                    FRONT_CLEAR_RANGE + 1
                ) % 360
            ]
        )
    )

    CRITICAL_DIST = (
        EMERGENCY_DIST * 2
    )

    # =====================================
    # STRAIGHT BIAS
    # =====================================
    if front_clear > FRONT_CLEAR_DIST:

        target = gap_angle * 0.2
        bias_label = "STRAIGHT"

    elif front_clear > CRITICAL_DIST:

        if abs(prev_angle) > 20:

            target = gap_angle

        else:

            target = gap_angle * 1.0

        smoothing = SMOOTHING_DANGER

        bias_label = "GAP"

    else:

        target = gap_angle
        smoothing = 0.0

        bias_label = "CRITICAL"

    # =====================================
    # SMOOTHING
    # =====================================
    target = (
        prev_angle * smoothing
        + target * (1.0 - smoothing)
    )

    prev_angle = target

    return (
        target,
        bias_label,
        front_clear
    )

# =========================================
# CONTROL
# =========================================
ALIGN_THRESHOLD = 10

def compute_cmd(target_angle):

    w = (
        math.radians(target_angle)
        * TURN_GAIN
    )

    w = float(
        np.clip(w, -MAX_W, MAX_W)
    )

    search_indices = (
        np.arange(
            -FRONT_RANGE,
            FRONT_RANGE + 1
        ) % 360
    )

    relevant_min = float(
        np.min(scan_data[search_indices])
    )

    if abs(target_angle) > ALIGN_THRESHOLD:

        return 0.05, w

    obstacle_scale = min(
        relevant_min / 25.0,
        1.0
    )

    speed = max(
        MAX_SPEED * obstacle_scale,
        MIN_SPEED
    )

    return speed, w

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
print("NAVIGATION START")

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

        dist_cm = (
            (raw[3]
            | (raw[4] << 8))
            / 40.0
        )

        if 3 < dist_cm < SCAN_LIMIT:

            apply_ema(angle, dist_cm)

        if s_flag != 1:
            continue

        apply_median_filter()

        now = time.time()

        # =================================
        # REVERSE
        # =================================
        if state == STATE_REVERSE:

            if now < maneuver_end_time:

                send_cmd(
                    REVERSE_SPEED,
                    0.0
                )

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

                reorient_dir = (
                    choose_reorient_direction()
                )

                reorient_start_time = now

                state = STATE_REORIENT

                prev_angle = 0.0

                for a in range(-45, 46):

                    scan_data[a % 360] = (
                        float(SCAN_LIMIT)
                    )

            continue

        # =================================
        # REORIENT
        # =================================
        if state == STATE_REORIENT:

            elapsed = (
                now - reorient_start_time
            )

            front_clear = float(
                np.min(
                    scan_data[
                        np.arange(
                            -FRONT_CLEAR_RANGE,
                            FRONT_CLEAR_RANGE + 1
                        ) % 360
                    ]
                )
            )

            if (
                front_clear
                > REORIENT_CLEAR_DIST
            ):

                state = STATE_NORMAL
                continue

            if elapsed > REORIENT_TIMEOUT:

                state = STATE_NORMAL
                continue

            send_cmd(
                0.0,
                REORIENT_W * reorient_dir
            )

            continue

        # =================================
        # NORMAL
        # =================================
        front_region = scan_data[
            np.arange(-6, 7) % 360
        ]

        front_mean = float(
            np.mean(front_region)
        )

        front_min = float(
            np.min(front_region)
        )

        front_score = (
            front_mean * 0.7
            + front_min * 0.3
        )

        # =================================
        # EMERGENCY DEBOUNCE
        # =================================
        if front_score < EMERGENCY_DIST:

            emergency_count += 1

        else:

            emergency_count = 0

        if emergency_count >= EMERGENCY_COUNT_LIMIT:

            rotate_dir = (
                choose_avoid_direction()
            )

            state = STATE_REVERSE

            maneuver_end_time = (
                now + REVERSE_DURATION
            )

            send_cmd(
                REVERSE_SPEED,
                0.0
            )

            emergency_count = 0

            continue

        # =================================
        # PLANNING
        # =================================
        smoothing = (
            SMOOTHING_DANGER
            if front_score < DANGER_DIST
            else SMOOTHING_NORMAL
        )

        result = find_best_direction(
            smoothing
        )

        if result is None:

            rotate_dir = (
                choose_avoid_direction()
            )

            state = STATE_REVERSE

            maneuver_end_time = (
                now + REVERSE_DURATION
            )

            send_cmd(
                REVERSE_SPEED,
                0.0
            )

            continue

        (
            target_angle,
            bias_label,
            front_clear
        ) = result

        v, w = compute_cmd(
            target_angle
        )

        send_cmd(v, w)

        print(
            f"TRG:{target_angle:5.1f}° | "
            f"v:{v:.2f} | "
            f"w:{w:.2f} | "
            f"score:{front_score:.1f} | "
            f"clear:{front_clear:.1f} | "
            f"bias:{bias_label}"
        )

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(
        bytes([0xA5, 0x25])
    )
