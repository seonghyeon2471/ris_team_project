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
# ROBOT PARAMETER
# =========================================
ROBOT_RADIUS = 17.0
WHEEL_BASE   = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED    = 0.14
MIN_SPEED    = 0.11

MAX_W        = 0.8
TURN_GAIN    = 1.8

SCAN_LIMIT   = 150
FRONT_RANGE  = 65

# =========================================
# FILTER
# =========================================
EMA_ALPHA = 0.3
MEDIAN_K  = 2

# =========================================
# SMOOTHING
# =========================================
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.20

DANGER_DIST = 8

# =========================================
# GAP PARAMETER
# =========================================
SAFE_DIST          = 14
INFLATION_MAX_DIST = 25

FRONT_CLEAR_DIST   = 12
FRONT_CLEAR_RANGE  = 15

# =========================================
# EMERGENCY
# =========================================
EMERGENCY_DIST   = 6

REVERSE_DURATION = 0.18
ROTATE_DURATION  = 1.00

REVERSE_SPEED    = -0.10
ROTATE_W         = 0.9

# =========================================
# ESCAPE MODE
# =========================================
ESCAPE_TRIGGER = 3

ESCAPE_BACK_TIME    = 0.60
ESCAPE_ROTATE_TIME  = 1.60
ESCAPE_FORWARD_TIME = 0.50

ESCAPE_BACK_SPEED   = -0.12
ESCAPE_ROTATE_W     = 1.2
ESCAPE_FORWARD_SPEED = 0.08

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL         = 0
STATE_REVERSE        = 1
STATE_ROTATE         = 2
STATE_ESCAPE_BACK    = 3
STATE_ESCAPE_ROTATE  = 4
STATE_ESCAPE_FORWARD = 5

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

escape_count = 0

# =========================================
# DATA
# =========================================
scan_data  = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0

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
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
        values = np.sort(scan_data[idx])
        filtered[i] = values[window // 2]

    scan_data[:] = filtered

# =========================================
# OBSTACLE INFLATION
# =========================================
def inflate_obstacles(dists):

    proc = dists.copy()

    for i in range(len(dists)):

        d = dists[i]

        if d < 5 or d >= INFLATION_MAX_DIST:
            continue

        alpha = math.degrees(
            math.asin(min(ROBOT_RADIUS / d, 1.0))
        )

        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists) - 1, int(i + alpha))

        proc[start_idx:end_idx + 1] = 0.0

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
                gaps.append((gap_start, i - 1))
                gap_start = None

    if gap_start is not None:
        gaps.append((gap_start, len(proc_dists) - 1))

    return gaps

def select_best_gap(gaps, proc_dists, angles):

    best_gap = None
    best_score = -1e9

    for gap in gaps:

        start, end = gap

        width = end - start

        center_i = int((start + end) / 2)

        center_angle = angles[center_i]

        avg_dist = np.mean(proc_dists[start:end + 1])

        score = (
            width * 0.5
            + avg_dist * 1.2
            - abs(center_angle) * 0.6
        )

        if score > best_score:
            best_score = score
            best_gap = gap

    return best_gap

def find_best_index(best_gap, proc_dists, angles):

    start, end = best_gap

    best_idx = int((start + end) / 2)
    best_score = -1e9

    for i in range(start, end + 1):

        dist = proc_dists[i]

        margin = min(i - start, end - i)

        angle = angles[i]

        score = (
            dist * 1.0
            + margin * 0.8
            - abs(angle) * 1.5
        )

        if score > best_score:
            best_score = score
            best_idx = i

    return best_idx

# =========================================
# PLANNING
# =========================================
def find_best_direction(smoothing):

    global prev_angle

    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)

    dists = np.array(
        [scan_data[a % 360] for a in angles],
        dtype=np.float32
    )

    proc_dists = inflate_obstacles(dists)

    gaps = find_gaps(proc_dists)

    if not gaps:
        return None

    best_gap = select_best_gap(gaps, proc_dists, angles)

    best_idx = find_best_index(best_gap, proc_dists, angles)

    gap_angle = float(angles[best_idx])

    front_clear = float(np.min(
        scan_data[
            np.arange(
                -FRONT_CLEAR_RANGE,
                FRONT_CLEAR_RANGE + 1
            ) % 360
        ]
    ))

    CRITICAL_DIST = EMERGENCY_DIST * 2

    if front_clear > FRONT_CLEAR_DIST:

        target = gap_angle * 0.2
        bias_label = "STRAIGHT"

    elif front_clear > CRITICAL_DIST:

        target = gap_angle
        smoothing = SMOOTHING_DANGER
        bias_label = "GAP"

    else:

        target = gap_angle
        smoothing = 0.0
        bias_label = "CRITICAL"

    target = (
        prev_angle * smoothing
        + target * (1.0 - smoothing)
    )

    prev_angle = target

    return target, bias_label, front_clear

# =========================================
# CONTROL
# =========================================
ALIGN_THRESHOLD = 10

def compute_cmd(target_angle):

    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    search_indices = (
        np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    )

    relevant_min = float(
        np.min(scan_data[search_indices])
    )

    if abs(target_angle) > ALIGN_THRESHOLD:
        return 0.05, w

    obstacle_scale = min(relevant_min / 25.0, 1.0)

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
# ESCAPE DIRECTION
# =========================================
def choose_avoid_direction():

    left_min = float(np.min(scan_data[0:90]))
    right_min = float(np.min(scan_data[270:359]))

    return 1 if left_min > right_min else -1

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

        if ((raw[0] & 0x02) >> 1) != (1 - s_flag):
            continue

        if (raw[1] & 0x01) != 1:
            continue

        if (raw[0] >> 2) < 3:
            continue

        angle = int(
            ((raw[1] >> 1) | (raw[2] << 7)) / 64.0
        ) % 360

        dist_cm = (
            (raw[3] | (raw[4] << 8)) / 40.0
        )

        if 3 < dist_cm < SCAN_LIMIT:
            apply_ema(angle, dist_cm)

        if s_flag != 1:
            continue

        apply_median_filter()

        now = time.time()

        # =====================================
        # ESCAPE BACK
        # =====================================
        if state == STATE_ESCAPE_BACK:

            if now < maneuver_end_time:

                send_cmd(
                    ESCAPE_BACK_SPEED,
                    0.0
                )

            else:

                state = STATE_ESCAPE_ROTATE

                maneuver_end_time = (
                    now + ESCAPE_ROTATE_TIME
                )

            continue

        # =====================================
        # ESCAPE ROTATE
        # =====================================
        if state == STATE_ESCAPE_ROTATE:

            if now < maneuver_end_time:

                send_cmd(
                    0.0,
                    ESCAPE_ROTATE_W * rotate_dir
                )

            else:

                state = STATE_ESCAPE_FORWARD

                maneuver_end_time = (
                    now + ESCAPE_FORWARD_TIME
                )

            continue

        # =====================================
        # ESCAPE FORWARD
        # =====================================
        if state == STATE_ESCAPE_FORWARD:

            if now < maneuver_end_time:

                send_cmd(
                    ESCAPE_FORWARD_SPEED,
                    0.0
                )

            else:

                state = STATE_NORMAL
                escape_count = 0
                prev_angle = 0.0

            continue

        # =====================================
        # NORMAL RECOVERY
        # =====================================
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

        if state == STATE_ROTATE:

            if now < maneuver_end_time:

                send_cmd(
                    0.0,
                    ROTATE_W * rotate_dir
                )

            else:

                rotate_dir *= -1
                state = STATE_NORMAL
                prev_angle = 0.0

            continue

        # =====================================
        # NORMAL
        # =====================================
        front_min = float(
            np.min(
                scan_data[
                    np.arange(-10, 11) % 360
                ]
            )
        )

        # -------------------------------------
        # EMERGENCY
        # -------------------------------------
        if front_min < EMERGENCY_DIST:

            escape_count += 1

            rotate_dir = choose_avoid_direction()

            # ESCAPE MODE
            if escape_count >= ESCAPE_TRIGGER:

                state = STATE_ESCAPE_BACK

                maneuver_end_time = (
                    now + ESCAPE_BACK_TIME
                )

                print("ESCAPE MODE")

            else:

                state = STATE_REVERSE

                maneuver_end_time = (
                    now + REVERSE_DURATION
                )

            continue

        else:
            escape_count = 0

        # -------------------------------------
        # FTG
        # -------------------------------------
        smoothing = (
            SMOOTHING_DANGER
            if front_min < DANGER_DIST
            else SMOOTHING_NORMAL
        )

        result = find_best_direction(smoothing)

        if result is None:

            escape_count += 1

            rotate_dir = choose_avoid_direction()

            if escape_count >= ESCAPE_TRIGGER:

                state = STATE_ESCAPE_BACK

                maneuver_end_time = (
                    now + ESCAPE_BACK_TIME
                )

                print("ESCAPE MODE")

            else:

                state = STATE_REVERSE

                maneuver_end_time = (
                    now + REVERSE_DURATION
                )

            continue

        target_angle, bias_label, front_clear = result

        v, w = compute_cmd(target_angle)

        send_cmd(v, w)

        print(
            f"TRG:{target_angle:5.1f}° | "
            f"v:{v:.2f} | "
            f"w:{w:.2f} | "
            f"front:{front_min:.1f} | "
            f"escape:{escape_count}"
        )

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(bytes([0xA5, 0x25]))
