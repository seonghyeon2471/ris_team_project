```python
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

ROBOT_RADIUS = 11.0
WHEEL_BASE   = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================

MAX_SPEED = 0.17
MIN_SPEED = 0.09

MAX_W     = 0.85
TURN_GAIN = 1.05

SCAN_LIMIT  = 150
FRONT_RANGE = 45

# =========================================
# FILTER
# =========================================

EMA_ALPHA = 0.30
MEDIAN_K  = 2

# =========================================
# SMOOTHING
# =========================================

SMOOTHING_NORMAL = 0.72
SMOOTHING_DANGER = 0.45

DANGER_DIST = 18

# =========================================
# GAP PARAMETER
# =========================================

SAFE_DIST          = 18
INFLATION_MAX_DIST = 20

FRONT_CLEAR_DIST  = 24
FRONT_CLEAR_RANGE = 12

MIN_GAP_WIDTH = 8

# =========================================
# EMERGENCY
# =========================================

EMERGENCY_DIST   = 6

REVERSE_DURATION = 0.30
ROTATE_DURATION  = 0.55

REVERSE_SPEED = -0.10
ROTATE_W      = 0.90

# =========================================
# STATE MACHINE
# =========================================

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

# =========================================
# STATE
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

    filtered = np.empty(360, dtype=np.float32)

    for i in range(360):

        idx = [(i + d) % 360 for d in range(-k, k + 1)]

        values = np.sort(scan_data[idx])

        filtered[i] = values[len(values)//2]

    scan_data[:] = filtered

# =========================================
# OBSTACLE INFLATION
# =========================================

def inflate_obstacles(dists):

    proc = dists.copy()

    for i in range(len(dists)):

        d = dists[i]

        if d < 6 or d >= INFLATION_MAX_DIST:
            continue

        alpha = math.degrees(
            math.atan2(ROBOT_RADIUS * 0.7, d)
        )

        alpha *= 0.7

        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists)-1, int(i + alpha))

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

                if (i - gap_start) >= MIN_GAP_WIDTH:
                    gaps.append((gap_start, i - 1))

                gap_start = None

    if gap_start is not None:

        if (len(proc_dists) - gap_start) >= MIN_GAP_WIDTH:
            gaps.append((gap_start, len(proc_dists)-1))

    return gaps

# =========================================
# PLANNER
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

    if len(gaps) == 0:
        return None

    candidate_angles = np.arange(-40, 41, 4)

    best_score = -1e9
    best_angle = 0.0

    front_clear = float(np.min(
        scan_data[
            np.arange(-FRONT_CLEAR_RANGE,
                      FRONT_CLEAR_RANGE + 1) % 360
        ]
    ))

    for angle in candidate_angles:

        idx = np.argmin(np.abs(angles - angle))

        dist_score = proc_dists[idx]

        if dist_score <= SAFE_DIST:
            continue

        straight_score = -abs(angle) * 0.35

        smooth_score = -abs(angle - prev_angle) * 0.85

        center_score = -abs(angle) * 0.15

        if front_clear > FRONT_CLEAR_DIST:

            front_bonus = (
                12.0 *
                (1.0 - abs(angle) / FRONT_RANGE)
            )

        else:
            front_bonus = 0.0

        score = (
            dist_score * 1.4
            + straight_score
            + smooth_score
            + center_score
            + front_bonus
        )

        if score > best_score:

            best_score = score
            best_angle = angle

    target = (
        prev_angle * smoothing
        + best_angle * (1.0 - smoothing)
    )

    prev_angle = target

    return target, front_clear

# =========================================
# CONTROL
# =========================================

def compute_cmd(target_angle):

    curvature = abs(target_angle) / 45.0

    w = math.radians(target_angle) * TURN_GAIN

    w = float(np.clip(w, -MAX_W, MAX_W))

    speed = MAX_SPEED * (
        1.0 - curvature * 0.40
    )

    front_min = float(
        np.min(scan_data[np.arange(-8, 9) % 360])
    )

    obstacle_scale = np.clip(
        front_min / 35.0,
        0.45,
        1.0
    )

    speed *= obstacle_scale

    speed = max(speed, MIN_SPEED)

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
# AVOID DIRECTION
# =========================================

def choose_avoid_direction():

    left_avg = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))

    if left_avg >= right_avg:
        return 1
    else:
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

        angle_raw = (raw[1] >> 1) | (raw[2] << 7)

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

        # =================================
        # STATE_REVERSE
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
        # STATE_ROTATE
        # =================================

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

        # =================================
        # NORMAL
        # =================================

        front_min = float(
            np.min(scan_data[np.arange(-10, 11) % 360])
        )

        if front_min < EMERGENCY_DIST:

            rotate_dir = choose_avoid_direction()

            state = STATE_REVERSE

            maneuver_end_time = (
                now + REVERSE_DURATION
            )

            send_cmd(REVERSE_SPEED, 0.0)

            continue

        smoothing = (
            SMOOTHING_DANGER
            if front_min < DANGER_DIST
            else SMOOTHING_NORMAL
        )

        result = find_best_direction(smoothing)

        if result is None:

            rotate_dir = choose_avoid_direction()

            state = STATE_REVERSE

            maneuver_end_time = (
                now + REVERSE_DURATION
            )

            send_cmd(REVERSE_SPEED, 0.0)

            continue

        target_angle, front_clear = result

        v, w = compute_cmd(target_angle)

        send_cmd(v, w)

        print(
            f"TARGET:{target_angle:6.1f}° | "
            f"v:{v:.2f} | "
            f"w:{w:.2f} | "
            f"front:{front_min:.1f}cm | "
            f"clear:{front_clear:.1f}cm"
        )

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(bytes([0xA5, 0x25]))

    lidar_ser.close()
    arduino_ser.close()

    print("END")
```
