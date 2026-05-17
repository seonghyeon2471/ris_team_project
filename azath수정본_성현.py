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
ROBOT_RADIUS = 8.5
WHEEL_BASE   = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED = 0.18
MIN_SPEED = 0.10

MAX_W     = 1.5
TURN_GAIN = 2.2

SCAN_LIMIT  = 150
FRONT_RANGE = 55

# =========================================
# FILTER
# =========================================
EMA_ALPHA        = 0.3
MEDIAN_K         = 2

SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.10

DANGER_DIST = 10

SAFE_DIST          = 8.5
INFLATION_MAX_DIST = 60.0

FRONT_CLEAR_DIST   = 20.0
FRONT_CLEAR_RANGE  = 12

# =========================================
# GLOBAL GOAL
# =========================================
pose_theta = 0.0
GLOBAL_GOAL_THETA = 0.0
GOAL_WEIGHT = 2.5

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state = STATE_NORMAL

maneuver_end_time = 0.0
rotate_dir = 1

# ↓ 기존 6 → 4.5로 완화
EMERGENCY_DIST = 4.5

REVERSE_DURATION = 0.12
ROTATE_DURATION  = 0.75

REVERSE_SPEED = -0.08
ROTATE_W      = 1.0

# =========================================
# RECOVERY SYSTEM
# =========================================
recovery_until = 0.0

# =========================================
# STATE DATA
# =========================================
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)

prev_angle = 0.0

last_odom_time = time.time()

is_recovering = False

# =========================================
# UTIL
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

        indices = [
            (i + d) % 360
            for d in range(-k, k + 1)
        ]

        values = np.sort(scan_data[indices])

        filtered[i] = values[window // 2]

    scan_data[:] = filtered

# =========================================
# HEADING UPDATE
# =========================================
def update_heading(w_radps):

    global pose_theta
    global last_odom_time

    now = time.time()

    dt = now - last_odom_time

    last_odom_time = now

    if dt <= 0 or dt > 0.5:
        return

    pose_theta += w_radps * dt

    pose_theta = (
        (pose_theta + math.pi)
        % (2 * math.pi)
        - math.pi
    )

# =========================================
# OBSTACLE INFLATION
# =========================================
def inflate_obstacles(dists):

    proc = dists.copy()

    for i in range(len(dists)):

        d = dists[i]

        if d < 5 or d >= INFLATION_MAX_DIST:
            continue

        ratio = min(ROBOT_RADIUS / d, 0.9)

        alpha = math.degrees(math.asin(ratio))

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

# =========================================
# GAP SCORE
# =========================================
def score_gap(gap, proc_dists, angles, is_critical=False):

    start, end = gap

    width = end - start

    center_i = int((start + end) / 2)

    center_angle = angles[center_i]

    avg_dist = np.mean(proc_dists[start:end + 1])

    base_score = (
        width * 0.5
        + avg_dist * 1.2
        - abs(center_angle) * 0.35
    )

    if is_critical:
        return base_score

    relative_goal_angle = math.degrees(
        GLOBAL_GOAL_THETA - pose_theta
    )

    relative_goal_angle = (
        (relative_goal_angle + 180)
        % 360
        - 180
    )

    angle_diff = abs(center_angle - relative_goal_angle)

    angle_diff = (
        (angle_diff + 180)
        % 360
        - 180
    )

    angle_diff = abs(angle_diff)

    goal_bonus = (
        (180.0 - angle_diff)
        / 180.0
        * GOAL_WEIGHT
    )

    return base_score + goal_bonus

# =========================================
# GAP SELECT
# =========================================
def select_best_gap(gaps, proc_dists, angles, is_critical=False):

    best_gap = None
    best_score = -1e9

    for gap in gaps:

        s = score_gap(
            gap,
            proc_dists,
            angles,
            is_critical
        )

        if s > best_score:
            best_score = s
            best_gap = gap

    return best_gap

# =========================================
# PLANNING
# =========================================
def find_best_direction(smoothing):

    global prev_angle
    global is_recovering

    angles = np.arange(
        -FRONT_RANGE,
        FRONT_RANGE + 1
    )

    dists = np.array(
        [scan_data[a % 360] for a in angles],
        dtype=np.float32
    )

    proc_dists = inflate_obstacles(dists)

    gaps = find_gaps(proc_dists)

    if not gaps:
        return None

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

    # =====================================
    # STRAIGHT DRIVE
    # =====================================
    if front_clear > FRONT_CLEAR_DIST:

        is_critical = False

        best_gap = select_best_gap(
            gaps,
            proc_dists,
            angles,
            is_critical
        )

        target = (
            float(
                angles[
                    int((best_gap[0] + best_gap[1]) / 2)
                ]
            )
            * 0.2
        )

        bias_label = "STRAIGHT"

    # =====================================
    # NORMAL GAP
    # =====================================
    elif front_clear > EMERGENCY_DIST * 2:

        is_critical = False

        best_gap = select_best_gap(
            gaps,
            proc_dists,
            angles,
            is_critical
        )

        target = float(
            angles[
                int((best_gap[0] + best_gap[1]) / 2)
            ]
        )

        smoothing = SMOOTHING_DANGER

        bias_label = "GAP"

    # =====================================
    # CRITICAL
    # =====================================
    else:

        is_critical = True

        best_gap = select_best_gap(
            gaps,
            proc_dists,
            angles,
            is_critical
        )

        target = float(
            angles[
                int((best_gap[0] + best_gap[1]) / 2)
            ]
        )

        smoothing = 0.0

        bias_label = "CRITICAL"

    # =====================================
    # RECOVERY
    # =====================================
    if is_recovering:

        smoothing = 0.85

        is_recovering = False

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

    w = float(
        np.clip(
            w,
            -MAX_W,
            MAX_W
        )
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

    # 급회전 시 속도 유지
    if abs(target_angle) > ALIGN_THRESHOLD:
        return MIN_SPEED, w

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

    left_avg = float(
        np.mean(scan_data[1:90])
    )

    right_avg = float(
        np.mean(scan_data[271:360])
    )

    return 1 if left_avg >= right_avg else -1

# =========================================
# MAIN LOOP
# =========================================
print("NAVIGATION START")

try:

    last_odom_time = time.time()

    v_active = 0.0
    w_active = 0.0

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

        update_heading(w_active)

        # =================================
        # STATE_REVERSE
        # =================================
        if state == STATE_REVERSE:

            if now < maneuver_end_time:

                v_active = REVERSE_SPEED
                w_active = 0.0

                send_cmd(v_active, w_active)

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

                v_active = 0.0
                w_active = ROTATE_W * rotate_dir

                send_cmd(v_active, w_active)

            else:

                rotate_dir *= -1

                state = STATE_NORMAL

                prev_angle = 0.0

                is_recovering = True

                # ★ recovery mode
                recovery_until = time.time() + 0.6

                # 회전 직후 가짜 emergency 방지
                for a in range(-35, 36):

                    scan_data[a % 360] = max(
                        scan_data[a % 360],
                        SAFE_DIST + 3
                    )

            continue

        # =================================
        # NORMAL STATE
        # =================================
        front_min = float(
            np.min(
                scan_data[
                    np.arange(-10, 11) % 360
                ]
            )
        )

        # =================================
        # EMERGENCY CHECK
        # =================================
        if time.time() > recovery_until:

            if front_min < EMERGENCY_DIST:

                rotate_dir = choose_avoid_direction()

                state = STATE_REVERSE

                maneuver_end_time = (
                    now + REVERSE_DURATION
                )

                v_active = REVERSE_SPEED
                w_active = 0.0

                send_cmd(v_active, w_active)

                print("EMERGENCY")

                continue

        # =================================
        # PLANNING
        # =================================
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

            continue

        target_angle, bias_label, front_clear = result

        v_active, w_active = compute_cmd(target_angle)

        send_cmd(v_active, w_active)

        print(
            f"Heading:{math.degrees(pose_theta):6.1f}° | "
            f"TRG:{target_angle:5.1f}° | "
            f"v:{v_active:.2f} | "
            f"w:{w_active:.2f} | "
            f"front:{front_min:5.1f} | "
            f"{bias_label}"
        )

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()

    lidar_ser.write(bytes([0xA5, 0x25]))
