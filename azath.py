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
# ROBOT PHYSICAL PARAMETER
# =========================================

ROBOT_RADIUS = 17.0
WHEEL_BASE   = 17.0

# =========================================
# DRIVE PARAMETER
# =========================================

MAX_SPEED    = 0.14
MIN_SPEED    = 0.05
MAX_W        = 1.5
TURN_GAIN    = 1.4
SCAN_LIMIT   = 150
FRONT_RANGE  = 60

# =========================================
# FILTER PARAMETER
# =========================================

EMA_ALPHA    = 0.3
MEDIAN_K     = 2

# =========================================
# SMOOTHING PARAMETER
# =========================================

SMOOTHING_NORMAL = 0.70
SMOOTHING_DANGER = 0.25
DANGER_DIST      = 18

# =========================================
# GAP PARAMETER
# =========================================

SAFE_DIST          = 22
INFLATION_MAX_DIST = 40
FRONT_CLEAR_DIST   = 23
FRONT_CLEAR_RANGE  = 15

# =========================================
# STATE MACHINE
# =========================================

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST   = 0
REVERSE_DURATION = 0.35
ROTATE_DURATION  = 0.7
REVERSE_SPEED    = -0.10
ROTATE_W         = 0.6

AVOID_COOLDOWN   = 0.9
avoid_until      = 0.0

# =========================================
# STATE
# =========================================

scan_data  = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0

def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def apply_median_filter():
    k = MEDIAN_K
    window = 2 * k + 1
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        indices = [(i + d) % 360 for d in range(-k, k + 1)]
        values = np.sort(scan_data[indices])
        filtered[i] = values[window // 2]
    scan_data[:] = filtered

def inflate_obstacles(dists):
    proc = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= INFLATION_MAX_DIST:
            continue
        alpha = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists) - 1, int(i + alpha))
        proc[start_idx:end_idx + 1] = 0.0
    return proc

def find_gaps(proc_dists, angles):
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

def score_gap(gap, proc_dists, angles):
    start, end = gap
    width = end - start
    center_i = (start + end) / 2.0
    center_angle = angles[int(center_i)]
    avg_dist = np.mean(proc_dists[start:end + 1])
    return width * 0.5 + avg_dist * 1.2 - abs(center_angle) * 0.3

def select_best_gap(gaps, proc_dists, angles):
    best_gap = None
    best_score = -1e9
    for gap in gaps:
        s = score_gap(gap, proc_dists, angles)
        if s > best_score:
            best_score = s
            best_gap = gap
    return best_gap

def find_best_direction(smoothing):
    global prev_angle
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    gaps = find_gaps(proc_dists, angles)

    if not gaps:
        return None

    best_gap = select_best_gap(gaps, proc_dists, angles)
    start, end = best_gap
    center_i = (start + end) / 2.0
    gap_angle = float(angles[int(center_i)])

    front_clear = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]))

    if front_clear > FRONT_CLEAR_DIST:
        target = gap_angle * 0.2
        bias_label = "STRAIGHT"
    else:
        target = gap_angle * 0.7
        bias_label = "GAP"

    target = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target

    return target, bias_label, front_clear

def compute_cmd(target_angle):
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    steering_ratio = min(abs(target_angle) / 90.0, 1.0)
    speed = MAX_SPEED * (1.0 - steering_ratio * 0.75)

    front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))
    obstacle_scale = min(front_min / 40.0, 1.0)
    speed *= obstacle_scale
    speed = max(speed, MIN_SPEED)
    return speed, w

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

def choose_avoid_direction():
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    if left_avg >= right_avg:
        print(f"  [AVOID DIR] LEFT  (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return 1
    else:
        print(f"  [AVOID DIR] RIGHT (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return -1

print("NAVIGATION START")

try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

        s_flag = raw[0] & 0x01
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

        if now < avoid_until:
            continue

        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
                print(f"  [REVERSE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                state = STATE_ROTATE
                maneuver_end_time = now + ROTATE_DURATION
                avoid_until = maneuver_end_time + AVOID_COOLDOWN
                print(f"  [ROTATE START] dir:{'+' if rotate_dir > 0 else '-'}")
            continue

        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(MIN_SPEED, ROTATE_W * rotate_dir * 0.8)
                print(f"  [ROTATE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                rotate_dir *= -1
                state = STATE_NORMAL
                prev_angle = 0.0
                for a in range(-45, 46):
                    scan_data[a % 360] = float(SCAN_LIMIT)
                print("  [NORMAL] maneuver done / scan reset ±45°")
            continue

        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))

        if front_min < EMERGENCY_DIST and EMERGENCY_DIST > 0:
            rotate_dir = choose_avoid_direction()
            state = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            avoid_until = maneuver_end_time + ROTATE_DURATION + AVOID_COOLDOWN
            print(f"EMERGENCY! front:{front_min:.1f}cm → REVERSE")
            send_cmd(REVERSE_SPEED, 0.0)
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result = find_best_direction(smoothing)

        if result is None:
            rotate_dir = choose_avoid_direction()
            state = STATE_ROTATE
            maneuver_end_time = now + ROTATE_DURATION
            avoid_until = maneuver_end_time + AVOID_COOLDOWN
            print("NO GAP! → ROTATE (no reverse)")
            send_cmd(MIN_SPEED, ROTATE_W * rotate_dir * 0.8)
            continue

        target_angle, bias_label, front_clear = result
        v, w = compute_cmd(target_angle)
        send_cmd(v, w)

        print(
            f"TARGET:{target_angle:6.1f}° | "
            f"v:{v:.2f}m/s | w:{w:.2f}r/s | "
            f"front:{front_min:.1f}cm | "
            f"clear:{front_clear:.1f}cm | "
            f"bias:{bias_label} | "
            f"smooth:{smoothing:.2f}"
        )

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
