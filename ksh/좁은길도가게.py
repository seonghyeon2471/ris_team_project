# =========================================
# ★ 벽 인식 안정화 버전 (연속 벽 대응)
# =========================================
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
# IMU
# =========================================
USE_IMU = True
try:
    import smbus2
    imu_bus = smbus2.SMBus(1)
    IMU_ADDR = 0x68
    imu_bus.write_byte_data(IMU_ADDR, 0x6B, 0)
except Exception:
    USE_IMU = False
    print("[WARN] IMU OFF")

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
# ROBOT PARAM (너가 수정한 값 반영)
# =========================================
ROBOT_RADIUS = 14
MAX_SPEED = 0.19
MIN_SPEED = 0.07
MAX_W = 0.78
TURN_GAIN = 0.72           # 살짝 올림
SCAN_LIMIT = 150
FRONT_RANGE = 90

# FILTER
EMA_ALPHA = 0.32
MEDIAN_K = 2

# SMOOTHING
SMOOTHING_NORMAL = 0.62
SMOOTHING_DANGER = 0.35
DANGER_DIST = 8

# GAP & INFLATION
SAFE_DIST_NORMAL = 12
SAFE_DIST_NARROW = 10
INFLATION_MAX_DIST_NORMAL = 25
INFLATION_MAX_DIST_NARROW = 17.5

FRONT_CLEAR_DIST = 23
FRONT_CLEAR_RANGE = 18

# STATE
STATE_NORMAL = 0
STATE_ROTATE = 1
STATE_RAMP = 2
state = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir = 1
EMERGENCY_DIST = 6.0
ROTATE_DURATION = 0.55
ROTATE_W = 1.0

# RAMP
RAMP_PITCH_THRESH = 8.0
RAMP_EXIT_PITCH = 3.0
RAMP_LIDAR_TIMEOUT = 3.0
RAMP_SPEED = 0.14
RAMP_INFLATION_MAX = 10
RAMP_SAFE_DIST = 8

# DATA
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0
prev_front_avg = float(SCAN_LIMIT)
ramp_start_time = 0.0
v_cmd = 0.0
w_cmd = 0.0

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
        pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
        return abs(pitch)
    except Exception:
        return 0.0

# =========================================
# RAMP
# =========================================
def detect_ramp(front_avg):
    global prev_front_avg
    pitch = read_imu_pitch()
    drop = front_avg - prev_front_avg
    prev_front_avg = front_avg
    if USE_IMU:
        return pitch >= RAMP_PITCH_THRESH
    return drop >= 30.0

def ramp_exited():
    if USE_IMU:
        return read_imu_pitch() < RAMP_EXIT_PITCH
    return (time.time() - ramp_start_time) > RAMP_LIDAR_TIMEOUT

# =========================================
# FILTER
# =========================================
def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def apply_median_filter():
    k = MEDIAN_K
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        indices = [(i + d) % 360 for d in range(-k, k + 1)]
        filtered[i] = np.median(scan_data[indices])
    scan_data[:] = filtered

# =========================================
# DYNAMIC RADIUS & INFLATION
# =========================================
def get_dynamic_radius(v, w):
    if abs(w) < 0.05:
        return ROBOT_RADIUS + 1.0
    R = abs(v / w)
    rear_orbit = math.sqrt(R**2 + 13.5**2)
    inner_cut = rear_orbit - R
    dyn = ROBOT_RADIUS + inner_cut
    return min(dyn, ROBOT_RADIUS + 4.0)

def inflate_obstacles(dists, inflation_max, dynamic_radius):
    proc = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= inflation_max:
            continue
        ratio = min(dynamic_radius / d, 0.92)
        alpha = math.degrees(math.asin(ratio))
        start = max(0, int(i - alpha))
        end = min(len(dists)-1, int(i + alpha))
        proc[start:end+1] = 0.0
    return proc

# =========================================
# GAP
# =========================================
def find_gaps(proc_dists, safe_dist):
    gaps = []
    gap_start = None
    for i, d in enumerate(proc_dists):
        if d > safe_dist:
            if gap_start is None:
                gap_start = i
        else:
            if gap_start is not None:
                if (i - gap_start) >= 20:
                    gaps.append((gap_start, i-1))
                gap_start = None
    if gap_start is not None and (len(proc_dists) - gap_start) >= 20:
        gaps.append((gap_start, len(proc_dists)-1))
    return gaps

def score_gap(gap, proc_dists, angles):
    start, end = gap
    width = end - start
    center_i = (start + end) // 2
    center_angle = angles[center_i]
    avg_dist = np.mean(proc_dists[start:end+1])
    
    score = (
        width * 2.0 +
        avg_dist * 1.45 -
        abs(center_angle) * 0.13 +
        (65 - abs(center_angle)) * 0.085
    )
    return score

def select_best_gap(gaps, proc_dists, angles):
    if not gaps:
        return None
    return max(gaps, key=lambda g: score_gap(g, proc_dists, angles))

# =========================================
# PLANNING - 벽 인식 강화
# =========================================
def find_best_direction(smoothing, on_ramp=False):
    global prev_angle
    global v_cmd, w_cmd

    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists = np.array([scan_data[a % 360] for a in angles])

    # 좁은 길 판단
    front_min = float(np.min(scan_data[np.arange(-45,46)%360]))
    side_left = np.mean(scan_data[45:115])
    side_right = np.mean(scan_data[245:315])
    side_clear = min(side_left, side_right)
    is_narrow = (front_min < 60 and side_clear < 25)

    infl_max = RAMP_INFLATION_MAX if on_ramp else (INFLATION_MAX_DIST_NARROW if is_narrow else INFLATION_MAX_DIST_NORMAL)
    safe_dist = RAMP_SAFE_DIST if on_ramp else (SAFE_DIST_NARROW if is_narrow else SAFE_DIST_NORMAL)

    dynamic_radius = get_dynamic_radius(v_cmd, w_cmd)
    proc = inflate_obstacles(dists, infl_max, dynamic_radius)
    gaps = find_gaps(proc, safe_dist)

    if not gaps:
        return None

    start, end = select_best_gap(gaps, proc, angles)
    center_i = (start + end) // 2
    target = float(angles[center_i])

    # ==================== 개선된 Wall Centering ====================
    left_dist  = np.median(scan_data[45:120])    # 왼쪽 벽
    right_dist = np.median(scan_data[240:315])   # 오른쪽 벽
    
    wall_bias = (right_dist - left_dist) * (0.20 if not is_narrow else 0.07)
    target += wall_bias

    # 앞이 트인 경우
    front_clear = float(np.min(scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE+1) % 360]))
    if front_clear > FRONT_CLEAR_DIST:
        target *= 0.35

    # Rate Limiter
    max_delta = 7.0 if is_narrow else 11.0
    target = max(min(target, prev_angle + max_delta), prev_angle - max_delta)

    # Smoothing
    target = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target

    return target, dynamic_radius, front_clear, is_narrow

# =========================================
# CONTROL
# =========================================
def compute_cmd(target_angle, on_ramp=False, is_narrow=False):
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))

    if abs(target_angle) > 22:
        return 0.0, w

    front_min = float(np.min(scan_data[np.arange(-12,13)%360]))

    if on_ramp:
        return RAMP_SPEED, w

    obstacle_scale = min(front_min / 45.0, 1.0)
    turn_scale = max(0.48, 1.0 - abs(w) / MAX_W)
    
    speed = MAX_SPEED * obstacle_scale * turn_scale
    speed = max(speed, MIN_SPEED)
    if is_narrow:
        speed = min(speed, 0.135)
    
    return speed, w

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

def choose_avoid_direction():
    left_avg = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:359]))
    return 1 if left_avg >= right_avg else -1

# =========================================
# MAIN LOOP
# =========================================
print("NAVIGATION START - Wall Detection Stabilized")
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
        front_min = float(np.min(scan_data[np.arange(-12,13)%360]))

        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
            else:
                state = STATE_NORMAL
                prev_angle = 0.0
            continue

        if state == STATE_RAMP:
            if ramp_exited():
                state = STATE_NORMAL
                print("[RAMP EXIT]")
            else:
                result = find_best_direction(SMOOTHING_NORMAL, True)
                if result:
                    ta, _, _, _ = result
                    v, w = compute_cmd(ta, True)
                    send_cmd(v, w)
            continue

        if front_min < EMERGENCY_DIST:
            rotate_dir = choose_avoid_direction()
            state = STATE_ROTATE
            maneuver_end_time = now + ROTATE_DURATION
            continue

        if detect_ramp(np.mean(scan_data[np.arange(-10,11)%360])):
            state = STATE_RAMP
            ramp_start_time = now
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result = find_best_direction(smoothing)

        if result is None:
            rotate_dir = choose_avoid_direction()
            state = STATE_ROTATE
            maneuver_end_time = now + ROTATE_DURATION
            continue

        target_angle, dynR, fclear, is_narrow = result
        v_cmd, w_cmd = compute_cmd(target_angle, False, is_narrow)

        send_cmd(v_cmd, w_cmd)

        print(f"T:{target_angle:6.1f} | v:{v_cmd:.2f} | w:{w_cmd:.2f} | "
              f"front:{front_min:.1f} | narrow:{is_narrow}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
