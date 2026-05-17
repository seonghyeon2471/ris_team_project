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
# ★★★ 여기서 OFFSET 조정하세요 ★★★
# =========================================
ANGLE_OFFSET = 0          # ← 0, 90, -90, 180, -180 중에서 테스트!
# =========================================

# =========================================
# ROBOT PARAMETER (기존 그대로)
# =========================================
ROBOT_RADIUS = 8.5
WHEEL_BASE = 17.0

MAX_SPEED = 0.18
MIN_SPEED = 0.10
MAX_W = 1.5
TURN_GAIN = 3.0           # ← 2.2 → 3.0으로 증가 (더 잘 돌게)
SCAN_LIMIT = 150
FRONT_RANGE = 90          # ← 55 → 90으로 확대 (측면도 더 잘 보게)

# 나머지 파라미터는 그대로...
EMA_ALPHA = 0.3
MEDIAN_K = 2
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.10
DANGER_DIST = 10
SAFE_DIST = 8.5
INFLATION_MAX_DIST = 60.0
FRONT_CLEAR_DIST = 20.0
FRONT_CLEAR_RANGE = 20     # ← 12 → 20으로 확대
EMERGENCY_DIST = 4.5
REVERSE_DURATION = 0.12
ROTATE_DURATION = 0.75
REVERSE_SPEED = -0.08
ROTATE_W = 1.2

# =========================================
# STATE MACHINE 등 (기존 그대로)
# =========================================
state = 0
STATE_NORMAL = 0
STATE_REVERSE = 1
STATE_ROTATE = 2
maneuver_end_time = 0.0
rotate_dir = 1
recovery_until = 0.0
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0
last_odom_time = time.time()
pose_theta = 0.0
GLOBAL_GOAL_THETA = 0.0
GOAL_WEIGHT = 2.5

# (apply_ema, apply_median_filter, update_heading, inflate_obstacles, find_gaps, score_gap, select_best_gap 함수들은 그대로 유지)

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

def update_heading(w_radps):
    global pose_theta, last_odom_time
    now = time.time()
    dt = now - last_odom_time
    last_odom_time = now
    if dt <= 0 or dt > 0.5:
        return
    pose_theta += w_radps * dt
    pose_theta = (pose_theta + math.pi) % (2 * math.pi) - math.pi

# (inflate_obstacles, find_gaps, score_gap, select_best_gap, find_best_direction, compute_cmd 함수들은 그대로)

# =========================================
# MAIN LOOP (ANGLE_OFFSET 적용 핵심 부분)
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

        # ================== ANGLE_OFFSET 적용 ==================
        angle = int(
            (((raw[1] >> 1) | (raw[2] << 7)) / 64.0)
        ) % 360
        angle = (angle + ANGLE_OFFSET) % 360
        # =====================================================

        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0

        if 3 < dist_cm < SCAN_LIMIT:
            apply_ema(angle, dist_cm)

        if s_flag != 1:
            continue

        apply_median_filter()
        now = time.time()
        update_heading(w_active)

        # STATE_REVERSE, STATE_ROTATE 부분은 그대로...

        # NORMAL STATE
        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))

        if time.time() > recovery_until:
            if front_min < EMERGENCY_DIST:
                rotate_dir = 1 if np.mean(scan_data[1:90]) >= np.mean(scan_data[271:360]) else -1
                state = STATE_REVERSE
                maneuver_end_time = now + REVERSE_DURATION
                v_active = REVERSE_SPEED
                w_active = 0.0
                arduino_ser.write(f"{v_active:.3f},{-w_active:.3f}\n".encode())
                print("EMERGENCY")
                continue

        # PLANNING
        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result = find_best_direction(smoothing)   # (기존 함수 그대로 사용)

        if result is None:
            rotate_dir = 1 if np.mean(scan_data[1:90]) >= np.mean(scan_data[271:360]) else -1
            state = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            continue

        target_angle, bias_label, front_clear = result
        v_active, w_active = compute_cmd(target_angle)

        # SEND
        arduino_ser.write(f"{v_active:.3f},{-w_active:.3f}\n".encode())

        print(
            f"Heading:{math.degrees(pose_theta):6.1f}° | "
            f"TRG:{target_angle:5.1f}° | "
            f"v:{v_active:.2f} | "
            f"w:{w_active:.2f} | "
            f"front:{front_min:5.1f} | "
            f"offset:{ANGLE_OFFSET} | {bias_label}"
        )

except KeyboardInterrupt:
    print("STOP")
finally:
    arduino_ser.write(b"0.0,0.0\n")
    lidar_ser.write(bytes([0xA5, 0x25]))
