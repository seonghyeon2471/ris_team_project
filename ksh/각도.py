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
# ★★★ OFFSET 여기서 바꿔가며 테스트하세요 ★★★
# =========================================
ANGLE_OFFSET = 0          # ← 0 → 90 → -90 → 180 → -180 순으로 테스트!!
# =========================================

# =========================================
# ROBOT PARAMETER (회전 강화)
# =========================================
ROBOT_RADIUS = 8.5
WHEEL_BASE = 17.0
MAX_SPEED = 0.18
MIN_SPEED = 0.10
MAX_W = 1.5
TURN_GAIN = 3.2           # ← 회전 더 강하게
SCAN_LIMIT = 150
FRONT_RANGE = 90          # 측면까지 더 넓게 보기

# (나머지 파라미터는 그대로 유지)

EMA_ALPHA = 0.3
MEDIAN_K = 2
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.10
DANGER_DIST = 10
SAFE_DIST = 8.5
INFLATION_MAX_DIST = 60.0
FRONT_CLEAR_DIST = 20.0
FRONT_CLEAR_RANGE = 20
EMERGENCY_DIST = 4.5
REVERSE_DURATION = 0.12
ROTATE_DURATION = 0.75
REVERSE_SPEED = -0.08
ROTATE_W = 1.2

# STATE MACHINE 등 (기존과 동일하게 유지)
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

# ====================== UTIL 함수들 (기존 그대로) ======================
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
    if dt <= 0 or dt > 0.5: return
    pose_theta += w_radps * dt
    pose_theta = (pose_theta + math.pi) % (2 * math.pi) - math.pi

# inflate_obstacles, find_gaps, score_gap, select_best_gap, find_best_direction, compute_cmd 함수들은 그대로 유지
# (코드 길이 때문에 생략했지만, 기존 코드에 있던 함수들은 그대로 복사해서 사용하세요)

# =========================================
# MAIN LOOP
# =========================================
print("NAVIGATION START - OFFSET:", ANGLE_OFFSET)
try:
    last_odom_time = time.time()
    v_active = 0.0
    w_active = 0.0
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5: continue

        s_flag = raw[0] & 0x01
        if (raw[0] & 0x02) >> 1 != (1 - s_flag): continue
        if (raw[1] & 0x01) != 1: continue
        if (raw[0] >> 2) < 3: continue

        # ================== ANGLE_OFFSET 적용 ==================
        angle = int((((raw[1] >> 1) | (raw[2] << 7)) / 64.0)) % 360
        angle = (angle + ANGLE_OFFSET) % 360
        # =====================================================

        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < SCAN_LIMIT:
            apply_ema(angle, dist_cm)

        if s_flag != 1: continue

        apply_median_filter()
        now = time.time()
        update_heading(w_active)

        # STATE_REVERSE, STATE_ROTATE 부분은 기존 그대로...

        # NORMAL STATE
        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))

        # 디버깅: 좌 / 우 / 전방 평균 거리 출력 (중요!)
        left_avg = float(np.mean(scan_data[0:90]))
        right_avg = float(np.mean(scan_data[270:360]))
        print(f"OFFSET:{ANGLE_OFFSET} | Left:{left_avg:5.1f}cm | Right:{right_avg:5.1f}cm | Front:{front_min:5.1f}cm")

        # ... (나머지 EMERGENCY, PLANNING, send_cmd 부분은 기존 코드 그대로)

except KeyboardInterrupt:
    print("STOP")
finally:
    arduino_ser.write(b"0.0,0.0\n")
    lidar_ser.write(bytes([0xA5, 0x25]))
