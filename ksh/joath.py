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
lidar_ser.write(bytes([0xA5, 0x40]))  # Reset
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))  # Scan Start
lidar_ser.read(7)
print("LIDAR START")

# =========================================
# ROBOT PARAMETERS
# =========================================
ROBOT_RADIUS = 17.0
WHEEL_BASE = 17.0

MAX_SPEED = 0.15
MIN_SPEED = 0.11
MAX_W = 1.6
TURN_GAIN = 2.0
SCAN_LIMIT = 150

# =========================================
# TUNED FOR THIS MAP
# =========================================
EMA_ALPHA = 0.35
MEDIAN_K = 2

SAFE_DIST = 18
INFLATION_MAX_DIST = 28
FRONT_CLEAR_DIST = 30      # 직진 판단 거리 ↑
FRONT_CLEAR_RANGE = 20

DANGER_DIST = 20
EMERGENCY_DIST = 7

# Goal bias (FINISH 방향 강하게 유도)
GOAL_BIAS = 0.45           # 0~1 (클수록 위쪽 강하게)
GOAL_ANGLE = 0.0           # 중앙선(0도) 기준

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL = 0
STATE_REVERSE = 1
STATE_ROTATE = 2
state = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir = 1

REVERSE_DURATION = 0.20
ROTATE_DURATION = 1.1
REVERSE_SPEED = -0.10
ROTATE_W = 1.0

# Loop trap
loop_counter = 0.0
LOOP_THRESHOLD = 18.0
VIRTUAL_WALL_DIST = 16.0

# Data
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0

# =========================================
# UTIL
# =========================================
def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (1 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

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
        start = max(0, int(i - alpha))
        end = min(len(dists) - 1, int(i + alpha))
        proc[start:end+1] = 0.0
    return proc

# =========================================
# GAP FINDING + GOAL BIAS
# =========================================
def find_best_direction():
    global prev_angle, loop_counter
    
    angles = np.arange(-70, 71)  # 좀 더 넓게 보기
    
    local_scan = scan_data.copy()
    
    # Loop escape
    if abs(loop_counter) > LOOP_THRESHOLD:
        if loop_counter > 0:   # 좌회전 루프 → 오른쪽 막기
            for a in range(10, 71):
                local_scan[a % 360] = min(local_scan[a % 360], VIRTUAL_WALL_DIST)
        else:                  # 우회전 루프 → 왼쪽 막기
            for a in range(-70, -9):
                local_scan[a % 360] = min(local_scan[a % 360], VIRTUAL_WALL_DIST)
    
    dists = np.array([local_scan[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    
    # Gap 찾기
    gaps = []
    start = None
    for i, d in enumerate(proc_dists):
        if d > SAFE_DIST:
            if start is None:
                start = i
        else:
            if start is not None:
                gaps.append((start, i-1))
                start = None
    if start is not None:
        gaps.append((start, len(proc_dists)-1))
    
    if not gaps:
        return None
    
    # Gap 점수 + Goal bias
    best_score = -1e9
    best_gap = None
    for s, e in gaps:
        center_i = (s + e) / 2
        center_angle = angles[int(center_i)]
        width = e - s + 1
        avg_dist = np.mean(proc_dists[s:e+1])
        
        # Goal bias (0도 방향 선호)
        goal_score = -abs(center_angle) * GOAL_BIAS
        score = width * 0.6 + avg_dist * 1.0 + goal_score
        
        if score > best_score:
            best_score = score
            best_gap = (s, e)
            best_angle = center_angle
    
    if best_gap is None:
        return None
    
    # Front clear check
    front_clear = float(np.min(local_scan[np.arange(-25, 26) % 360]))
    
    if front_clear > FRONT_CLEAR_DIST:
        target = best_angle * 0.25          # 강한 직진
        bias = "STRAIGHT"
    elif front_clear > 22:
        target = best_angle * 0.75
        bias = "GAP"
    else:
        target = best_angle * 1.1
        bias = "CRITICAL"
    
    # Smoothing
    smoothing = 0.65 if front_clear > 25 else 0.25
    target = prev_angle * smoothing + target * (1 - smoothing)
    prev_angle = target
    
    return target, bias, front_clear

# =========================================
# CONTROL
# =========================================
def compute_cmd(target_angle):
    global loop_counter
    
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -MAX_W, MAX_W))
    
    loop_counter = loop_counter * 0.97 + target_angle * 0.03
    
    # Side check
    side_min = float(np.min(scan_data[np.arange(-70, 71) % 360]))
    
    if abs(target_angle) > 12:
        v = 0.06 if side_min > 22 else 0.0
    else:
        obstacle_scale = min(side_min / 28.0, 1.0)
        v = max(MAX_SPEED * obstacle_scale, MIN_SPEED)
    
    return v, w

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# MAIN
# =========================================
print("=== RoCoGMan - TRACK OPTIMIZED NAVIGATION START ===")

try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5: 
            continue
            
        # LIDAR packet validation (기존 그대로)
        s_flag = raw[0] & 0x01
        if (raw[0] & 0x02) >> 1 != (1 - s_flag): continue
        if (raw[1] & 0x01) != 1: continue
        if (raw[0] >> 2) < 3: continue
            
        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        
        if 3 < dist_cm < SCAN_LIMIT:
            apply_ema(angle, dist_cm)
        
        if s_flag != 1: 
            continue
            
        apply_median_filter()
        now = time.time()
        
        # State Machine
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
            else:
                state = STATE_ROTATE
                maneuver_end_time = now + ROTATE_DURATION
            continue
            
        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
            else:
                rotate_dir *= -1
                state = STATE_NORMAL
                prev_angle = 0.0
                loop_counter = 0.0
                for a in range(-50, 51):
                    scan_data[a % 360] = float(SCAN_LIMIT)
            continue
        
        # Normal navigation
        front_min = float(np.min(scan_data[np.arange(-12, 13) % 360]))
        
        if front_min < EMERGENCY_DIST:
            # Emergency back
            rotate_dir = 1 if np.mean(scan_data[1:90]) > np.mean(scan_data[271:360]) else -1
            state = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            continue
        
        result = find_best_direction()
        if result is None:
            send_cmd(0.05, 0.0)
            continue
            
        target_angle, bias, front_clear = result
        v, w = compute_cmd(target_angle)
        send_cmd(v, w)
        
        print(f"TRG:{target_angle:6.1f}° | v:{v:.2f} | w:{w:6.2f} | f:{front_clear:5.1f} | LOOP:{loop_counter:+6.1f} | {bias}")

except KeyboardInterrupt:
    print("\n=== STOP ===")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
