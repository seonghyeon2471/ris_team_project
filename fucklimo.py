import serial
import math
import time
import numpy as np

# =========================================
# SERIAL & IMU
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

USE_IMU = True
try:
    import smbus2
    imu_bus  = smbus2.SMBus(1)
    IMU_ADDR = 0x68
    imu_bus.write_byte_data(IMU_ADDR, 0x6B, 0)
except Exception:
    USE_IMU = False
    print("[WARN] IMU 초기화 실패")

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
# PARAMETERS
# =========================================
ROBOT_RADIUS = 14.0
MAX_SPEED = 0.20
MIN_SPEED = 0.07
MAX_W     = 1.5   # 최대 각속도 살짝 하향
TURN_GAIN = 1.1

SCAN_LIMIT   = 150
FRONT_RANGE  = 60

# EMERGENCY (정면 충돌 방지)
EMERGENCY_DIST  = 12.0  # ★ 8 -> 12cm (인식률 향상)
EMERGENCY_RANGE = 10    # ★ 5 -> 10 (감지 폭 확대)
REVERSE_DURATION = 0.3  # 후진 시간
ROTATE_DURATION  = 0.6  # ★ 1.0 -> 0.6 (회전 각도 축소)
REVERSE_SPEED    = -0.10
ROTATE_W         = 0.7  # ★ 0.9 -> 0.7 (회전 속도 축소)

# RAMP & WALL FOLLOW
RAMP_PITCH_THRESH = 8.0
RAMP_EXIT_PITCH   = 3.0
RAMP_SPEED        = 0.12
WALL_FOLLOW_TARGET = 22.0
WALL_FOLLOW_KP     = 0.35
WALL_RAMP_DIST     = 20.0

# STATE MACHINE
STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2
STATE_RAMP    = 3

RAMP_MODE_NORMAL     = 0
RAMP_MODE_WALL_LEFT  = 1
RAMP_MODE_WALL_RIGHT = 2

# Global State
state = STATE_NORMAL
ramp_mode = RAMP_MODE_NORMAL
maneuver_end_time = 0.0
rotate_dir = 1
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0
prev_front_avg = float(SCAN_LIMIT)
ramp_start_time = 0.0

# =========================================
# FUNCTIONS
# =========================================

def read_imu_pitch():
    if not USE_IMU: return 0.0
    try:
        def read_word(reg):
            h = imu_bus.read_byte_data(IMU_ADDR, reg)
            l = imu_bus.read_byte_data(IMU_ADDR, reg + 1)
            val = (h << 8) | l
            return val - 65536 if val >= 0x8000 else val
        ax, ay, az = read_word(0x3B)/16384.0, read_word(0x3D)/16384.0, read_word(0x3F)/16384.0
        return abs(math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2))))
    except: return 0.0

def detect_side_ramp():
    left_dist = float(np.mean(scan_data[85:96]))
    right_dist = float(np.mean(scan_data[265:276]))
    if left_dist < WALL_RAMP_DIST: return RAMP_MODE_WALL_LEFT
    if right_dist < WALL_RAMP_DIST: return RAMP_MODE_WALL_RIGHT
    return RAMP_MODE_NORMAL

def compute_wall_follow_cmd(mode):
    if mode == RAMP_MODE_WALL_LEFT:
        wall_dist = float(np.mean(scan_data[265:276])) # 오른쪽 벽 기준
        error = WALL_FOLLOW_TARGET - wall_dist
        w = float(np.clip(-error * WALL_FOLLOW_KP, -0.7, 0.7))
    else:
        wall_dist = float(np.mean(scan_data[85:96])) # 왼쪽 벽 기준
        error = WALL_FOLLOW_TARGET - wall_dist
        w = float(np.clip(error * WALL_FOLLOW_KP, -0.7, 0.7))
    return RAMP_SPEED, w

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def find_best_direction(smoothing, on_ramp=False):
    global prev_angle
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    
    # Simple Gap Search (Simplified for reliability)
    safe_threshold = 12.0 if on_ramp else 18.0
    valid_indices = np.where(dists > safe_threshold)[0]
    
    if len(valid_indices) == 0: return None
    
    # 가고자 하는 방향의 중앙값 찾기
    gap_idx = valid_indices[len(valid_indices)//2]
    target = float(angles[gap_idx])
    
    target = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    return target

# =========================================
# MAIN LOOP
# =========================================
try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) < 5: continue
        
        # Lidar Parsing
        s_flag = raw[0] & 0x01
        if (raw[1] & 0x01) != 1: continue
        
        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        
        if 3 < dist_cm < SCAN_LIMIT:
            scan_data[angle] = (0.5 * scan_data[angle]) + (0.5 * dist_cm)

        if s_flag != 1: continue # 한 바퀴 다 돌 때까지 대기
        
        now = time.time()
        front_min = float(np.min(scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE + 1) % 360]))
        pitch = read_imu_pitch()

        # 1. EMERGENCY CHECK (모든 상태에서 공통)
        if state != STATE_REVERSE and state != STATE_ROTATE:
            if front_min < EMERGENCY_DIST:
                print(f"!!! EMERGENCY !!! Dist: {front_min:.1f}cm")
                state = STATE_REVERSE
                maneuver_end_time = now + REVERSE_DURATION
                # 왼쪽/오른쪽 공간 중 넓은 곳으로 회전 방향 결정
                rotate_dir = 1 if np.mean(scan_data[1:45]) > np.mean(scan_data[315:359]) else -1
                continue

        # 2. STATE MACHINE
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
            else:
                state = STATE_ROTATE
                maneuver_end_time = now + ROTATE_DURATION
            continue

        elif state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
            else:
                state = STATE_NORMAL
                prev_angle = 0.0
            continue

        elif state == STATE_RAMP:
            if pitch < RAMP_EXIT_PITCH:
                state = STATE_NORMAL
                ramp_mode = RAMP_MODE_NORMAL
                print("RAMP EXIT")
                continue
            
            # 벽 추종 혹은 경사로 Gap 이동
            rmode = detect_side_ramp()
            if rmode != RAMP_MODE_NORMAL:
                v, w = compute_wall_follow_cmd(rmode)
                send_cmd(v, w)
            else:
                target = find_best_direction(0.7, on_ramp=True)
                if target is not None:
                    w = float(np.clip(math.radians(target) * 1.2, -0.8, 0.8))
                    send_cmd(RAMP_SPEED, w)
                else:
                    send_cmd(RAMP_SPEED, 0.0)

        elif state == STATE_NORMAL:
            if pitch > RAMP_PITCH_THRESH:
                state = STATE_RAMP
                print("RAMP ENTER")
                continue
            
            target = find_best_direction(0.6)
            if target is None:
                state = STATE_REVERSE
                maneuver_end_time = now + REVERSE_DURATION
            else:
                w = float(np.clip(math.radians(target) * TURN_GAIN, -MAX_W, MAX_W))
                # 정면이 어느 정도 비어있으면 속도 유지, 아니면 감속
                v = MAX_SPEED if front_min > 40 else MIN_SPEED
                send_cmd(v, w)
                print(f"NOR: Ang:{target:.1f} V:{v:.2f} W:{w:.2f}")

except KeyboardInterrupt:
    print("STOP")
finally:
    send_cmd(0.0, 0.0)
    lidar_ser.write(bytes([0xA5, 0x25]))
