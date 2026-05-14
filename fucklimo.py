import serial
import math
import time
import numpy as np

# =========================================
# SERIAL & IMU 초기화
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
    print("[WARN] IMU 비활성화")

# =========================================
# 핵심 파라미터 (최적화 완료)
# =========================================
# 1. 인식 범위 및 주행
FRONT_RANGE  = 90     # ★ 인식 범위를 좌우 90도로 확대 (측면 장애물 완벽 감지)
SCAN_LIMIT   = 150
MAX_SPEED    = 0.20
MIN_SPEED    = 0.07
MAX_W        = 1.5
TURN_GAIN    = 1.1

# 2. 감속 및 긴급 정지 (답답함 해소)
SLOW_DIST       = 20.0   # ★ 25cm까지는 최대 속도로 주행 (감속 시작 거리 단축)
EMERGENCY_DIST  = 12.0   # 정면 충돌 방지 거리
EMERGENCY_RANGE = 10     # 정면 감시 각도 (±10도)

# 3. 회피 동작 (각도 축소)
REVERSE_DURATION = 0.3
ROTATE_DURATION  = 0.6   # 회전 시간 단축 (살짝만 틀기)
ROTATE_W         = 0.7   # 회전 속도 하향

# 4. 경사로 및 벽 추종
RAMP_PITCH_THRESH  = 8.0
RAMP_EXIT_PITCH    = 3.0
RAMP_SPEED         = 0.12
WALL_RAMP_DIST     = 22.0  # 벽으로 인식할 거리
WALL_FOLLOW_TARGET = 22.0  # 유지할 벽 거리
WALL_FOLLOW_KP     = 0.35

# 상태 상수
STATE_NORMAL, STATE_REVERSE, STATE_ROTATE, STATE_RAMP = 0, 1, 2, 3
RAMP_MODE_NORMAL, RAMP_MODE_WALL_LEFT, RAMP_MODE_WALL_RIGHT = 0, 1, 2

# 전역 변수
state = STATE_NORMAL
ramp_mode = RAMP_MODE_NORMAL
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0
maneuver_end_time = 0.0
rotate_dir = 1

# =========================================
# 주요 함수
# =========================================

def read_imu_pitch():
    if not USE_IMU: return 0.0
    try:
        def read_word(reg):
            h = imu_bus.read_byte_data(IMU_ADDR, reg); l = imu_bus.read_byte_data(IMU_ADDR, reg + 1)
            val = (h << 8) | l
            return val - 65536 if val >= 0x8000 else val
        ax, ay, az = read_word(0x3B)/16384.0, read_word(0x3D)/16384.0, read_word(0x3F)/16384.0
        return abs(math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2))))
    except: return 0.0

def detect_side_ramp():
    """ 측면 60~100도 범위를 체크하여 벽 추종 모드 결정 """
    # 왼쪽 체크 (60~100도) / 오른쪽 체크 (260~300도)
    left_side  = float(np.mean(scan_data[60:101]))
    right_side = float(np.mean(scan_data[260:301]))

    if left_side < WALL_RAMP_DIST: return RAMP_MODE_WALL_LEFT
    if right_side < WALL_RAMP_DIST: return RAMP_MODE_WALL_RIGHT
    return RAMP_MODE_NORMAL

def compute_wall_follow_cmd(mode):
    """ 벽과의 거리를 유지하는 각속도 계산 """
    if mode == RAMP_MODE_WALL_LEFT:
        # 왼쪽 벽은 90도 근처 데이터로 거리 유지
        wall_dist = float(np.mean(scan_data[80:101]))
        error = WALL_FOLLOW_TARGET - wall_dist
        w = float(np.clip(error * WALL_FOLLOW_KP, -0.7, 0.7))
    else:
        # 오른쪽 벽은 270도 근처 데이터로 거리 유지
        wall_dist = float(np.mean(scan_data[260:281]))
        error = WALL_FOLLOW_TARGET - wall_dist
        w = float(np.clip(-error * WALL_FOLLOW_KP, -0.7, 0.7))
    return RAMP_SPEED, w

def find_best_direction(smoothing, on_ramp=False):
    """ FRONT_RANGE(90도) 내에서 가장 열린 공간 탐색 """
    global prev_angle
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    
    threshold = 12.0 if on_ramp else 18.0
    valid_indices = np.where(dists > threshold)[0]
    
    if len(valid_indices) == 0: return None
    
    # 가야할 목표 각도 산출
    target = float(angles[valid_indices[len(valid_indices)//2]])
    target = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    return target

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

# =========================================
# 메인 루프
# =========================================
lidar_ser.write(bytes([0xA5, 0x40])) # Reset
time.sleep(1)
lidar_ser.write(bytes([0xA5, 0x20])) # Scan
lidar_ser.read(7)

try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) < 5: continue
        
        # 데이터 파싱
        s_flag = raw[0] & 0x01
        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        
        if 3 < dist_cm < SCAN_LIMIT:
            scan_data[angle] = (0.4 * scan_data[angle]) + (0.6 * dist_cm)

        if s_flag != 1: continue # 한 바퀴 수집 완료 시 제어 시작
        
        now = time.time()
        pitch = read_imu_pitch()
        front_min = float(np.min(scan_data[np.arange(-EMERGENCY_RANGE, EMERGENCY_RANGE+1) % 360]))

        # --- 제어 로직 (State Machine) ---
        
        # 1. 공통 긴급 회피
        if state in [STATE_NORMAL, STATE_RAMP] and front_min < EMERGENCY_DIST:
            state = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            rotate_dir = 1 if np.mean(scan_data[1:60]) > np.mean(scan_data[300:359]) else -1
            continue

        # 2. 상태별 동작
        if state == STATE_REVERSE:
            if now < maneuver_end_time: send_cmd(REVERSE_SPEED, 0.0)
            else:
                state, maneuver_end_time = STATE_ROTATE, now + ROTATE_DURATION
            continue

        elif state == STATE_ROTATE:
            if now < maneuver_end_time: send_cmd(0.0, ROTATE_W * rotate_dir)
            else: state, prev_angle = STATE_NORMAL, 0.0
            continue

        elif state == STATE_RAMP:
            if pitch < RAMP_EXIT_PITCH:
                state, ramp_mode = STATE_NORMAL, RAMP_MODE_NORMAL
                print("RAMP EXIT")
                continue
            
            rmode = detect_side_ramp()
            if rmode != RAMP_MODE_NORMAL:
                v, w = compute_wall_follow_cmd(rmode)
                send_cmd(v, w)
            else:
                target = find_best_direction(0.7, on_ramp=True)
                w = float(np.clip(math.radians(target or 0) * 1.2, -0.8, 0.8))
                send_cmd(RAMP_SPEED, w)

        elif state == STATE_NORMAL:
            if pitch > RAMP_PITCH_THRESH:
                state = STATE_RAMP
                print("RAMP ENTER")
                continue
            
            target = find_best_direction(0.6)
            if target is None:
                state, maneuver_end_time = STATE_REVERSE, now + REVERSE_DURATION
            else:
                v = MAX_SPEED if front_min > SLOW_DIST else MIN_SPEED
                w = float(np.clip(math.radians(target) * TURN_GAIN, -MAX_W, MAX_W))
                send_cmd(v, w)
                if int(now*10) % 5 == 0:
                    print(f"NORMAL: Target:{target:.1f} V:{v:.2f} Front:{front_min:.1f}")

except KeyboardInterrupt:
    print("STOP")
finally:
    send_cmd(0.0, 0.0)
    lidar_ser.write(bytes([0xA5, 0x25]))
