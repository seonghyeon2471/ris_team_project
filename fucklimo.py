import serial
import math
import time

# =========================================
# 1. SERIAL CONFIGURATION
# =========================================
MOTOR_PORT = "/dev/serial0"
MOTOR_BAUD = 115200
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

try:
    motor_ser = serial.Serial(MOTOR_PORT, MOTOR_BAUD, timeout=0.1)
    lidar_ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
except Exception as e:
    print(f"Serial Connection Error: {e}")
    exit()

# =========================================
# 2. PARAMETERS (3cm 마진 & 직진성 강화)
# =========================================
# 차량 물리 크기
CAR_WIDTH = 0.20
SAFETY_RADIUS = 0.13   # 차량 반폭(10cm) + 여유(3cm)

# 주행 성능
BASE_SPEED = 0.25      # 직선 주행 속도 (상향)
MIN_SPEED = 0.08
MAX_W = 1.2            # 최대 회전 속도

# 시간 제어 (상태 유지 시간)
DRIVE_DURATION = 0.7   # 한 번 방향 잡고 직진할 시간 (초)
TURN_DURATION = 0.25   # 회전 보정 시간 (초)

# 거리 임계값
SAFE_DIST = 0.18       # 좁은 통로 인식을 위해 하향
EMERGENCY_DIST = 0.12  # 충돌 방지 최소 거리
MAX_LIDAR_DIST = 4.0

# =========================================
# 3. STATE & UTIL
# =========================================
STATE_SCAN = 0
STATE_TURN = 1
STATE_DRIVE = 2

current_state = STATE_SCAN
state_start_time = 0
target_v, target_w = 0, 0

scan_data = [MAX_LIDAR_DIST] * 360

def normalize_angle(angle):
    return int(angle % 360)

def get_range(angle):
    return scan_data[normalize_angle(angle)]

def send_cmd(v, w):
    motor_ser.write(f"{v:.3f},{w:.3f}\n".encode())

# =========================================
# 4. 핵심 로직 함수
# =========================================

def smooth_scan():
    global scan_data
    filtered = scan_data[:]
    for i in range(360):
        # Median Filter (노이즈 제거)
        values = [scan_data[normalize_angle(i + k)] for k in range(-2, 3)]
        values.sort()
        filtered[i] = values[2]
    scan_data = filtered

def find_closest_obstacle():
    min_dist = 999
    min_angle = 0
    # 탐색 범위를 정면 위주(-80~80)로 제한하여 무한회전 방지
    for angle in range(-80, 81):
        d = get_range(angle)
        if 0.05 < d < min_dist:
            min_dist = d
            min_angle = angle
    return min_angle, min_dist

def create_bubble(masked_scan, obs_angle, obs_dist):
    # 3cm 마진을 포함한 동적 버블 계산
    dist = max(obs_dist, SAFETY_RADIUS + 0.01)
    try:
        # asin(0.13 / 거리)로 필요한 각도 산출
        bubble_radius_deg = math.degrees(math.asin(SAFETY_RADIUS / dist))
    except:
        bubble_radius_deg = 40
    
    bubble_radius_deg = min(bubble_radius_deg, 45) # 너무 커서 길을 막지 않게 제한

    for a in range(int(obs_angle - bubble_radius_deg), int(obs_angle + bubble_radius_deg + 1)):
        masked_scan[normalize_angle(a)] = 0.0

def find_gaps(masked_scan):
    gaps = []
    gap_start = None
    for angle in range(-80, 81):
        if masked_scan[normalize_angle(angle)] > SAFE_DIST:
            if gap_start is None: gap_start = angle
        else:
            if gap_start is not None:
                gaps.append((gap_start, angle - 1))
                gap_start = None
    if gap_start is not None: gaps.append((gap_start, 80))
    return gaps

def score_gap_linear(gap):
    start, end = gap
    center = (start + end) / 2.0
    avg_dist = sum(scan_data[normalize_angle(a)] for a in range(start, end + 1)) / (end - start + 1)
    
    # 직진성 강조: 정면(0도)에서 멀어질수록 감점 대폭 강화
    score = (avg_dist * 2.0) - (abs(center) * 2.5)
    return score

def recovery_behavior():
    print("!!! EMERGENCY !!!")
    send_cmd(-0.12, 0.0) # 후진
    time.sleep(0.5)
    send_cmd(0.0, 1.0)   # 탈출 회전
    time.sleep(0.4)

# =========================================
# 5. MAIN LOOP
# =========================================
lidar_ser.write(bytes([0xA5, 0x40])) # 모터 시작
time.sleep(1)
lidar_ser.write(bytes([0xA5, 0x20])) # 스캔 시작
lidar_ser.read(7)

print("RACE START - LINEAR MODE")

try:
    while True:
        # --- LIDAR DATA READING ---
        raw = lidar_ser.read(5)
        if len(raw) < 5: continue
        
        s_flag = raw[0] & 0x01
        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0)
        dist = ((raw[3] | (raw[4] << 8)) / 4.0) / 1000.0

        if 0 <= angle < 360 and 0.02 < dist < MAX_LIDAR_DIST:
            scan_data[angle] = (0.5 * scan_data[angle]) + (0.5 * dist)

        # --- STATE MACHINE ---
        now = time.time()

        if s_flag == 1: # 한 바퀴 스캔 완료 시점
            smooth_scan()
            obs_angle, obs_dist = find_closest_obstacle()

            # 긴급 상황은 상태와 관계없이 체크
            if obs_dist < EMERGENCY_DIST:
                recovery_behavior()
                current_state = STATE_SCAN
                continue

            if current_state == STATE_SCAN:
                masked_scan = scan_data[:]
                create_bubble(masked_scan, obs_angle, obs_dist)
                gaps = find_gaps(masked_scan)

                if gaps:
                    best_gap = max(gaps, key=score_gap_linear)
                    target_angle = (best_gap[0] + best_gap[1]) / 2.0
                    
                    # 조향 및 속도 설정
                    target_w = math.radians(target_angle) * 1.3
                    target_w = max(min(target_w, MAX_W), -MAX_W)
                    target_v = BASE_SPEED * 0.8 # 회전 시 살짝 감속
                    
                    current_state = STATE_TURN
                    state_start_time = now
                    print(f"SCAN: Path Found at {target_angle:.1f} deg. Turning...")
                else:
                    recovery_behavior()

        # 회전 상태 제어
        if current_state == STATE_TURN:
            send_cmd(target_v, target_w)
            if now - state_start_time > TURN_DURATION:
                current_state = STATE_DRIVE
                state_start_time = now
                print("TURN: Done. Full Speed Ahead!")

        # 직진 상태 제어
        elif current_state == STATE_DRIVE:
            send_cmd(BASE_SPEED, 0.0) # 직진 시 조향 0으로 고정 (직진성 극대화)
            
            # 주행 중 정면 장애물 상시 감시
            front_dist = min(get_range(a) for a in range(-15, 16))
            if front_dist < SAFE_DIST or (now - state_start_time > DRIVE_DURATION):
                current_state = STATE_SCAN
                print("DRIVE: Re-scanning for next path.")

except KeyboardInterrupt:
    print("STOP")
finally:
    send_cmd(0, 0)
    lidar_ser.write(bytes([0xA5, 0x25]))
    lidar_ser.close()
    motor_ser.close()
