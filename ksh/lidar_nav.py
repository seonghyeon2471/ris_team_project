import serial
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
lidar_ser.write(bytes([0xA5, 0x40]))   # Reset
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))   # Scan Start
lidar_ser.read(7)                       # 응답 헤더 소비
print("LIDAR START")

# =========================================
# PARAMETER
# =========================================
SCAN_LIMIT      = 200.0   # 유효 인식 거리 (cm) — 2m
MAX_SPEED       = 0.14    # 선속도 (m/s)
MAX_W           = 1.5     # 최대 각속도 (rad/s)

# 장애물 포인트 → w 변환 게인
# 거리 d(cm)일 때 해당 점의 기여: gain / d
# 부호: 오른쪽(1~179°) → +w(좌회전), 왼쪽(181~359°) → -w(우회전)
W_GAIN          = 300.0

# 긴급 정지 & 회전 조건
EMERGENCY_DIST  = 25.0    # 양쪽 모두 이 거리 이하면 정지 후 회전 (cm)
ROTATE_W        = 0.9     # 회전 각속도
ROTATE_DURATION = 1.0     # 회전 지속 시간 (s)

# EMA 필터
EMA_ALPHA       = 0.3

# =========================================
# STATE
# =========================================
STATE_NORMAL = 0
STATE_ROTATE = 1

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1      # +1: 좌회전, -1: 우회전

scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)

# =========================================
# UTIL
# =========================================
def apply_ema(angle, new_dist_cm):
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# STEERING FROM OBSTACLE POINTS
# =========================================
def compute_w_from_points():
    """
    2m 이내 장애물 점들을 순회.
    오른쪽(각도 1~179°)  → +w  (좌로 틀어 회피)
    왼쪽(각도 181~359°)  → -w  (우로 틀어 회피)
    기여도 = W_GAIN / distance  (가까울수록 크게)
    """
    w = 0.0
    for angle in range(1, 360):
        d = float(scan_data[angle])
        if d >= SCAN_LIMIT:          # 장애물 없음
            continue
        contribution = W_GAIN / d
        if 1 <= angle <= 179:        # 오른쪽 장애물 → 왼쪽으로
            w += contribution
        else:                        # 왼쪽 장애물(181~359) → 오른쪽으로
            w -= contribution
    return float(np.clip(w, -MAX_W, MAX_W))

def choose_rotate_dir():
    """공간이 더 넓은 쪽으로 회전"""
    right_avg = float(np.mean(scan_data[1:90]))
    left_avg  = float(np.mean(scan_data[271:360]))
    return 1 if left_avg >= right_avg else -1   # +1: 좌회전, -1: 우회전

def is_emergency():
    """정면 ±90° 내 좌우 양쪽 모두 너무 가까운지 확인"""
    right_min = float(np.min(scan_data[1:90]))
    left_min  = float(np.min(scan_data[271:360]))
    return right_min < EMERGENCY_DIST and left_min < EMERGENCY_DIST

# =========================================
# MAIN LOOP
# =========================================
print("NAVIGATION START")
try:
    while True:
        # --- 라이다 패킷 수신 ---
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

        # 패킷 유효성 검사
        s_flag = raw[0] & 0x01
        if (raw[0] & 0x02) >> 1 != (1 - s_flag):
            continue
        if (raw[1] & 0x01) != 1:
            continue
        if (raw[0] >> 2) < 3:
            continue

        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0

        if 3 < dist_cm < SCAN_LIMIT:
            apply_ema(angle, dist_cm)
        elif dist_cm >= SCAN_LIMIT:
            scan_data[angle] = float(SCAN_LIMIT)   # 범위 밖은 빈 공간 처리

        # 한 스캔 완료 시(s_flag==1)마다 제어 실행
        if s_flag != 1:
            continue

        now = time.time()

        # --- STATE: ROTATE ---
        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
            else:
                state = STATE_NORMAL
                print("ROTATE DONE → NORMAL")
            continue

        # --- STATE: NORMAL ---
        if is_emergency():
            rotate_dir        = choose_rotate_dir()
            state             = STATE_ROTATE
            maneuver_end_time = now + ROTATE_DURATION
            send_cmd(0.0, ROTATE_W * rotate_dir)
            print(f"EMERGENCY → ROTATE {'LEFT' if rotate_dir==1 else 'RIGHT'}")
            continue

        w = compute_w_from_points()
        v = MAX_SPEED
        send_cmd(v, w)
        print(f"v:{v:.2f} | w:{w:.3f}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))   # Scan Stop
