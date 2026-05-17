import serial
import time
import math

# =========================================
# SERIAL 설정 (당신 이전 코드 그대로)
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)   # Arduino (UART)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)    # LiDAR

# =========================================
# LIDAR START (당신 이전 코드 그대로)
# =========================================
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)          # 헤더 스킵
print("LIDAR START")

# =========================================
# 파라미터
# =========================================
DESIRED_WALL_DISTANCE = 0.25   # 벽 유지 거리 (미터)
FRONT_THRESHOLD       = 0.35   # 앞벽 감지 거리 (미터)
TURN_SPEED            = 0.45   # 원 그리며 돌 때 각속도 (rad/s)
LINEAR_SPEED          = 0.25   # 기본 직진 속도 (m/s)

# =========================================
# LiDAR 스캔 읽기 (rplidar 라이브러리 없이 직접 파싱)
# =========================================
def get_lidar_scan():
    scan_data = []                     # [(angle_deg, distance_m), ...]
    buffer = b''
    
    start_time = time.time()
    while len(scan_data) < 360 and time.time() - start_time < 0.25:   # 최대 0.25초 동안 수집
        if lidar_ser.in_waiting > 0:
            chunk = lidar_ser.read(lidar_ser.in_waiting)
            buffer += chunk
            
            # 5바이트씩 파싱 (RPLIDAR 표준 측정 프레임)
            while len(buffer) >= 5:
                # 체크 비트 확인 (새 스캔 시작 표시)
                if (buffer[0] & 0x01) == 0:
                    buffer = buffer[1:]
                    continue
                
                quality = buffer[0] >> 2
                angle_raw = ((buffer[1] >> 1) | (buffer[2] << 7))
                dist_raw  = (buffer[3] | (buffer[4] << 8))
                
                angle_deg = angle_raw / 64.0          # 0.64도 단위
                dist_m    = dist_raw / 4000.0         # 4mm 단위 → 미터
                
                if quality > 0 and 0 < dist_m < 5.0:  # 유효한 데이터만
                    scan_data.append((angle_deg, dist_m))
                
                buffer = buffer[5:]
    
    return scan_data

# =========================================
# 모터 명령 (Arduino v,w 방식)
# =========================================
def send_motor_command(v, w):
    """v: 선속도(m/s), w: 각속도(rad/s)"""
    cmd = f"{v:.3f},{w:.3f}\n"
    arduino_ser.write(cmd.encode())
    # print(f"→ v={v:.2f}  w={w:.2f}")

# =========================================
# 거리 계산 (front / right / left)
# =========================================
def find_distances(scan_data):
    front = right = left = 999.0
    for angle, dist in scan_data:
        if dist <= 0: continue
        if angle <= 15 or angle >= 345:           # 앞
            front = min(front, dist)
        elif 75 <= angle <= 105:                  # 오른쪽
            right = min(right, dist)
        elif 255 <= angle <= 285:                 # 왼쪽
            left = min(left, dist)
    return front, right, left

# =========================================
# 메인 루프
# =========================================
print("=== 벽 따라 원 그리기 + 앞벽 반전 시작 ===")
follow_side = "right"   # 처음 오른쪽 벽 따라가기

try:
    while True:
        scan = get_lidar_scan()
        if len(scan) < 50:                     # 데이터 부족하면 대기
            time.sleep(0.05)
            continue
            
        front, right, left = find_distances(scan)
        
        # 1. 앞벽 감지 → 방향 반전
        if front < FRONT_THRESHOLD:
            print(f"!!! 앞벽 감지 ({front:.2f}m) → 방향 반전 !!!")
            follow_side = "left" if follow_side == "right" else "right"
            
            # 강하게 반대 방향으로 턴
            if follow_side == "right":
                send_motor_command(0.0, 1.2)     # 왼쪽으로 빠르게 턴
            else:
                send_motor_command(0.0, -1.2)    # 오른쪽으로 빠르게 턴
            time.sleep(0.9)
            continue
        
        # 2. 선택된 쪽 벽 따라가며 원 그리기 (P 제어)
        if follow_side == "right":
            wall_dist = right
            error = DESIRED_WALL_DISTANCE - wall_dist
            angular = -TURN_SPEED + 1.8 * error     # 오른쪽으로 돌기
        else:
            wall_dist = left
            error = DESIRED_WALL_DISTANCE - wall_dist
            angular = TURN_SPEED - 1.8 * error      # 왼쪽으로 돌기
        
        # 앞이 좀 막히면 속도 줄임
        linear = LINEAR_SPEED * (0.7 if front < 0.8 else 1.0)
        
        send_motor_command(linear, angular)
        time.sleep(0.05)   # 20Hz 제어

except KeyboardInterrupt:
    send_motor_command(0, 0)
    print("\n=== 정지 ===")
finally:
    lidar_ser.close()
    arduino_ser.close()
