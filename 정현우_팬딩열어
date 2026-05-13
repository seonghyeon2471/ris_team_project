import serial
import math
import time
import numpy as np

# =========================================
# 설정 (HARDWARE & PARAMETERS)
# =========================================
# 아두이노 연결 (UART)
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

# 라이다 연결 (RPLIDAR C1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# 로봇 물리 파라미터
ROBOT_RADIUS = 0.125  # 로봇 폭(195mm) 기반 안전 반경 (m)
WHEEL_BASE = 0.17     # 아두이노 코드와 일치 (m)

# 주행 파라미터
MAX_SPEED = 0.22      # 최대 선속도 (m/s)
MAX_W = 1.5           # 최대 각속도 (rad/s)
SCAN_LIMIT = 3.5      # 유효 인식 거리 (m)
FRONT_RANGE = 90      # 전방 탐색 각도 (좌우 90도씩 총 180도)

# =========================================
# 핵심 알고리즘: Follow the Gap
# =========================================

def get_gap_navigation(scan_data):
    """
    라이다 데이터를 받아 갭 팔로잉 알고리즘으로 v, w를 반환
    """
    # 1. 전방 데이터만 슬라이싱 (-90도 ~ 90도)
    # scan_data는 360개 리스트라고 가정
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists = np.array([scan_data[a % 360] for a in angles])

    # 2. 장애물 팽창 (Obstacle Inflation)
    # 로봇이 지나갈 수 없는 공간을 0으로 메움
    proc_dists = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if 0.05 < d < 0.6:  # 0.6m 이내 장애물에 대해서만 팽창 적용
            # 팽창시킬 각도 계산: alpha = arcsin(R/D)
            alpha = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
            
            # 주변 인덱스 계산
            start_idx = max(0, int(i - alpha))
            end_idx = min(len(dists) - 1, int(i + alpha))
            proc_dists[start_idx:end_idx+1] = 0

    # 3. 갭(Gap) 찾기 (0이 아닌 연속된 구간)
    # 가장 먼 거리가 포함된 유효한 갭을 선택
    if np.max(proc_dists) == 0:
        return 0.0, 0.5  # 사방이 막혔으면 제자리 회전

    # 가장 깊은(먼) 지점의 인덱스 찾기
    best_idx = np.argmax(proc_dists)
    target_angle = angles[best_idx] # -90 ~ 90 사이의 값

    # 4. 제어 명령(v, w) 계산
    # 목표 각도가 정면(0)에서 멀수록 속도를 줄임
    v = MAX_SPEED * (1.0 - abs(target_angle) / 100.0)
    v = max(v, 0.05) # 최소 속도 유지

    # 각속도 P 제어 (목표 각도 rad 변환 후 게인 곱함)
    w = math.radians(target_angle) * 1.8
    w = np.clip(w, -MAX_W, MAX_W)

    return v, w

# =========================================
# MAIN LOOP
# =========================================

# 라이다 초기화 (기존 코드의 Start 시퀀스 필요)
lidar_ser.write(bytes([0xA5, 0x40])) # Reset
time.sleep(1.0)
lidar_ser.write(bytes([0xA5, 0x20])) # Scan Start

scan_data = [SCAN_LIMIT] * 360

try:
    print("Navigation Start...")
    while True:
        # 라이다 데이터 읽기 (5바이트 패킷 파싱)
        raw = lidar_ser.read(5)
        if len(raw) < 5: continue
        
        # 데이터 파싱 (상태 비트, 체크 비트 확인 생략 - 핵심 로직 집중)
        angle_raw = (raw[1] >> 1) | (raw[2] << 7)
        angle = int(angle_raw / 64.0) % 360
        dist_raw = raw[3] | (raw[4] << 8)
        dist = (dist_raw / 4.0) / 1000.0 # mm -> m

        if 0.02 < dist < SCAN_LIMIT:
            scan_data[angle] = dist
        
        # 주기적으로(예: 360개 데이터가 어느정도 모였을 때) 계산 실행
        # 여기서는 매 패킷마다 계산하면 너무 무거우므로 카운터 사용 권장
        if angle == 0: # 한 바퀴 돌 때마다 한 번 계산
            v, w = get_gap_navigation(scan_data)
            
            # 아두이노로 전송 (v,w\n 포맷)
            msg = f"{v:.3f},{w:.3f}\n"
            arduino_ser.write(msg.encode())
            
            print(f"Target: {v:.2f}m/s, {w:.2f}rad/s | FrontDist: {scan_data[0]:.2f}m")

except KeyboardInterrupt:
    arduino_ser.write("0.0,0.0\n".encode())
    print("Stop.")
finally:
    lidar_ser.close()
    arduino_ser.close()
