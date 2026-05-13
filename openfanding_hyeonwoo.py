import serial
import math
import time
import numpy as np

# =========================================
# 설정 (HARDWARE & PARAMETERS)
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# 로봇 물리 파라미터 (폭 195mm, 길이 200mm 반영)
ROBOT_RADIUS = 0.15   # 외접원 반지름 약 140mm + 공차 포함 (m)
WHEEL_BASE = 0.17     # 아두이노 코드와 일치

# 주행 및 히스테리시스 파라미터
MAX_SPEED = 0.20      
MAX_W = 1.2           
SCAN_LIMIT = 0.3     
FRONT_RANGE = 90      

# 히스테리시스 관련
current_target_angle = 0.0
HYSTERESIS_THRESHOLD = 15.0  # 새로운 갭이 현재보다 15도 이상 차이 나야 방향 전환
SMOOTHING = 0.7              # 70%는 이전 각도 유지 (결단력 강화)

# =========================================
# 알고리즘: Follow the Gap with Hysteresis
# =========================

def get_gap_navigation(scan_data, prev_angle):
    global current_target_angle
    
    # 1. 전방 데이터 슬라이싱
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists = np.array([scan_data[a % 360] for a in angles])

    # 2. 장애물 팽창 (Obstacle Inflation)
    proc_dists = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if 0.05 < d < 0.7: # 0.7m 이내 장애물 팽창
            # asin(R/D)를 통해 장애물이 가리는 각도 계산
            alpha = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
            start_idx = max(0, int(i - alpha))
            end_idx = min(len(dists) - 1, int(i + alpha))
            proc_dists[start_idx:end_idx+1] = 0

    # 3. 최적의 방향 찾기 (Max Depth 방식)
    if np.max(proc_dists) == 0:
        return 0.0, 0.6 # 막히면 제자리 회전

    best_idx = np.argmax(proc_dists)
    new_angle = float(angles[best_idx])

    # 4. 히스테리시스 적용 (Hysteresis Logic)
    # 현재 가고 있는 방향과 차이가 크지 않으면 기존 방향을 고수함
    if abs(new_angle - prev_angle) < HYSTERESIS_THRESHOLD:
        final_angle = prev_angle
    else:
        # 급격한 변화를 막기 위한 평활화(Smoothing)
        final_angle = (prev_angle * SMOOTHING) + (new_angle * (1.0 - SMOOTHING))

    # 5. 제어 명령(v, w) 계산
    # 꺾는 각도가 클수록 속도를 대폭 줄임
    v = MAX_SPEED * (1.0 - abs(final_angle) / 110.0)
    v = max(v, 0.04) 

    # 각속도 제어 (P 게인)
    w = math.radians(final_angle) * 1.5
    w = np.clip(w, -MAX_W, MAX_W)

    return v, w, final_angle

# =========================================
# MAIN LOOP (생략된 데이터 파싱 부분 포함 필요)
# =========================================

# (라이다 초기화 시퀀스 생략...)

scan_data = [SCAN_LIMIT] * 360
last_angle = 0.0

try:
    while True:
        # [라이다 데이터 수신 및 scan_data 업데이트 로직 위치]
        # (기존에 작성한 5바이트 파싱 코드를 여기에 넣으세요)
        
        # 한 바퀴(angle == 0) 돌 때마다 명령 전송
        # if angle == 0:
            v, w, last_angle = get_gap_navigation(scan_data, last_angle)
            
            msg = f"{v:.3f},{w:.3f}\n"
            arduino_ser.write(msg.encode())
            
            # 아두이노 디버그와 맞춤
            print(f"v:{v:.2f} | w:{w:.2f} | angle:{last_angle:.1f}")

except KeyboardInterrupt:
    arduino_ser.write("0.0,0.0\n".encode())
