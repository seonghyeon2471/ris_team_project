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
SCAN_LIMIT = 2.5      # [수정] 0.3은 너무 짧음. 최소 1m 이상 권장 (벽 인식용)
FRONT_RANGE = 90      

# 히스테리시스 관련
HYSTERESIS_THRESHOLD = 15.0  
SMOOTHING = 0.6              

# =========================================
# 알고리즘: Follow the Gap with Hysteresis
# =========================================

def get_gap_navigation(scan_data, prev_angle):
    # 1. 전방 데이터 슬라이싱 (인덱스 에러 방지)
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    # scan_data가 리스트이므로 안전하게 인덱싱
    dists = np.array([float(scan_data[int(a) % 360]) for a in angles])

    # 2. 장애물 팽창 (Obstacle Inflation)
    proc_dists = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        
        # [수정] d가 너무 작으면 분모가 0이 되어 에러 발생. 최소 0.05m 보장.
        if 0.05 < d < 0.7: 
            # [수정] ROBOT_RADIUS / d 가 1을 넘지 않도록 제한 (asin 에러 방지)
            ratio = ROBOT_RADIUS / d
            alpha = math.degrees(math.asin(min(ratio, 1.0)))
            
            start_idx = max(0, int(i - alpha))
            end_idx = min(len(dists) - 1, int(i + alpha))
            proc_dists[start_idx:end_idx+1] = 0

    # 3. 최적의 방향 찾기 (Max Depth 방식)
    # [수정] 모든 방향이 0(장애물)인 경우 예외 처리
    if np.max(proc_dists) <= 0:
        return 0.0, 0.6, prev_angle 

    best_idx = np.argmax(proc_dists)
    new_angle = float(angles[best_idx])

    # 4. 히스테리시스 및 평활화
    if abs(new_angle - prev_angle) < HYSTERESIS_THRESHOLD:
        final_angle = prev_angle
    else:
        final_angle = (prev_angle * SMOOTHING) + (new_angle * (1.0 - SMOOTHING))

    # 5. 제어 명령(v, w) 계산
    v = MAX_SPEED * (1.0 - abs(final_angle) / 110.0)
    v = max(v, 0.04) 

    w = math.radians(final_angle) * 1.5
    w = np.clip(w, -MAX_W, MAX_W)

    return v, w, final_angle

# =========================================
# MAIN LOOP
# =========================================

# [수정] 초기 scan_data는 충분히 큰 값(빈 공간)으로 채워야 로봇이 출발함
scan_data = [SCAN_LIMIT] * 360
last_angle = 0.0

print("LIDAR START...")

try:
    while True:
        # 여기에 라이다 5바이트 파싱 코드가 들어가야 합니다.
        # 예시: angle, dist = parse_lidar_data()
        # scan_data[angle] = dist
        
        # [임시 실행부] 실제로는 특정 각도(예: angle == 0)마다 실행하는 것이 효율적
        v, w, last_angle = get_gap_navigation(scan_data, last_angle)
        
        msg = f"{v:.3f},{w:.3f}\n"
        arduino_ser.write(msg.encode())
        
        # 출력 오버헤드를 줄이기 위해 간헐적으로 출력 권장
        # print(f"v:{v:.2f} | w:{w:.2f} | angle:{last_angle:.1f}")
        time.sleep(0.05) # 루프 속도 조절

except KeyboardInterrupt:
    arduino_ser.write("0.0,0.0\n".encode())
    print("\nSTOP")
finally:
    arduino_ser.close()
    lidar_ser.close()
