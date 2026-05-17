import time
import serial
from rplidar import RPLidar

# ================== 설정 (사용자 요청 반영) ==================
SERIAL_PORT = '/dev/ttyUSB0'   # Arduino 연결 포트 (ls /dev/tty* 로 확인)
BAUDRATE = 115200
LIDAR_PORT = '/dev/ttyUSB1'    # RPLIDAR 포트

# 튜닝 파라미터
MIN_DIST = 0.10        # 10cm (사용자 요청)
MAX_TURN = 85          # 최대 steering 강도 (0~100)
BASE_SPEED = 62        # 기본 직진 속도 (튜닝 포인트)
FRONT_ANGLE_MAX = 120  # 120도까지 전방으로 고려

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
time.sleep(2)

lidar = RPLidar(LIDAR_PORT)
print("LiDAR 및 Arduino 연결 완료! (60/90/120도 + 10cm 모드)")

def send_motor(left, right):
    cmd = f"L:{int(left)} R:{int(right)}\n"
    ser.write(cmd.encode())

def get_steering(scan):
    min_dist = float('inf')
    min_angle = 0.0

    for meas in scan:
        dist = meas[2] / 1000.0      # mm → m
        angle = meas[1]              # 0~360
        
        # 전방만 고려 (±120도)
        if (360 - FRONT_ANGLE_MAX <= angle or angle <= FRONT_ANGLE_MAX):
            if 0.08 < dist < min_dist:   # 8cm 이하는 노이즈 무시
                min_dist = dist
                min_angle = angle if angle <= 180 else angle - 360   # -180~180

    if min_dist == float('inf'):
        return 0  # 장애물 없음 → 직진

    theta = min_angle                     # 장애물 각도 (-120 ~ 120)
    abs_theta = abs(theta)

    # ================== 60/90/120도 tiered steering ==================
    if abs_theta <= 60:           # 좁은 각도 → 강하게
        strength = 1.0
    elif abs_theta <= 90:         # 중간
        strength = 0.65
    elif abs_theta <= 120:        # 넓은 각도 → 약하게
        strength = 0.35
    else:
        return 0

    # 방향: 장애물 쪽으로 theta가 양수면 왼쪽으로 회전 (steering 음수)
    steering = - (theta / abs_theta) * MAX_TURN * strength
    # 거리 가까울수록 더 강하게 (10cm 기준)
    dist_factor = MIN_DIST / max(min_dist, MIN_DIST * 0.5)
    steering *= dist_factor

    # steering 범위 제한
    steering = max(-MAX_TURN, min(MAX_TURN, steering))

    # 모터 속도 계산
    left_speed = BASE_SPEED + steering
    right_speed = BASE_SPEED - steering

    left_speed = max(20, min(100, left_speed))   # 최소 20은 직진 유지
    right_speed = max(20, min(100, right_speed))

    return left_speed, right_speed

# ================== 메인 루프 ==================
try:
    for scan in lidar.iter_scans(max_buf_meas=2000):
        left, right = get_steering(scan)
        send_motor(left, right)
        
        # 실시간 디버깅 (필요하면 주석 해제)
        # print(f"θ: {theta:+6.1f}°  dist: {min_dist:.2f}m  L:{left:3.0f} R:{right:3.0f}")

except KeyboardInterrupt:
    print("\n종료")
    send_motor(0, 0)
finally:
    lidar.stop()
    lidar.disconnect()
    ser.close()
