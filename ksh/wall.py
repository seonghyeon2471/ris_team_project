import serial
import time
import math
from rplidar import RPLidar   # pip install rplidar (또는 당신이 쓰는 LiDAR 라이브러리)

# ================== 설정 ==================
SERIAL_PORT = '/dev/ttyACM0'   # Arduino와 연결된 포트 (확인: ls /dev/tty*)
BAUDRATE = 115200
DESIRED_WALL_DISTANCE = 0.25   # 벽까지 유지할 거리 (m)
FRONT_THRESHOLD = 0.35         # 앞벽 감지 threshold (m)
TURN_SPEED = 0.4               # 원 그리며 돌 때 angular speed (rad/s)
LINEAR_SPEED = 0.25            # 기본 직진 속도 (m/s)

# Arduino에 보낼 명령 형식 예시: "L{left_pwm},R{right_pwm}\n"
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
time.sleep(2)

# LiDAR 초기화 (USB 포트 확인)
lidar = RPLidar('/dev/ttyUSB0')   # ← 당신 LiDAR 포트로 변경

follow_side = "right"   # 처음엔 오른쪽 벽 따라가기 (left로 바꿔도 OK)

def send_motor_command(left_speed, right_speed):
    # PWM 값으로 변환 (0~255, 음수면 후진)
    cmd = f"L{int(left_speed*255)},R{int(right_speed*255)}\n"
    ser.write(cmd.encode())
    print(f"→ Motor: L{left_speed:.2f} R{right_speed:.2f}")

def get_lidar_scan():
    # 한 번 스캔 (실제로는 continuous scan 권장)
    for scan in lidar.iter_scans(max_buf_meas=2000):
        return scan   # [(quality, angle, distance), ...]

def find_distances(scan):
    # angle 0° = 앞, 90° = 오른쪽, 270° = 왼쪽 (rplidar 기준)
    front = right = left = 999.0
    for _, angle, dist in scan:
        dist_m = dist / 1000.0
        if 355 <= angle or angle <= 5:      # front
            front = min(front, dist_m)
        elif 80 <= angle <= 100:            # right
            right = min(right, dist_m)
        elif 260 <= angle <= 280:           # left
            left = min(left, dist_m)
    return front, right, left

# ================== 메인 루프 ==================
try:
    print("장애물 회피 시작! (벽 따라 원 그리기 + front-wall reverse)")
    while True:
        scan = get_lidar_scan()
        front, right, left = find_distances(scan)
        
        # 1. 앞벽 감지 → 방향 반전
        if front < FRONT_THRESHOLD:
            print("!!! 앞벽 감지 → 방향 반전 !!!")
            follow_side = "left" if follow_side == "right" else "right"
            # 순간적으로 강하게 반대 방향으로 턴 (약 1초)
            if follow_side == "right":
                send_motor_command(0.6, -0.6)   # 오른쪽으로 빠르게 턴
            else:
                send_motor_command(-0.6, 0.6)
            time.sleep(0.8)
            continue
        
        # 2. 선택된 쪽 벽 따라가며 원 그리기 (P-control)
        if follow_side == "right":
            wall_dist = right
            # 오른쪽 벽이 멀면 오른쪽으로 턴 (시계방향 원)
            error = DESIRED_WALL_DISTANCE - wall_dist
            angular = TURN_SPEED + 1.2 * error   # P gain
        else:
            wall_dist = left
            error = DESIRED_WALL_DISTANCE - wall_dist
            angular = -TURN_SPEED - 1.2 * error  # 반대 방향
            
        # linear speed는 앞이 좀 막히면 살짝 줄임
        linear = LINEAR_SPEED * (1 if front > 0.6 else 0.6)
        
        # diff-drive 변환
        left_vel = linear - angular
        right_vel = linear + angular
        
        send_motor_command(left_vel, right_vel)
        time.sleep(0.05)   # 20Hz 제어

except KeyboardInterrupt:
    send_motor_command(0, 0)
    print("정지")
finally:
    lidar.stop()
    lidar.disconnect()
    ser.close()
