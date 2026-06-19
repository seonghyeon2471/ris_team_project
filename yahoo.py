import time
import cv2
import numpy as np

# ─── [가상/실제 로봇 제어 및 센서 인터페이스 정의] ───
def send_cmd(linear_vel, angular_vel):
    # 실제 모터 드라이버나 ROS 토픽으로 속도 명령 전송
    pass

def get_lidar_scan():
    # 실제 LiDAR 센서로부터 360도 거리 배열(cm 단위) 반환 (0도 정면, 90도 좌측, 270도 우측)
    return np.ones(360) * 300 

def get_camera_frame():
    return np.zeros((480, 640, 3), dtype=np.uint8)

def detect_target(frame):
    # 기존 카메라 기반 객체 검출 로직
    found = False
    cx, cy = 0, 0
    return found, cx, cy

# ─── [파라미터 설정] ─────────────────────────────────
WALL_TARGET = 30.0      # 벽 타기 유지 거리 (cm)
THRESH_TURN = 50.0      # 전방 벽 감지 및 회전 기준 거리 (cm)
THRESH_STOP = 20.0      # 전방 충돌 위험 급정거 거리 (cm)

# ─── [초기 상태 설정] ─────────────────────────────────
# main_state: "LIDAR" -> "PARK" -> "SCAN_360"
main_state = "LIDAR"

# 세부 하위 상태
lidar_state = "WALL_SEARCH"
park_state = "WALL_SEARCH"

# 타이머 및 스캔용 변수
search_start_time = time.time()
wall_follow_start_time = time.time()
escape_t = time.time()

scan_360_start_time = time.time()
closest_dist = 9999.0
closest_angle = 0

follow_side = "left"  # 기본 벽타기 방향 (SCAN_360 완료 후 자동 변경됨)
adir = 1              # 회전 방향 가중치 (1: 좌회전, -1: 우회전)
target = "yellow"     # 추적 목적지 색상 (필요시 조건문으로 분기 활용 가능)

# ─── [유틸리티 함수] ─────────────────────────────────
def get_front_dist(scan):
    front_angles = list(range(0, 11)) + list(range(350, 360))
    return min([scan[a] for a in front_angles if scan[a] > 0])

def side_dist(scan, side):
    if side == "left":
        return min([scan[a] for a in range(80, 101) if scan[a] > 0])
    else:
        return min([scan[a] for a in range(260, 281) if scan[a] > 0])

def wall_follow(scan, fm, adir, side):
    sd = side_dist(scan, side)
    error = sd - WALL_TARGET
    
    if fm < THRESH_TURN:
        return 0.0, adir * 1.0
        
    kp = 0.05
    w = -error * kp if side == "left" else error * kp
    w = np.clip(w, -0.5, 0.5)
    return 0.15, w


# ─── [메인 루프] ─────────────────────────────────────
try:
    while True:
        frame = get_camera_frame()
        scan = get_lidar_scan()
        
        if frame is None or scan is None:
            continue
            
        fm = get_front_dist(scan)
        found, cx, cy = detect_target(frame)
        
        # ----------------------------------------------------------------
        # 1. LIDAR 모드 (기본 벽 타며 경기장 탐색)
        # ----------------------------------------------------------------
        if main_state == "LIDAR":
            if lidar_state == "WALL_SEARCH":
                if time.time() - search_start_time > 5.0:
                    send_cmd(0.15, 0.0)
                else:
                    send_cmd(0.0, adir * 0.8)
                    
                if side_dist(scan, follow_side) < 80.0:
                    lidar_state = "WALL_APPROACH"
                    print(f"[LIDAR] 벽 감지 ({follow_side}) -> 접근 시작")
            
            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    lidar_state = "WALL_FOLLOW"
                    wall_follow_start_time = time.time()
                    print(f"[LIDAR] 벽 도달 ({follow_side}:{sd:.0f}cm) -> wall-following 시작")
                    continue
                w_turn = 0.3 if follow_side == "left" else -0.3
                send_cmd(0.12, w_turn)
                
            elif lidar_state == "WALL_FOLLOW":
                if time.time() - wall_follow_start_time > 7.0:
                    lidar_state = "ESCAPE"
                    escape_t = time.time()
                    print("[LIDAR] 장애물 갇힘 감지! -> 탈출 모드 가동")
                    continue
                    
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)

                # 조건 만족 시 주차 모드 진입 (예시)
                # if 특정조건: main_state = "PARK"

            elif lidar_state == "ESCAPE":
                elapsed = time.time() - escape_t
                if elapsed < 0.8:
                    escape_turn_dir = -1.2 if follow_side == "left" else 1.2
                    send_cmd(0.0, escape_turn_dir)
                elif elapsed < 2.8:
                    if fm < THRESH_TURN or fm > 180:
                        lidar_state = "WALL_SEARCH"
                        search_start_time = time.time()
                        continue
                    else:
                        send_cmd(0.20, 0.0)
                else:
                    lidar_state = "WALL_SEARCH"
                    search_start_time = time.time()

        # ----------------------------------------------------------------
        # 2. PARK 모드 (카메라로 타겟 추적 및 주차)
        # ----------------------------------------------------------------
        elif main_state == "PARK":
            if found and park_state != "ESCAPE":
                park_state = "TRACK"
                
            if park_state == "WALL_SEARCH":
                send_cmd(0.0, adir * 0.8)
                if side_dist(scan, follow_side) < 80.0:
                    park_state = "WALL_APPROACH"
                    
            elif park_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    park_state = "WALL_FOLLOW"
                    wall_follow_start_time = time.time()
                    continue
                w_turn = 0.3 if follow_side == "left" else -0.3
                send_cmd(0.12, w_turn)
                
            elif park_state == "WALL_FOLLOW":
                if time.time() - wall_follow_start_time > 7.0:
                    park_state = "ESCAPE"
                    escape_t = time.time()
                    continue
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                
            elif park_state == "TRACK":
                if not found:
                    park_state = "WALL_SEARCH"
                    search_start_time = time.time()
                    continue
                    
                error_x = cx - (frame.shape[1] / 2)
                v = 0.12
                w = -error_x * 0.005
                
                # 도착 판단 라인 (cy > 400) 도달 시
                if cy > 400:
                    send_cmd(0.0, 0.0)
                    
                    # (선택) 만약 '시작점'과 '노란색 객체'의 처리를 나누고 싶다면 여기에 if문을 추가할 수 있습니다.
                    # 예: if target == "start_point": break 
                    
                    print(f"★ [{target}] 주차 완료 -> 360도 주변 스캔 시퀀스 가동 ★")
                    main_state = "SCAN_360"
                    scan_360_start_time = time.time()
                    closest_dist = 9999.0  # 최솟값 초기화
                    closest_angle = 0
                    continue
                    
                send_cmd(v, w)

            elif park_state == "ESCAPE":
                # 기존 PARK ESCAPE 로직 (간략화)
                if found:
                    park_state = "TRACK"
                    continue
                
                elapsed = time.time() - escape_t
                if elapsed < 0.8:
                    send_cmd(0.0, -1.2 if follow_side == "left" else 1.2)
                elif elapsed < 2.8:
                    if fm < THRESH_TURN or fm > 180:
                        park_state = "WALL_SEARCH"
                        search_start_time = time.time()
                        continue
                    send_cmd(0.20, 0.0)
                else:
                    park_state = "WALL_SEARCH"
                    search_start_time = time.time()

        # ----------------------------------------------------------------
        # 3. SCAN_360 모드 (한바퀴 회전하며 가장 가까운 장애물 탐색 및 방향 결정)
        # ----------------------------------------------------------------
        elif main_state == "SCAN_360":
            elapsed = time.time() - scan_360_start_time
            
            # [튜닝 포인트] 로봇이 1회전(360도)하는 데 걸리는 실제 시간으로 수정
            SCAN_DURATION = 4.5 
            
            if elapsed < SCAN_DURATION:
                # 제자리 회전 (각속도 1.0)
                send_cmd(0.0, 1.0) 
                
                # 회전하는 동안 실시간 LiDAR 측정값 중 최소 거리/각도 갱신
                for angle in range(360):
                    dist = scan[angle]
                    if 0 < dist < closest_dist:
                        closest_dist = dist
                        closest_angle = angle
                        
                cv2.putText(frame, f"SCANNING... Min: {closest_dist:.1f}cm", (10, 25), 0, 0.6, (255, 0, 255), 2)
                
            else:
                print(f"★ 스캔 완료! 가장 가까운 장애물: {closest_dist:.1f}cm (상대각도: {closest_angle}도) ★")
                
                # 최소 거리를 가진 각도가 왼쪽인지 오른쪽인지 판별하여 방향 결정
                if 0 <= closest_angle <= 180:
                    follow_side = "left"
                    adir = 1
                else:
                    follow_side = "right"
                    adir = -1
                
                print(f"-> 벽타기 방향을 [{follow_side}]로 자동 갱신하고 복귀 탐색(WALL_APPROACH) 시작!")
                
                # 탐색 모드로 복귀
                main_state = "LIDAR"
                lidar_state = "WALL_APPROACH"
                search_start_time = time.time()
                continue

        # 화면 출력 및 강제 종료키(q) 설정
        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("사용자에 의해 종료되었습니다.")
finally:
    send_cmd(0.0, 0.0)
    cv2.destroyAllWindows()
