import time
import cv2
import numpy as np

# ─── [가상/실제 로봇 제어 및 센서 인터페이스 정의] ───
# 사용자 환경에 맞는 라이브러리(예: rospy, serial 등)로 send_cmd와 scan을 연동해야 합니다.
def send_cmd(linear_vel, angular_vel):
    # 실제 모터 드라이버나 ROS 토픽으로 속도 명령을 보내는 함수
    pass

def get_lidar_scan():
    # 실제 LiDAR 센서로부터 360도 거리 배열(cm 단위)을 받아오는 함수
    # 예시로 360개 요소의 배열을 반환한다고 가정 (0도: 정면, 90도: 왼쪽, 270도: 오른쪽)
    return np.ones(360) * 300 

def get_camera_frame():
    # 실제 카메라나 비디오 캡처 객체로부터 프레임을 받아오는 함수
    return np.zeros((480, 640, 3), dtype=np.uint8)

def detect_target(frame):
    # 기존 카메라 기반 객체 검출 로직 (Found 여부, 중심점 X, Y 등을 반환)
    # 여기서는 가상의 값 반환
    found = False
    cx, cy = 0, 0
    return found, cx, cy

# ─── [파라미터 설정] ─────────────────────────────────
WALL_TARGET = 30.0      # 벽 타기 유지 거리 (cm)
THRESH_TURN = 50.0      # 전방 벽 감지 및 회전 기준 거리 (cm)
THRESH_STOP = 20.0      # 전방 충돌 위험 급정거 거리 (cm)

# ─── [초기 상태 설정] ─────────────────────────────────
# 전체 메인 상태: "LIDAR" (벽타기 탐색) -> "PARK" (카메라 기반 주차)
main_state = "LIDAR"

# 세부 하위 상태
lidar_state = "WALL_SEARCH"
park_state = "WALL_SEARCH"

# 타이머 및 방향 제어 변수
search_start_time = time.time()
wall_follow_start_time = time.time()
escape_t = time.time()

follow_side = "left"  # 기본 벽타기 방향
adir = 1              # 회전 방향 가중치 (1: 좌회전, -1: 우회전)
target = "red"        # 예시 목적지 색상

# ─── [유틸리티 함수] ─────────────────────────────────
def get_front_dist(scan):
    # 정면 (고개 흔들림 감안하여 -10도 ~ 10도 사이 최솟값)
    front_angles = list(range(0, 11)) + list(range(350, 360))
    return min([scan[a] for a in front_angles if scan[a] > 0])

def side_dist(scan, side):
    if side == "left":
        return min([scan[a] for a in range(80, 101) if scan[a] > 0])
    else:
        return min([scan[a] for a in range(260, 281) if scan[a] > 0])

def wall_follow(scan, fm, adir, side):
    # 기본적인 벽타기 조향 연산 (P 제어 기법 예시)
    sd = side_dist(scan, side)
    error = sd - WALL_TARGET
    
    # 정면에 벽이 가깝다면 회전 우선
    if fm < THRESH_TURN:
        return 0.0, adir * 1.0
        
    # 벽과의 거리에 따른 조향 보정
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
            
        # 기본 센서 가공
        fm = get_front_dist(scan)
        found, cx, cy = detect_target(frame)
        
        # ----------------------------------------------------------------
        # 1. LIDAR 모드 (기본 벽 타며 경기장 탐색)
        # ----------------------------------------------------------------
        if main_state == "LIDAR":
            
            # ── A. 벽 찾는 중 (WALL_SEARCH) ──
            if lidar_state == "WALL_SEARCH":
                if time.time() - search_start_time > 5.0:
                    # 5초 동안 벽을 못 찾으면 전진하면서 탐색
                    send_cmd(0.15, 0.0)
                else:
                    # 제자리 회전하며 벽 탐색
                    send_cmd(0.0, adir * 0.8)
                    
                # 측면에 벽이 감지되면 접근 모드로 변경
                if side_dist(scan, follow_side) < 80.0:
                    lidar_state = "WALL_APPROACH"
                    print("[LIDAR] 벽 감지 -> 접근(WALL_APPROACH) 시작")
            
            # ── B. 벽으로 접근 중 (WALL_APPROACH) ──
            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    lidar_state = "WALL_FOLLOW"
                    wall_follow_start_time = time.time()  # 벽타기 시작 시점 타이머 리셋
                    print(f"[LIDAR] 벽 도달 ({follow_side}:{sd:.0f}cm) -> wall-following 시작")
                    continue
                
                # 벽에 붙기 위해 비스듬히 전진
                w_turn = 0.3 if follow_side == "left" else -0.3
                send_cmd(0.12, w_turn)
                
            # ── C. 벽 타기 주행 (WALL_FOLLOW) ──
            elif lidar_state == "WALL_FOLLOW":
                # [핵심 수정] 한 장애물을 7초 이상 잡고 돌 때 루프 탈출 발동
                if time.time() - wall_follow_start_time > 7.0:
                    lidar_state = "ESCAPE"
                    escape_t = time.time()
                    print("[LIDAR] 장애물 루프 갇힘 감지! -> 안전 탈출(ESCAPE) 모드 가동")
                    continue
                    
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                
                # (예시 트trigger) 탐색 도중 특정 조건 만족 시 주차 모드로 전환 가능
                # if 특정_조건: main_state = "PARK"

            # ── D. 논리적 안전 탈출 모드 (ESCAPE) ──
            elif lidar_state == "ESCAPE":
                elapsed = time.time() - escape_t
                
                # [단계 1] 0.8초 동안 내가 타던 장애물의 반대 방향으로 기습 90도 회전
                if elapsed < 0.8:
                    escape_turn_dir = -1.2 if follow_side == "left" else 1.2
                    send_cmd(0.0, escape_turn_dir)
                    cv2.putText(frame, "LIDAR-ESC: TURN AWAY", (10, 25), 0, 0.5, (0, 0, 255), 1)
                    
                # [단계 2] 이후 2초 동안 전방 LiDAR 공간을 실시간 필터링하며 안전 대시(Dash)
                elif elapsed < 2.8:
                    # 안전장치 1: 대시 도중 정면에 외곽 벽(진짜 벽)이 너무 가까워지면 즉시 중단
                    if fm < THRESH_TURN: 
                        print("[LIDAR-ESCAPE] 전방 벽 근접! 대시 조기 종료 후 탐색 복귀")
                        lidar_state = "WALL_SEARCH"
                        search_start_time = time.time()
                        continue
                        
                    # 안전장치 2: 정면이 1.8m 이상으로 너무 트여있으면 이미 탈출한 것이므로 조기 종료
                    elif fm > 180:
                        print("[LIDAR-ESCAPE] 공간 확보 완료 -> 대시 조기 종료 후 탐색 복귀")
                        lidar_state = "WALL_SEARCH"
                        search_start_time = time.time()
                        continue
                    
                    # 두 안전 필터링을 통과하면 안심하고 풀 직진
                    else:
                        send_cmd(0.20, 0.0)
                        cv2.putText(frame, "LIDAR-ESC: DASHING", (10, 25), 0, 0.5, (0, 255, 0), 1)
                        
                # [단계 3] 조기 종료 없이 2.8초 타임아웃을 다 채우면 안전하게 탐색 복귀
                else:
                    lidar_state = "WALL_SEARCH"
                    search_start_time = time.time()

        # ----------------------------------------------------------------
        # 2. PARK 모드 (카메라로 타겟 색상을 포착하여 주차 구역 진입)
        # ----------------------------------------------------------------
        elif main_state == "PARK":
            
            # 카메라가 목표를 찾으면 최우선 순위로 추적(TRACK) 상태 변환
            if found and park_state != "ESCAPE":
                park_state = "TRACK"
                
            # ── A. 벽 찾는 중 (WALL_SEARCH) ──
            if park_state == "WALL_SEARCH":
                send_cmd(0.0, adir * 0.8)
                if side_dist(scan, follow_side) < 80.0:
                    park_state = "WALL_APPROACH"
                    
            # ── B. 벽으로 접근 중 (WALL_APPROACH) ──
            elif park_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    park_state = "WALL_FOLLOW"
                    wall_follow_start_time = time.time()  # 타이머 리셋
                    continue
                w_turn = 0.3 if follow_side == "left" else -0.3
                send_cmd(0.12, w_turn)
                
            # ── C. 벽 타기 주행 (WALL_FOLLOW) ──
            elif park_state == "WALL_FOLLOW":
                # [핵심 수정] 주차 진입용 벽타기 중 기둥에 가두어졌을 때 타임아웃 처리
                if time.time() - wall_follow_start_time > 7.0:
                    park_state = "ESCAPE"
                    escape_t = time.time()
                    print(f"[{target}] 주차 탐색 중 갇힘! -> 안전 탈출(ESCAPE) 가동")
                    continue
                    
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                
            # ── D. 카메라 기반 객체 추적 (TRACK) ──
            elif park_state == "TRACK":
                if not found:
                    print(f"[{target}] 타겟 놓침 -> 다시 벽타기 탐색 복귀")
                    park_state = "WALL_SEARCH"
                    search_start_time = time.time()
                    continue
                    
                # 카메라 영상 중심점 오차 기반 조향 제어 (주차 정렬)
                error_x = cx - (frame.shape[1] / 2)
                v = 0.12
                w = -error_x * 0.005
                
                # 도착 판단 라인 안쪽으로 들어오면 정지 (예시)
                if cy > 400:
                    send_cmd(0.0, 0.0)
                    print("★ ARRIVED / PARK COMPLETE ★")
                    break
                send_cmd(v, w)

            # ── E. 주차 모드 전용 안전 탈출 모드 (ESCAPE) ──
            elif park_state == "ESCAPE":
                # 탈출 주행 중이라도 운좋게 카메라 시야에 타겟(목적지)이 걸려들면 즉시 추적 복귀
                if found:
                    print("[ESCAPE SUCCESS] 탈출 중 카메라로 타겟 재포착 -> TRACK 전환")
                    park_state = "TRACK"
                    continue
                    
                elapsed = time.time() - escape_t
                
                # [단계 1] 0.8초간 기둥 반대 방향 강제 90도 회전
                if elapsed < 0.8:
                    escape_turn_dir = -1.2 if follow_side == "left" else 1.2
                    send_cmd(0.0, escape_turn_dir)
                    cv2.putText(frame, "PARK-ESC: TURN AWAY", (10, 25), 0, 0.6, (0, 0, 255), 2)
                    
                # [단계 2] 2초간 전방 LiDAR 피드백 필터링하며 전속 대시
                elif elapsed < 2.8:
                    # 안전 브레이크 1: 외곽 벽 박기 직전 조기 종료
                    if fm < THRESH_TURN: 
                        print("[PARK-ESCAPE] 전방 맵 경계 근접 -> 대시 중단 후 재탐색")
                        park_state = "WALL_SEARCH"
                        search_start_time = time.time()
                        continue
                        
                    # 안전 브레이크 2: 너무 넓은 공간(맵 밖) 유입 방지 조기 종료
                    elif fm > 180:
                        print("[PARK-ESCAPE] 장애물 반경 이탈 완료 -> 대시 종료 후 재탐색")
                        park_state = "WALL_SEARCH"
                        search_start_time = time.time()
                        continue
                        
                    else:
                        send_cmd(0.20, 0.0)
                        cv2.putText(frame, "PARK-ESC: DASHING", (10, 25), 0, 0.6, (0, 255, 0), 2)
                        
                # [단계 3] 타임아웃 종료
                else:
                    park_state = "WALL_SEARCH"
                    search_start_time = time.time()

        # 화면 출력 처리
        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("사용자에 의해 종료되었습니다.")
finally:
    send_cmd(0.0, 0.0)
    cv2.destroyAllWindows()
