import cv2
import serial
import numpy as np
import time
import threading

# ─────────────────────────────────────────────────────────────
# 시리얼 포트 연결
# 아두이노: 모터 제어
# 라이다: 거리 스캔
# ─────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# ─────────────────────────────────────────────────────────────
# 카메라 설정
# 해상도, 버퍼, MJPG, 자동노출/자동화이트밸런스 설정
# ─────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# ─────────────────────────────────────────────────────────────
# 라이다 초기화
# 장치 부팅 후 스캔 시작
# ─────────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR OK")

# ─────────────────────────────────────────────────────────────
# 라이다 필터 파라미터
# EMA: 지수이동평균으로 노이즈 완화
# MEDIAN_K: 중앙값 필터 크기
# FRONT_RANGE: 정면 범위
# ─────────────────────────────────────────────────────────────
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 90
THRESH_SLOW = 55.0
THRESH_TURN = 30.0
THRESH_STOP = 18.0

# 360도 거리 배열
_scan     = np.full(360, 150.0, dtype=np.float32)
_scan_pub = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

# ─────────────────────────────────────────────────────────────
# EMA 필터: 특정 각도 거리값을 부드럽게 갱신
# ─────────────────────────────────────────────────────────────
def _ema(a, d):
    if d > 0:
        _scan[a] = (1 - EMA_ALPHA) * _scan[a] + EMA_ALPHA * d

# ─────────────────────────────────────────────────────────────
# 중앙값 필터: 주변 각도값과 비교해 튀는 값 제거
# ─────────────────────────────────────────────────────────────
def _median():
    k = MEDIAN_K
    buf = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
        buf[i] = np.sort(_scan[idx])[k]
    _scan[:] = buf

# ─────────────────────────────────────────────────────────────
# 라이다 수신 스레드
# 패킷을 계속 읽으며 angle / distance를 갱신
# sf == 1이면 한 바퀴 스캔 완료로 보고 pub 배열 갱신
# ─────────────────────────────────────────────────────────────
def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

        sf = raw[0] & 0x01

        # 패킷 유효성 체크
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue

        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0

        # 거리값 갱신
        if 3 < dist_cm < 150:
            _ema(angle, dist_cm)

        # 한 바퀴 스캔 완료 시 중앙값 필터 적용 후 공개
        if sf == 1:
            _median()
            with scan_lock:
                _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

# ─────────────────────────────────────────────────────────────
# 스캔 데이터 읽기
# ─────────────────────────────────────────────────────────────
def get_scan():
    with scan_lock:
        return _scan_pub.copy()

# ─────────────────────────────────────────────────────────────
# 정면 최소거리
# ±90도 범위를 정면으로 보고 가장 가까운 장애물 거리 반환
# ─────────────────────────────────────────────────────────────
def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

# ─────────────────────────────────────────────────────────────
# 좌/우 어느 쪽이 더 열려 있는지 판단
# 1이면 왼쪽이 더 열림, -1이면 오른쪽이 더 열림
# ─────────────────────────────────────────────────────────────
def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

# ─────────────────────────────────────────────────────────────
# 좌우 벽 거리 측정
# side == "L" : 왼쪽
# side == "R" : 오른쪽
# ─────────────────────────────────────────────────────────────
def side_dist(scan, side):
    if side == "L":
        idx = np.arange(85, 96) % 360
    else:
        idx = np.arange(265, 276) % 360
    return float(np.mean(scan[idx]))

# ─────────────────────────────────────────────────────────────
# 특정 각도 구간의 최소거리
# 벽 근접/충돌 판단에 사용
# ─────────────────────────────────────────────────────────────
def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# ─────────────────────────────────────────────────────────────
# 모터 명령 전송
# v: 전진 속도
# w: 회전 속도
# 아두이노로 "v,w" 형태 문자열 전송
# ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# ─────────────────────────────────────────────────────────────
# 색상별 마스크 설정
# hsv1: HSV 범위
# bgr:  BGR 범위
# draw: 화면 표시 색상
# ─────────────────────────────────────────────────────────────
COLOR_CFG = {
    "red": {
        "hsv1": ([169, 136, 114], [179, 220, 255]),
        "hsv2": None,
        "bgr": ([20, 20, 80], [255, 255, 255]),
        "draw": (0, 0, 255)
    },
    "yellow": {
        "hsv1": ([24, 19, 193], [45, 165, 255]),
        "hsv2": None,
        "bgr": ([0, 80, 80], [255, 255, 255]),
        "draw": (0, 200, 255)
    },
    "blue": {
        "hsv1": ([98, 100, 95], [138, 207, 246]),
        "hsv2": None,
        "bgr": ([40, 0, 0], [255, 220, 220]),
        "draw": (255, 80, 0)
    },
}

# 주차해야 할 색 순서
MISSION = ["red", "yellow", "blue"]

# ─────────────────────────────────────────────────────────────
# 색 마스크 생성
# HSV + BGR 조건을 동시에 만족하는 픽셀만 남김
# ─────────────────────────────────────────────────────────────
def make_mask(frame, hsv, name):
    cfg = COLOR_CFG[name]
    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    m = cv2.inRange(hsv, lo1, hi1)

    if cfg["hsv2"]:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        m = cv2.bitwise_or(m, cv2.inRange(hsv, lo2, hi2))

    bm = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    return cv2.bitwise_and(m, bm)

# ─────────────────────────────────────────────────────────────
# 주요 파라미터
# MIN_AREA: 색지로 인정할 최소 면적
# KP_ROT: 화면 중심 오차에 대한 회전 비례값
# W_MIN: 최소 회전 보장값
# APPROACH_V: 색지를 향해 접근할 때의 전진 속도
# ─────────────────────────────────────────────────────────────
MIN_AREA = 400
KP_ROT = 0.030
W_MIN = 0.20
APPROACH_V = 0.17
PARK_SEC = 1.2
DETECT_CONFIRM = 6

# 도착 판정 영역
ARRIVE_Y_TOP = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V = 0.17
ARRIVE_CONFIRM = 8

# wall-following 관련 파라미터
WALL_TARGET = 20.0
WALL_SCAN_DIST = 150.0
WALL_APPROACH_V = 0.20
WALL_KP = 0.012
WALL_V = 0.22
WALL_TURN_V = 0.10
WALL_LOST_W = 0.5
WALL_SEARCH_W = 1.1

# ─────────────────────────────────────────────────────────────
# SEARCH 상태 파라미터 (나선형 탐색)
# 제자리 회전(ROT) + 짧은 직진(FWD) 반복 대신,
# 회전속도(w)는 고정하고 전진속도(v)를 시간에 따라 점점 키워서
# 같은 자리를 빙글빙글 도는 게 아니라 반경이 점점 커지는
# 나선(스파이럴) 경로로 움직이며 색지를 찾는다.
# v/w 비율이 곧 회전 반경이므로, w 고정 + v 증가 = 반경 증가.
# ─────────────────────────────────────────────────────────────
SEARCH_SPIRAL_W       = 0.85   # 나선 회전 각속도 (고정)
SEARCH_SPIRAL_V_START = 0.05   # 첫 번째 나선의 시작 속도
SEARCH_SPIRAL_V_STEP  = 0.03   # 시도(try)가 늘어날 때마다 시작 속도를 키워서, 다음 나선은 더 바깥쪽부터 시작
SEARCH_SPIRAL_V_GROWTH = 0.05  # 한 나선이 진행되는 동안 시간에 따라 속도가 늘어나는 비율 (m/s per sec)
SEARCH_SPIRAL_V_MAX    = 0.22  # 전진 속도 상한
SEARCH_SPIRAL_SEC      = 2.2   # 한 나선을 진행하는 최대 시간(초). 지나면 방향을 반전하고 다음(더 큰) 나선으로
SEARCH_MAX_TRY         = 3     # 나선을 이만큼 반복해도 못 찾으면 WALL_SEARCH로 복귀

# 경계 안전 파라미터
BOUNDARY_MODE = "LINE_ON_DARK"
BOUNDARY_ROI_Y1 = 170
BOUNDARY_ROI_Y2 = 239
BOUNDARY_ROI_X1 = 0
BOUNDARY_ROI_X2 = 319
BOUNDARY_CONFIRM = 3
BOUNDARY_BACK_SEC = 0.35
BOUNDARY_TURN_SEC = 0.45
BOUNDARY_BACK_V = -0.12
BOUNDARY_TURN_W = 0.9

# ─────────────────────────────────────────────────────────────
# 상태 변수
# mode: LIDAR / PARK
# park_state: 현재 주차 상태
# safe_state: 경계 안전 상태
# search_state: 객체 놓쳤을 때 탐색 상태 (나선형 탐색의 단일 상태)
# ─────────────────────────────────────────────────────────────
mode = "LIDAR"
mission_idx = 0
detect_count = 0
arrive_count = 0
follow_side = "L"
lidar_state = "WALL_SEARCH"
park_state = "TRACK"
last_seen_x = 160
park_t = None
last_cmd = (0.0, 0.0)

safe_state = "OK"   # OK | BACK | TURN
safe_t = None
boundary_count = 0

search_state = "SPIRAL"     # 나선형 탐색 단일 상태
search_t = None             # 현재 나선이 시작된 시각
search_dir = 1               # 회전 방향 (+1 / -1), 나선이 끝날 때마다 반전
search_try = 0                # 몇 번째 나선인지 (커질수록 더 바깥쪽부터 시작)
search_v_base = SEARCH_SPIRAL_V_START  # 현재 나선의 시작 속도

print(f"START | MISSION: {MISSION}")

# ─────────────────────────────────────────────────────────────
# 바닥 경계선/테이프 감지
# 바닥 ROI를 잘라서 경계선이 보이는지 비율 계산
# ─────────────────────────────────────────────────────────────
def boundary_check(frame):
    roi = frame[BOUNDARY_ROI_Y1:BOUNDARY_ROI_Y2 + 1, BOUNDARY_ROI_X1:BOUNDARY_ROI_X2 + 1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # 어두운 바닥 위 밝은 라인
    if BOUNDARY_MODE == "LINE_ON_DARK":
        m1 = cv2.inRange(gray, 180, 255)
        m2 = cv2.inRange(hsv, np.array([0, 0, 170]), np.array([179, 80, 255]))
        mask = cv2.bitwise_and(m1, m2)
    else:
        # 밝은 바닥 위 어두운 라인
        m1 = cv2.inRange(gray, 0, 90)
        m2 = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([179, 80, 120]))
        mask = cv2.bitwise_and(m1, m2)

    # 노이즈 제거
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    ratio = float(np.count_nonzero(mask)) / float(mask.size + 1e-6)
    return ratio

# ─────────────────────────────────────────────────────────────
# 카메라 오차를 회전값으로 변환
# ex > 0 이면 목표가 화면 오른쪽에 있음
# ─────────────────────────────────────────────────────────────
def cam_w(ex):
    raw = -KP_ROT * ex
    if abs(raw) < W_MIN and ex != 0:
        return -W_MIN if ex > 0 else W_MIN
    return raw

# ─────────────────────────────────────────────────────────────
# 메인 루프
# 매 프레임마다:
# 1) 카메라 처리
# 2) 경계 안전 검사
# 3) 상태에 따라 LIDAR / PARK / SEARCH 제어
# ─────────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.flip(frame, 1)
        H, W = frame.shape[:2]
        cx_mid = W // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan = get_scan()
        fm = front_min(scan)
        adir = avoid_dir(scan)

        # 미션이 끝나면 정지
        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        target = MISSION[mission_idx]
        draw = COLOR_CFG[target]["draw"]

        # 목표 색 마스크 생성
        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        # 색지 중심 좌표 계산
        cx_obj, cy_obj = -1, -1
        if found:
            M_mom = cv2.moments(big)
            if M_mom["m00"] > 0:
                cx_obj = int(M_mom["m10"] / M_mom["m00"])
                cy_obj = int(M_mom["m01"] / M_mom["m00"])

            bx, by_top, bw, bh = cv2.boundingRect(big)
            last_seen_x = cx_obj

            # 화면에 인식 결과 표시
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)

        # 도착 판정 박스 표시
        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        # 색 중심이 도착 영역 안에 있는지 판단
        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # ─────────────────────────────────────────────────────
        # 경계 안전 체크
        # 위험하면 BACK -> TURN으로 빠져나감
        # ─────────────────────────────────────────────────────
        boundary_ratio = boundary_check(frame)
        cv2.putText(frame, f"BOUNDARY {boundary_ratio:.2f} SAFE:{safe_state}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if safe_state == "OK":
            if boundary_ratio > 0.12:
                boundary_count += 1
            else:
                boundary_count = 0

            if boundary_count >= BOUNDARY_CONFIRM:
                safe_state = "BACK"
                safe_t = time.time()
                boundary_count = 0
                stop_robot()

        # 후진 단계
        if safe_state == "BACK":
            send_cmd(BOUNDARY_BACK_V, 0.0)
            if time.time() - safe_t >= BOUNDARY_BACK_SEC:
                safe_state = "TURN"
                safe_t = time.time()

        # 회전 단계
        elif safe_state == "TURN":
            send_cmd(0.0, BOUNDARY_TURN_W)
            if time.time() - safe_t >= BOUNDARY_TURN_SEC:
                safe_state = "OK"
                safe_t = None

        # 안전상태가 정상일 때만 아래 로직 실행
        else:
            # ─────────────────────────────────────────────
            # LIDAR 모드
            # 첫 번째 색지 탐색 / wall-following
            # ─────────────────────────────────────────────
            if mode == "LIDAR":
                if found:
                    detect_count += 1
                else:
                    detect_count = 0

                # 색지를 일정 프레임 이상 보면 PARK 모드로 전환
                if detect_count >= DETECT_CONFIRM:
                    detect_count = 0
                    mode = "PARK"
                    park_state = "TRACK"
                    continue

                # 벽 탐색
                if lidar_state == "WALL_SEARCH":
                    if fm < WALL_SCAN_DIST:
                        l = side_dist(scan, "L")
                        r = side_dist(scan, "R")
                        follow_side = "L" if l <= r else "R"
                        lidar_state = "WALL_APPROACH"
                    else:
                        send_cmd(0.0, WALL_SEARCH_W)

                # 벽으로 접근
                elif lidar_state == "WALL_APPROACH":
                    sd = side_dist(scan, follow_side)
                    if sd <= WALL_TARGET * 1.3:
                        lidar_state = "WALL_FOLLOW"
                    else:
                        sign = 1 if follow_side == "L" else -1
                        if fm < THRESH_STOP:
                            v, w = 0.08, adir * 1.0
                        elif fm < THRESH_TURN:
                            v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                        else:
                            v, w = WALL_APPROACH_V, sign * 0.3
                        send_cmd(v, w)

                # 벽 따라 이동
                elif lidar_state == "WALL_FOLLOW":
                    v, w = wall_follow(scan, fm, adir, follow_side)
                    send_cmd(v, w)

            # ─────────────────────────────────────────────
            # PARK 모드
            # 목표 색을 추적해서 주차하는 단계
            # ─────────────────────────────────────────────
            elif mode == "PARK":
                # 1) 도착 후 전진
                if park_state == "FORWARD":
                    elapsed = time.time() - park_t
                    if elapsed >= ARRIVE_FORWARD_SEC:
                        stop_robot()
                        park_state = "PARKING"
                        park_t = time.time()
                    else:
                        send_cmd(*last_cmd)

                # 2) 주차 정지 유지
                elif park_state == "PARKING":
                    stop_robot()
                    elapsed = time.time() - park_t
                    if elapsed >= PARK_SEC:
                        mission_idx += 1
                        arrive_count = 0
                        detect_count = 0
                        if mission_idx < len(MISSION):
                            park_state = "WALL_SEARCH"
                        continue

                # 3) 벽 탐색
                elif park_state == "WALL_SEARCH":
                    if found:
                        detect_count += 1
                        if detect_count >= DETECT_CONFIRM:
                            detect_count = 0
                            park_state = "TRACK"
                            continue
                    else:
                        detect_count = 0

                    if fm < WALL_SCAN_DIST:
                        l = side_dist(scan, "L")
                        r = side_dist(scan, "R")
                        follow_side = "L" if l <= r else "R"
                        park_state = "WALL_APPROACH"
                        continue

                    send_cmd(0.0, WALL_SEARCH_W)

                # 4) 벽 접근
                elif park_state == "WALL_APPROACH":
                    if found:
                        detect_count += 1
                        if detect_count >= DETECT_CONFIRM:
                            detect_count = 0
                            park_state = "TRACK"
                            continue
                    else:
                        detect_count = 0

                    sd = side_dist(scan, follow_side)
                    if sd <= WALL_TARGET * 1.3:
                        park_state = "WALL_FOLLOW"
                        continue

                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP:
                        v, w = 0.08, adir * 1.0
                    elif fm < THRESH_TURN:
                        v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                    else:
                        v, w = WALL_APPROACH_V, sign * 0.3
                    send_cmd(v, w)

                # 5) 벽 따라 이동
                elif park_state == "WALL_FOLLOW":
                    if found:
                        detect_count += 1
                        if detect_count >= DETECT_CONFIRM:
                            detect_count = 0
                            park_state = "TRACK"
                            continue
                    else:
                        detect_count = 0

                    v, w = wall_follow(scan, fm, adir, follow_side)
                    send_cmd(v, w)

                # 6) 색지 추적
                elif found:
                    # 도착 영역 안에 색 중심이 들어오면 arrive_count 증가
                    arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                    # 충분히 안정적으로 들어오면 전진 상태로 전환
                    if arrive_count >= ARRIVE_CONFIRM:
                        arrive_count = 0
                        park_state = "FORWARD"
                        park_t = time.time()
                        last_cmd = (ARRIVE_FORWARD_V, 0.0)
                        send_cmd(ARRIVE_FORWARD_V, 0.0)
                        continue

                    # 화면 중심 기준 색지 좌우 오차
                    err_x = cx_obj - cx_mid

                    # 목표가 한쪽으로 크게 치우치면 속도를 줄임
                    err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                    reduced_v = APPROACH_V * (1.0 - err_ratio)

                    # 카메라 기준 회전 제어
                    if fm >= THRESH_SLOW:
                        v = reduced_v
                        w = cam_w(err_x)
                    else:
                        # 앞이 막히면 LIDAR 회피를 섞어서 움직임
                        w_cam = cam_w(err_x)
                        w_lid = adir * 0.7
                        if fm < THRESH_STOP:
                            v, w = 0.09, w_lid
                        elif fm < THRESH_TURN:
                            v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                        else:
                            v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                    last_cmd = (v, w)
                    send_cmd(v, w)

                # 7) 색지를 놓쳤을 때 검색 (나선형 탐색)
                #    제자리 회전이 아니라, w는 고정하고 v를 점점 키워서
                #    바깥쪽으로 점점 퍼지는 나선 경로를 그리며 찾는다.
                else:
                    if search_state == "SPIRAL":
                        if search_t is None:
                            # 새 나선 시작: try가 늘어날수록 더 큰 속도(=더 바깥쪽)부터 시작
                            search_t = time.time()
                            search_v_base = SEARCH_SPIRAL_V_START + search_try * SEARCH_SPIRAL_V_STEP

                        elapsed = time.time() - search_t
                        v = min(search_v_base + SEARCH_SPIRAL_V_GROWTH * elapsed, SEARCH_SPIRAL_V_MAX)
                        w = search_dir * SEARCH_SPIRAL_W
                        send_cmd(v, w)

                        cv2.putText(frame, f"SPIRAL try{search_try} v={v:.2f} w={w:.2f}", (10, 85),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

                        # 한 나선이 끝나면 방향을 반전하고 다음 나선(더 큰 반경) 준비
                        if elapsed >= SEARCH_SPIRAL_SEC:
                            search_try += 1
                            search_dir *= -1
                            search_t = None

                    # 나선을 충분히 반복해도 못 찾으면 벽 탐색으로 복귀
                    if search_try >= SEARCH_MAX_TRY:
                        search_try = 0
                        search_t = None
                        search_state = "SPIRAL"
                        park_state = "WALL_SEARCH"

        # 화면 출력
        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
