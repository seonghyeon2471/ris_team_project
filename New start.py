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

# SEARCH 상태 파라미터
SEARCH_ROT_SEC = 1.0
SEARCH_FWD_SEC = 0.22
SEARCH_ROT_W = 0.9
SEARCH_FWD_V = 0.10
SEARCH_MAX_TRY = 3

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
# search_state: 객체 놓쳤을 때 탐색 상태
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

search_state = "ROT"   # ROT | FWD
search_t = None
search_dir = 1
search_try = 0

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
            
