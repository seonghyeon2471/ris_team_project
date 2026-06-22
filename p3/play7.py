import cv2
import serial
import numpy as np
import time
import math
import threading

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0",  460800, timeout=0.1)

# ── CAMERA ────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR ─────────────────────────────────────────────────────────────
EMA_ALPHA    = 0.35
MEDIAN_K     = 2
FRONT_RANGE  = 90
THRESH_SLOW  = 55.0
THRESH_TURN  = 35.0
THRESH_STOP  = 20.0

_scan     = np.full(360, 150.0, dtype=np.float32)
_scan_pub = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

def _ema(a, d):
    if d > 0:
        _scan[a] = (1 - EMA_ALPHA) * _scan[a] + EMA_ALPHA * d

def _median():
    k = MEDIAN_K
    buf = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
        buf[i] = np.sort(_scan[idx])[k]
    _scan[:] = buf

def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5: continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150: _ema(angle, dist_cm)
        if sf == 1:
            _median()
            with scan_lock: _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock: return _scan_pub.copy()

def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    vals = scan[idx]
    return float(np.percentile(vals, 15))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

# 어느 쪽 벽을 추종할지 결정 (타겟이 보이면 타겟 쪽, 안 보이면 라이다 기준 더 넓게 트인 쪽)
# WALL_SEARCH -> WALL_APPROACH로 넘어가는 순간에만 호출, 추종 중에는 절대 다시 호출하지 않음
def decide_follow_side(adir, found, cx_obj, cx_mid):
    if found and cx_obj >= 0:
        return "L" if cx_obj < cx_mid else "R"
    return "R" if adir == 1 else "L"

def side_dist(scan, side):
    if side == "L":
        idx = np.arange(265, 296) % 360
    else:
        idx = np.arange(65, 96) % 360
    return float(np.min(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# follow_side 진행방향 기준 "더 앞쪽"/"더 뒤쪽" 측면 빔.
# side_dist()는 정확히 측면(거의 90도/270도)만 보기 때문에, 벽이 凸 코너로 꺾여 멀어지기
# *시작하는* 순간에는 반응이 늦다(이미 옆을 완전히 지나친 뒤에야 sd가 커짐).
# front 쪽 빔이 back 쪽 빔보다 멀어지기 시작하면 "곧 코너"라는 조짐이므로 이걸로 미리 꺾어준다.
def side_subdist(scan, side, part):
    if side == "R":
        rng = (40, 70) if part == "front" else (95, 125)
    else:  # L
        rng = (290, 320) if part == "front" else (235, 265)
    idx = np.arange(rng[0], rng[1]) % 360
    return float(np.min(scan[idx]))

# ── GAP SEARCH ────────────────────────────────────────────────────────
# 360도 스캔에서 "진짜 좁은 틈"(양쪽이 막혀있고 그 사이만 트인 구간)을 찾는다.
# 단순히 평균적으로 더 트인 쪽(avoid_dir)으로 가는 게 아니라, 통과 가능한 폭의
# 진짜 갭(문/틈)을 찾아 그 중심 각도를 반환한다. 후보가 없으면 None.
def find_best_gap(scan):
    free = scan > GAP_FREE_THRESH
    n = 360
    if free.all() or not free.any():
        return None  # 사방이 다 트였거나 다 막혀있으면 "틈"이라 부를 후보가 없음

    # "막힘 -> 뚫림" 전환 지점을 시작점으로 잡아 원형 run-length encoding
    start_idx = None
    for k in range(n):
        if free[k] and not free[(k - 1) % n]:
            start_idx = k
            break
    if start_idx is None:
        return None

    runs = []
    idx = start_idx
    visited = 0
    while visited < n:
        if free[idx % n]:
            run_start = idx % n
            length = 0
            while free[idx % n] and length < n:
                idx += 1
                length += 1
                visited += 1
            run_end = (idx - 1) % n
            runs.append((run_start, run_end, length))
        else:
            idx += 1
            visited += 1

    candidates = []
    for run_start, run_end, length in runs:
        if length < GAP_MIN_ANGLE_DEG or length > GAP_MAX_ANGLE_DEG:
            continue  # 노이즈로 보기엔 너무 좁거나, "틈"이라기엔 너무 넓음(=그냥 열린 공간)
        before_d = float(scan[(run_start - 1) % n])   # 갭 직전(낮은 인덱스 쪽=R 방향) 벽
        after_d  = float(scan[(run_end + 1) % n])      # 갭 직후(높은 인덱스 쪽=L 방향) 벽
        flank_d  = min(before_d, after_d)
        if flank_d > GAP_MAX_FLANK_DIST:
            continue  # 양쪽 벽이 너무 멀면 신뢰도 낮은 노이즈로 취급
        gap_width = 2.0 * flank_d * math.sin(math.radians(length) / 2.0)
        if gap_width < GAP_MIN_PHYS_WIDTH or gap_width > GAP_MAX_PHYS_WIDTH:
            continue  # 로봇이 못 지나가거나, 충분히 좁지 않은 갭은 제외
        center = (run_start + (length - 1) / 2.0) % n
        depth = float(scan[int(round(center)) % n])
        if depth < GAP_MIN_DEPTH:
            continue  # 갭 안쪽으로 진입할 공간이 충분치 않으면 제외
        candidates.append((gap_width, center, before_d, after_d))

    if not candidates:
        return None

    # "진짜 좁은 틈" 우선: 통과 가능한 갭 중 물리적으로 가장 좁은(타이트한) 것부터 선택
    candidates.sort(key=lambda c: c[0])
    return candidates[0][1]

# 320x240 카메라 기준 cx_obj(색지 중심 x좌표) -> LiDAR 각도(정면=0 기준 오프셋)로 변환
def cx_to_lidar_angle(cx_obj, cx_mid, frame_w=320):
    if cx_obj < 0:
        return 0.0
    ratio = (cx_obj - cx_mid) / (frame_w / 2.0)
    ratio = float(np.clip(ratio, -1.0, 1.0))
    return CAM_TO_LIDAR_SIGN * ratio * (CAMERA_HFOV / 2.0)

# 색지 방향(target_angle, 정면 기준 오프셋) ±TARGET_CHECK_HALFWIDTH 범위의 LiDAR 최소 거리
def obstacle_on_target(scan, target_angle):
    center = int(round(target_angle)) % 360
    idx = np.arange(center - TARGET_CHECK_HALFWIDTH,
                     center + TARGET_CHECK_HALFWIDTH + 1) % 360
    return float(np.min(scan[idx]))

def wall_follow(scan, fm, adir, follow_side):
    sd          = side_dist(scan, follow_side)
    left_close  = side_min(scan, 240, 300)
    right_close = side_min(scan, 60, 120)
    sign = -1 if follow_side == "L" else 1

    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    if left_close < SIDE_STOP:
        return (WALL_V * 0.7,  0.7)
    if right_close < SIDE_STOP:
        return (WALL_V * 0.7, -0.7)

    if left_close < BOTTLENECK_ENTER and right_close < BOTTLENECK_ENTER:
        err_center = left_close - right_close
        w_center = float(np.clip(-CENTER_KP * err_center, -0.9, 0.9))
        return (BOTTLENECK_V, w_center)

    # ── 코너 예측(lookahead) ──────────────────────────────────────────
    # 진행방향 쪽(front) 측면 빔이 후방(back) 측면 빔보다 멀어지고 있으면
    # = 벽이 앞쪽에서 꺾여 멀어지는 凸 코너에 다다르고 있다는 신호.
    # sd(순수 측면 거리)가 실제로 커지기 *전에* 미리 follow_side 방향으로 더 꺾어서
    # 코너를 둥글게 따라가도록 한다. (front <= back, 즉 코너 조짐이 없으면 0)
    front_side = side_subdist(scan, follow_side, "front")
    back_side  = side_subdist(scan, follow_side, "back")
    corner_err = max(0.0, front_side - back_side)
    w_corner   = sign * float(np.clip(WALL_CORNER_KP * corner_err, 0.0, WALL_CORNER_WMAX))

    if sd > WALL_TARGET * 2.0:
        # 완전히 벽을 놓친 상태. 기존엔 거의 제자리(v=0.05)로 WALL_LOST_W만큼 스핀했는데,
        # 이러면 코너에서 "벽 없는 빈 공간으로 돌진 -> 한참 헌팅 스핀"처럼 보인다.
        # 대신 전진은 유지한 채(WALL_LOST_ARC_V) follow_side 쪽으로 호를 그리며 돌아
        # 코너를 자연스럽게 감아 도는 모양이 되도록 한다.
        return (WALL_LOST_ARC_V, sign * WALL_LOST_ARC_W)

    err = sd - WALL_TARGET
    w   = sign * WALL_KP * err + w_corner
    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V

    w = float(np.clip(w, -0.9, 0.9))
    return (v, w)

# ── MOTOR ─────────────────────────────────────────────────────────────
# 마지막으로 실제 전송된 (v, w)를 기록 — Watchdog(P1)이 "최근 얼마나 회전했는지"를
#   판단하는 데 사용. send_cmd를 거치는 모든 상태에서 동일하게 누적되므로 상태별로 따로
#   계측 코드를 넣지 않아도 된다.
last_sent_v, last_sent_w = 0.0, 0.0

def send_cmd(v, w):
    global last_sent_v, last_sent_w
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    last_sent_v, last_sent_w = float(v), float(w)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]), "hsv2": None, "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]),    "hsv2": None, "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 95], [138, 207, 246]),   "hsv2": None, "bgr":  ([40,  0,   0], [255, 220, 220]), "draw": (255, 80, 0)},
}
MISSION = ["red", "yellow", "blue"]

def make_mask(frame, hsv, name):
    cfg = COLOR_CFG[name]
    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    m = cv2.inRange(hsv, lo1, hi1)
    if cfg["hsv2"]:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        m = cv2.bitwise_or(m, cv2.inRange(hsv, lo2, hi2))
    bm = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    return cv2.bitwise_and(m, bm)

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA       = 400
KP_ROT         = 0.030
W_MIN          = 0.20
APPROACH_V     = 0.13
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8
BIG_TARGET_AREA    = 9000   # 색지 면적이 이보다 크면(=충분히 가까움) arrive_count 확인을 생략하고 즉시 도착 처리

WALL_TARGET    = 12.0
SIDE_STOP      = 6.0
BOTTLENECK_ENTER = 25.0
CENTER_KP      = 0.030
BOTTLENECK_V   = 0.15
WALL_APPROACH_V = 0.20
WALL_KP        = 0.024
WALL_V         = 0.22
WALL_TURN_V    = 0.10
WALL_LOST_W    = 1.3
WALL_SEARCH_W  = 0.8

# ── 코너 추종(corner hugging) 신규 파라미터 ───────────────────────────
WALL_CORNER_KP   = 0.018          # front_side-back_side 차이 -> 추가 회전량 게인
WALL_CORNER_WMAX = 0.5            # 코너 예측으로 추가되는 회전량의 최대치
WALL_LOST_ARC_V  = WALL_V * 0.6   # 벽을 완전히 놓쳤을 때도 제자리 스핀 대신 전진을 유지(호 그리기)
WALL_LOST_ARC_W  = WALL_LOST_W * 0.65

# ── GAP SEARCH PARAMS ─────────────────────────────────────────────────
# WALL_SEARCH 상태에서 "더 트인 쪽"이 아니라 "진짜 좁은 틈"을 찾아 그쪽으로 정렬/출발하기 위한 값들.
# 로봇 실제 폭/안전여유, 아레나 벽 재질/반사율에 맞게 튜닝 필요.
GAP_FREE_THRESH    = 45.0   # 이 거리(cm) 이상이면 "뚫린 방향"으로 간주
GAP_MIN_ANGLE_DEG  = 10      # 최소 갭 각도폭 (이보다 좁으면 노이즈로 보고 무시)
GAP_MAX_ANGLE_DEG  = 110     # 최대 갭 각도폭 (이보다 넓으면 "틈"이 아니라 그냥 열린 공간으로 보고 제외)
GAP_MIN_PHYS_WIDTH = 26.0    # cm, 로봇이 실제로 통과 가능한 최소 물리 폭(로봇 폭 + 여유)
GAP_MAX_PHYS_WIDTH = 70.0    # cm, 이보다 넓으면 "좁은 틈"이 아니라고 보고 후보에서 제외
GAP_MAX_FLANK_DIST = 120.0   # cm, 갭 양쪽 벽까지 거리가 이보다 멀면 신뢰도 낮은 값으로 보고 무시
GAP_MIN_DEPTH      = 30.0    # cm, 갭 정면 방향으로 최소 이만큼은 트여 있어야 진입 가능으로 인정
GAP_ALIGN_TOL_DEG  = 6.0     # 갭 중심과 정면의 오차가 이 안에 들어오면 "정렬 완료"로 보고 접근 시작

# P1: 같은 색을 이 시간 넘게 못 찾으면 LOOP_ESCAPE (시간 기반 트리거)
# ★ 변경: 10.0 -> 6.0 -> 8.0 (WALL_SEARCH_W를 낮춰 회전을 늦췄으므로 타임아웃도 다시 늘림)
MISSION_TIMEOUT_SEC = 8.0

TARGET_CLEAR_DIST      = 40.0
TARGET_CHECK_HALFWIDTH = 15
CAMERA_HFOV            = 60.0
CAM_TO_LIDAR_SIGN      = 1

# P1: 같은 자리를 맴도는 것(회전 누적)을 감지하기 위한 임계값/탈출 동작
# ★ 변경: 400도 -> 260도 -> 300도 (회전 속도를 낮췄으므로 임계값도 살짝 다시 늘림)
FULL_LOOP_THRESH   = math.radians(300)
LOOP_ESCAPE_BACK_V = -0.12
LOOP_ESCAPE_SEC    = 0.6
# ★ 신규: 탈출 시 회전 단계의 길이/속도를 별도 상수로 분리 (기존엔 1.5초, WALL_SEARCH_W*1.2로 하드코딩)
LOOP_ESCAPE_TURN_SEC = 2.6                  # 1.5 -> 2.6초로 연장 (더 오래 회전)
LOOP_ESCAPE_TURN_W   = WALL_SEARCH_W * 1.6  # 1.2배 -> 1.6배로 증가 (더 빠르게/많이 회전)

# ── DODGE (정면 장애물 + 색지 인식 시 소각도 회피 후 원선회) ─────────────
# TRACK 중 fm(정면 LiDAR 최소거리) < DODGE_TRIGGER_DIST 이면 색지를 무시하고
#   회피 모드(DODGE_TURN90 -> DODGE_CIRCLE)로 진입한다. (DODGE_TURN90은 상태 이름일 뿐,
#   실제 꺾는 각도는 DODGE_TURN_DEG로 조절)
DODGE_TRIGGER_DIST = THRESH_TURN     # 정면 장애물 판정 임계값 (기존 THRESH_TURN 재사용)
DODGE_REENGAGE_DIST = THRESH_TURN    # 이 이상으로 전방이 열리면 회피 그만하고 WALL_SEARCH 복귀
DODGE_TURN_W        = 1.0            # 제자리 회전 각속도 크기
DODGE_TURN_DEG       = 30             # 1회 꺾는 각도 (요청에 따라 90 -> 30도로 축소)
DODGE_TURN90_SEC    = math.radians(DODGE_TURN_DEG) / DODGE_TURN_W  # w=DODGE_TURN_W로 DODGE_TURN_DEG만큼 도는 데 걸리는 시간
DODGE_CIRCLE_V      = 0.15           # 원을 그리며 전진하는 속도
DODGE_CIRCLE_W      = 0.55           # 원을 그리며 도는 각속도 크기
DODGE_WALL_HIT_DIST = THRESH_STOP    # "벽 인식" 기준: 거의 부딪히기 직전 (THRESH_STOP)

# ── TURN_AROUND (색지 도착 후 유턴) ───────────────────────────────────
# 색지가 맵 가장자리에 붙어 있으면, 도착 시점엔 로봇이 맵 바깥셄을 보고 있을 수 있다.
# PARKING이 끝나면 제자리에서 약 180도 돌려 맵 안쪽을 보게 한 뒤 다음 색지를 탐색한다.
# (오도메트리가 없어 시간 기반으로 각도를 추정 — DODGE_TURN90_SEC과 동일한 방식)
TURN_AROUND_W   = 1.0
TURN_AROUND_SEC = math.pi / TURN_AROUND_W   # 약 180도

# ── 우선순위(P1 > P2 > P3) ──────────────────────────────────────────────
# P0 비상정지   : 전방 < THRESH_STOP → wall_follow()/각 상태 내부에서 즉시 처리. 상태와 무관하게 항상 최우선.
# P1 Watchdog   : (회전 누적 > FULL_LOOP_THRESH) OR (시간 초과 > MISSION_TIMEOUT_SEC) → LOOP_ESCAPE로 강제 전환.
#                 탐색 상태(WALL_SEARCH/APPROACH/FOLLOW)에서만 감시하고, TRACK/FORWARD/PARKING처럼
#                 "진행 중"인 상태에서는 누적을 멈춘다(=watchdog 일시정지) → 목표를 쫓는 중에 오작동 개입 방지.
# P2 Target     : 색지가 카메라에서 DETECT_CONFIRM 프레임 연속 검출되면, 탐색/탈출/회피 상태를 즉시 덮고 TRACK으로 전환.
# P3 Explore    : WALL_SEARCH(제자리 회전 + LiDAR로 "진짜 좁은 틈" 탐색 → 정렬되면 그 방향으로 출발)
#                 → WALL_APPROACH → WALL_FOLLOW (기본 탐색, 항상 실행 가능한 fallback).
#                 틈을 못 찾으면 기존처럼 일정 방향으로 계속 회전하며 탐색을 이어간다.
EXPLORE_STATES = ("WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW")
TARGET_INTERRUPTIBLE_STATES = EXPLORE_STATES + ("LOOP_ESCAPE", "DODGE_TURN90", "DODGE_CIRCLE")

# ── STATE ─────────────────────────────────────────────────────────────
state         = "WALL_SEARCH"   # mode + lidar_state + park_state를 단일 상태로 통합 (중복 로직 제거)
mission_idx   = 0
detect_count  = 0
arrive_count  = 0
follow_side   = "L"   # 초기값일 뿐, WALL_SEARCH -> WALL_APPROACH 전환 시 decide_follow_side() 또는
                       # 갭의 좌/우 벽 중 더 가까운 쪽 기준으로 항상 재결정됨

park_t        = None
mission_start_t = time.time()
last_cmd      = (0.0, 0.0)

escape_t      = None   # LOOP_ESCAPE 진입 시각
loop_accum    = 0.0    # P1 Watchdog: 탐색 상태에서 누적된 |w|*dt (대략적인 "누적 회전각", 단위: rad)
loop_t_prev   = time.time()

was_found     = False  # 색지가 "보이다가 사라지는 순간"을 감지하기 위한 직전 프레임 found 상태
last_seen_x   = 160    # 색지를 마지막으로 본 화면 x좌표 (잃어버렸을 때 follow_side 결정에 사용)

# DODGE 전용 상태 변수
dodge_dir     = "L"    # 색지가 보였던 쪽. "L"이면 (오른쪽으로 DODGE_TURN_DEG 꺾고) 왼쪽으로 원 선회, "R"이면 그 반대
dodge_t       = None   # 현재 DODGE_TURN90/DODGE_CIRCLE 단계 진입 시각
dodge_turn_w  = 0.0    # 이번 DODGE_TURN90 단계에서 사용할 회전 방향(+-DODGE_TURN_W)
last_target_side = "L" # TRACK 중 색지가 마지막으로 있었던 쪽 (DODGE 종료 후 다시 그 방향으로 접근하는 데 사용)
turn_t        = None   # TURN_AROUND(유턴) 진입 시각

print(f"START | MISSION: {MISSION}")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        now_t = time.time()
        dt = now_t - loop_t_prev
        loop_t_prev = now_t

        ret, frame = cap.read()
        if not ret: continue

        frame  = cv2.flip(frame, 1)
        H, W   = frame.shape[:2]
        cx_mid = W // 2
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan   = get_scan()
        fm     = front_min(scan)
        adir   = avoid_dir(scan)

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        cv2.putText(frame, f"TARGET: {target.upper()}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
        cv2.putText(frame, f"STATE:{state} LOOP:{math.degrees(loop_accum):.0f}/{math.degrees(FULL_LOOP_THRESH):.0f}deg",
                    (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # ══ P1: Watchdog (회전 누적 / 타임아웃) ════════════════════════════
        # 탐색 상태에서만 누적하고, TRACK처럼 목표를 향해 "진행 중"일 때는 누적을 멈춘다.
        if state in EXPLORE_STATES:
            loop_accum += abs(last_sent_w) * dt
            if (now_t - mission_start_t > MISSION_TIMEOUT_SEC) or (loop_accum > FULL_LOOP_THRESH):
                state = "LOOP_ESCAPE"
                escape_t = now_t
                loop_accum = 0.0
                detect_count = 0
                cv2.imshow("f", frame); cv2.waitKey(1); continue
        elif state != "LOOP_ESCAPE":
            # TRACK/FORWARD/PARKING/DODGE 등 "진행 중" 상태에서는 watchdog 시계/누적을 계속 리셋해 둔다.
            mission_start_t = now_t
            loop_accum = 0.0

        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big    = max(cnts, key=cv2.contourArea) if cnts else None
        area   = cv2.contourArea(big) if big is not None else 0.0
        found  = big is not None and area > MIN_AREA

        cx_obj, cy_obj = -1, -1
        if found:
            M_mom = cv2.moments(big)
            if M_mom["m00"] > 0:
                cx_obj = int(M_mom["m10"] / M_mom["m00"])
                cy_obj = int(M_mom["m01"] / M_mom["m00"])
            bx, by_top, bw, bh = cv2.boundingRect(big)
            last_seen_x = cx_obj
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # 색지가 "보이다가 사라지는 순간" 감지 -> 그 직전 위치 기준으로 follow_side 즉시 갱신
        if was_found and not found:
            follow_side = "L" if last_seen_x < cx_mid else "R"
        was_found = found

        # ══ P2: Target — 색지 발견 시 탐색/탈출/회피 상태를 즉시 덮고 TRACK으로 ═══════════
        if state in TARGET_INTERRUPTIBLE_STATES:
            if found:
                detect_count += 1
                if detect_count >= DETECT_CONFIRM:
                    detect_count = 0
                    state = "TRACK"
            else:
                detect_count = 0

        # ══ P3 + 상태별 디스패치 ═══════════════════════════════════════════
        if state == "WALL_SEARCH":
            if fm < THRESH_STOP:
                # 회전 중에도 정면이 거의 부딪히기 직전이면 비상 회피가 우선
                send_cmd(0.05, adir * 1.0)
            else:
                gap_angle = find_best_gap(scan)
                cv2.putText(frame, f"GAP:{'-' if gap_angle is None else f'{gap_angle:.0f}deg'}",
                            (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                if gap_angle is not None:
                    err = ((gap_angle + 180) % 360) - 180  # 정면(0) 기준 부호있는 오차, +면 R쪽, -면 L쪽
                    if abs(err) < GAP_ALIGN_TOL_DEG:
                        # 갭이 정면에 정렬됨 -> 그쪽으로 출발. follow_side는 색지가 보이면 그쪽,
                        # 아니면 갭 양쪽 벽 중 더 가까운(=더 타이트하게 붙은) 쪽으로 결정
                        before_d = float(scan[(int(round(gap_angle)) - 1) % 360])
                        after_d  = float(scan[(int(round(gap_angle)) + 1) % 360])
                        follow_side = decide_follow_side(adir, found, cx_obj, cx_mid) if found else \
                                      ("R" if before_d <= after_d else "L")
                        state = "WALL_APPROACH"
                    else:
                        # 오차 부호 방향으로 제자리 회전(같은 속도, 방향만 오차에 맞춤)
                        send_cmd(0.0, WALL_SEARCH_W if err >= 0 else -WALL_SEARCH_W)
                else:
                    # 유효한 갭을 못 찾았으면 기존처럼 한 방향으로 계속 회전하며 탐색
                    send_cmd(0.0, WALL_SEARCH_W)

        elif state == "WALL_APPROACH":
            sd = side_dist(scan, follow_side)
            if sd <= WALL_TARGET * 1.3:
                state = "WALL_FOLLOW"
            else:
                sign = -1 if follow_side == "L" else 1
                if fm < THRESH_STOP: send_cmd(0.08, adir * 1.0)
                elif fm < THRESH_TURN: send_cmd(WALL_APPROACH_V * 0.6, adir * 0.7)
                else: send_cmd(WALL_APPROACH_V, sign * 0.3)

        elif state == "WALL_FOLLOW":
            v, w = wall_follow(scan, fm, adir, follow_side)
            send_cmd(v, w)

        elif state == "LOOP_ESCAPE":
            # 1) 잠시 후진 → 2) 추가 회전으로 새 방향 모색(연장됨) → 3) WALL_SEARCH로 복귀
            elapsed_esc = now_t - escape_t
            if elapsed_esc < LOOP_ESCAPE_SEC:
                send_cmd(LOOP_ESCAPE_BACK_V, 0.0)
            elif elapsed_esc < LOOP_ESCAPE_SEC + LOOP_ESCAPE_TURN_SEC:
                send_cmd(0.0, LOOP_ESCAPE_TURN_W)
            else:
                state = "WALL_SEARCH"
                mission_start_t = now_t
                loop_accum = 0.0

        elif state == "TRACK":
            if found:
                last_target_side = "L" if cx_obj < cx_mid else "R"
                target_angle = cx_to_lidar_angle(cx_obj, cx_mid, W)
                target_clear = obstacle_on_target(scan, target_angle)
                # 정면(±90도, fm 측정 범위) 자체가 막혀 있으면: 색지 방향 각도와 무관하게 무조건 DODGE로 진입.
                # (target_clear는 "색지 방향" 기준이라 정면이 막혔어도 색지가 옆쪽이면 안 걸릴 수 있음.
                #  요청하신 동작은 "로봇과 색지 사이 정면"이므로 fm(정면 최소거리) 기준을 우선 사용.)
                if fm < DODGE_TRIGGER_DIST:
                    # 색지 무시하고 회피 모드 진입. dodge_dir = 색지가 보였던 쪽(L/R)
                    dodge_dir = "L" if cx_obj < cx_mid else "R"
                    # dodge_dir == "L" -> 오른쪽으로 DODGE_TURN_DEG 꺾고(w<0) 왼쪽으로 원 선회
                    # dodge_dir == "R" -> 왼쪽으로 DODGE_TURN_DEG 꺾고(w>0) 오른쪽으로 원 선회
                    dodge_turn_w = -DODGE_TURN_W if dodge_dir == "L" else DODGE_TURN_W
                    state = "DODGE_TURN90"
                    dodge_t = now_t
                    arrive_count = 0
                    detect_count = 0
                    cv2.imshow("f", frame); cv2.waitKey(1); continue
                elif target_clear < TARGET_CLEAR_DIST:
                    # 색지 방향이 비스듬히 막혀있음 (정면은 열려있음) -> 기존 동작: 벽 우회 탐색으로 전환
                    arrive_count = 0
                    follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                    state = "WALL_APPROACH"
                    cv2.imshow("f", frame); cv2.waitKey(1); continue
            else:
                # 색지를 완전히 놓침 (cx_obj=-1인 채로 조향 계산을 계속하면 한쪽으로 발산하는 조향이 나옴).
                # 마지막으로 봤던 쪽(last_seen_x) 기준으로 follow_side를 잡고 벽 추종 접근으로 전환해서
                # 다시 시야에 들어올 기회를 만든다. (DETECT_CONFIRM 프레임 연속 검출되면 P2가 다시 TRACK으로 덮음)
                arrive_count = 0
                detect_count = 0
                follow_side = "L" if last_seen_x < cx_mid else "R"
                state = "WALL_APPROACH"
                cv2.imshow("f", frame); cv2.waitKey(1); continue

            if found and area > BIG_TARGET_AREA:
                # 색지가 화면을 거의 가릴 만큼 가까움 -> arrive zone 정렬 확인 없이 곧장 도착 처리
                # (바로 앞에서 좌우로 흔들리며 arrive_count가 못 차는 것을 방지)
                arrive_count = ARRIVE_CONFIRM
            else:
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

            if arrive_count >= ARRIVE_CONFIRM:
                arrive_count = 0
                state = "FORWARD"
                park_t = now_t
                send_cmd(ARRIVE_FORWARD_V, 0.0)
            else:
                err_x = cx_obj - cx_mid
                err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                reduced_v = APPROACH_V * (1.0 - err_ratio)

                def cam_w(ex):
                    raw = -KP_ROT * ex
                    if abs(raw) < W_MIN and ex != 0: return -W_MIN if ex > 0 else W_MIN
                    return raw

                if fm >= THRESH_SLOW: v, w = reduced_v, cam_w(err_x)
                else:
                    w_cam = cam_w(err_x); w_lid = adir * 0.7
                    if fm < THRESH_STOP: v, w = 0.09, w_lid
                    elif fm < THRESH_TURN: v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                    else: v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                last_cmd = (v, w); send_cmd(v, w)

        elif state == "DODGE_TURN90":
            # 정지 상태에서 제자리로 DODGE_TURN_DEG만큼 회전 (v=0, dodge_turn_w 방향)
            # 회전 도중에도 전방이 충분히 열리면(=장애물이 옆으로 비껴갔거나 사라짐) 바로 탐색 복귀
            if fm >= DODGE_REENGAGE_DIST:
                follow_side = last_target_side
                state = "WALL_APPROACH"
            elif now_t - dodge_t >= DODGE_TURN90_SEC:
                state = "DODGE_CIRCLE"
                dodge_t = now_t
            else:
                send_cmd(0.0, dodge_turn_w)

        elif state == "DODGE_CIRCLE":
            # dodge_dir 방향으로 원을 그리며 전진하다가,
            # (a) 전방이 다시 열리면(장애물 비껴감/소실) -> WALL_SEARCH로 정상 복귀
            # (b) 벽을 거의 부딪히기 직전까지 인식하면(fm < DODGE_WALL_HIT_DIST)
            #     -> 반대 방향으로 DODGE_TURN_DEG 꺾었다가 다시 원래 dodge_dir로 원 선회 반복
            if fm >= DODGE_REENGAGE_DIST:
                follow_side = last_target_side
                state = "WALL_APPROACH"
            elif fm < DODGE_WALL_HIT_DIST:
                # 반대 방향으로 DODGE_TURN_DEG 꺾기 (원래 회전 방향의 부호를 뒤집음)
                dodge_turn_w = -dodge_turn_w
                state = "DODGE_TURN90"
                dodge_t = now_t
            else:
                circle_w = -DODGE_CIRCLE_W if dodge_dir == "L" else DODGE_CIRCLE_W
                send_cmd(DODGE_CIRCLE_V, circle_w)

        elif state == "FORWARD":
            elapsed = now_t - park_t
            if elapsed >= ARRIVE_FORWARD_SEC:
                stop_robot()
                state = "PARKING"
                park_t = now_t
            else:
                send_cmd(*last_cmd)

        elif state == "PARKING":
            stop_robot()
            if now_t - park_t >= PARK_SEC:
                mission_idx += 1
                arrive_count = 0
                detect_count = 0
                if mission_idx < len(MISSION):
                    # 색지가 맵 가장자리에 있을 수 있으니, 다음 색지를 찾으러 가기 전에
                    # 먼저 제자리에서 유턴해 맵 안쪽을 보도록 한다.
                    state = "TURN_AROUND"
                    turn_t = now_t

        elif state == "TURN_AROUND":
            # 시간 기반으로 약 180도 회전. 도중에 새 색지가 보여도(TARGET_INTERRUPTIBLE_STATES에
            # 포함 안 함) 끝까지 돌려 맵 안쪽을 보게 만든 뒤 탐색을 시작한다.
            if now_t - turn_t >= TURN_AROUND_SEC:
                state = "WALL_SEARCH"
                mission_start_t = now_t
                loop_accum = 0.0
            else:
                send_cmd(0.0, TURN_AROUND_W)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
