import cv2
import serial
import numpy as np
import time
import math
import threading
from collections import deque

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

# ── LIDAR 파라미터 ────────────────────────────────────────────────────
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 45
THRESH_SLOW = 55.0
THRESH_TURN = 35.0
THRESH_STOP = 18.0

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
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

# ── 점유 격자 맵 (Occupancy Grid) ─────────────────────────────────────
# 2cm/셀, 200×200 셀 → 4m×4m 공간 표현 (로봇 시작점이 중앙)
CELL_SIZE   = 0.02          # 셀 크기 (m)
GRID_W      = 200           # 격자 가로 셀 수 (4m)
GRID_H      = 200           # 격자 세로 셀 수 (4m)
GRID_ORIGIN = (100, 100)    # 로봇 시작점 = 격자 중앙 (셀 인덱스)

# 셀 값: 128=unknown, 0=free, 255=occupied
grid      = np.full((GRID_H, GRID_W), 128, dtype=np.uint8)
grid_lock = threading.Lock()

# 로그 오즈 기반 업데이트용 버퍼
log_odds  = np.zeros((GRID_H, GRID_W), dtype=np.float32)
L_OCC     =  0.85   # 점유 로그 오즈 증분
L_FREE    = -0.40   # 자유 로그 오즈 증분
L_MAX     =  5.0
L_MIN     = -5.0

def world_to_grid(wx, wy):
    """월드 좌표(m) → 격자 인덱스"""
    gx = int(GRID_ORIGIN[0] + wx / CELL_SIZE)
    gy = int(GRID_ORIGIN[1] - wy / CELL_SIZE)   # y축 반전 (위쪽이 +y)
    return gx, gy

def bresenham(x0, y0, x1, y1):
    """두 셀 사이 경로의 셀 목록 (Bresenham)"""
    cells = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1: break
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 <  dx: err += dx; y0 += sy
    return cells

def update_map(robot_x, robot_y, robot_th, scan):
    """현재 위치에서의 스캔으로 맵 업데이트"""
    rx, ry = world_to_grid(robot_x, robot_y)
    if not (0 <= rx < GRID_W and 0 <= ry < GRID_H):
        return

    angles_rad = np.deg2rad(np.arange(360))
    with grid_lock:
        for a_deg in range(0, 360, 2):   # 2도 간격으로 처리 (속도 절충)
            d = scan[a_deg]
            if d <= 0 or d >= 149:
                continue
            # 글로벌 각도 = 로봇 방향 + 스캔 각도
            global_angle = robot_th + math.radians(a_deg)
            ex = robot_x + (d / 100.0) * math.cos(global_angle)
            ey = robot_y + (d / 100.0) * math.sin(global_angle)
            ex_g, ey_g = world_to_grid(ex, ey)

            # 레이 경로 = free
            ray = bresenham(rx, ry, ex_g, ey_g)
            for (cx, cy) in ray[:-1]:
                if 0 <= cx < GRID_W and 0 <= cy < GRID_H:
                    log_odds[cy, cx] = max(L_MIN, log_odds[cy, cx] + L_FREE)

            # 끝점 = occupied
            if 0 <= ex_g < GRID_W and 0 <= ey_g < GRID_H:
                log_odds[ey_g, ex_g] = min(L_MAX, log_odds[ey_g, ex_g] + L_OCC)

        # 로그 오즈 → 그레이스케일 변환
        occ = (log_odds > 0.5).astype(np.uint8) * 255
        free = (log_odds < -0.3).astype(np.uint8) * 128
        grid[:] = 128
        grid[log_odds < -0.3] = 0
        grid[log_odds >  0.5] = 255

def get_grid_copy():
    with grid_lock: return grid.copy()

# 바운더리 체크: 로봇 주변 셀이 occupied인지 확인
WALL_CHECK_CM  = 25     # 이 거리 안에 occupied 셀이 있으면 벽으로 판단
WALL_CHECK_ANG = 30     # 전방 ±각도 안에서만 체크

def near_wall_in_dir(robot_x, robot_y, robot_th, direction_th, check_cm=WALL_CHECK_CM):
    """direction_th 방향 check_cm 이내에 occupied 셀이 있으면 True"""
    with grid_lock:
        for d_cm in range(5, check_cm, 4):
            ex = robot_x + (d_cm / 100.0) * math.cos(direction_th)
            ey = robot_y + (d_cm / 100.0) * math.sin(direction_th)
            gx, gy = world_to_grid(ex, ey)
            if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                if grid[gy, gx] == 255:
                    return True
    return False

def map_boundary_turn(robot_x, robot_y, robot_th, scan):
    """맵 기반 경계 판단: 전방/좌/우 벽 근접 시 회전 방향 반환. 없으면 None"""
    fm = front_min(scan)
    # 전방 장애물 우선 (라이다 직접)
    if fm < THRESH_STOP:
        return avoid_dir(scan)
    # 맵에서 전방 벽 확인
    if near_wall_in_dir(robot_x, robot_y, robot_th, robot_th, check_cm=30):
        return avoid_dir(scan)
    # 좌/우 벽 근접 → 반대 방향으로 살짝 틀기
    left_wall  = near_wall_in_dir(robot_x, robot_y, robot_th, robot_th + math.pi/2, check_cm=20)
    right_wall = near_wall_in_dir(robot_x, robot_y, robot_th, robot_th - math.pi/2, check_cm=20)
    if left_wall and not right_wall:
        return -1   # 오른쪽으로
    if right_wall and not left_wall:
        return 1    # 왼쪽으로
    return None

# ── 스캔 매칭 (ICP-lite, point-to-point) ─────────────────────────────
# 이전 스캔의 XY 포인트를 저장해 두고, 다음 스캔과 비교해 Δpose 추출
prev_points      = None
MATCH_MAX_DIST   = 0.20   # 매칭 허용 최대 거리 (m)
MATCH_MIN_PTS    = 30     # 최소 매칭 포인트 수
MATCH_SKIP_DEG   = 3      # 몇 도 간격으로 샘플링할지

def scan_to_points(scan, robot_x, robot_y, robot_th):
    """스캔 배열 → 글로벌 XY 포인트 클라우드 (numpy Nx2)"""
    pts = []
    for a in range(0, 360, MATCH_SKIP_DEG):
        d = scan[a]
        if 3 < d < 148:
            angle = robot_th + math.radians(a)
            x = robot_x + (d / 100.0) * math.cos(angle)
            y = robot_y + (d / 100.0) * math.sin(angle)
            pts.append((x, y))
    return np.array(pts, dtype=np.float32) if pts else None

def icp_once(src, dst):
    """
    단순 최근접점 ICP 1회 반복.
    src(현재 스캔 포인트) → dst(이전 스캔 포인트)에 맞게
    최적 회전(dth)과 이동(dx, dy) 반환.
    """
    if src is None or dst is None or len(src) < MATCH_MIN_PTS or len(dst) < MATCH_MIN_PTS:
        return 0.0, 0.0, 0.0

    # 각 src 포인트에서 dst 최근접점 탐색 (brute-force, N이 작아서 충분)
    matched_src = []
    matched_dst = []
    for p in src:
        dists = np.linalg.norm(dst - p, axis=1)
        idx   = np.argmin(dists)
        if dists[idx] < MATCH_MAX_DIST:
            matched_src.append(p)
            matched_dst.append(dst[idx])

    if len(matched_src) < MATCH_MIN_PTS:
        return 0.0, 0.0, 0.0

    ms = np.array(matched_src)
    md = np.array(matched_dst)

    # SVD로 최적 회전/이동 계산
    cs = ms.mean(axis=0)
    cd = md.mean(axis=0)
    hs = ms - cs
    hd = md - cd
    H  = hs.T @ hd
    U, _, Vt = np.linalg.svd(H)
    R  = Vt.T @ U.T
    # 회전각 추출
    dth = math.atan2(R[1, 0], R[0, 0])
    # 이동
    t   = cd - R @ cs
    dx, dy = float(t[0]), float(t[1])

    # 너무 큰 점프는 무시 (튀는 값 방어)
    if abs(dx) > 0.15 or abs(dy) > 0.15 or abs(dth) > 0.3:
        return 0.0, 0.0, 0.0

    return dx, dy, dth

# ── 위치 추정 (SLAM Pose) ─────────────────────────────────────────────
SLAM_X  = 0.0
SLAM_Y  = 0.0
SLAM_TH = 0.0
slam_lock = threading.Lock()

# 오도메트리 (ICP 실패 시 폴백 + ICP 초기값)
ODOM_X   = 0.0
ODOM_Y   = 0.0
ODOM_TH  = 0.0
last_odom_t = time.time()

def update_odom(v, w):
    global ODOM_X, ODOM_Y, ODOM_TH, last_odom_t
    now = time.time()
    dt  = now - last_odom_t
    last_odom_t = now
    ODOM_X  += v * math.cos(ODOM_TH) * dt
    ODOM_Y  += v * math.sin(ODOM_TH) * dt
    ODOM_TH += w * dt

# 스캔 매칭 주기 제어
SCAN_MATCH_PERIOD = 0.25   # 초
last_match_t      = time.time()

def try_scan_match(scan):
    """
    스캔 매칭을 시도해 SLAM 위치를 보정.
    ICP가 실패하면 오도메트리 값을 그대로 사용.
    """
    global prev_points, SLAM_X, SLAM_Y, SLAM_TH, last_match_t

    now = time.time()
    if now - last_match_t < SCAN_MATCH_PERIOD:
        return
    last_match_t = now

    with slam_lock:
        sx, sy, sth = SLAM_X, SLAM_Y, SLAM_TH

    # 현재 스캔 → 글로벌 포인트 (현재 추정 위치 기준)
    cur_pts = scan_to_points(scan, sx, sy, sth)

    if prev_points is not None and cur_pts is not None:
        dx, dy, dth = icp_once(cur_pts, prev_points)
        # ICP 보정이 유의미하면 적용, 아니면 오도메트리 델타 사용
        if abs(dx) + abs(dy) + abs(dth) > 1e-6:
            with slam_lock:
                SLAM_X  = sx + dx
                SLAM_Y  = sy + dy
                SLAM_TH = sx + dth   # 각도 누적
        else:
            # 오도메트리 델타 반영
            with slam_lock:
                SLAM_X  = ODOM_X
                SLAM_Y  = ODOM_Y
                SLAM_TH = ODOM_TH
    else:
        # 첫 스캔이거나 포인트 부족 → 오도메트리
        with slam_lock:
            SLAM_X  = ODOM_X
            SLAM_Y  = ODOM_Y
            SLAM_TH = ODOM_TH

    prev_points = cur_pts

    # 맵 비동기 업데이트 (스레드로 돌려서 메인루프 블로킹 방지)
    with slam_lock:
        rx, ry, rth = SLAM_X, SLAM_Y, SLAM_TH
    threading.Thread(target=update_map, args=(rx, ry, rth, scan.copy()), daemon=True).start()

def get_pose():
    with slam_lock: return SLAM_X, SLAM_Y, SLAM_TH

# ── 모터 ──────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    update_odom(v, w)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

last_odom_t = time.time()

# ── 맵 시각화 ─────────────────────────────────────────────────────────
MAP_VIS_SIZE = 200   # 화면에 표시할 맵 크기 (픽셀)

def render_map(robot_x, robot_y, robot_th):
    """점유 격자를 컬러 이미지로 변환 + 로봇 위치 표시"""
    g = get_grid_copy()
    vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    vis[g == 128] = (80, 80, 80)      # unknown = 회색
    vis[g == 0]   = (200, 200, 200)   # free = 밝은 회색
    vis[g == 255] = (0, 0, 200)       # occupied = 파랑

    # 로봇 위치 표시
    rx, ry = world_to_grid(robot_x, robot_y)
    if 0 <= rx < GRID_W and 0 <= ry < GRID_H:
        cv2.circle(vis, (rx, ry), 3, (0, 255, 0), -1)
        # 방향 화살표
        ex = int(rx + 8 * math.cos(robot_th))
        ey = int(ry - 8 * math.sin(robot_th))
        cv2.arrowedLine(vis, (rx, ry), (ex, ey), (0, 255, 255), 1, tipLength=0.4)

    # 2x2m 바운더리 표시
    b_x1, b_y1 = world_to_grid(-1.0,  2.0)
    b_x2, b_y2 = world_to_grid( 1.0, -0.1)
    cv2.rectangle(vis, (b_x1, b_y1), (b_x2, b_y2), (0, 200, 100), 1)

    return cv2.resize(vis, (MAP_VIS_SIZE, MAP_VIS_SIZE), interpolation=cv2.INTER_NEAREST)

# ── 색상 설정 ─────────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 175], [179, 207, 255]),
               "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 48, 193], [45, 170, 255]),
               "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 123], [138, 207, 246]),
               "hsv2": None,
               "bgr":  ([40,  0,   0], [255, 220, 220]), "draw": (255, 80, 0)},
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

# ── 파라미터 ──────────────────────────────────────────────────────────
MIN_AREA        = 400
KP_ROT          = 0.030
W_MIN           = 0.25
APPROACH_V      = 0.17
PARK_SEC        = 1.2
DETECT_CONFIRM  = 6

ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 40
ARRIVE_FORWARD_SEC = 0.7
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8

# ── 상태 ──────────────────────────────────────────────────────────────
mode         = "LIDAR"
mission_idx  = 0
detect_count = 0
arrive_count = 0

park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
last_cmd      = (0.0, 0.0)

print(f"START | MISSION: {MISSION}")

# ── 메인 루프 ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame  = cv2.flip(frame, 1)
        H, W   = frame.shape[:2]
        cx_mid = W // 2
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan   = get_scan()
        fm     = front_min(scan)
        adir   = avoid_dir(scan)

        # 스캔 매칭 + 맵 업데이트 (주기적)
        try_scan_match(scan)
        robot_x, robot_y, robot_th = get_pose()

        # 맵 시각화 오버레이 (카메라 프레임 옆에 표시)
        map_vis = render_map(robot_x, robot_y, robot_th)
        # 카메라 프레임과 맵을 가로로 합치기
        cam_h, cam_w = frame.shape[:2]
        map_resized = cv2.resize(map_vis, (MAP_VIS_SIZE, cam_h))
        display = np.hstack([frame, map_resized])

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(display, "ALL MISSIONS DONE", (30, cam_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("robot | map", display); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big   = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        cx_obj, cy_obj = -1, -1
        if found:
            M_mom = cv2.moments(big)
            if M_mom["m00"] > 0:
                cx_obj = int(M_mom["m10"] / M_mom["m00"])
                cy_obj = int(M_mom["m01"] / M_mom["m00"])

            bx, by_top, bw, bh = cv2.boundingRect(big)
            last_seen_x   = cx_obj
            last_bottom_y = min(by_top + bh, 239)

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (arrive_x1 <= cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # ══ LIDAR 모드 ═══════════════════════════════════════════════
        if mode == "LIDAR":
            if found: detect_count += 1
            else:     detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                print(f"[{target}] 발견 → 추적 시작")
                continue

            # ── 맵 기반 경계/장애물 판단 ─────────────────────────────
            bt = map_boundary_turn(robot_x, robot_y, robot_th, scan)

            if bt is not None:
                v, w = 0.10, bt * 0.9
            elif fm < THRESH_STOP: v, w = 0.09, adir * 0.9
            elif fm < THRESH_TURN: v, w = 0.13, adir * 0.7
            elif fm < THRESH_SLOW: v, w = 0.18, adir * 0.4
            else:                  v, w = 0.28, 0.0

            send_cmd(v, w)

            pos_txt = f"({robot_x:.2f},{robot_y:.2f}) {math.degrees(robot_th):.0f}d"
            cv2.putText(frame, "SLAM-LIDAR", (10, 20), 0, 0.45, (255, 255, 255), 1)
            cv2.putText(frame, pos_txt,       (10, 36), 0, 0.40, (200, 255, 200), 1)

        # ══ PARK 모드 ════════════════════════════════════════════════
        elif mode == "PARK":

            # 1. 전진 중
            if park_state == "FORWARD":
                elapsed = time.time() - park_t
                if elapsed >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t = time.time()
                    print(f"[{target}] 전진 완료 → 정차")
                else:
                    send_cmd(*last_cmd)
                cv2.putText(frame, f"FORWARD: {target}", (10, 25), 0, 0.6, draw, 2)

            # 2. 정차 중
            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    if mission_idx < len(MISSION):
                        mode = "LIDAR"
                        detect_count = 0
                        arrive_count = 0
                        print(f"다음 미션 [{MISSION[mission_idx]}] → LIDAR 모드 복귀")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # 3. 객체 추적
            elif found:
                park_state = "TRACK"

                if centroid_in_arrive_zone():
                    arrive_count += 1
                else:
                    arrive_count = 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    print(f"[{target}] 도착 확정 → {ARRIVE_FORWARD_SEC}s 전진")
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                    continue
                else:
                    err_x = cx_obj - cx_mid
                    def cam_w(ex):
                        raw = -KP_ROT * ex
                        if abs(raw) < W_MIN and ex != 0:
                            return -W_MIN if ex > 0 else W_MIN
                        return raw

                    if fm >= THRESH_SLOW:
                        v = APPROACH_V; w = cam_w(err_x)
                    else:
                        w_cam = cam_w(err_x); w_lid = adir * 0.7
                        if fm < THRESH_STOP:
                            v, w = 0.09, w_lid
                        elif fm < THRESH_TURN:
                            v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                        else:
                            v, w = 0.18, 0.3 * w_lid + 0.7 * w_cam

                    last_cmd = (v, w)
                    send_cmd(v, w)

                cv2.putText(frame, f"TRACK: {target}", (10, 25), 0, 0.6, draw, 1)
                cv2.putText(frame, f"arr:{arrive_count}/{ARRIVE_CONFIRM}", (10, 42), 0, 0.4, draw, 1)

            # 4. 놓침 → 제자리 회전 탐색
            else:
                park_state = "SEARCH"
                w = -1.0 if last_seen_x > cx_mid else 1.0
                send_cmd(0.0, w)
                cv2.putText(frame, f"SEARCH: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        # 맵을 display에 반영 (frame은 이미 그림이 그려진 상태)
        display[:, :cam_w] = frame
        cv2.imshow("robot | map", display)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
