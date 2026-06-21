import cv2
import serial
import numpy as np
import time
import threading
from math import acos, pi

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
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 90
THRESH_SLOW = 55.0
THRESH_TURN = 30.0
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


# ══════════════════════════════════════════════════════════════════════
#  벽 추종 — 5번(Wall_following.py) 방식 그대로 적용
#
#  교체 범위: WALL_FOLLOW 뿐 아니라 기존의 WALL_SEARCH / WALL_APPROACH
#  단계도 모두 이 방식으로 대체한다. 5번 알고리즘은 judge_distance()의
#  "forward"(벽 없음 → 그냥 직진 = 기존 WALL_SEARCH 역할)와
#  "far"(벽이 멀어서 다가가야 함 = 기존 WALL_APPROACH 역할),
#  "close"/"maintaining"(기존 WALL_FOLLOW 역할)을 한 번의 step()
#  호출 안에서 전부 판단하므로, 별도의 SEARCH/APPROACH 상태가
#  더 이상 필요 없다. 따라서 세 상태를 모두 단일 "WALL_FOLLOW"
#  상태 하나로 합치고, 그 안에서 WallFollower.step()만 호출한다.
# ══════════════════════════════════════════════════════════════════════

LEFT  = "LEFT"
RIGHT = "RIGHT"

class WallFollower:
    """
    5번(Wall_following.py)의 클래스 구조·파라미터·알고리즘을 그대로 유지.
    ROS msg 대신 numpy scan 배열(360, cm 단위)을 직접 받는다.

    scan 배열 인덱스 = 각도(°):
      0   = 전방
      90  = 왼쪽
      180 = 후방
      270 = 오른쪽
    """

    default_speed = 0.15
    default_angle = 0.2
    scan_dist     = 50.0   # cm (원본 0.5 m)
    offset        = 20.0   # cm (원본 0.2 m)
    DEFAULT_THETA = 0.28   # rad

    def __init__(self, direction: str = LEFT):
        self.DIRECTION = direction
        self.condition = None
        self.speed     = 0.0
        self.angle     = 0.0

        self.distance            = []
        self.obstacle_data_range = []
        self.obstacle_data_idx   = []

    def LiDAR_scan(self, scan: np.ndarray):
        for deg in range(0, 91):
            d = float(scan[deg])
            if 0 < d <= self.scan_dist:
                self.distance.append(d)
        for deg in range(270, 360):
            d = float(scan[deg])
            if 0 < d <= self.scan_dist:
                self.distance.append(d)

        for deg in range(0, 6):
            d = float(scan[deg])
            if 0 < d < self.scan_dist:
                self.obstacle_data_idx.append(deg)
                self.obstacle_data_range.append(d)
        for deg in range(355, 360):
            d = float(scan[deg])
            if 0 < d < self.scan_dist:
                self.obstacle_data_idx.append(deg)
                self.obstacle_data_range.append(d)

    def judge_distance(self):
        if len(self.obstacle_data_range) == 0:
            self.condition = "forward"
            self.speed     = self.default_speed
            if len(self.distance) > 0:
                if min(self.distance) < self.scan_dist - self.offset:
                    self.condition = "close"
                elif (self.scan_dist - self.offset
                      <= max(self.distance)
                      <= self.scan_dist + self.offset):
                    self.condition = "maintaining"
            else:
                self.condition = "far"
        else:
            self.condition = "obstacle"

    def angle_distance(self, scan: np.ndarray, degree: int):
        for delta in range(0, 2):
            idx = (degree + delta) % 360
            d = float(scan[idx])
            if 0 < d < 60.0:
                return d
        return None

    def maintain_direction(self, scan: np.ndarray):
        if self.DIRECTION == LEFT:
            angle1 = self.angle_distance(scan,  70)
            angle2 = self.angle_distance(scan,  80)
        else:
            angle1 = self.angle_distance(scan, -70 % 360)
            angle2 = self.angle_distance(scan, -80 % 360)

        if angle1 is None or angle2 is None:
            return

        try:
            ratio = angle2 / angle1
            ratio = max(-1.0, min(1.0, ratio))
            theta  = acos(ratio)
            thetad = theta * 180 / pi
            print(f"maintain θ={thetad:.1f}°")

            if 0 < thetad < 20:
                print("세타가 작습니다")
                if self.DIRECTION == LEFT:
                    self.angle = theta - self.DEFAULT_THETA
                else:
                    self.angle = -(theta - self.DEFAULT_THETA)
                self.speed = self.default_speed

            elif thetad >= 20:
                print("세타가 큽니다.")
                if self.DIRECTION == LEFT:
                    self.angle = -(theta - self.DEFAULT_THETA)
                else:
                    self.angle = theta - self.DEFAULT_THETA
                self.speed = self.default_speed

        except ValueError:
            pass

    def obstacle_motion(self):
        print("obstacle_motion")
        angle_incre        = len(self.obstacle_data_idx) / 10 * pi / 180
        obstacle_end_point = self.obstacle_data_idx[-1]
        blank_space        = len(self.obstacle_data_idx) - obstacle_end_point
        turn_angle         = angle_incre * blank_space / 2

        if self.DIRECTION == LEFT:
            self.angle = turn_angle
        else:
            self.angle = -turn_angle
        self.speed = -turn_angle / 10

    def move_control(self, scan: np.ndarray):
        if self.condition == "forward":
            self.speed = self.default_speed
            self.angle = 0.0

        elif self.condition == "close":
            print("벽과의 거리가 너무 가깝습니다.")
            if self.DIRECTION == LEFT:
                self.angle = -self.default_angle / (min(self.distance) * 10)
            else:
                self.angle =  self.default_angle / (min(self.distance) * 10)

        elif self.condition == "far":
            print("벽과의 거리가 너무 멉니다.")
            if self.DIRECTION == LEFT:
                self.angle =  self.default_angle * 2
            else:
                self.angle = -self.default_angle * 2

        elif self.condition == "maintaining":
            self.maintain_direction(scan)

        elif self.condition == "obstacle":
            self.obstacle_motion()

        self.obstacle_data_idx   = []
        self.obstacle_data_range = []
        self.distance            = []

    def step(self, scan: np.ndarray):
        self.LiDAR_scan(scan)
        self.judge_distance()
        self.move_control(scan)
        return self.speed, self.angle


wall_follower = WallFollower(direction=LEFT)

def pick_wall_side(scan: np.ndarray, current_side: str) -> str:
    """
    좌/우 어느 쪽에 벽이 있는지 매 프레임 판단해서 따라갈 방향을 정한다.
    (한쪽으로 고정하지 않고, 왼쪽 벽이든 오른쪽 벽이든 있는 쪽을 따라가도록)

      - 왼쪽(0~90°)에만 벽이 있으면 → "L"
      - 오른쪽(270~359°)에만 벽이 있으면 → "R"
      - 양쪽 다 있으면(복도) → 현재 따라가던 쪽 유지 (좌우 떨림 방지)
      - 양쪽 다 없으면 → 현재 방향 유지 (다음 벽 나올 때까지 직진)
    """
    d = WallFollower.scan_dist
    left_has  = np.any((scan[0:91]   > 0) & (scan[0:91]   <= d))
    right_has = np.any((scan[270:360] > 0) & (scan[270:360] <= d))

    if left_has and not right_has:
        return "L"
    if right_has and not left_has:
        return "R"
    return current_side


# ── 복도 중앙 주행 (양쪽 벽이 동시에 있을 때) ─────────────────────────
#
# 좌/우 정측면(90°, 270°) 기준 ±CENTER_BAND° 안에서 가장 가까운 거리를
# 각각 left_d / right_d로 구해서, 둘 다 유효하면(=양쪽 다 벽) 그 차이를
# 0으로 만들도록(=정중앙) 조향한다. 한쪽 벽만 있을 때는 기존
# pick_wall_side + WallFollower 로직을 그대로 사용한다.

CENTER_BAND    = 25     # 좌/우 정측면 기준 ± 범위(°)
CENTER_KP      = 0.018  # 중앙 정렬 게인 (좌우 거리차 → 조향)
CENTER_V       = 0.18   # 중앙 주행 기본 속도
CENTER_TURN_V  = 0.10   # 전방 장애물 근접 시 속도
CENTER_MAX_W   = 0.9

def corridor_distances(scan: np.ndarray):
    """좌측 정측면(90°)·우측 정측면(270°) 기준 ±CENTER_BAND° 최소거리."""
    left_idx  = np.arange(90  - CENTER_BAND, 90  + CENTER_BAND + 1) % 360
    right_idx = np.arange(270 - CENTER_BAND, 270 + CENTER_BAND + 1) % 360
    left_d  = float(np.min(scan[left_idx]))
    right_d = float(np.min(scan[right_idx]))
    return left_d, right_d

def corridor_has_both(left_d: float, right_d: float) -> bool:
    d = WallFollower.scan_dist
    return (0 < left_d <= d) and (0 < right_d <= d)

def corridor_center_cmd(scan, fm, adir, left_d, right_d):
    """
    err = left_d - right_d
      > 0 : 왼쪽이 더 멀다 = 오른쪽 벽에 더 붙어있다 → 왼쪽(+w)으로 조향
      < 0 : 오른쪽이 더 멀다 = 왼쪽 벽에 더 붙어있다 → 오른쪽(-w)으로 조향
    """
    err   = left_d - right_d
    w_raw = CENTER_KP * err

    if fm < THRESH_STOP:
        v, w = 0.08, adir * 1.1
    elif fm < THRESH_TURN:
        v, w = CENTER_TURN_V, adir * 0.85
    elif fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1.0 - blend) * w_raw + blend * adir * 0.5
        v = CENTER_V * (1.0 - 0.4 * blend)
    else:
        v, w = CENTER_V, w_raw

    w = float(np.clip(w, -CENTER_MAX_W, CENTER_MAX_W))
    return v, w


# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]), "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]),    "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 95], [138, 207, 246]),   "hsv2": None,
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

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA       = 400
KP_ROT         = 0.030
W_MIN          = 0.20
APPROACH_V     = 0.13
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8

HOP_MOVE_V          = 0.22   # SAFE_HOP 전진 속도 (구 WALL_V)
MISSION_TIMEOUT_SEC  = 10.0

# ── STATE ─────────────────────────────────────────────────────────────
mode            = "LIDAR"
mission_idx     = 0
detect_count    = 0
arrive_count    = 0
follow_side     = "L"   # "L" → WallFollower LEFT, "R" → RIGHT

park_state      = "TRACK"
last_seen_x     = 160
last_bottom_y   = 0
park_t          = None
search_t        = None
mission_start_t = time.time()
hop_start_t     = None
last_cmd        = (0.0, 0.0)

# 디버그용
dbg_condition = "-"
dbg_v, dbg_w  = 0.0, 0.0
dbg_left_d, dbg_right_d = 0.0, 0.0

print(f"START | MISSION: {MISSION}")

def sync_follower_direction():
    wall_follower.DIRECTION = LEFT if follow_side == "L" else RIGHT

# ── MAIN LOOP ─────────────────────────────────────────────────────────
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

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        # ── 10초 타임아웃 감시 ────────────────────────────────────────
        is_searching = (mode == "LIDAR") or (
            mode == "PARK" and park_state in ["WALL_FOLLOW", "SEARCH"])

        if is_searching:
            if time.time() - mission_start_t > MISSION_TIMEOUT_SEC:
                print(f"🚨 [{target}] {MISSION_TIMEOUT_SEC}초 경과! 도약합니다.")
                mode        = "PARK"
                park_state  = "SAFE_HOP"
                hop_start_t = time.time()
                detect_count = 0
                continue
        elif park_state not in ["SAFE_HOP"]:
            mission_start_t = time.time()

        # ── 색상 검출 ─────────────────────────────────────────────────
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

        # ══ LIDAR 모드 ════════════════════════════════════════════════
        # (구 WALL_SEARCH / WALL_APPROACH / WALL_FOLLOW 세 단계를
        #  5번 방식 WallFollower.step() 하나로 전부 대체)
        if mode == "LIDAR":
            if found: detect_count += 1
            else:     detect_count  = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode       = "PARK"
                park_state = "TRACK"
                continue

            left_d, right_d = corridor_distances(scan)
            if corridor_has_both(left_d, right_d):
                dbg_v, dbg_w = corridor_center_cmd(scan, fm, adir, left_d, right_d)
                dbg_condition = "centering"
            else:
                follow_side = pick_wall_side(scan, follow_side)
                sync_follower_direction()
                dbg_v, dbg_w = wall_follower.step(scan)
                dbg_condition = wall_follower.condition
            dbg_left_d, dbg_right_d = left_d, right_d
            send_cmd(dbg_v, dbg_w)

        # ══ PARK 모드 ═════════════════════════════════════════════════
        elif mode == "PARK":

            # ── SAFE_HOP ─────────────────────────────────────────────
            if park_state == "SAFE_HOP":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        print(f"[{target}] 도약 중 타겟 발견!")
                        detect_count    = 0
                        park_state      = "TRACK"
                        mission_start_t = time.time()
                        continue
                else:
                    detect_count = 0

                elapsed_hop = time.time() - hop_start_t
                if elapsed_hop < 2.0:
                    send_cmd(0.0, 1.2)
                    cv2.putText(frame, "HOP: TURNING", (10, 45), 0, 0.5, (0, 0, 255), 2)
                else:
                    if fm > 130.0:
                        send_cmd(0.0, 1.0)
                        cv2.putText(frame, "HOP: SCANNING", (10, 45), 0, 0.5, (0, 150, 255), 2)
                    elif fm > 50.0:
                        send_cmd(HOP_MOVE_V, 0.0)
                        cv2.putText(frame, f"HOP: MOVING ({fm:.0f}cm)", (10, 45), 0, 0.5, (0, 255, 0), 2)
                    else:
                        print("   → 새 장애물 도착! 벽 추종 재시작")
                        park_state      = "WALL_FOLLOW"
                        mission_start_t = time.time()
                        continue

                cv2.imshow("f", frame); cv2.waitKey(1); continue

            # ── FORWARD ──────────────────────────────────────────────
            if park_state == "FORWARD":
                if time.time() - park_t >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t     = time.time()
                else:
                    send_cmd(*last_cmd)

            # ── PARKING ──────────────────────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                if time.time() - park_t >= PARK_SEC:
                    mission_idx  += 1
                    arrive_count  = 0
                    detect_count  = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_FOLLOW"
                    continue

            # ── WALL_FOLLOW (5번 방식, search/approach 통합) ──────────
            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else:
                    detect_count = 0

                left_d, right_d = corridor_distances(scan)
                if corridor_has_both(left_d, right_d):
                    dbg_v, dbg_w = corridor_center_cmd(scan, fm, adir, left_d, right_d)
                    dbg_condition = "centering"
                else:
                    follow_side = pick_wall_side(scan, follow_side)
                    sync_follower_direction()
                    dbg_v, dbg_w = wall_follower.step(scan)
                    dbg_condition = wall_follower.condition
                dbg_left_d, dbg_right_d = left_d, right_d
                send_cmd(dbg_v, dbg_w)

            # ── TRACK ─────────────────────────────────────────────────
            elif park_state == "TRACK":
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state   = "FORWARD"
                    park_t       = time.time()
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                    continue

                err_x     = cx_obj - cx_mid
                err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                reduced_v = APPROACH_V * (1.0 - err_ratio)

                def cam_w(ex):
                    raw = -KP_ROT * ex
                    if abs(raw) < W_MIN and ex != 0:
                        return -W_MIN if ex > 0 else W_MIN
                    return raw

                if fm >= THRESH_SLOW:
                    v, w = reduced_v, cam_w(err_x)
                else:
                    w_cam = cam_w(err_x); w_lid = adir * 0.7
                    if fm < THRESH_STOP:        v, w = 0.09, w_lid
                    elif fm < THRESH_TURN:      v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                    else:                       v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                last_cmd = (v, w)
                send_cmd(v, w)

            # ── SEARCH ───────────────────────────────────────────────
            elif park_state == "SEARCH":
                if search_t is None:
                    search_t = time.time(); arrive_count = 0
                if time.time() - search_t > 5.0:
                    park_state = "WALL_FOLLOW"; search_t = None
                else:
                    send_cmd(0.0, -1.0 if last_seen_x > cx_mid else 1.0)

        # ── 디버그 오버레이 ───────────────────────────────────────────
        search_time_left = MISSION_TIMEOUT_SEC - (time.time() - mission_start_t)
        if is_searching and search_time_left > 0:
            cv2.putText(frame, f"Timeout: {search_time_left:.1f}s | Side:{follow_side}",
                        (10, 20), 0, 0.50, (255, 150, 0), 2)

        in_wall_follow = (mode == "LIDAR") or (mode == "PARK" and park_state == "WALL_FOLLOW")
        if in_wall_follow:
            side_label = "CENTER" if dbg_condition == "centering" else follow_side
            cv2.putText(frame,
                f"WF[{side_label}] cond={dbg_condition} v={dbg_v:.2f} w={dbg_w:.2f} "
                f"L={dbg_left_d:.0f} R={dbg_right_d:.0f}",
                (10, H - 10), 0, 0.42, (0, 255, 100), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
