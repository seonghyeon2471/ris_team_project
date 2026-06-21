import cv2
import serial
import numpy as np
import time
import math
import threading
from collections import deque  # ★ PID 이력용

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
THRESH_SLOW  = 40.0
THRESH_TURN  = 25.0
THRESH_STOP  = 18.0

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

# ★ ── 강화된 side_dist ────────────────────────────────────────────────
# 기존: 10개 각도 단순 평균 → 코너/노이즈에 취약
# 변경: 31개 각도(±15도) 수집 후 상위 20% 제거한 중앙값 사용
SIDE_ANGLE_HALF = 15

def side_dist(scan, side):
    center = 90 if side == "L" else 270
    idx = np.arange(center - SIDE_ANGLE_HALF,
                    center + SIDE_ANGLE_HALF + 1) % 360
    vals = scan[idx]
    cutoff = np.percentile(vals, 80)       # 상위 20% 이상치 제거
    filtered = vals[vals <= cutoff]
    if len(filtered) == 0:
        return float(np.median(vals))
    return float(np.median(filtered))

def left_dist(scan):
    return side_dist(scan, "L")

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# ★ ── 전방 측면 거리 측정 (코너 전방 벽까지 거리) ─────────────────────
# follow_side 쪽 전방 45도 구간의 최솟값
# ex) L 팔로우 시 30~60도(좌전방), R 팔로우 시 300~330도(우전방)
def front_side_dist(scan, follow_side):
    if follow_side == "L":
        idx = np.arange(30, 61) % 360
    else:
        idx = np.arange(300, 331) % 360
    return float(np.min(scan[idx]))

# ★ ── 삼각형 이등분 법칙 기반 코너 조향각 계산 ───────────────────────
# 직사각형 장애물 코너에서:
#   - d_side  : 현재 따라가는 측면 벽까지 거리 (수평 벡터)
#   - d_front : 코너 앞 새 벽까지 거리       (수직 벡터)
# 두 벡터의 각도 이등분선 방향으로 조향하면
# 코너를 자연스러운 대각선 호로 통과 가능
#
# θ = atan2(d_side, d_front)   (두 벡터가 이루는 각)
# 목표 조향 = θ / 2           (이등분선)
# 정규화: sign * (θ/2) / (π/4) → [-1, 1] 범위로 매핑
CORNER_ENTRY_THRESH = 1.8   # d_front < d_side * 이 비율이면 코너 진입으로 판단
CORNER_BISECT_W_MAX = 1.0   # 이등분 조향 최대값

def bisect_corner_w(d_side, d_front, sign):
    """
    삼각형 이등분 법칙으로 코너 조향각 반환.
    d_side: 측면 거리(cm), d_front: 전방 측면 거리(cm), sign: L=+1, R=-1
    """
    d_side  = max(d_side,  1.0)
    d_front = max(d_front, 1.0)
    theta   = math.atan2(d_side, d_front)   # 두 벡터 사잇각 (0 ~ π/2)
    half    = theta / 2.0                   # 이등분선 각도
    # π/4(45도)를 기준으로 정규화 → 코너가 직각일 때 w ≈ 1.0
    w_norm  = half / (math.pi / 4.0)
    return float(np.clip(sign * w_norm * CORNER_BISECT_W_MAX, -1.2, 1.2))

# ★ ── PD 제어기 ───────────────────────────────────────────────────────
class WallPID:
    def __init__(self, kp, kd, dt=0.05):
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self.prev_err = 0.0

    def reset(self):
        self.prev_err = 0.0

    def compute(self, err):
        d_err = (err - self.prev_err) / self.dt
        self.prev_err = err
        return self.kp * err + self.kd * d_err

# ── WALL PARAMS ───────────────────────────────────────────────────────
WALL_TARGET     = 20.0
WALL_SCAN_DIST  = 150.0
WALL_APPROACH_V = 0.20
WALL_KP         = 0.012
WALL_KD         = 0.006
WALL_V          = 0.22
WALL_TURN_V     = 0.10
WALL_LOST_W     = 0.5
WALL_SEARCH_W   = 1.1
CORNER_V_SCALE  = 0.60   # 코너 진입 시 감속 비율

wall_pid = WallPID(kp=WALL_KP, kd=WALL_KD)

# ★ ── 삼각형 이등분 법칙 적용 wall_follow ────────────────────────────
def wall_follow(scan, fm, adir, follow_side):
    """
    직사각형 장애물 코너 처리 강화:
    - 평소: PD 제어로 측면 거리 유지
    - 코너 감지 시: d_side + d_front 이등분선 방향으로 자연스럽게 선회
      → 급격한 회전 없이 대각선 호를 그리며 코너 통과
    """
    sd      = side_dist(scan, follow_side)
    fsd     = front_side_dist(scan, follow_side)   # ★ 코너 전방 벽 거리
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1

    # ① 정면 위험 → 긴급 회피
    if fm < THRESH_STOP:
        wall_pid.reset()
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        wall_pid.reset()
        return (WALL_TURN_V, adir * 0.85)

    # ② 측면 충돌 위험
    if left_close < THRESH_STOP:
        wall_pid.reset()
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        wall_pid.reset()
        return (WALL_V * 0.7,  0.7)

    # ③ 벽 손실
    if sd > WALL_TARGET * 2.0:
        wall_pid.reset()
        return (WALL_V * 0.7, sign * WALL_LOST_W)

    # ★ ④ 코너 감지: 전방 측면 거리가 측면 거리보다 충분히 가까우면 코너
    #    (직사각형 장애물이므로 코너 직전에 fsd가 급격히 줄어듦)
    is_corner = (fsd < sd * CORNER_ENTRY_THRESH) and (fsd < THRESH_SLOW)

    if is_corner:
        # ★ 삼각형 이등분 법칙: d_side와 d_front 벡터의 각도 이등분선으로 조향
        wall_pid.reset()
        w_bisect = bisect_corner_w(sd, fsd, sign)
        v_corner = WALL_V * CORNER_V_SCALE * float(
            np.clip(fsd / THRESH_SLOW, 0.4, 1.0))  # 가까울수록 더 감속
        return (v_corner, w_bisect)

    # ⑤ 직선 구간: PD 제어
    err = sd - WALL_TARGET
    w   = sign * wall_pid.compute(err)

    # ⑥ 전방 장애물 접근 블렌딩
    if fm < THRESH_SLOW:
        blend = float(np.clip(
            (THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6),
            0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V

    w = float(np.clip(w, -0.9, 0.9))
    return (v, w)

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]),
               "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([25, 60, 160], [32, 161, 255]),
               "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([96, 100, 95], [138, 207, 246]),
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

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA       = 400
KP_ROT         = 0.030
W_MIN          = 0.20
APPROACH_V     = 0.17
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
mission_idx   = 0
detect_count  = 0
arrive_count  = 0
follow_side   = "L"
lidar_state   = "WALL_SEARCH"
park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
last_cmd      = (0.0, 0.0)

print(f"START | MISSION: {MISSION}")

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
            cv2.line(frame, (cx_obj, by_top), (cx_obj, by_top + bh), (0, 255, 255), 1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and
                    cy_obj >= ARRIVE_Y_TOP)

        # ★ 코너 표시: front_side_dist vs side_dist 비교로 판단
        _fsd_disp = front_side_dist(scan, follow_side)
        _sd_disp  = side_dist(scan, follow_side)
        is_corner_disp = (_fsd_disp < _sd_disp * CORNER_ENTRY_THRESH) and (_fsd_disp < THRESH_SLOW)

        # ══ LIDAR 모드 ══════════════════════════════════════════════
        if mode == "LIDAR":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                wall_pid.reset()  # ★
                mode = "PARK"
                park_state = "TRACK"
                print(f"[{target}] 발견 → 추적 시작")
                continue

            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    lidar_state = "WALL_APPROACH"
                    print(f"[LIDAR] 벽 감지 fm:{fm:.0f}cm, follow_side={follow_side}")
                else:
                    send_cmd(0.0, WALL_SEARCH_W)
                    cv2.putText(frame, f"LIDAR-WALL-SEARCH fm:{fm:.0f}",
                                (10, 25), 0, 0.5, (0, 255, 0), 1)

            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    wall_pid.reset()  # ★
                    lidar_state = "WALL_FOLLOW"
                    print(f"[LIDAR] 벽 도달 {follow_side}:{sd:.0f}cm → wall-following")
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP:
                        v, w = 0.08, adir * 1.0
                    elif fm < THRESH_TURN:
                        v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                    else:
                        v, w = WALL_APPROACH_V, sign * 0.3
                    send_cmd(v, w)
                    cv2.putText(frame, f"LIDAR-WALL-APPROACH {follow_side}:{sd:.0f}cm",
                                (10, 25), 0, 0.5, (0, 200, 0), 1)

            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                sd_disp = side_dist(scan, follow_side)
                corner_txt = " [CORNER]" if is_corner_disp else ""
                cv2.putText(frame, f"LIDAR-WALL-FOLLOW {follow_side}:{sd_disp:.0f}cm{corner_txt}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            cv2.putText(frame, "MODE: LIDAR", (10, 45), 0, 0.5, (255, 255, 255), 1)

        # ══ PARK 모드 ═══════════════════════════════════════════════
        elif mode == "PARK":

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

            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    if mission_idx < len(MISSION):
                        wall_pid.reset()  # ★
                        park_state = "WALL_SEARCH"
                        print(f"다음 미션 [{MISSION[mission_idx]}] wall-following 탐색")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

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
                cv2.putText(frame, f"WALL-SEARCH [{target}] fm:{fm:.0f}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

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
                    wall_pid.reset()  # ★
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
                cv2.putText(frame, f"WALL-APPROACH [{target}] {follow_side}:{sd:.0f}cm",
                            (10, 25), 0, 0.5, (0, 200, 0), 1)

            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        wall_pid.reset()  # ★
                        park_state = "TRACK"
                        continue
                else:
                    detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                sd_disp = side_dist(scan, follow_side)
                corner_txt = " [CORNER]" if is_corner_disp else ""
                cv2.putText(frame, f"WALL-FOLLOW [{target}] {follow_side}:{sd_disp:.0f}cm{corner_txt}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            elif found:
                park_state = "TRACK"
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    print(f"[{target}] centroid 확정 → {ARRIVE_FORWARD_SEC}초 전진")
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                    continue
                else:
                    err_x = cx_obj - cx_mid
                    err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                    reduced_v = APPROACH_V * (1.0 - err_ratio)

                    def cam_w(ex):
                        raw = -KP_ROT * ex
                        if abs(raw) < W_MIN and ex != 0:
                            return -W_MIN if ex > 0 else W_MIN
                        return raw

                    if fm >= THRESH_SLOW:
                        v = reduced_v
                        w = cam_w(err_x)
                    else:
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

                cv2.putText(frame, f"TRACKING: {target}", (10, 25), 0, 0.6, draw, 1)

            else:
                park_state = "SEARCH"
                arrive_count = 0
                v = 0.0
                w = (-1.0 if last_seen_x > cx_mid else 1.0)
                send_cmd(v, w)
                cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
