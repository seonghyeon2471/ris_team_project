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
FRONT_RANGE  = 45
THRESH_SLOW  = 55.0
THRESH_TURN  = 35.0
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

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 175], [179, 207, 255]),
               "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 214], [41, 143, 255]),
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

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA       = 400
KP_ROT         = 0.030
W_MIN          = 0.25
APPROACH_V     = 0.17
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 40
ARRIVE_FORWARD_SEC = 1.0
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8

# ── SEARCH (장애물 추종) 파라미터 ─────────────────────────────────────
ROBOT_RADIUS = 10.0          # 로봇 반경 (cm), 실측 후 조정

# 탐색 전용 임계값 (주행용보다 훨씬 바짝)
SEARCH_THRESH_TURN = 12.0    # 이 거리까지는 직진 유지
SEARCH_THRESH_STOP =  7.0    # 실제 정지/회전 (ROBOT_RADIUS - 3)

# 장애물 추종 거리
SEARCH_FOLLOW_DIST =  8.0    # 측면 유지 목표 거리 (ROBOT_RADIUS - 2)
SEARCH_FOLLOW_MAX  = 14.0    # 이 이상 멀어지면 장애물 놓침 판정

SEARCH_FOLLOW_SIDE = "LEFT"  # 장애물을 왼쪽에 두고 순환 (일관된 방향)
SEARCH_FOLLOW_V    = 0.13    # 추종 전진 속도
SEARCH_FOLLOW_KP   = 0.018   # 거리 오차 → 조향 게인
SEARCH_APPROACH_V  = 0.13    # FIND 단계 접근 속도
SEARCH_LOST_TIMEOUT = 1.2    # 장애물 놓친 후 FIND 복귀까지 대기 시간 (초)

# 순환 감지: 누적 회전량 기반
FULL_CIRCLE_RAD   = 2 * math.pi * 0.85  # 360도의 85% 이상이면 순환으로 판단
ESCAPE_V          = 0.13                 # ESCAPE 단계 전진 속도
ESCAPE_W          = 1.0                  # ESCAPE 단계 회전 속도 (장애물 반대 방향)
ESCAPE_RAD        = math.pi * 0.6       # ESCAPE 회전 목표량 (~108도)

# 측면 섹터 각도 (LEFT: 로봇 왼쪽 270도 부근, RIGHT: 90도 부근)
SIDE_SECTORS = {
    "LEFT":  np.arange(240, 300),
    "RIGHT": np.arange(60,  120),
}

def side_dist(scan, side="LEFT"):
    """측면 최솟값 거리"""
    return float(np.min(scan[SIDE_SECTORS[side]]))

def find_nearest_obstacle_angle(scan):
    """전방위에서 가장 가까운 포인트의 각도 반환"""
    return int(np.argmin(scan))

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
mission_idx   = 0
detect_count  = 0
arrive_count  = 0

park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
last_cmd      = (0.0, 0.0)

# SEARCH 세부 상태
search_phase      = "FIND"   # "FIND" → "FOLLOW" → "ESCAPE" → "FIND" ...
search_phase_t    = None
follow_lost_t     = None
follow_angle_accum = 0.0     # FOLLOW 중 누적 회전량 (rad)
follow_last_t      = None    # 직전 프레임 시각 (dt 계산용)
escape_angle_accum = 0.0     # ESCAPE 중 누적 회전량 (rad)

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
            err_x  = cx_obj - cx_mid
            last_seen_x   = cx_obj
            last_bottom_y = min(by_top + bh, 239)

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)
            cv2.line(frame, (cx_obj, by_top), (cx_obj, by_top + bh), (0, 255, 255), 1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame,
                      (arrive_x1, ARRIVE_Y_TOP),
                      (arrive_x2, H - 1),
                      (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and
                    cy_obj >= ARRIVE_Y_TOP)

        # ══ LIDAR 모드 ═══════════════════════════════════════════
        if mode == "LIDAR":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                print(f"[{target}] 발견 → 추적 시작")
                continue

            if fm < THRESH_STOP: v, w = 0.09, adir * 0.9
            elif fm < THRESH_TURN: v, w = 0.13, adir * 0.7
            elif fm < THRESH_SLOW: v, w = 0.18, adir * 0.4
            else: v, w = 0.28, 0.0
            send_cmd(v, w)
            cv2.putText(frame, "MODE: LIDAR", (10, 25), 0, 0.5, (255, 255, 255), 1)

        # ══ PARK 모드 ════════════════════════════════════════════
        elif mode == "PARK":

            # ── 1. 도착 후 전진 중 (FORWARD) ──────────────────────
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

            # ── 2. 정차 중 (PARKING) ───────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    if mission_idx < len(MISSION):
                        # 다음 미션 진입 시 SEARCH 상태 초기화
                        park_state     = "SEARCH"
                        search_phase   = "FIND"
                        search_phase_t = time.time()
                        follow_lost_t  = None
                        last_seen_x    = cx_mid + 40
                        print(f"다음 미션 [{MISSION[mission_idx]}] 탐색 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 3. 객체 추적 중 (TRACK) ────────────────────────────
            elif found:
                park_state = "TRACK"
                # SEARCH에서 TRACK으로 복귀 시 상태 초기화
                search_phase       = "FIND"
                search_phase_t     = None
                follow_lost_t      = None
                follow_angle_accum = 0.0
                follow_last_t      = None
                escape_angle_accum = 0.0

                if centroid_in_arrive_zone():
                    arrive_count += 1
                else:
                    arrive_count = 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    print(f"[{target}] centroid {ARRIVE_CONFIRM}프레임 확정 → {ARRIVE_FORWARD_SEC}초 전진")
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
                        v = APPROACH_V
                        w = cam_w(err_x)
                    else:
                        w_cam = cam_w(err_x)
                        w_lid = adir * 0.7
                        if fm < THRESH_STOP:
                            v, w = 0.09, w_lid
                        elif fm < THRESH_TURN:
                            v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                        else:
                            v, w = 0.18, 0.3 * w_lid + 0.7 * w_cam

                    last_cmd = (v, w)
                    send_cmd(v, w)

                cv2.putText(frame, f"TRACKING: {target}", (10, 25), 0, 0.6, draw, 1)

            # ── 4. 장애물 추종 탐색 (SEARCH) ──────────────────────
            else:
                # SEARCH 최초 진입 초기화
                if park_state != "SEARCH":
                    park_state         = "SEARCH"
                    search_phase       = "FIND"
                    search_phase_t     = time.time()
                    follow_lost_t      = None
                    follow_angle_accum = 0.0
                    follow_last_t      = None
                    escape_angle_accum = 0.0

                sd  = side_dist(scan, SEARCH_FOLLOW_SIDE)
                now = time.time()
                sign = 1 if SEARCH_FOLLOW_SIDE == "LEFT" else -1

                # ── 4-1. 가장 가까운 장애물 방향으로 접근 ──────────
                if search_phase == "FIND":
                    nearest_angle = find_nearest_obstacle_angle(scan)
                    nearest_dist  = float(scan[nearest_angle])

                    if nearest_dist <= SEARCH_FOLLOW_MAX:
                        # 장애물 포착 → 추종 시작, 누적값 초기화
                        search_phase       = "FOLLOW"
                        search_phase_t     = now
                        follow_angle_accum = 0.0
                        follow_last_t      = now
                        print(f"[{target}] 장애물 포착 ({nearest_dist:.1f}cm) → 추종 시작")
                    else:
                        # 가장 가까운 방향으로 회전 후 접근
                        err_a = nearest_angle if nearest_angle <= 180 else nearest_angle - 360
                        w_a   = np.clip(-err_a / 90.0 * 1.0, -1.2, 1.2)
                        v_a   = SEARCH_APPROACH_V if fm > SEARCH_THRESH_STOP else 0.0
                        send_cmd(v_a, w_a)

                # ── 4-2. 장애물 측면 유지하며 순환 ─────────────────
                elif search_phase == "FOLLOW":

                    # dt 계산
                    dt            = now - follow_last_t if follow_last_t else 0.0
                    follow_last_t = now

                    if sd > SEARCH_FOLLOW_MAX:
                        # 장애물 놓침
                        if follow_lost_t is None:
                            follow_lost_t = now
                        elif now - follow_lost_t > SEARCH_LOST_TIMEOUT:
                            search_phase       = "FIND"
                            search_phase_t     = now
                            follow_lost_t      = None
                            follow_angle_accum = 0.0
                            follow_last_t      = None
                            print(f"[{target}] 장애물 놓침 → FIND 복귀")
                        send_cmd(SEARCH_FOLLOW_V * 0.5, sign * 0.5)

                    else:
                        follow_lost_t = None

                        # 거리 오차 기반 조향
                        dist_err = sd - SEARCH_FOLLOW_DIST
                        w_follow = sign * SEARCH_FOLLOW_KP * dist_err

                        # 전방 장애물 처리 (탐색 전용 임계값 사용)
                        if fm < SEARCH_THRESH_STOP:
                            v, w = 0.0, sign * 1.0
                        elif fm < SEARCH_THRESH_TURN:
                            v, w = 0.08, sign * 0.7 + w_follow * 0.3
                        else:
                            v, w = SEARCH_FOLLOW_V, w_follow

                        # 누적 회전량 갱신 (절댓값: 방향 무관하게 총 회전량)
                        follow_angle_accum += abs(w) * dt

                        # 순환 감지: 누적 회전량이 FULL_CIRCLE_RAD 초과
                        if follow_angle_accum >= FULL_CIRCLE_RAD:
                            search_phase       = "ESCAPE"
                            search_phase_t     = now
                            escape_angle_accum = 0.0
                            follow_angle_accum = 0.0
                            follow_last_t      = None
                            print(f"[{target}] 순환 감지 ({math.degrees(follow_angle_accum):.0f}도) → 이탈")

                        send_cmd(v, w)

                # ── 4-3. 순환 이탈 (ESCAPE) ─────────────────────────
                elif search_phase == "ESCAPE":
                    # 장애물 반대 방향으로 틀면서 전진 → 다음 장애물 탐색 위치로 이동
                    escape_w = -sign * ESCAPE_W   # 추종 반대 방향 회전
                    send_cmd(ESCAPE_V, escape_w)

                    dt = now - (search_phase_t if search_phase_t else now)
                    escape_angle_accum += abs(escape_w) * dt

                    if escape_angle_accum >= ESCAPE_RAD:
                        search_phase       = "FIND"
                        search_phase_t     = now
                        escape_angle_accum = 0.0
                        follow_last_t      = None
                        print(f"[{target}] 이탈 완료 → FIND (다음 장애물 탐색)")

                sd_disp = side_dist(scan, SEARCH_FOLLOW_SIDE)
                accum_deg = math.degrees(follow_angle_accum)
                cv2.putText(frame,
                            f"SEARCH/{search_phase} side={sd_disp:.0f}cm "
                            f"rot={accum_deg:.0f}deg",
                            (10, 25), 0, 0.45, (0, 255, 255), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
