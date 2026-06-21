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
THRESH_TURN  = 30.0
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

# ── 개선된 벽 거리 측정 ────────────────────────────────────────────────
# 기존: 10도 범위의 단순 평균 → 개선: 넓은 범위(30도)에서 최솟값 기반 가중평균
WALL_MEASURE_HALF = 15   # 중심에서 ±15도 = 총 30도 범위로 측정

def side_dist(scan, side):
    """
    개선된 측면 거리 측정.
    넓은 각도(30도)에서 측정하여 노이즈에 강하게 만들고,
    최솟값 주변에 가중치를 주어 실제 벽까지의 거리를 정확히 파악.
    """
    if side == "L":
        center = 90
    else:
        center = 270
    half = WALL_MEASURE_HALF
    idx = np.arange(center - half, center + half + 1) % 360
    vals = scan[idx]
    # 최솟값 근처(하위 30%)에 더 높은 가중치를 부여
    threshold = np.percentile(vals, 30)
    weights = np.where(vals <= threshold, 2.0, 1.0)
    return float(np.average(vals, weights=weights))

def side_angle_dist(scan, side):
    """
    벽까지의 거리뿐만 아니라 벽과의 각도 오차도 계산.
    전방 45도, 측면 90도 두 점의 거리 차이를 이용해 각도 추정.
    반환: (거리, 각도_오차_cm)
    """
    if side == "L":
        c_side   = 90
        c_front  = 45
    else:
        c_side   = 270
        c_front  = 315

    half = 8
    idx_s = np.arange(c_side  - half, c_side  + half + 1) % 360
    idx_f = np.arange(c_front - half, c_front + half + 1) % 360

    d_side  = float(np.min(scan[idx_s]))
    d_front = float(np.min(scan[idx_f]))

    # 이상적으로 벽에 평행하면 d_side_45 ≈ d_side / sin(45°) ≈ d_side * 1.414
    # 실제 d_front 값과 비교해 각도 오차를 cm 단위로 표현
    d_front_ideal = d_side * 1.414
    angle_err = d_front - d_front_ideal   # 양수: 벽에서 멀어지는 방향, 음수: 벽에 파고드는 방향

    return d_side, angle_err

def left_dist(scan):
    return side_dist(scan, "L")

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# ── 개선된 PD 벽 추종 ─────────────────────────────────────────────────
# 기존: 단순 P 제어 + adir blend 혼용 → 개선: PD 제어로 벽과의 거리 + 각도 동시 제어
WALL_TARGET    = 25.0   # 기존 20cm에서 25cm로 늘려 여유 확보
WALL_SCAN_DIST = 150.0
WALL_APPROACH_V = 0.20
WALL_KP        = 0.018  # 거리 비례 게인 (기존 0.012에서 상향)
WALL_KD        = 0.008  # 각도(미분) 게인 추가
WALL_V         = 0.22
WALL_TURN_V    = 0.10
WALL_LOST_W    = 0.55
WALL_SEARCH_W  = 1.1
WALL_RECOVER_W = 0.7    # 벽 재탐색 시 천천히 회전

# 이전 거리 오차 저장 (D항 계산용)
_prev_wall_err = {"L": 0.0, "R": 0.0}
_prev_wall_t   = {"L": 0.0, "R": 0.0}

def wall_follow(scan, fm, adir, follow_side):
    """
    개선된 PD 벽 추종.
    - 거리 오차(P항) + 각도 오차(D항)을 결합해 더 안정적으로 추종
    - 정면 장애물 처리를 단계적으로 구분
    - 벽을 잃었을 때 반대쪽 회전 방향으로 복구
    """
    global _prev_wall_err, _prev_wall_t

    sign = 1 if follow_side == "L" else -1

    d_side, angle_err = side_angle_dist(scan, follow_side)
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)

    # ① 정면 위험 → 긴급 회피 (adir 사용, 전진 최소화)
    if fm < THRESH_STOP:
        return (0.08, adir * 1.2)

    # ② 정면 주의 → 완만하게 adir 방향 틀며 속도 감소
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.9)

    # ③ 측면 근접 위험 (다른 방향 벽에 너무 붙었을 때)
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.8)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.8)

    # ④ 벽을 잃음 → 해당 방향으로 완만히 회전하며 재탐색
    if d_side > WALL_TARGET * 2.5:
        return (WALL_V * 0.6, sign * WALL_LOST_W)

    # ⑤ 정상 PD 벽 추종
    dist_err = d_side - WALL_TARGET   # 양수: 벽에서 너무 멀리, 음수: 너무 가까이

    # D항: 각도 오차 기반 (벽에 대한 평행도)
    # angle_err 양수 → 앞부분이 멀어지는 중 → sign 방향으로 더 틀어야 함
    d_term = WALL_KD * angle_err

    # P항 + D항 결합 (sign을 곱해 L/R 방향 구분)
    w = sign * (WALL_KP * dist_err + d_term)

    # 정면이 가까워질수록 속도 감소 + adir 혼합 비율 증가
    if fm < THRESH_SLOW:
        blend = float(np.clip(
            (THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0
        ))
        w = (1 - blend) * w + blend * (adir * 0.4)
        v = WALL_V * (1.0 - 0.35 * blend)
    else:
        v = WALL_V

    w = float(np.clip(w, -1.0, 1.0))
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

ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
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

        # ── 디버그 표시: 측면 거리 ──
        sd_l = side_dist(scan, "L")
        sd_r = side_dist(scan, "R")
        cv2.putText(frame, f"L:{sd_l:.0f} R:{sd_r:.0f} F:{fm:.0f}",
                    (10, H - 10), 0, 0.4, (200, 200, 200), 1)

        # ══ LIDAR 모드 ════════════════════════════════════════════════
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

            # ── A. 벽 탐색 중 제자리 회전 ──
            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    lidar_state = "WALL_APPROACH"
                    print(f"[LIDAR] 벽 감지 fm:{fm:.0f}cm → {follow_side} 추종 접근")
                else:
                    send_cmd(0.0, WALL_SEARCH_W)
                    cv2.putText(frame, f"LIDAR-WALL-SEARCH fm:{fm:.0f}",
                                (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── B. 벽으로 접근 중 ──
            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.4:
                    lidar_state = "WALL_FOLLOW"
                    print(f"[LIDAR] 벽 도달 {follow_side}:{sd:.0f}cm → wall-following 시작")
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP:
                        v, w = 0.08, adir * 1.0
                    elif fm < THRESH_TURN:
                        v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                    else:
                        # follow_side 방향으로 완만히 틀면서 접근
                        steer = sign * 0.25
                        v, w = WALL_APPROACH_V, steer
                    send_cmd(v, w)
                    cv2.putText(frame, f"LIDAR-APPROACH {follow_side}:{sd:.0f}cm",
                                (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── C. PD 벽 추종 탐색 ──
            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                sd_disp = side_dist(scan, follow_side)
                _, ang_err = side_angle_dist(scan, follow_side)
                cv2.putText(frame, f"LIDAR-WF {follow_side}:{sd_disp:.0f}cm ang:{ang_err:.1f}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            cv2.putText(frame, "MODE: LIDAR", (10, 45), 0, 0.5, (255, 255, 255), 1)

        # ══ PARK 모드 ════════════════════════════════════════════════
        elif mode == "PARK":

            # ── 1. 도착 후 전진 중 ──
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

            # ── 2. 정차 중 ──
            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"
                        print(f"다음 미션 [{MISSION[mission_idx]}] wall-following 탐색 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 3-A. 벽 탐색 제자리 회전 ──
            elif park_state == "WALL_SEARCH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] 탐색 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                if fm < WALL_SCAN_DIST:
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    park_state = "WALL_APPROACH"
                    print(f"회전 중 벽 감지 fm:{fm:.0f}cm → {follow_side} 접근")
                    continue

                send_cmd(0.0, WALL_SEARCH_W)
                cv2.putText(frame, f"WALL-SEARCH [{target}] fm:{fm:.0f}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── 3-B. 벽으로 접근 중 ──
            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] 접근 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.4:
                    park_state = "WALL_FOLLOW"
                    print(f"벽 도달 {follow_side}:{sd:.0f}cm → wall-following 시작")
                    continue

                sign = 1 if follow_side == "L" else -1
                if fm < THRESH_STOP:
                    v, w = 0.08, adir * 1.0
                elif fm < THRESH_TURN:
                    v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else:
                    v, w = WALL_APPROACH_V, sign * 0.25
                send_cmd(v, w)
                cv2.putText(frame, f"WALL-APPROACH [{target}] {follow_side}:{sd:.0f}cm",
                            (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── 3-C. PD 벽 추종 ──
            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] wall-following 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                sd_disp = side_dist(scan, follow_side)
                _, ang_err = side_angle_dist(scan, follow_side)
                cv2.putText(frame, f"WALL-FOLLOW [{target}] {follow_side}:{sd_disp:.0f} ang:{ang_err:.1f}",
                            (10, 25), 0, 0.45, (0, 255, 0), 1)

            # ── 4. 객체 추적 중 ──
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

            # ── 5. 객체 놓침 / 제자리 탐색 ──
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
