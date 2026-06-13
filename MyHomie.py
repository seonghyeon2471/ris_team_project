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
    "yellow": {"hsv1": ([24, 48, 193], [45, 165, 255]),
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

# ── 화면 영역 기준 ─────────────────────────────────────────────────────
# 좌우 15% 영역으로 타이트하게 조정
ALIGN_MARGIN   = int(320 * 0.15)          # 48px  (양쪽 15%)
ALIGN_LEFT     = ALIGN_MARGIN             # 48
ALIGN_RIGHT    = 320 - ALIGN_MARGIN       # 272
# centroid가 중앙 정렬된 것으로 볼 허용 오차 (±픽셀) → 더 타이트하게
CENTER_TOL     = 10                       # ±10px 이내면 "정렬 완료"
# 정렬 완료 확인용 연속 프레임 수 → 더 엄격하게
ALIGN_CONFIRM  = 8

# ── 도착 판정 ──────────────────────────────────────────────────────────
ARRIVE_Y_TOP        = int(240 * 0.85)
ARRIVE_X_MARGIN     = 40
ARRIVE_CONFIRM      = 8
ARRIVE_FORWARD_SEC  = 0.7
ARRIVE_FORWARD_V    = 0.15

# ── 색상 미인식 타임아웃 + 한바퀴 회전 탐색 ──────────────────────────────
NO_DETECT_TIMEOUT  = 2.5
SPIN_W             = 1.6
SPIN_ONE_ROUND_SEC = (2 * math.pi) / SPIN_W  # ~3.93초

# ── SPIN2: 주차 후 SEARCH 한바퀴 돌고 색상 못찾으면 추가 한바퀴 ───────────
GAP_OBS_THRESH  = 40.0
GAP_FREE_THRESH = 80.0
GAP_MIN_DEG     = 20
SPIN2_W         = 1.6
SPIN2_SEC       = (2 * math.pi) / SPIN2_W

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
mission_idx   = 0
detect_count  = 0
arrive_count  = 0

park_state    = "ALIGN"
align_count   = 0
last_seen_x   = 160
park_t        = None
last_cmd      = (0.0, 0.0)

no_detect_t   = time.time()
spin_t        = None
spin_found    = False

spin2_t       = None
spin2_samples = []
spin2_heading = 0.0
spin2_last_t  = None
spin2_gap_deg = None

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
            last_seen_x = cx_obj
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)

        # ── 시각화 ─────────────────────────────────────────────────────
        cv2.line(frame, (ALIGN_LEFT,  0), (ALIGN_LEFT,  H - 1), (200, 200, 0), 1)
        cv2.line(frame, (ALIGN_RIGHT, 0), (ALIGN_RIGHT, H - 1), (200, 200, 0), 1)
        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame,
                      (arrive_x1, ARRIVE_Y_TOP),
                      (arrive_x2, H - 1),
                      (0, 0, 255), 1)

        # ── 헬퍼 함수 ──────────────────────────────────────────────────
        def centroid_out_of_center():
            """centroid가 좌우 15% 밖에 있는가"""
            return cx_obj < ALIGN_LEFT or cx_obj > ALIGN_RIGHT

        def centroid_centered():
            """centroid가 중앙 ±CENTER_TOL(10px) 이내인가"""
            return abs(cx_obj - cx_mid) <= CENTER_TOL

        def centroid_in_arrive_zone():
            return (arrive_x1 <= cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        def cam_w(ex):
            """카메라 오차 기반 회전속도 (최솟값 보장)"""
            raw = -KP_ROT * ex
            if abs(raw) < W_MIN and ex != 0:
                return -W_MIN if ex > 0 else W_MIN
            return raw

        # ══ LIDAR 탐색 모드 ══════════════════════════════════════════
        if mode == "LIDAR":
            if found:
                detect_count += 1
                no_detect_t = time.time()
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "ALIGN"
                align_count = 0
                no_detect_t = time.time()
                print(f"[{target}] 발견 → PARK 모드 진입")
                continue

            if time.time() - no_detect_t >= NO_DETECT_TIMEOUT:
                mode = "SPIN"
                spin_t = time.time()
                spin_found = False
                no_detect_t = time.time()
                print(f"[{target}] {NO_DETECT_TIMEOUT}초 미인식 → SPIN 탐색 시작")
                continue

            if fm < THRESH_STOP: v, w = 0.09, adir * 0.9
            elif fm < THRESH_TURN: v, w = 0.13, adir * 0.7
            elif fm < THRESH_SLOW: v, w = 0.18, adir * 0.4
            else: v, w = 0.28, 0.0
            send_cmd(v, w)
            cv2.putText(frame, "MODE: LIDAR", (10, 25), 0, 0.5, (255, 255, 255), 1)

        # ══ SPIN 탐색 모드 (한바퀴 회전) ════════════════════════════
        elif mode == "SPIN":
            elapsed_spin = time.time() - spin_t

            if found and not spin_found:
                spin_found = True
                detect_count = DETECT_CONFIRM
                mode = "PARK"
                park_state = "ALIGN"
                align_count = 0
                no_detect_t = time.time()
                print(f"[{target}] SPIN 중 발견 → PARK 모드 진입")
                continue

            elif elapsed_spin >= SPIN_ONE_ROUND_SEC:
                mode = "LIDAR"
                no_detect_t = time.time()
                detect_count = 0
                print(f"[{target}] SPIN 완료, 미발견 → LIDAR 주행 복귀")
                send_cmd(0.28, 0.0)
            else:
                send_cmd(0.0, SPIN_W)
                remain = SPIN_ONE_ROUND_SEC - elapsed_spin
                cv2.putText(frame, f"SPIN: {remain:.1f}s", (10, 25), 0, 0.6, (0, 255, 0), 2)

        # ══ PARK 모드 ════════════════════════════════════════════════
        elif mode == "PARK":

            # ── 도착 후 전진 중 (FORWARD) ─────────────────────────────
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

            # ── 정차 중 (PARKING) ──────────────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    if mission_idx < len(MISSION):
                        park_state = "SEARCH"
                        last_seen_x = cx_mid + 40
                        align_count = 0
                        print(f"다음 미션 [{MISSION[mission_idx]}] 탐색 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 객체 없음: 탐색 회전 (SEARCH) ────────────────────────
            elif not found:
                if park_state != "SEARCH":
                    park_state   = "SEARCH"
                    spin_t       = time.time()
                    align_count  = 0
                    arrive_count = 0

                elapsed_search = time.time() - spin_t
                if elapsed_search >= SPIN_ONE_ROUND_SEC:
                    park_state    = "SPIN2"
                    spin2_t       = time.time()
                    spin2_last_t  = time.time()
                    spin2_heading = 0.0
                    spin2_samples = []
                    spin2_gap_deg = None
                    print(f"[{target}] SEARCH 한바퀴 완료, 미발견 → SPIN2(라이다 패턴 탐색)")
                    continue

                # ── 주차 후 탐색 각속도: 1.6 rad/s ──────────────────
                v = 0.0
                w = -1.6 if last_seen_x > cx_mid else 1.6
                send_cmd(v, w)
                remain = SPIN_ONE_ROUND_SEC - elapsed_search
                cv2.putText(frame, f"SEARCHING({remain:.1f}s): {target}",
                            (10, 25), 0, 0.55, (0, 255, 255), 1)

            # ── SPIN2: 라이다로 [장애물-공간-장애물] 패턴 탐색 ───────────
            elif park_state == "SPIN2":
                now      = time.time()
                dt       = now - spin2_last_t
                spin2_last_t = now

                spin2_heading += SPIN2_W * dt

                spin2_samples.append((spin2_heading, float(np.mean(scan[355:360].tolist() + scan[0:6].tolist()))))

                if found:
                    park_state  = "ALIGN"
                    align_count = 0
                    no_detect_t = time.time()
                    print(f"[{target}] SPIN2 중 색상 발견 → ALIGN")
                    continue

                if spin2_heading >= 2 * math.pi:
                    total = len(spin2_samples)
                    best_gap_center = None
                    best_gap_width  = 0

                    if total > 10:
                        dists  = np.array([s[1] for s in spin2_samples])
                        angles = np.array([math.degrees(s[0]) % 360 for s in spin2_samples])

                        is_free = dists >= GAP_FREE_THRESH
                        is_obs  = dists <= GAP_OBS_THRESH

                        in_gap      = False
                        gap_start   = 0
                        best_score  = -1

                        for i in range(total + 1):
                            idx = i % total
                            if not in_gap and is_free[idx]:
                                in_gap    = True
                                gap_start = i
                            elif in_gap and (not is_free[idx] or i == total):
                                gap_end   = i
                                gap_width = gap_end - gap_start

                                pre_idx  = (gap_start - 1) % total
                                post_idx = gap_end % total
                                has_obs_before = is_obs[pre_idx]
                                has_obs_after  = is_obs[post_idx]

                                if (has_obs_before and has_obs_after and
                                        gap_width >= GAP_MIN_DEG and
                                        gap_width > best_score):
                                    best_score     = gap_width
                                    mid_idx        = (gap_start + gap_width // 2) % total
                                    best_gap_center = angles[mid_idx]

                                in_gap = False

                    if best_gap_center is not None:
                        spin2_gap_deg = best_gap_center
                        park_state    = "SPIN2_TURN"
                        spin2_t       = time.time()
                        print(f"[{target}] SPIN2 패턴 발견 → 방향 {spin2_gap_deg:.1f}도 조준")
                    else:
                        mode         = "LIDAR"
                        no_detect_t  = time.time()
                        detect_count = 0
                        print(f"[{target}] SPIN2 패턴 미발견 → LIDAR 복귀")
                else:
                    send_cmd(0.0, SPIN2_W)
                    remain = SPIN2_SEC - (time.time() - spin2_t)
                    cv2.putText(frame, f"SPIN2({remain:.1f}s): {target}",
                                (10, 25), 0, 0.55, (255, 165, 0), 2)

            # ── SPIN2_TURN: 찾은 공간 방향으로 로봇 회전 후 전진 ──────────
            elif park_state == "SPIN2_TURN":
                turn_sec = math.radians(spin2_gap_deg) / SPIN2_W
                elapsed  = time.time() - spin2_t
                if elapsed < turn_sec:
                    send_cmd(0.0, SPIN2_W)
                    cv2.putText(frame, f"SPIN2_TURN: {spin2_gap_deg:.0f}deg",
                                (10, 25), 0, 0.55, (255, 165, 0), 2)
                else:
                    park_state  = "SPIN2_GO"
                    spin2_t     = time.time()
                    print(f"[{target}] 조준 완료 → 공간으로 전진")

            # ── SPIN2_GO: 공간 방향으로 전진 ──────────────────────────────
            elif park_state == "SPIN2_GO":
                if found:
                    park_state  = "ALIGN"
                    align_count = 0
                    no_detect_t = time.time()
                    print(f"[{target}] SPIN2_GO 중 색상 발견 → ALIGN")
                    continue

                if fm < THRESH_STOP:
                    v, w = 0.09, adir * 0.9
                elif fm < THRESH_TURN:
                    v, w = 0.13, adir * 0.7
                elif fm < THRESH_SLOW:
                    v, w = 0.18, adir * 0.4
                else:
                    v, w = 0.28, 0.0

                send_cmd(v, w)
                cv2.putText(frame, f"SPIN2_GO: {target}", (10, 25), 0, 0.55, (255, 165, 0), 2)

            # ── 객체 발견 시 메인 로직 ─────────────────────────────────
            elif found and park_state not in ("SPIN2", "SPIN2_TURN", "SPIN2_GO"):
                no_detect_t = time.time()
                err_x = cx_obj - cx_mid

                # [도착 판정]
                if centroid_in_arrive_zone():
                    arrive_count += 1
                else:
                    arrive_count = 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    align_count  = 0
                    park_state   = "FORWARD"
                    park_t       = time.time()
                    print(f"[{target}] 도착 확정 → {ARRIVE_FORWARD_SEC}초 전진")
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                    continue

                # ── ALIGN: centroid 중앙 정렬 (카메라 조향) ──────────
                if park_state in ("ALIGN", "SEARCH"):
                    park_state = "ALIGN"

                if park_state == "ALIGN":
                    if centroid_centered():
                        align_count += 1
                    else:
                        align_count = 0

                    if align_count >= ALIGN_CONFIRM:
                        # 정렬 완료 → 라이다 전진 모드
                        align_count = 0
                        park_state  = "LIDAR_GO"
                        print(f"[{target}] 정렬 완료(±{CENTER_TOL}px, {ALIGN_CONFIRM}프레임) → LIDAR_GO")
                    else:
                        # 카메라 조향: 오차 60px 초과 시 제자리 회전
                        w = cam_w(err_x)
                        v = 0.0 if abs(err_x) > 60 else 0.08
                        last_cmd = (v, w)
                        send_cmd(v, w)

                    cv2.putText(frame, f"ALIGN({align_count}/{ALIGN_CONFIRM}): {target}",
                                (10, 25), 0, 0.55, draw, 1)

                # ── LIDAR_GO: 정렬 후 라이다로 장애물 피하며 전진 ──────
                elif park_state == "LIDAR_GO":
                    # centroid가 다시 15% 밖으로 벗어나면 ALIGN 복귀
                    if centroid_out_of_center():
                        park_state  = "ALIGN"
                        align_count = 0
                        print(f"[{target}] 이탈 감지 → ALIGN 복귀")
                        continue

                    # 라이다 장애물 회피 + 전진
                    if fm < THRESH_STOP:
                        v, w = 0.09, adir * 0.9
                    elif fm < THRESH_TURN:
                        v, w = 0.13, adir * 0.7
                    elif fm < THRESH_SLOW:
                        v, w = 0.18, adir * 0.4
                    else:
                        v, w = APPROACH_V, 0.0

                    last_cmd = (v, w)
                    send_cmd(v, w)
                    cv2.putText(frame, f"LIDAR_GO: {target}", (10, 25), 0, 0.55, draw, 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
