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
_last_v = 0.0
_last_w = 0.0

# 정지→이동 시 부드러운 전환을 위한 램프업 설정
RAMP_STEP_V  = 0.04   # 프레임당 v 최대 변화량
RAMP_STEP_W  = 0.15   # 프레임당 w 최대 변화량

def send_cmd(v, w, ramp=True):
    """
    ramp=True 이면 급격한 속도 변화를 막아줌 (정지→이동 부드럽게).
    즉각 정지가 필요할 땐 ramp=False.
    """
    global _last_v, _last_w
    v = float(np.clip(v, -0.4, 0.4))
    w = float(np.clip(w, -1.6, 1.6))
    if ramp:
        dv = v - _last_v
        dw = w - _last_w
        if abs(dv) > RAMP_STEP_V:
            v = _last_v + math.copysign(RAMP_STEP_V, dv)
        if abs(dw) > RAMP_STEP_W:
            w = _last_w + math.copysign(RAMP_STEP_W, dw)
    _last_v, _last_w = v, w
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    global _last_v, _last_w
    _last_v, _last_w = 0.0, 0.0
    arduino_ser.write(b"0.000,0.000\n")

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 175], [179, 207, 255]),
               "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]),
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
MIN_AREA           = 400
KP_ROT             = 0.030
W_MIN              = 0.25
APPROACH_V         = 0.17
PARK_SEC           = 1.2
DETECT_CONFIRM     = 6

ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 40
ARRIVE_FORWARD_SEC = 0.7
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8

# ── SCAN SEARCH (제자리 회전 스캔) ────────────────────────────────────
SPIN_W             = 0.6        # 제자리 회전 각속도 (rad/s)
SPIN_FULL_SEC      = 10.5       # 360° 완료 기준 시간 (여유 포함)
SPIN_DETECT_FRAMES = 4          # 회전 중 색상 연속 인식 프레임 수

# ── WANDER SEARCH (30cm 반경 배회) ────────────────────────────────────
SEARCH_RADIUS_CM   = 30.0       # 원점에서 최대 이동 반경
WANDER_V           = 0.18       # 배회 전진 속도
WANDER_W_BASE      = 0.4        # 배회 시 기본 회전
WANDER_STEP_SEC    = 1.5        # 한 방향 전진 시간
WANDER_TURN_SEC    = 1.0        # 방향 전환 시간
STOP_BETWEEN_SEC   = 0.3        # 동작 사이 정지 시간

# ── 간단한 오도메트리 (LiDAR 없이 cmd 적분) ──────────────────────────
# 로봇 heading 을 추적하여 원점 대비 상대 위치를 추정함
odo_x    = 0.0   # cm
odo_y    = 0.0   # cm
odo_yaw  = 0.0   # rad
odo_t    = time.time()
odo_lock = threading.Lock()

def update_odometry(v, w):
    """send_cmd 후 호출하여 위치 적분"""
    global odo_x, odo_y, odo_yaw, odo_t
    now = time.time()
    dt  = now - odo_t
    odo_t = now
    if dt > 0.2: dt = 0.2   # 이상값 방지
    with odo_lock:
        odo_yaw += w * dt
        dist_cm  = v * 100.0 * dt   # m → cm
        odo_x   += dist_cm * math.cos(odo_yaw)
        odo_y   += dist_cm * math.sin(odo_yaw)

def get_dist_from_origin():
    with odo_lock:
        return math.hypot(odo_x, odo_y)

def get_odo():
    with odo_lock:
        return odo_x, odo_y, odo_yaw

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "SPIN_SCAN"   # 시작 모드
mission_idx   = 0
detect_count  = 0
arrive_count  = 0

# SPIN_SCAN 세부
spin_start_t  = time.time()
spin_found    = False         # 회전 중 색상 발견 여부

# WANDER 세부
wander_state     = "STOP_PRE"  # STOP_PRE → FORWARD → STOP_MID → TURN → STOP_MID → ...
wander_state_t   = time.time()
wander_dir       = 1            # +1 or -1
wander_phase_idx = 0            # 배회 패턴 인덱스

# PARK 세부
park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
last_cmd      = (0.0, 0.0)

print(f"START | MISSION: {MISSION}")
print(">>> 제자리 회전 스캔 시작")
stop_robot()
time.sleep(0.3)   # 초기 정지 대기

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
            err_x         = cx_obj - cx_mid
            last_seen_x   = cx_obj
            last_bottom_y = min(by_top + bh, 239)
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # ══════════════════════════════════════════════════════════════
        # MODE: SPIN_SCAN  ─ 제자리 360° 회전으로 색상 탐색
        # ══════════════════════════════════════════════════════════════
        if mode == "SPIN_SCAN":
            elapsed = time.time() - spin_start_t

            if found:
                detect_count += 1
            else:
                detect_count = 0

            # 회전 중 색상 발견 → 즉시 PARK 모드로 전환
            if detect_count >= SPIN_DETECT_FRAMES:
                stop_robot()
                time.sleep(STOP_BETWEEN_SEC)
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                print(f"[SPIN_SCAN] [{target}] 발견 → 추적 시작")
                continue

            # 360° 회전 완료 → WANDER 모드로 전환
            if elapsed >= SPIN_FULL_SEC:
                stop_robot()
                time.sleep(STOP_BETWEEN_SEC)
                mode = "WANDER"
                wander_state   = "STOP_PRE"
                wander_state_t = time.time()
                wander_dir     = 1
                # 오도메트리 원점 리셋 (현재 위치가 새로운 탐색 원점)
                with odo_lock:
                    odo_x, odo_y, odo_yaw = 0.0, 0.0, 0.0
                odo_t = time.time()
                print(f"[SPIN_SCAN] 완료, 미발견 → 30cm 반경 배회 시작")
                continue

            # 회전 중 (정지 직후라면 램프업으로 부드럽게 가속)
            send_cmd(0.0, SPIN_W)
            update_odometry(0.0, SPIN_W)
            cv2.putText(frame, f"SPIN SCAN: {target}  {elapsed:.1f}s/{SPIN_FULL_SEC:.0f}s",
                        (10, 25), 0, 0.5, (255, 255, 255), 1)

        # ══════════════════════════════════════════════════════════════
        # MODE: WANDER  ─ 30cm 반경 배회 탐색
        # ══════════════════════════════════════════════════════════════
        elif mode == "WANDER":
            now = time.time()

            # 색상 발견 확인
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= SPIN_DETECT_FRAMES:
                stop_robot()
                time.sleep(STOP_BETWEEN_SEC)
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                print(f"[WANDER] [{target}] 발견 → 추적 시작")
                continue

            dist_from_origin = get_dist_from_origin()
            over_radius = dist_from_origin >= SEARCH_RADIUS_CM

            # ─ 배회 상태 머신 ────────────────────────────────────────
            if wander_state == "STOP_PRE":
                # 동작 전 짧은 정지
                stop_robot()
                if now - wander_state_t >= STOP_BETWEEN_SEC:
                    wander_state   = "FORWARD"
                    wander_state_t = now
                label = "WANDER STOP"

            elif wander_state == "FORWARD":
                elapsed_w = now - wander_state_t

                if over_radius:
                    # 반경 초과 → 즉시 정지 후 방향 전환
                    stop_robot()
                    wander_dir     = -wander_dir   # 반대 방향으로 전환
                    wander_state   = "STOP_MID"
                    wander_state_t = now
                    print(f"[WANDER] 반경 초과({dist_from_origin:.1f}cm) → 방향 전환")
                elif elapsed_w >= WANDER_STEP_SEC:
                    # 전진 시간 완료 → 정지 후 회전
                    stop_robot()
                    wander_state   = "STOP_MID"
                    wander_state_t = now
                else:
                    # 전진 중 (장애물 회피 포함)
                    if fm < THRESH_STOP:
                        v, w = 0.0, adir * 0.7
                        stop_robot()  # 장애물 직전 정지
                        wander_state   = "STOP_MID"
                        wander_state_t = now
                    elif fm < THRESH_TURN:
                        v, w = 0.10, adir * 0.5
                        send_cmd(v, w)
                        update_odometry(v, w)
                    else:
                        v, w = WANDER_V, WANDER_W_BASE * wander_dir * 0.3
                        send_cmd(v, w)
                        update_odometry(v, w)
                label = f"WANDER FWD  r={dist_from_origin:.0f}cm"

            elif wander_state == "STOP_MID":
                # 전진↔회전 사이 정지
                stop_robot()
                if now - wander_state_t >= STOP_BETWEEN_SEC:
                    wander_state   = "TURN"
                    wander_state_t = now
                label = "WANDER STOP"

            elif wander_state == "TURN":
                elapsed_w = now - wander_state_t
                if elapsed_w >= WANDER_TURN_SEC:
                    wander_dir     = -wander_dir        # 다음 턴은 반대 방향
                    stop_robot()
                    wander_state   = "STOP_PRE"
                    wander_state_t = now
                else:
                    # 원점 방향 편향: 원점과 멀수록 원점 쪽을 향하도록 보정
                    ox, oy, oyaw = get_odo()
                    angle_to_origin = math.atan2(-oy, -ox)   # 원점 방향 각도
                    angle_err = angle_to_origin - oyaw
                    # -π ~ π 정규화
                    angle_err = (angle_err + math.pi) % (2 * math.pi) - math.pi
                    # 반경 50% 초과 시 원점 방향 회전 가중치 추가
                    if dist_from_origin > SEARCH_RADIUS_CM * 0.5:
                        bias = np.clip(angle_err * 0.5, -0.4, 0.4)
                    else:
                        bias = 0.0
                    w = WANDER_W_BASE * wander_dir + bias
                    send_cmd(0.0, w)
                    update_odometry(0.0, w)
                label = f"WANDER TURN r={dist_from_origin:.0f}cm"

            else:
                label = "WANDER ?"

            cv2.putText(frame, f"WANDER [{target}] {label}", (10, 25), 0, 0.45, (255, 200, 0), 1)
            ox, oy, _ = get_odo()
            cv2.putText(frame, f"odo ({ox:.0f},{oy:.0f})cm", (10, 45), 0, 0.4, (200, 200, 200), 1)

        # ══════════════════════════════════════════════════════════════
        # MODE: LIDAR  ─ 일반 주행 (미션 간 이동)
        # ══════════════════════════════════════════════════════════════
        elif mode == "LIDAR":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                stop_robot()
                time.sleep(STOP_BETWEEN_SEC)
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                print(f"[LIDAR] [{target}] 발견 → 추적 시작")
                continue

            if fm < THRESH_STOP: v, w = 0.09, adir * 0.9
            elif fm < THRESH_TURN: v, w = 0.13, adir * 0.7
            elif fm < THRESH_SLOW: v, w = 0.18, adir * 0.4
            else: v, w = 0.28, 0.0
            send_cmd(v, w)
            cv2.putText(frame, "MODE: LIDAR", (10, 25), 0, 0.5, (255, 255, 255), 1)

        # ══════════════════════════════════════════════════════════════
        # MODE: PARK  ─ 색상 추적 & 주차
        # ══════════════════════════════════════════════════════════════
        elif mode == "PARK":

            # ── 1. 도착 후 전진 (FORWARD) ─────────────────────────
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

            # ── 2. 정차 (PARKING) ──────────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    if mission_idx < len(MISSION):
                        # 다음 미션: 제자리 회전 스캔으로 시작
                        spin_start_t = time.time()
                        detect_count = 0
                        mode = "SPIN_SCAN"
                        print(f"다음 미션 [{MISSION[mission_idx]}] → 제자리 회전 스캔")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 3. 객체 추적 (TRACK) ───────────────────────────────
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

            # ── 4. 놓침 → 소규모 재탐색 (SEARCH) ──────────────────
            else:
                park_state = "SEARCH"
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
