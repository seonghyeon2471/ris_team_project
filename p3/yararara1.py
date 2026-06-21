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
FRONT_RANGE  = 60      # 변경: 90° → 60° (실내 측벽 오반응 방지)
THRESH_SLOW  = 55.0
THRESH_TURN  = 30.0    # 변경: 35.0 → 30.0
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

# ── WALL-FOLLOWING HELPERS ────────────────────────────────────────────
def side_dist(scan, side):
    """측면(90° or 270°) 부근 평균 거리 반환"""
    if side == "RIGHT":
        idx = np.arange(90 - WF_SIDE_RANGE, 90 + WF_SIDE_RANGE + 1) % 360
    else:  # LEFT
        idx = np.arange(270 - WF_SIDE_RANGE, 270 + WF_SIDE_RANGE + 1) % 360
    return float(np.mean(scan[idx]))

def choose_wall_side(scan):
    """전방 장애물 감지 시 더 가까운 쪽 벽을 선택"""
    right = side_dist(scan, "RIGHT")
    left  = side_dist(scan, "LEFT")
    # 더 가까운 쪽(먼저 만난 쪽)을 따라감
    return "RIGHT" if right <= left else "LEFT"

def wall_follow_cmd(scan, side, prev_err):
    """
    선택한 벽을 WF_TARGET_DIST 거리로 유지하며 전진 (PD 제어).
    코너/탈출 판단은 호출부에서 처리.
    반환: (v, w, new_err)
    """
    d    = side_dist(scan, side)
    err  = d - WF_TARGET_DIST   # 양수: 벽에서 멀어짐, 음수: 너무 가까움
    derr = err - prev_err

    sign = 1.0 if side == "RIGHT" else -1.0
    w = sign * (WF_KP * err + WF_KD * derr)
    w = float(np.clip(w, -1.2, 1.2))

    # 전방 장애물 긴급 처리 — THRESH_SLOW부터 미리 꺾기 시작
    fm_local = front_min(scan)
    if fm_local < THRESH_STOP:
        return 0.0, -sign * WF_TURN_W, err
    elif fm_local < THRESH_TURN:
        return 0.0, -sign * WF_TURN_W, err          # 정지 + 최대 회전
    elif fm_local < THRESH_SLOW:
        return 0.07, -sign * WF_TURN_W * 0.85, err  # 감속 + 강한 회전
    else:
        return WF_V, w, err

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
W_MIN          = 0.20    # 변경: 0.25 → 0.20
APPROACH_V     = 0.13    # 변경: 0.17 → 0.13
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 30               # 변경: 40 → 30
ARRIVE_FORWARD_SEC = 0.8              # 변경: 0.7 → 0.8
ARRIVE_FORWARD_V   = 0.13            # 변경: 0.15 → 0.13
ARRIVE_CONFIRM     = 8

# ── WALL-FOLLOWING PARAMS ─────────────────────────────────────────────
WF_ENGAGE_DIST  = 65.0   # cm — 전방 이 거리 이내 장애물 감지 시 WALL 진입
WF_TARGET_DIST  = 20.0   # cm — 벽과 유지할 목표 거리 (30 → 20)
WF_SIDE_RANGE   = 20     # 측면 각도 범위 (90°±20° / 270°±20°)
WF_KP           = 0.020  # PD 비례 게인
WF_KD           = 0.008  # PD 미분 게인
WF_V            = 0.15   # wall-following 전진 속도
WF_TURN_W       = 1.3    # 전방 긴급 회전 속도

# 코너 감지 & 오버슈트 파라미터
# 측면 벽이 갑자기 사라지면(거리가 크게 늘면) 코너로 판정
WF_CORNER_SIDE_THRESH = 45.0  # cm — 측면 거리가 이 값 초과 시 코너 감지
WF_CORNER_CONFIRM     = 5     # 코너 판정 연속 프레임 수 (노이즈 오판 방지)
WF_CORNER_FWD_SEC     = 2.0   # 초 — 코너 감지 후 직진 시간 (바퀴 걸림 방지)
WF_CORNER_FWD_V       = 0.15  # 코너 직진 속도
WF_CORNER_TURN_W      = 1.3   # 코너 직진 후 회전 속도 (벽 방향으로 꺾기)

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"   # "LIDAR" | "WALL" | "PARK"
mission_idx   = 0
detect_count  = 0
arrive_count  = 0

park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
last_cmd      = (0.0, 0.0)

# wall-following 상태
wf_side         = None   # "LEFT" | "RIGHT"
wf_prev_err     = 0.0    # PD 미분항용 이전 오차
wf_corner_state = "NONE" # "NONE" | "FWD" | "TURN"  — 코너 처리 단계
wf_corner_t     = None   # 코너 직진/회전 시작 시각
wf_corner_cnt   = 0      # 코너 연속 감지 카운터

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

        # ══ 공통: 색지 발견 시 PARK 전환 (LIDAR / WALL 공통) ═══════
        if mode in ("LIDAR", "WALL"):
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                wf_side         = None
                wf_prev_err     = 0.0
                wf_corner_state = "NONE"
                wf_corner_cnt   = 0
                mode        = "PARK"
                park_state  = "TRACK"
                print(f"[{target}] 발견 → PARK 모드")
                continue

        # ══ LIDAR 모드 ═══════════════════════════════════════════
        if mode == "LIDAR":
            if fm < WF_ENGAGE_DIST:
                # 장애물 감지 → 어느 쪽 벽 따라갈지 결정 후 WALL 진입
                wf_side     = choose_wall_side(scan)
                wf_prev_err = 0.0
                mode        = "WALL"
                print(f"장애물 감지 (fm={fm:.1f}cm) → WALL [{wf_side}] 모드")
                continue

            # 장애물 없음: 직진
            v, w = 0.28, 0.0
            send_cmd(v, w)
            cv2.putText(frame, "MODE: LIDAR", (10, 25), 0, 0.5, (255, 255, 255), 1)

        # ══ WALL-FOLLOWING 모드 ══════════════════════════════════
        elif mode == "WALL":
            sign = 1.0 if wf_side == "RIGHT" else -1.0
            sd   = side_dist(scan, wf_side)
            col  = (0, 255, 128)

            # ── 코너 직진 단계 (FWD) ──────────────────────────────
            if wf_corner_state == "FWD":
                elapsed = time.time() - wf_corner_t
                if elapsed >= WF_CORNER_FWD_SEC:
                    wf_corner_state = "TURN"
                    wf_corner_t     = time.time()
                    print(f"코너 직진 완료 → 회전 시작 [{wf_side}]")
                else:
                    send_cmd(WF_CORNER_FWD_V, 0.0)
                cv2.putText(frame, f"CORNER FWD {elapsed:.1f}s", (10, 25), 0, 0.5, col, 1)

            # ── 코너 회전 단계 (TURN) ─────────────────────────────
            elif wf_corner_state == "TURN":
                # 측면에 벽이 다시 잡히면 회전 종료 → 일반 following 복귀
                if sd < WF_CORNER_SIDE_THRESH:
                    wf_corner_state = "NONE"
                    wf_prev_err     = 0.0
                    print(f"코너 완료 → wall-following 재개")
                else:
                    # 벽 방향으로 꺾기 (sign 반전: 벽이 있던 쪽으로 돌아야 함)
                    send_cmd(WF_CORNER_FWD_V * 0.5, sign * WF_CORNER_TURN_W)
                cv2.putText(frame, "CORNER TURN", (10, 25), 0, 0.5, col, 1)

            # ── 일반 wall-following ────────────────────────────────
            else:
                # 장애물 통과 판정: 전방 열리고 측면 벽도 없어짐
                if fm >= WF_ENGAGE_DIST and sd > WF_CORNER_SIDE_THRESH:
                    mode    = "LIDAR"
                    wf_side = None
                    wf_corner_state = "NONE"
                    print("장애물 완전 통과 → LIDAR 복귀")
                    continue

                # 코너 감지: 측면 벽이 갑자기 사라짐 (전방은 아직 막혀 있음)
                # 코너 감지: 측면 벽 사라짐 + 전방 열림 → N프레임 연속 확인
                if sd > WF_CORNER_SIDE_THRESH and fm >= THRESH_SLOW:
                    wf_corner_cnt += 1
                else:
                    wf_corner_cnt = 0

                if wf_corner_cnt >= WF_CORNER_CONFIRM:
                    wf_corner_cnt   = 0
                    wf_corner_state = "FWD"
                    wf_corner_t     = time.time()
                    print(f"코너 확정 (side={sd:.0f}cm, fm={fm:.0f}cm) → {WF_CORNER_FWD_SEC}초 직진")
                    send_cmd(WF_CORNER_FWD_V, 0.0)
                    continue

                v, w, wf_prev_err = wall_follow_cmd(scan, wf_side, wf_prev_err)
                send_cmd(v, w)
                cv2.putText(frame, f"WALL [{wf_side}] fm={fm:.0f}", (10, 25), 0, 0.5, col, 1)
                cv2.putText(frame, f"side={sd:.0f}cm", (10, 45), 0, 0.5, col, 1)

        # ══ PARK 모드 ════════════════════════════════════════════
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
                    mode            = "LIDAR"
                    park_state      = "TRACK"
                    detect_count    = 0
                    arrive_count    = 0
                    last_cmd        = (0.0, 0.0)
                    wf_side         = None
                    wf_prev_err     = 0.0
                    wf_corner_state = "NONE"
                    wf_corner_cnt   = 0
                    if mission_idx < len(MISSION):
                        print(f"다음 미션 [{MISSION[mission_idx]}] → LIDAR 모드")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

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


        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
