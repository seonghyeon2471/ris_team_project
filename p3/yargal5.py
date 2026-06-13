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

def left_dist(scan):
    # 왼쪽 85~95도 구간 평균 거리 (cm)
    idx = np.arange(85, 96) % 360
    return float(np.mean(scan[idx]))

def side_min(scan, start, end):
    # 지정 각도 구간 최솟값
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

def wall_follow(scan, fm, adir):
    ld = left_dist(scan)

    # 측면 근접 장애물 감지 (좌: 60~120도, 우: 240~300도)
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)

    # 1. 정면 위험 -> 긴급 회피 (adir로 덜 막힌 쪽)
    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    # 2. 측면 너무 가까움 -> 반대쪽으로 밀어내기
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

    # 3. 왼쪽 장애물 없음 -> 좌회전해서 찾기
    if ld > WALL_TARGET * 2.0:
        return (WALL_V * 0.7, WALL_LOST_W)

    # 4. 정상 wall-following: 왼쪽 거리 오차 비례 보정
    err = ld - WALL_TARGET  # 양수: 너무 멀다->좌회전, 음수: 너무 가깝다->우회전
    w = WALL_KP * err

    # 정면이 THRESH_SLOW 이내면 adir 회피 혼합
    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V

    w = float(np.clip(w, -0.9, 0.9))
    return (v, w)

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
MIN_AREA       = 400
KP_ROT         = 0.030   # 픽셀당 회전속도
W_MIN          = 0.25    # 최소 회전 속도
APPROACH_V     = 0.17
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

# ── WALL FOLLOWING (LIDAR 탐색 모드) ────────────────────────────────
WALL_TARGET   = 25.0   # 왼쪽 장애물까지 유지할 목표 거리 (cm)
WALL_KP       = 0.012  # 거리 오차 → 회전 비례 게인
WALL_RANGE    = 20     # 왼쪽 측정에 쓸 각도 범위 (85~95도 평균)
WALL_V        = 0.22   # wall-following 전진 속도
WALL_TURN_V   = 0.10   # 정면 장애물 회피 시 전진 속도
WALL_LOST_W   = 0.4    # 왼쪽 벽 놓쳤을 때 좌회전 강도 (벽 찾기)

# ── 도착 판정 영역 ────────────────────────────────────────────────────
ARRIVE_Y_TOP    = int(240 * 0.85)  # 세로 기준선
ARRIVE_X_MARGIN = 40               # 화면 중앙에서 ±40px 이내

# 도착 후 이전 속도로 전진하는 시간 (초)
ARRIVE_FORWARD_SEC = 1.0

# centroid가 연속 이 프레임 수만큼 영역 안에 있어야 도착 판정
ARRIVE_CONFIRM     = 8

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

        # centroid 계산
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

        # 도착 판정 영역 시각화
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

            v, w = wall_follow(scan, fm, adir)
            send_cmd(v, w)
            ld_disp = left_dist(scan)
            cv2.putText(frame, f"WALL-FOLLOW L:{ld_disp:.0f}cm", (10, 25),
                        0, 0.5, (255, 255, 255), 1)

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
                    send_cmd(*last_cmd)  # 도착 직전 v, w 그대로 유지
                cv2.putText(frame, f"FORWARD: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 2. 정차 중 (PARKING) ───────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    if mission_idx < len(MISSION):
                        park_state = "SEARCH"
                        last_seen_x = cx_mid + 40
                        print(f"다음 미션 [{MISSION[mission_idx]}] 탐색 회전 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 3. 객체 추적 중 (TRACK) ────────────────────────────
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
                    send_cmd(*last_cmd)
                    continue

                else:
                    err_x = cx_obj - cx_mid

                    def cam_w(ex):
                        raw = -KP_ROT * ex
                        if abs(raw) < W_MIN and ex != 0:
                            return -W_MIN if ex > 0 else W_MIN
                        return raw

                    # 오차가 클수록 전진속도 줄이고 회전 위주로
                    # err_x=0(정중앙) → v=APPROACH_V, err_x=±cx_mid(화면 끝) → v=0
                    err_ratio = min(abs(err_x) / float(cx_mid), 1.0)
                    reduced_v = APPROACH_V * (1.0 - err_ratio)

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

            # ── 4. 객체 놓침 / 탐색 (SEARCH) ──────────────────────
            else:
                park_state = "SEARCH"
                arrive_count = 0
                v = 0.0
                w = (-1.3 if last_seen_x > cx_mid else 1.3)
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
