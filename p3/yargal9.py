import cv2
import serial
import numpy as np
import time
import math
import random
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
cap.set(cv2.CAP_PROP_BRIGHTNESS, -10)   # v4l2 기준값 0 대비 살짝 낮춤 (더 낮추려면 -20 등으로 조정)

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR ─────────────────────────────────────────────────────────────
EMA_ALPHA    = 0.35
MEDIAN_K     = 2
FRONT_RANGE  = 90   # 정면 감지 범위 (±90도 = 총 180도, 충돌 회피용 광각)
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
    """광각(±90도) 정면 최소 거리 — 충돌 회피/감속 판단용."""
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
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]),
               "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]),
               "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 95], [138, 207, 246]),
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

# 도착 판정 영역
ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8

# 직진 탐색 (GO_STRAIGHT)
GO_V = 0.20             # 직진 속도

# 지그재그(좌앞/우앞) 탐색 파라미터
ZIGZAG_MIN_SEC = 1.0     # 좌/우 한 쪽으로 유지하는 최소 시간
ZIGZAG_MAX_SEC = 2.0     # 좌/우 한 쪽으로 유지하는 최대 시간
ZIGZAG_W       = 1.1     # 지그재그 회전 강도 (강하게 꺾이는 좌앞/우앞)
SEARCH_W       = 1.4     # SEARCH 상태 제자리 회전 속도

# 장애물 회피 시 사용하는 회전 속도(직진 v는 그대로, 급커브용 보조값)
WALL_TURN_V    = 0.10

# ── STATE ─────────────────────────────────────────────────────────────
# state 종류:
#   GO_STRAIGHT  : 목표 색지를 찾을 때까지, 좌앞/우앞을 번갈아 가며(지그재그) 전진 + 라이다 단순 회피
#   TRACK        : 카메라로 색지를 추적
#   SEARCH       : 추적 중 색지를 놓쳐서 제자리 탐색 (기존 방식 유지)
#   FORWARD      : 도착 판정 후 일정 시간 직진
#   PARKING      : 정차

state         = "GO_STRAIGHT"
mission_idx   = 0
detect_count  = 0
arrive_count  = 0

park_t        = None
last_seen_x   = 160
last_bottom_y = 0
last_cmd      = (0.0, 0.0)

# ── 지그재그 상태 변수 ────────────────────────────────────────────────
# zigzag_sign : 현재 좌(+1)/우(-1) 중 어느 쪽으로 틀고 있는지 (시간 기반 토글)
# zigzag_bias : 장애물 회피 이후 "원래 돌아가야 할 방향" 보정값 (+1 또는 -1)
#               회피 시 장애물 반대방향(adir)으로 피했다면, 그 다음부터는
#               장애물이 있던 방향(=피한 반대, 즉 -adir)을 기준으로 지그재그가
#               흔들리도록 bias를 -adir로 갱신한다. 다음 장애물을 만나면 다시 갱신.
zigzag_sign      = 1
zigzag_bias      = 1
zigzag_next_t    = time.time() + random.uniform(ZIGZAG_MIN_SEC, ZIGZAG_MAX_SEC)
was_avoiding     = False   # 직전 프레임에 라이다 회피 중이었는지 (회피 종료 시점 검출용)

print(f"START | MISSION: {MISSION}")

def zigzag_update_and_dir():
    """지그재그 좌/우 부호를 시간 기반으로 토글하고, bias를 곱한 실제 방향을 반환."""
    global zigzag_sign, zigzag_next_t
    now = time.time()
    if now >= zigzag_next_t:
        zigzag_sign *= -1
        zigzag_next_t = now + random.uniform(ZIGZAG_MIN_SEC, ZIGZAG_MAX_SEC)
    return zigzag_sign * zigzag_bias

# ── 직진(지그재그) + 라이다 단순 회피 ────────────────────────────────
def go_straight_cmd(fm, adir):
    """
    장애물이 없으면 좌앞/우앞을 번갈아가며 완만하게 지그재그 전진.
    장애물을 만나면 기존처럼 adir 반대방향(장애물 반대쪽)으로 피하고,
    회피가 끝나는 순간 zigzag_bias를 갱신해 이후 지그재그가
    장애물이 있던 방향 쪽으로 다시 틀어지도록 만든다.
    """
    global zigzag_bias, was_avoiding

    avoiding_now = fm < THRESH_SLOW

    if avoiding_now:
        was_avoiding = True
        if fm < THRESH_STOP:
            return (0.08, adir * 1.1)
        elif fm < THRESH_TURN:
            return (WALL_TURN_V, adir * 0.85)
        else:
            blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
            v = GO_V * (1.0 - 0.5 * blend)
            w = adir * 0.5 * blend
            return (v, w)
    else:
        # 회피가 막 끝난 시점이면 → 지그재그 편향을 "장애물이 있던 방향"으로 보정
        if was_avoiding:
            zigzag_bias = -adir   # adir: 회피 시 튼 방향(장애물 반대쪽) → 그 반대가 장애물 방향
            was_avoiding = False
        zdir = zigzag_update_and_dir()
        return (GO_V, zdir * ZIGZAG_W)

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
        fm     = front_min(scan)          # 광각(±90도) — 일반 회피, 충돌 안전용
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

        # ══ GO_STRAIGHT: 지그재그(좌앞/우앞) 전진하며 목표 색지 탐색 ══
        if state == "GO_STRAIGHT":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                state = "TRACK"
                print(f"[{target}] 발견 → 추적 시작")
                continue

            v, w = go_straight_cmd(fm, adir)
            send_cmd(v, w)
            cv2.putText(frame, f"GO-STRAIGHT(ZIGZAG) [{target}] fm:{fm:.0f}",
                        (10, 25), 0, 0.5, (0, 255, 0), 1)

        # ══ TRACK: 카메라로 색지 추적 ═══════════════════════════════
        elif state == "TRACK":
            if found:
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    state = "FORWARD"
                    park_t = time.time()
                    print(f"[{target}] centroid {ARRIVE_CONFIRM}프레임 확정 → {ARRIVE_FORWARD_SEC}초 전진")
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
                state = "SEARCH"
                arrive_count = 0
                v = 0.0
                w = (-SEARCH_W if last_seen_x > cx_mid else SEARCH_W)
                send_cmd(v, w)
                cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        # ══ SEARCH: 추적 중 놓친 색지를 제자리에서 다시 탐색 (기존 방식 유지) ══
        elif state == "SEARCH":
            if found:
                state = "TRACK"
                continue
            v = 0.0
            w = (-SEARCH_W if last_seen_x > cx_mid else SEARCH_W)
            send_cmd(v, w)
            cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        # ══ FORWARD: 도착 판정 후 일정 시간 전진 ════════════════════
        elif state == "FORWARD":
            elapsed = time.time() - park_t
            if elapsed >= ARRIVE_FORWARD_SEC:
                stop_robot()
                state = "PARKING"
                park_t = time.time()
                print(f"[{target}] 전진 완료 → 정차")
            else:
                send_cmd(*last_cmd)
            cv2.putText(frame, f"FORWARD: {target}", (10, 25), 0, 0.6, draw, 2)

        # ══ PARKING: 정차, 다음 미션으로 전환 ════════════════════════
        elif state == "PARKING":
            stop_robot()
            elapsed = time.time() - park_t
            if elapsed >= PARK_SEC:
                mission_idx += 1
                arrive_count = 0
                detect_count = 0
                if mission_idx < len(MISSION):
                    state = "GO_STRAIGHT"
                    print(f"다음 미션 [{MISSION[mission_idx]}] 직진 탐색 시작")
                continue
            cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

        cv2.putText(frame, f"MISSION:{target} STATE:{state}", (10, 45),
                    0, 0.5, (255, 255, 255), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
