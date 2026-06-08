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
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR PARAMS ──────────────────────────────────────────────────────
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 45       # 전방 판단 범위 ±45°

# 라이다 거리 임계값 (cm)
THRESH_SLOW = 55.0     # 감속 시작
THRESH_TURN = 35.0     # 회피 조향 개입
THRESH_STOP = 18.0     # 강한 회피

_scan     = np.full(360, 150.0, dtype=np.float32)
_scan_pub = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

def _ema(a, d):
    if d > 0:
        _scan[a] = (1 - EMA_ALPHA) * _scan[a] + EMA_ALPHA * d

def _median():
    k   = MEDIAN_K
    buf = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx    = [(i + d) % 360 for d in range(-k, k + 1)]
        buf[i] = np.sort(_scan[idx])[k]
    _scan[:] = buf

def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150:
            _ema(angle, dist_cm)
        if sf == 1:
            _median()
            with scan_lock:
                _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock:
        return _scan_pub.copy()

def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    # 왼쪽(1~90°)과 오른쪽(271~360°) 평균 비교 → 공간 넓은 쪽으로
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# ── CAMERA GEOMETRY ───────────────────────────────────────────────────
CAM_H      = 73.0
CAM_PITCH  = 30.0
CAM_FOV_V  = 50.0
RES_W, RES_H = 320, 240
FY = (RES_H / 2) / math.tan(math.radians(CAM_FOV_V / 2))
CY = RES_H / 2

def px_to_dist(y):
    deg = CAM_PITCH + math.degrees(math.atan2(y - CY, FY))
    return CAM_H / math.tan(math.radians(deg)) if deg > 0 else float('inf')

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([0,  60, 120], [12,  255, 255]),
               "hsv2": ([160, 60, 120], [179, 255, 255]),
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([18, 15, 180], [45,  255, 255]),
               "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([95, 50,  50], [130, 255, 255]),
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

# ── PARK PARAMS ───────────────────────────────────────────────────────
MIN_AREA     = 400
TARGET_AREA  = 6000
KP_ROT       = 0.003
APPROACH_V   = 0.22
BLIND_V      = 0.28
BLIND_SEC    = 1.7
PARK_SEC     = 3.0
ALIGN_MARGIN = 15      # px: 중심 일치 허용 오차
ALIGN_V      = 0.20
ALIGN_SEC    = 0.15
APPROACH_MAX = 4.5

# 라이다-카메라 융합 가중치 임계
# 장애물이 가까울수록 α→1 (라이다 주도), 멀수록 α→0 (카메라 주도)
def blend_w(fm, w_cam, w_lidar):
    """전방 거리(fm)에 따라 카메라/라이다 각속도를 블렌딩"""
    if fm < THRESH_STOP:
        return w_lidar          # 라이다 100%
    elif fm < THRESH_TURN:
        alpha = 0.7
        return alpha * w_lidar + (1 - alpha) * w_cam
    elif fm < THRESH_SLOW:
        alpha = 0.3
        return alpha * w_lidar + (1 - alpha) * w_cam
    else:
        return w_cam            # 카메라 100%

def lidar_v(fm, base_v):
    """전방 거리에 따라 선속도 결정"""
    if fm < THRESH_STOP:
        return 0.09
    elif fm < THRESH_TURN:
        return min(base_v, 0.13)
    elif fm < THRESH_SLOW:
        return min(base_v, 0.18)
    else:
        return base_v

# ── STATE ─────────────────────────────────────────────────────────────
# mode: "LIDAR" | "PARK"
mode         = "LIDAR"
mission_idx  = 0
park_state   = "SEARCH"   # SEARCH / TRACK / APPROACH / BLIND / ALIGN / PARKING
last_seen_x  = 160
last_dist_cm = 150.0
blind_t      = None
align_t      = None
park_t       = None
approach_t   = None

# 연속 인식 확정 카운터 (오인식 방지)
DETECT_CONFIRM = 6
detect_count   = 0

print("▶ START — LIDAR mode")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame  = cv2.flip(frame, 1)
        H, W   = frame.shape[:2]
        cx_mid = W // 2
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan   = get_scan()
        fm     = front_min(scan)
        adir   = avoid_dir(scan)

        # ── 미션 완료 ─────────────────────────────────────────────────
        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL DONE", (60, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        # ══════════════════════════════════════════════════════════════
        # LIDAR 주행 모드
        # ══════════════════════════════════════════════════════════════
        if mode == "LIDAR":
            # 장애물 회피 v/w
            if fm < THRESH_STOP:
                v, w = 0.09, adir * 0.9
            elif fm < THRESH_TURN:
                v, w = 0.13, adir * 0.7
            elif fm < THRESH_SLOW:
                v, w = 0.18, adir * 0.4
            else:
                v, w = 0.28, 0.0

            # 색지 연속 인식 확정
            mask = make_mask(frame, hsv, target)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            big = max(cnts, key=cv2.contourArea) if cnts else None
            if big is not None and cv2.contourArea(big) > MIN_AREA:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode         = "PARK"
                park_state   = "TRACK"
                blind_t      = None
                approach_t   = None
                stop_robot()
                print(f"🎯 [{target}] 인식 확정 → PARK 모드 전환")
            else:
                send_cmd(v, w)

            cv2.putText(frame, f"LIDAR  front={fm:.0f}cm  [{detect_count}/{DETECT_CONFIRM}]",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)
            cv2.putText(frame, f"TARGET: {target}", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 1)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        # ══════════════════════════════════════════════════════════════
        # PARK 모드
        # ══════════════════════════════════════════════════════════════

        # ─ PARKING 정차 대기 ──────────────────────────────────────────
        if park_state == "PARKING":
            stop_robot()
            elapsed = time.time() - park_t
            remain  = max(0.0, PARK_SEC - elapsed)
            cv2.putText(frame, f"PARKING [{target}]  {remain:.1f}s",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw, 2)
            cv2.imshow("f", frame)
            if elapsed >= PARK_SEC:
                mission_idx += 1
                mode         = "LIDAR"
                park_state   = "SEARCH"
                blind_t      = None
                last_dist_cm = 150.0
                print(f"✅ [{target}] 완료 → LIDAR 복귀")
            cv2.waitKey(1)
            continue

        # ─ ALIGN 정밀 전진 ────────────────────────────────────────────
        if park_state == "ALIGN":
            if time.time() - align_t < ALIGN_SEC:
                send_cmd(ALIGN_V, 0.0)
                cv2.putText(frame, "ALIGN FWD", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 150, 0), 2)
                cv2.imshow("f", frame)
                cv2.waitKey(1)
                continue
            park_state = "PARKING"
            park_t     = time.time()
            continue

        # ─ APPROACH 타임아웃 안전장치 ────────────────────────────────
        if park_state == "APPROACH" and approach_t and \
                time.time() - approach_t > APPROACH_MAX:
            print("🚨 APPROACH timeout → PARKING")
            park_state = "PARKING"
            park_t     = time.time()
            continue

        # ─ 마스크 & 컨투어 ────────────────────────────────────────────
        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big   = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        v, w = 0.0, 0.0

        if found:
            area       = cv2.contourArea(big)
            bx, by_top, bw, bh = cv2.boundingRect(big)

            # ★ x축 가로 중심 (boundingRect 기반)
            ox    = bx + bw // 2
            oy    = by_top + bh // 2   # 시각화용
            err_x = ox - cx_mid        # 양수=오른쪽, 음수=왼쪽

            last_seen_x = ox

            # 하단 y로 거리 추정
            by_bot = min(by_top + bh, RES_H - 1)
            d      = px_to_dist(by_bot)
            if d != float('inf'):
                last_dist_cm = d

            # 시각화
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), (0, 255, 0), 2)
            cv2.circle(frame, (ox, oy), 5, (0, 255, 0), -1)
            cv2.line(frame, (cx_mid, 0), (cx_mid, H), (100, 100, 100), 1)   # 화면 중심선
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 1)  # 색지 중심선

            # 카메라 기반 각속도
            w_cam   = -KP_ROT * err_x
            # 라이다 기반 각속도 (회피 방향)
            w_lidar = adir * 0.7

            # ─ 중심 일치 → ALIGN ──────────────────────────────────
            if abs(err_x) <= ALIGN_MARGIN and park_state in ["TRACK", "APPROACH"]:
                park_state = "ALIGN"
                align_t    = time.time()
                print(f"🎯 중심 일치 (err={err_x}px) → ALIGN")
                stop_robot()
                continue

            # ─ APPROACH 조건 ──────────────────────────────────────
            if park_state == "APPROACH" or area > TARGET_AREA or last_dist_cm < 65.0:
                if park_state != "APPROACH":
                    park_state = "APPROACH"
                    approach_t = time.time()
                base_v = APPROACH_V
            else:
                park_state = "TRACK"
                rem    = max(0.0, TARGET_AREA - area)
                base_v = np.clip(0.10 + 0.14 * (rem / TARGET_AREA), 0.10, 0.24)

            # ★ 라이다-카메라 융합
            v = lidar_v(fm, base_v)
            w = blend_w(fm, w_cam, w_lidar)

        else:
            # ─ 객체 유실 ──────────────────────────────────────────
            if park_state in ["APPROACH", "BLIND"]:
                if blind_t is None:
                    blind_t = time.time()
                    print("🚀 BLIND DASH 시작")
                v, w       = BLIND_V, 0.0
                park_state = "BLIND"
                if time.time() - blind_t > BLIND_SEC:
                    park_state = "PARKING"
                    park_t     = time.time()
                    print("🅿️ 블라인드 완료 → PARKING")
            else:
                # SEARCH: 마지막 방향으로 회전
                v          = 0.03
                w          = -1.30 if last_seen_x > cx_mid else 1.30
                park_state = "SEARCH"

        send_cmd(v, w)

        # ─ 디스플레이 ─────────────────────────────────────────────
        cv2.putText(frame, f"PARK:{park_state}  [{target}]  d={last_dist_cm:.0f}cm  front={fm:.0f}cm",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.50, draw, 1)
        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("INTERRUPTED")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
    print("SHUTDOWN")
