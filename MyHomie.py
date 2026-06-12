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

# ── LIDAR ─────────────────────────────────────────────────────────────
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 45
THRESH_SLOW = 55.0
THRESH_TURN = 35.0
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

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red": {
        "hsv1": ([169, 168,  96], [179, 222, 157]),
        "hsv2": None,
        "bgr":  ([20,  20,   80], [255, 255, 255]),
        "draw": (0, 0, 255),
    },
    "yellow": {
        "hsv1": ([16,  137, 142], [30,  214, 195]),
        "hsv2": None,
        "bgr":  ([0,   80,   80], [255, 255, 255]),
        "draw": (0, 200, 255),
    },
    "blue": {
        "hsv1": ([106, 168,  54], [131, 210,  82]),
        "hsv2": None,
        "bgr":  ([40,   0,    0], [255, 220, 220]),
        "draw": (255, 80, 0),
    },
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
KP_ROT         = 0.003
KD_ROT         = 0.001   # PD제어 미분항
APPROACH_V     = 0.22
PARK_SEC       = 1.2
DETECT_CONFIRM = 6
BOTTOM_10PCT   = int(240 * 0.90)   # y=216 하단 판정선
BOTTOM_CONFIRM = 4                  # 연속 N프레임 하단 확인

# 주차 판정 X 구역: 양옆 10% 제외 → 중앙 80%만 주차 판정
# W=320 기준: 좌 32px ~ 우 288px
PARK_X_LEFT  = int(320 * 0.10)   # 32px
PARK_X_RIGHT = int(320 * 0.90)   # 288px

# 양옆 구역 진입 시 각속도
SIDE_W = 0.6

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
mission_idx   = 0
detect_count  = 0

park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0
bottom_count  = 0
park_t        = None
prev_err_x    = 0   # PD제어용 이전 오차

print(f"START | MISSION: {MISSION}")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        # 버퍼 비우고 최신 프레임
        for _ in range(3):
            cap.grab()
        ret, frame = cap.retrieve()
        if not ret: continue

        frame  = cv2.flip(frame, 1)
        H, W   = frame.shape[:2]
        cx_mid = W // 2
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan   = get_scan()
        fm     = front_min(scan)
        adir   = avoid_dir(scan)

        # 전체 미션 완료
        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big   = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        ox = cx_mid  # 색상 중심 x 기본값
        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            ox     = bx + bw // 2
            by_bot = min(by_top + bh, 239)
            err_x  = ox - cx_mid
            last_seen_x   = ox
            last_bottom_y = by_bot

            # 바운딩박스 그리기
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)
            cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)

        # 주차 판정 구역 시각화
        cv2.line(frame, (PARK_X_LEFT,  0), (PARK_X_LEFT,  H), (100, 100, 255), 1)
        cv2.line(frame, (PARK_X_RIGHT, 0), (PARK_X_RIGHT, H), (100, 100, 255), 1)

        # ══ LIDAR 모드 ═══════════════════════════════════════════
        if mode == "LIDAR":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                bottom_count = 0
                prev_err_x   = 0
                mode         = "PARK"
                park_state   = "TRACK"
                print(f"[{target}] 발견 → PARK 모드 진입")
                continue

            if fm < THRESH_STOP:   v, w = 0.09, adir * 0.9
            elif fm < THRESH_TURN: v, w = 0.13, adir * 0.7
            elif fm < THRESH_SLOW: v, w = 0.18, adir * 0.4
            else:                  v, w = 0.28, 0.0
            send_cmd(v, w)
            cv2.putText(frame, "MODE: LIDAR", (10, 25), 0, 0.5, (255, 255, 255), 1)

        # ══ PARK 모드 ════════════════════════════════════════════
        elif mode == "PARK":

            # 1. 정차 중 (PARKING)
            if park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                cv2.putText(frame, f"PARKING {PARK_SEC - elapsed:.1f}s",
                            (10, 25), 0, 0.6, draw, 2)
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    if mission_idx < len(MISSION):
                        mode         = "LIDAR"
                        detect_count = 0
                        bottom_count = 0
                        prev_err_x   = 0
                        last_seen_x  = cx_mid
                        print(f"주차 완료 → LIDAR 복귀 | 다음: {MISSION[mission_idx]}")
                    else:
                        print("모든 미션 완료")

            # 2. 객체 추적 중 (TRACK)
            elif found:
                park_state = "TRACK"

                # ── 양옆 10% 구역 판정 ──────────────────────────
                in_left_zone  = ox < PARK_X_LEFT
                in_right_zone = ox > PARK_X_RIGHT
                in_center_zone = not in_left_zone and not in_right_zone

                # 하단 주차 판정은 중앙 80% 구역에서만
                if by_bot >= BOTTOM_10PCT and in_center_zone:
                    bottom_count += 1
                else:
                    bottom_count = 0

                if bottom_count >= BOTTOM_CONFIRM:
                    park_state   = "PARKING"
                    park_t       = time.time()
                    bottom_count = 0
                    print(f"[{target}] 도착 확정 → PARKING")
                    continue

                # ── PD 조향 계산 ─────────────────────────────────
                err_x  = ox - cx_mid
                d_err  = err_x - prev_err_x
                prev_err_x = err_x
                w_cam  = -KP_ROT * err_x - KD_ROT * d_err

                # 양옆 구역이면 해당 방향으로 각속도 추가
                if in_left_zone:
                    w_cam = SIDE_W    # 왼쪽 → 왼쪽으로 회전
                    cv2.putText(frame, "LEFT ZONE", (10, 55), 0, 0.5, (0, 255, 255), 1)
                elif in_right_zone:
                    w_cam = -SIDE_W   # 오른쪽 → 오른쪽으로 회전
                    cv2.putText(frame, "RIGHT ZONE", (10, 55), 0, 0.5, (0, 255, 255), 1)

                # 라이다 + 카메라 조향 합산
                if fm >= THRESH_SLOW:
                    v, w = APPROACH_V, w_cam
                else:
                    w_lid = adir * 0.7
                    if fm < THRESH_STOP:
                        v, w = 0.09, w_lid
                    elif fm < THRESH_TURN:
                        v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                    else:
                        v, w = 0.18, 0.3 * w_lid + 0.7 * w_cam

                send_cmd(v, w)
                cv2.putText(frame, f"TRACKING err:{err_x:+d}", (10, 25), 0, 0.6, draw, 1)

            # 3. 색상 놓침
            else:
                bottom_count = 0
                prev_err_x   = 0

                # 마지막으로 본 위치가 하단이었으면 → 사라진 것 → PARKING
                if last_bottom_y >= BOTTOM_10PCT:
                    park_state   = "PARKING"
                    park_t       = time.time()
                    print(f"[{target}] 화면 아래로 사라짐 → PARKING")
                else:
                    # 위쪽에서 놓친 것 → 탐색
                    park_state = "SEARCH"
                    v = 0.0
                    w = -1.3 if last_seen_x > cx_mid else 1.3
                    send_cmd(v, w)
                    cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        cv2.imshow("f", frame)
        cv2.imshow("m", mask)
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
    print("SHUTDOWN")
