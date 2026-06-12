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

def lidar_v_limit(fm):
    """전방 거리에 따라 최대 선속도 제한"""
    if fm < THRESH_STOP: return 0.09
    elif fm < THRESH_TURN: return 0.13
    elif fm < THRESH_SLOW: return 0.18
    else: return 0.28

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red": {
        "hsv": [(np.array([169, 168,  96]), np.array([179, 222, 157]))],
        "draw": (0, 0, 255),
    },
    "yellow": {
        "hsv": [(np.array([16,  137, 142]), np.array([30,  214, 195]))],
        "draw": (0, 200, 255),
    },
    "blue": {
        "hsv": [(np.array([106, 168,  54]), np.array([131, 210,  82]))],
        "draw": (255, 80, 0),
    },
}
MISSION = ["red", "yellow", "blue"]

def make_mask(frame, hsv_img, name):
    cfg = COLOR_CFG[name]
    m = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
    for lo, hi in cfg["hsv"]:
        m = cv2.bitwise_or(m, cv2.inRange(hsv_img, lo, hi))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  kernel)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
    return m

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA         = 400
KP_ROT           = 0.006    # 각속도 상향 (기존 0.003 → 0.006)
APPROACH_V       = 0.22
PARK_SEC         = 1.2
DETECT_CONFIRM   = 6
BOTTOM_10PCT     = int(240 * 0.90)  # 216px  하단 10% 기준
SIDE_LEFT_LIM    = int(320 * 0.10)  # 32px   좌측 10% 경계
SIDE_RIGHT_LIM   = int(320 * 0.90)  # 288px  우측 10% 경계
PARK_ENTRY_GRACE = 8                # PARK 진입 직후 안정화 프레임

# ── STATE ─────────────────────────────────────────────────────────────
mode             = "LIDAR"
mission_idx      = 0
detect_count     = 0
park_state       = "TRACK"
last_seen_x      = 160
last_bottom_y    = 0
was_in_bottom    = False
park_t           = None
park_entry_count = 0

print(f"START | 하단 10% 기준: y > {BOTTOM_10PCT}px")

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

        # ── 미션 완료 ─────────────────────────────────────────────────
        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL DONE", (60, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        # ── 마스크 & 컨투어 (공통) ────────────────────────────────────
        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big   = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            ox     = bx + bw // 2
            by_bot = min(by_top + bh, 239)
            err_x  = ox - cx_mid
            last_seen_x   = ox
            last_bottom_y = by_bot
            # 한번 하단 진입하면 True 유지 (사라지기 전까지 초기화 안 함)
            if by_bot >= BOTTOM_10PCT:
                was_in_bottom = True

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)
            cv2.line(frame, (cx_mid, 0), (cx_mid, H), (100, 100, 100), 1)

        cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)
        cv2.line(frame, (SIDE_LEFT_LIM, 0), (SIDE_LEFT_LIM, H), (0, 0, 255), 1)
        cv2.line(frame, (SIDE_RIGHT_LIM, 0), (SIDE_RIGHT_LIM, H), (0, 0, 255), 1)

        # ══ LIDAR 주행 모드 ═══════════════════════════════════════════
        if mode == "LIDAR":
            # 색지 인식 카운트
            if found:
                detect_count += 1
                cv2.putText(frame, f"DETECT [{detect_count}/{DETECT_CONFIRM}]",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 2)
            else:
                detect_count = 0

            # PARK 진입 조건1: 색지 N프레임 연속 인식
            # PARK 진입 조건2: 전방 장애물 없음 (fm >= THRESH_SLOW)
            if detect_count >= DETECT_CONFIRM and fm >= THRESH_SLOW:
                detect_count     = 0
                mode             = "PARK"
                park_state       = "TRACK"
                last_bottom_y    = 0
                was_in_bottom    = False
                park_t           = None
                park_entry_count = 0
                stop_robot()
                print(f"[{target}] 인식확정 + 전방여유({fm:.0f}cm) → PARK")
                cv2.imshow("f", frame); cv2.waitKey(1); continue

            if detect_count >= DETECT_CONFIRM and fm < THRESH_SLOW:
                # 색지는 보이지만 전방 장애물 있음 → LIDAR 주행 유지
                cv2.putText(frame, f"WAIT obstacle {fm:.0f}cm",
                            (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)

            # 라이다 장애물 회피 (색지 보이면 구역별 조향)
            if found:
                OUTER_L = int(320 * 0.20)   # 64px  바깥 20% 경계
                OUTER_R = int(320 * 0.80)   # 256px
                CENTER_L = int(320 * 0.40)  # 128px 중앙 20% 경계
                CENTER_R = int(320 * 0.60)  # 192px

                if ox < OUTER_L or ox > OUTER_R:
                    # 양끝 20% → 카메라 조향 (제자리 강한 회전)
                    v = 0.0
                    w = -1.4 if err_x > 0 else 1.4
                elif CENTER_L <= ox <= CENTER_R:
                    # 중앙 20% → 라이다 조향 (장애물 회피 직진)
                    if fm < THRESH_STOP:
                        v, w = 0.09, adir * 0.9
                    elif fm < THRESH_TURN:
                        v, w = 0.13, adir * 0.7
                    elif fm < THRESH_SLOW:
                        v, w = 0.18, adir * 0.4
                    else:
                        v, w = 0.28, 0.0
                else:
                    # 중간 40% → 카메라 조향 유지 (부드럽게 중앙으로)
                    v = min(lidar_v_limit(fm), APPROACH_V)
                    w = -KP_ROT * err_x
            else:
                # 색지 없음 → 순수 라이다 회피
                if fm < THRESH_STOP:
                    v, w = 0.09, adir * 0.9
                elif fm < THRESH_TURN:
                    v, w = 0.13, adir * 0.7
                elif fm < THRESH_SLOW:
                    v, w = 0.18, adir * 0.4
                else:
                    v, w = 0.28, 0.0

            send_cmd(v, w)
            cv2.putText(frame, f"LIDAR  front={fm:.0f}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27: break
            continue

        # ══ PARK 모드 ══════════════════════════════════════════════════

        # ─ PARKING 정차 ───────────────────────────────────────────────
        if park_state == "PARKING":
            stop_robot()
            elapsed = time.time() - park_t
            remain  = max(0.0, PARK_SEC - elapsed)
            cv2.putText(frame, f"PARKING [{target}]  {remain:.1f}s",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw, 2)
            cv2.imshow("f", frame)
            if elapsed >= PARK_SEC:
                mission_idx      += 1
                mode              = "LIDAR"
                park_state        = "TRACK"
                last_bottom_y     = 0
                was_in_bottom     = False
                detect_count      = 0
                park_entry_count  = 0
                print(f"[{target}] 완료 → LIDAR 복귀")
            cv2.waitKey(1); continue

        # ─ PARK 진입 직후 안정화 (카메라 버퍼 flush) ─────────────────
        if park_entry_count < PARK_ENTRY_GRACE:
            park_entry_count += 1
            send_cmd(APPROACH_V, 0.0)
            cv2.putText(frame, f"STABILIZE ({park_entry_count}/{PARK_ENTRY_GRACE})",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 1)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        # ─ TRACK: 색지 보임 ───────────────────────────────────────────
        if found:
            park_state = "TRACK"

            OUTER_L  = int(320 * 0.20)   # 64px
            OUTER_R  = int(320 * 0.80)   # 256px
            CENTER_L = int(320 * 0.40)   # 128px
            CENTER_R = int(320 * 0.60)   # 192px

            if ox < OUTER_L or ox > OUTER_R:
                # 양끝 20% → 카메라 조향 (제자리 강한 회전)
                v = 0.0
                w = -1.4 if err_x > 0 else 1.4
                lidar_label = "CAM_TURN"
            elif CENTER_L <= ox <= CENTER_R:
                # 중앙 20% → 라이다 조향 (장애물 회피 직진)
                if fm < THRESH_STOP:
                    v, w = 0.09, adir * 0.9
                elif fm < THRESH_TURN:
                    v, w = 0.13, adir * 0.7
                elif fm < THRESH_SLOW:
                    v, w = 0.18, adir * 0.4
                else:
                    v, w = APPROACH_V, 0.0
                lidar_label = "LIDAR_FWD"
            else:
                # 중간 40% → 카메라 조향 유지 (부드럽게 중앙으로)
                v = APPROACH_V
                w = -KP_ROT * err_x
                lidar_label = "CAM_ALIGN"

            send_cmd(v, w)
            cv2.putText(frame, f"TRACK [{target}] {lidar_label} front={fm:.0f}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.48, draw, 1)

        # ─ 색지 사라짐 ────────────────────────────────────────────────
        else:
            in_x_range = SIDE_LEFT_LIM < last_seen_x < SIDE_RIGHT_LIM
            if was_in_bottom and in_x_range:
                # 조건1: 하단 10%에서 사라짐
                # 조건2: 마지막 x중심이 양옆 10% 제외 구역 안
                park_state = "PARKING"
                park_t     = time.time()
                stop_robot()
                print(f"[{target}] 주차 확정 (last_y={last_bottom_y}px, last_x={last_seen_x}px) → {PARK_SEC}s 정차")
            else:
                # 조건 미달 → 재탐색
                reason = []
                if not was_in_bottom: reason.append(f"y={last_bottom_y}<{BOTTOM_10PCT}")
                if not in_x_range:    reason.append(f"x={last_seen_x} 사이드구역")
                print(f"[{target}] 주차 무효 ({', '.join(reason)}) → SEARCH")
                park_state = "SEARCH"
                v = 0.0
                w = -1.30 if last_seen_x > cx_mid else 1.30
                send_cmd(v, w)
                cv2.putText(frame, f"SEARCH [{target}] last_x={last_seen_x}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("INTERRUPTED")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
    print("SHUTDOWN")
