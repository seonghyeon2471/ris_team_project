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

def lidar_vw(fm, adir):
    if fm < THRESH_STOP:  return 0.09, adir * 0.9
    elif fm < THRESH_TURN: return 0.13, adir * 0.7
    elif fm < THRESH_SLOW: return 0.18, adir * 0.4
    else:                  return 0.28, 0.0

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv": [(np.array([169, 168,  96]), np.array([179, 222, 157]))], "draw": (0, 0, 255)},
    "yellow": {"hsv": [(np.array([16,  137, 142]), np.array([30,  214, 195]))], "draw": (0, 200, 255)},
    "blue":   {"hsv": [(np.array([106, 168,  54]), np.array([131, 210,  82]))], "draw": (255, 80, 0)},
}
MISSION = ["red", "yellow", "blue"]

def make_mask(frame, hsv_img, name):
    m = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_CFG[name]["hsv"]:
        m = cv2.bitwise_or(m, cv2.inRange(hsv_img, lo, hi))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    return m

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA       = 400
KP_ROT         = 0.006
APPROACH_V     = 0.22
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

BOTTOM_Y    = int(240 * 0.90)   # 216px  하단 10% 기준
SIDE_LIM_L  = int(320 * 0.10)   # 32px   주차 유효 x 좌측
SIDE_LIM_R  = int(320 * 0.90)   # 288px  주차 유효 x 우측

OUTER_L  = int(320 * 0.20)      # 64px   강한 회전 구역
OUTER_R  = int(320 * 0.80)      # 256px
CENTER_L = int(320 * 0.40)      # 128px  라이다 직진 구역
CENTER_R = int(320 * 0.60)      # 192px

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
mission_idx   = 0
detect_count  = 0

# PARK 전용 상태
#   "TRACK"   : 색지 추적 중
#   "LOST"    : 색지 유실 → 주차 판단 대기
#   "SEARCH"  : 주차 조건 미달 → 재탐색
#   "PARKING" : 정차 중
park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0
was_in_bottom = False   # 한번 True → 유실 전까지 유지
park_t        = None

print(f"START | BOTTOM_Y={BOTTOM_Y}  SIDE={SIDE_LIM_L}~{SIDE_LIM_R}")

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

        # ── 마스크 & 컨투어 ───────────────────────────────────────────
        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big   = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        ox, err_x, by_bot = cx_mid, 0, 0
        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            ox     = bx + bw // 2
            by_bot = min(by_top + bh, 239)
            err_x  = ox - cx_mid
            last_seen_x   = ox
            last_bottom_y = by_bot
            if by_bot >= BOTTOM_Y:          # 한번 하단 진입하면 유지
                was_in_bottom = True

            # 시각화
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)
            cv2.line(frame, (cx_mid, 0), (cx_mid, H), (80, 80, 80), 1)

        # 구역 기준선 표시
        cv2.line(frame, (0, BOTTOM_Y), (W, BOTTOM_Y), (0, 0, 255), 1)
        cv2.line(frame, (SIDE_LIM_L, 0), (SIDE_LIM_L, H), (0, 0, 200), 1)
        cv2.line(frame, (SIDE_LIM_R, 0), (SIDE_LIM_R, H), (0, 0, 200), 1)
        cv2.line(frame, (OUTER_L, 0), (OUTER_L, H), (0, 200, 0), 1)
        cv2.line(frame, (OUTER_R, 0), (OUTER_R, H), (0, 200, 0), 1)
        cv2.line(frame, (CENTER_L, 0), (CENTER_L, H), (200, 200, 0), 1)
        cv2.line(frame, (CENTER_R, 0), (CENTER_R, H), (200, 200, 0), 1)

        # ══ LIDAR 주행 모드 ═══════════════════════════════════════════
        if mode == "LIDAR":
            if found:
                detect_count += 1
                cv2.putText(frame, f"DETECT [{detect_count}/{DETECT_CONFIRM}]",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 2)

                # 색지 보이는 동안 카메라 조향
                if ox < OUTER_L or ox > OUTER_R:
                    v, w = 0.0, (-1.4 if err_x > 0 else 1.4)
                elif CENTER_L <= ox <= CENTER_R:
                    v, w = lidar_vw(fm, adir)
                else:
                    v = min(0.22, lidar_vw(fm, adir)[0])
                    w = -KP_ROT * err_x
            else:
                detect_count = 0
                v, w = lidar_vw(fm, adir)

            # PARK 진입: 색지 N프레임 확정 + 전방 여유
            if detect_count >= DETECT_CONFIRM and fm >= THRESH_SLOW:
                detect_count  = 0
                mode          = "PARK"
                park_state    = "TRACK"
                was_in_bottom = False
                last_bottom_y = 0
                park_t        = None
                stop_robot()
                print(f"[{target}] PARK 진입 (front={fm:.0f}cm)")
                cv2.imshow("f", frame); cv2.waitKey(1); continue

            send_cmd(v, w)
            cv2.putText(frame, f"LIDAR front={fm:.0f}cm  [{target}]",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27: break
            continue

        # ══ PARK 모드 ══════════════════════════════════════════════════

        # ─ PARKING 정차 ───────────────────────────────────────────────
        if park_state == "PARKING":
            stop_robot()
            elapsed = time.time() - park_t
            remain  = max(0.0, PARK_SEC - elapsed)
            cv2.putText(frame, f"PARKING [{target}] {remain:.1f}s",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw, 2)
            cv2.imshow("f", frame)
            if elapsed >= PARK_SEC:
                mission_idx   += 1
                mode           = "LIDAR"
                park_state     = "TRACK"
                was_in_bottom  = False
                last_bottom_y  = 0
                detect_count   = 0
                print(f"[{target}] 완료 → LIDAR")
            cv2.waitKey(1)
            continue

        # ─ TRACK: 색지 보임 ───────────────────────────────────────────
        if park_state in ["TRACK", "SEARCH"]:
            if found:
                park_state = "TRACK"

                if ox < OUTER_L or ox > OUTER_R:
                    # 양끝 20% → 카메라 강한 회전
                    v, w  = 0.0, (-1.4 if err_x > 0 else 1.4)
                    label = "CAM_TURN"
                elif CENTER_L <= ox <= CENTER_R:
                    # 중앙 20% → 라이다 직진
                    v, w  = lidar_vw(fm, adir)
                    label = "LIDAR_FWD"
                else:
                    # 중간 40% → 카메라 정렬
                    v     = APPROACH_V
                    w     = -KP_ROT * err_x
                    label = "CAM_ALIGN"

                send_cmd(v, w)
                cv2.putText(frame,
                    f"TRACK [{target}] {label} y={last_bottom_y} bot={was_in_bottom}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw, 1)

            else:
                # ── 색지 사라짐 ───────────────────────────────────────
                in_x_ok = SIDE_LIM_L < last_seen_x < SIDE_LIM_R
                print(f"[LOST] was_in_bottom={was_in_bottom} last_y={last_bottom_y} in_x={in_x_ok} last_x={last_seen_x}")

                if was_in_bottom and in_x_ok:
                    park_state = "PARKING"
                    park_t     = time.time()
                    stop_robot()
                    print(f"[{target}] 주차 확정 → {PARK_SEC}s")
                else:
                    park_state = "SEARCH"
                    v = 0.0
                    w = -1.30 if last_seen_x > cx_mid else 1.30
                    send_cmd(v, w)
                    cv2.putText(frame, f"SEARCH [{target}]",
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
