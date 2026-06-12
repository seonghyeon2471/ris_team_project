import cv2
import serial
import numpy as np
import time
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
    "red":    {"hsv1": ([169, 168,  96], [179, 222, 157]),
               "hsv2": ([160,  60, 120], [179, 255, 255]),
               "bgr":  ([20,  20,  80], [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([16,  137, 142], [30,  214, 195]),
               "hsv2": None,
               "bgr":  ([0,   80,  80], [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([106, 168,  54], [131, 210, 195]),
               "hsv2": None,
               "bgr":  ([40,    0,   0], [255, 220, 220]), "draw": (255, 80, 0)},
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
MIN_AREA        = 400
DETECT_CONFIRM  = 6

# ── APPROACH: x축 + y축 동시 제어 ─────────────────────────────────────
KP_X            = 0.003    # x 오차 → 회전 (좌우 조향)
KP_Y            = 0.004    # y 오차 → 전진/후진
APPROACH_W_MAX  = 0.9      # 최대 조향 각속도
APPROACH_V_MAX  = 0.22     # 최대 접근 속도
APPROACH_V_MIN  = 0.07     # 최소 전진 속도 (너무 느리면 정지 판정 오류 방지)

# 카메라 목표 위치: 화면 중앙 x, y축은 화면 하단 30% 지점 (색지가 발 아래 오는 시점)
TARGET_X_RATIO  = 0.5      # 화면 가로 중심 (0.5 = 정중앙)
TARGET_Y_RATIO  = 0.72     # 화면 세로 목표 (0.72 = 상단72% 위치, 즉 하단28% 근방)

# 정지 판정: x, y 오차가 모두 이 범위 안에 들어오면 도착
ARRIVE_X_PX     = 35       # x 허용 오차 (픽셀)
ARRIVE_Y_PX     = 30       # y 허용 오차 (픽셀)
ARRIVE_CONFIRM  = 8        # 연속 N프레임 이상 조건 충족 시 정차

# ── SPIN: 정차 후 제자리 한 바퀴 회전 ────────────────────────────────
PARK_SEC        = 1.2      # 정차 유지 시간 (초)
SPIN_W          = 0.50     # 제자리 회전 각속도 (작을수록 천천히)
# 한 바퀴(2π) 도는 데 걸리는 시간 = 2π / SPIN_W (rad/s 기준)
# SPIN_W=0.5 → 약 12.6초. 실제 로봇은 rad/s ≠ 물리값이므로 아래 SPIN_SEC로 조정
SPIN_SEC        = 6.0      # 제자리 회전 지속 시간 (실험으로 한 바퀴에 맞게 조정)

# ── STATE ─────────────────────────────────────────────────────────────
#
# mode: "LIDAR" | "PARK"
#
# LIDAR 서브상태 (park_state):
#   "DRIVE"   → 일반 장애물 회피 주행
#   "SPIN"    → 주차 완료 후 제자리 회전
#
# PARK 서브상태 (park_state):
#   "TRACK"   → x+y 축 제어로 색지 중심 접근
#   "SEARCH"  → 색지 소실, 마지막 위치 기준 제자리 탐색
#   "PARKING" → 정차 대기
#
mode           = "LIDAR"
park_state     = "DRIVE"
mission_idx    = 0
detect_count   = 0
arrive_count   = 0          # 도착 조건 연속 카운트

last_seen_x    = 160
park_t         = None
spin_t         = None
spin_dir       = 1

print("START")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame  = cv2.flip(frame, 1)
        H, W   = frame.shape[:2]
        cx_mid = int(W * TARGET_X_RATIO)   # 목표 x (픽셀)
        cy_tgt = int(H * TARGET_Y_RATIO)   # 목표 y (픽셀)
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan   = get_scan()
        fm     = front_min(scan)
        adir   = avoid_dir(scan)

        # ── 목표 위치 시각화 (항상 표시) ─────────────────────────────
        cv2.line(frame, (cx_mid - 15, cy_tgt), (cx_mid + 15, cy_tgt), (180, 180, 180), 1)
        cv2.line(frame, (cx_mid, cy_tgt - 15), (cx_mid, cy_tgt + 15), (180, 180, 180), 1)

        # ── 미션 완료 ─────────────────────────────────────────────────
        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL DONE", (60, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        # ── 공통: 마스크 & 컨투어 ─────────────────────────────────────
        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big   = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        # 색지 정보 추출
        obj_cx = obj_cy = err_x = err_y = 0
        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            obj_cx = bx + bw // 2          # 색지 x 중심
            obj_cy = by_top + bh // 2      # 색지 y 중심
            err_x  = obj_cx - cx_mid       # +: 오른쪽, -: 왼쪽
            err_y  = obj_cy - cy_tgt       # +: 멀다(더 전진), -: 너무 가까움

            last_seen_x = obj_cx

            # 시각화
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (obj_cx, obj_cy), 5, (0, 255, 255), -1)   # 색지 중심
            cv2.circle(frame, (cx_mid, cy_tgt), 5, (255, 255, 255), -1) # 목표점
            cv2.line(frame, (obj_cx, obj_cy), (cx_mid, cy_tgt), (100, 100, 255), 1)
            cv2.putText(frame, f"ex={err_x:+d} ey={err_y:+d}",
                        (bx, by_top - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, draw, 1)

        # ══════════════════════════════════════════════════════════════
        # LIDAR 모드
        # ══════════════════════════════════════════════════════════════
        if mode == "LIDAR":

            # ── SPIN: 주차 완료 후 제자리 한 바퀴 ────────────────────
            if park_state == "SPIN":
                elapsed = time.time() - spin_t
                remain  = max(0.0, SPIN_SEC - elapsed)
                send_cmd(0.0, spin_dir * SPIN_W)
                cv2.putText(frame, f"SPIN {remain:.1f}s  (1 round)",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 200), 2)
                cv2.imshow("f", frame); cv2.waitKey(1)
                if elapsed >= SPIN_SEC:
                    stop_robot()
                    park_state   = "DRIVE"
                    arrive_count = 0
                    detect_count = 0
                    print(f"[prev] SPIN 완료 → LIDAR 탐색 재개 | 다음 목표: {MISSION[mission_idx] if mission_idx < len(MISSION) else 'DONE'}")
                continue

            # ── DRIVE: 색지 감지 카운트 + 장애물 회피 ────────────────
            if found:
                detect_count += 1
                cv2.putText(frame, f"DETECT [{detect_count}/{DETECT_CONFIRM}]",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 1)
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                arrive_count = 0
                mode         = "PARK"
                park_state   = "TRACK"
                stop_robot()
                print(f"[{target}] 인식 확정 → PARK/TRACK")
                cv2.imshow("f", frame); cv2.waitKey(1); continue

            # 장애물 회피 주행
            if fm < THRESH_STOP:
                v, w = 0.09, adir * 0.9
            elif fm < THRESH_TURN:
                v, w = 0.13, adir * 0.7
            elif fm < THRESH_SLOW:
                v, w = 0.18, adir * 0.4
            else:
                v, w = 0.28, 0.0

            send_cmd(v, w)
            cv2.putText(frame, f"LIDAR  front={fm:.0f}cm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27: break
            continue

        # ══════════════════════════════════════════════════════════════
        # PARK 모드
        # ══════════════════════════════════════════════════════════════

        # ── PARKING: 정차 대기 ────────────────────────────────────────
        if park_state == "PARKING":
            stop_robot()
            elapsed = time.time() - park_t
            remain  = max(0.0, PARK_SEC - elapsed)
            cv2.putText(frame, f"PARKING [{target}]  {remain:.1f}s",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw, 2)
            cv2.imshow("f", frame); cv2.waitKey(1)
            if elapsed >= PARK_SEC:
                mission_idx += 1
                mode         = "LIDAR"
                park_state   = "SPIN"
                spin_t       = time.time()
                spin_dir     = 1            # 반시계 원하면 -1
                detect_count = 0
                arrive_count = 0
                stop_robot()
                print(f"[{target}] 정차 완료 → SPIN 시작 ({SPIN_SEC}초 한 바퀴)")
            continue

        # ── TRACK: x + y 축 동시 제어 ────────────────────────────────
        if found:
            park_state = "TRACK"

            # ── 도착 판정 ─────────────────────────────────────────────
            x_ok = abs(err_x) <= ARRIVE_X_PX
            y_ok = abs(err_y) <= ARRIVE_Y_PX

            if x_ok and y_ok:
                arrive_count += 1
                cv2.putText(frame, f"ARRIVE [{arrive_count}/{ARRIVE_CONFIRM}]",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                arrive_count = 0

            if arrive_count >= ARRIVE_CONFIRM:
                # 중심 일치 확정 → 정차
                stop_robot()
                park_state   = "PARKING"
                park_t       = time.time()
                arrive_count = 0
                print(f"[{target}] 중심 일치 확정 (ex={err_x}, ey={err_y}) → PARKING")
                cv2.imshow("f", frame); cv2.waitKey(1); continue

            # ── x + y 동시 제어 ───────────────────────────────────────
            # x 오차 → 회전 (w)
            w_cam = -KP_X * err_x
            w_cam = float(np.clip(w_cam, -APPROACH_W_MAX, APPROACH_W_MAX))

            # y 오차 → 전진 속도 (v)
            #   err_y > 0: 색지가 목표보다 위 = 아직 멀다 → 전진
            #   err_y < 0: 색지가 목표보다 아래 = 너무 가깝다 → 후진 or 정지
            v_cam = KP_Y * err_y
            v_cam = float(np.clip(v_cam, -0.10, APPROACH_V_MAX))
            if v_cam > 0 and v_cam < APPROACH_V_MIN:
                v_cam = APPROACH_V_MIN   # 전진 시 최소 속도 보장

            # 장애물이 가까우면 라이다 블렌딩
            if fm < THRESH_STOP:
                v = 0.07
                w = float(adir) * 0.9
            elif fm < THRESH_TURN:
                v = max(0.07, v_cam * 0.5)
                w = 0.7 * float(adir) * 0.7 + 0.3 * w_cam
            elif fm < THRESH_SLOW:
                v = max(0.07, v_cam * 0.8)
                w = 0.3 * float(adir) * 0.4 + 0.7 * w_cam
            else:
                v = v_cam
                w = w_cam

            send_cmd(v, w)

            status = "LIDAR_ON" if fm < THRESH_SLOW else "LIDAR_OFF"
            cv2.putText(frame,
                        f"TRACK [{target}] ex={err_x:+d} ey={err_y:+d} v={v:.2f} w={w:.2f} {status}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.38, draw, 1)

        # ── SEARCH: 색지 소실 ─────────────────────────────────────────
        else:
            arrive_count = 0
            park_state   = "SEARCH"
            v = 0.0
            w = -1.30 if last_seen_x > W // 2 else 1.30
            send_cmd(v, w)
            cv2.putText(frame, f"SEARCH [{target}]  last_x={last_seen_x}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 0), 1)

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
