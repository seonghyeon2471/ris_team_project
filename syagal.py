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

# ── ENCODER RECEIVER ──────────────────────────────────────────────────
# 아두이노가 200ms마다 "E:<dist_r>,<dist_l>\n" 포맷으로 전송
MAP_SIZE_CM      = 200.0   # 2x2m 맵 한 변 길이
ESCAPE_DIST_CM   = 180.0   # 이 거리 이상 이동 시 맵 이탈 경고 (맵 크기의 90%)

enc_dist_r  = 0.0
enc_dist_l  = 0.0
enc_lock    = threading.Lock()
map_escaped = False   # 맵 이탈 플래그

def encoder_loop():
    """아두이노 Serial1 → 라즈베리파이 수신 스레드"""
    global enc_dist_r, enc_dist_l, map_escaped
    buf = ""
    while True:
        try:
            raw = arduino_ser.read(arduino_ser.in_waiting or 1)
            if not raw:
                continue
            buf += raw.decode(errors="ignore")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line.startswith("E:"):
                    parts = line[2:].split(",")
                    if len(parts) == 2:
                        r = float(parts[0])
                        l = float(parts[1])
                        avg = (abs(r) + abs(l)) / 2.0
                        with enc_lock:
                            enc_dist_r  = r
                            enc_dist_l  = l
                            map_escaped = avg >= ESCAPE_DIST_CM
        except Exception:
            pass

threading.Thread(target=encoder_loop, daemon=True).start()

def get_encoder():
    with enc_lock:
        return enc_dist_r, enc_dist_l

def get_traveled_cm():
    r, l = get_encoder()
    return (abs(r) + abs(l)) / 2.0

def reset_encoder_baseline():
    """미션 완료 후 기준 초기화 — 아두이노 엔코더는 누적이므로
       파이 쪽에서 기준값을 저장해 상대 거리를 계산"""
    global _enc_baseline_r, _enc_baseline_l
    r, l = get_encoder()
    _enc_baseline_r = r
    _enc_baseline_l = l

_enc_baseline_r = 0.0
_enc_baseline_l = 0.0

def get_relative_dist_cm():
    """마지막 reset_encoder_baseline() 이후 이동 거리 (cm)"""
    r, l = get_encoder()
    return (abs(r - _enc_baseline_r) + abs(l - _enc_baseline_l)) / 2.0

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR ─────────────────────────────────────────────────────────────
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 45
THRESH_SLOW = 38.0   # cm: 감속         (55.0 → 38.0, -17cm)
THRESH_TURN = 18.0   # cm: 회피 조향    (35.0 → 18.0, -17cm)
THRESH_STOP =  8.0   # cm: 강한 회피    (18.0 →  8.0, 안전 마진 고려)

# ── 벽 추종 ───────────────────────────────────────────────────────────
WALL_DIST_REF = 35.0  # cm: 장애물과 유지할 목표 거리 (2x2m 맵 기준)
KP_WALL       = 0.018 # 벽 추종 P 게인

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

def wall_follow(scan, adir):
    """벽 추종: adir 방향 반대쪽 벽과의 거리 오차로 각속도 보정"""
    right_dist = float(np.mean(scan[280:350]))
    left_dist  = float(np.mean(scan[10:80]))
    if adir == 1:   # 오른쪽으로 피하는 중 → 왼쪽 벽 추종
        err = left_dist - WALL_DIST_REF
    else:           # 왼쪽으로 피하는 중 → 오른쪽 벽 추종
        err = WALL_DIST_REF - right_dist
    return KP_WALL * err

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
# ── COLOR CONFIG (요청하신 새로운 색상 파라미터 적용) ──────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 168,  96], [179, 222, 157]),
               "hsv2": None,  # 새로운 단일 범위 적용으로 hsv2는 제외
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
               
    "yellow": {"hsv1": ([16,  137, 142], [30,  214, 195]),
               "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
               
    "blue":   {"hsv1": ([106, 168,  54], [131, 210, 82]),
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
MIN_AREA        = 400
KP_ROT          = 0.003
APPROACH_V      = 0.22
PARK_SEC        = 1.2
DETECT_CONFIRM  = 6
BOTTOM_10PCT    = int(240 * 0.90)   # 216px
SCAN360_W       = 1.1               # 제자리 회전 각속도
SCAN360_TIMEOUT = 22.0              # 22초 = 약 2바퀴 탐색 (2x2m 맵 기준)

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
lidar_submode = "SCAN360"
scan360_start = None
mission_idx   = 0
detect_count  = 0

# PARK 상태
park_state    = "SEARCH"
last_seen_x   = 160
last_bottom_y = 0
was_in_bottom = False
park_t        = None

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
        rel_dist = get_relative_dist_cm()

        # ── 맵 이탈 경고 표시 ─────────────────────────────────────────
        if map_escaped:
            cv2.putText(frame, f"!! MAP ESCAPE ({rel_dist:.0f}cm)",
                        (10, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

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
        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            ox     = bx + bw // 2
            by_bot = min(by_top + bh, 239)
            err_x  = ox - cx_mid

            last_seen_x   = ox
            last_bottom_y = by_bot
            was_in_bottom = (by_bot >= BOTTOM_10PCT)

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)
            cv2.line(frame, (cx_mid, 0), (cx_mid, H), (100, 100, 100), 1)
            cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)
            cv2.putText(frame, f"x_err={err_x}  y={by_bot}",
                        (bx, by_top - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, draw, 1)

        # ══ LIDAR 주행 모드 ═══════════════════════════════════════════
        if mode == "LIDAR":

            # ── SCAN360: 제자리 360도 회전 탐색 ──────────────────────
            if lidar_submode == "SCAN360":

                if scan360_start is None:
                    scan360_start = time.time()

                elapsed_scan = time.time() - scan360_start

                if found:
                    detect_count += 1
                    cv2.putText(frame, f"SCAN360 DETECT [{detect_count}/{DETECT_CONFIRM}]",
                                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 1)
                else:
                    detect_count = 0

                if detect_count >= DETECT_CONFIRM:
                    detect_count  = 0
                    lidar_submode = "DRIVE"
                    scan360_start = None
                    stop_robot()
                    print(f"[{target}] SCAN360 발견 → DRIVE 전환")

                elif elapsed_scan >= SCAN360_TIMEOUT:
                    lidar_submode = "DRIVE"
                    scan360_start = None
                    detect_count  = 0
                    stop_robot()
                    print(f"[{target}] SCAN360 타임아웃 → DRIVE 강제 전환")

                else:
                    send_cmd(0.0, SCAN360_W)
                    remain_str = f"{max(0.0, SCAN360_TIMEOUT - elapsed_scan):.0f}s"
                    cv2.putText(frame, f"SCAN360 [{target}]  timeout={remain_str}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 0), 1)

                cv2.imshow("f", frame)
                if cv2.waitKey(1) & 0xFF == 27: break
                continue

            # ── DRIVE: 벽 추종 + 라이다 장애물 회피 주행 ─────────────
            if found:
                detect_count += 1
                cv2.putText(frame, f"DETECT [{detect_count}/{DETECT_CONFIRM}]",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 1)
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count  = 0
                mode          = "PARK"
                park_state    = "TRACK"
                last_bottom_y = 0
                was_in_bottom = False
                park_t        = None
                stop_robot()
                print(f"[{target}] 인식 확정 → PARK")
                cv2.imshow("f", frame); cv2.waitKey(1); continue

            # 라이다 장애물 회피 + 벽 추종 혼합
            w_wall = wall_follow(scan, adir)

            if fm < THRESH_STOP:
                # 충돌 임박: 회피 60% + 벽 추종 40%
                v = 0.09
                w = 0.6 * (adir * 0.9) + 0.4 * w_wall
            elif fm < THRESH_TURN:
                # 회피 조향: 회피 40% + 벽 추종 60%
                v = 0.13
                w = 0.4 * (adir * 0.7) + 0.6 * w_wall
            elif fm < THRESH_SLOW:
                # 감속 구간: 벽 추종 100%
                v = 0.18
                w = w_wall
            else:
                # 열린 공간: 직진 (0.28 → 0.22, 좁은 맵 대응)
                v, w = 0.22, 0.0

            send_cmd(v, w)
            cv2.putText(frame, f"DRIVE  front={fm:.0f}cm  wall_w={w_wall:.2f}  dist={rel_dist:.0f}cm",
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
            cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)
            cv2.putText(frame, f"PARKING [{target}]  {remain:.1f}s",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw, 2)
            cv2.imshow("f", frame)
            if elapsed >= PARK_SEC:
                mission_idx   += 1
                mode           = "LIDAR"
                lidar_submode  = "SCAN360"
                scan360_start  = None
                park_state     = "TRACK"
                last_bottom_y  = 0
                was_in_bottom  = False
                detect_count   = 0
                reset_encoder_baseline()   # 다음 미션 기준 거리 초기화
                print(f"[{target}] 완료 → LIDAR 복귀 (SCAN360 시작)")
            cv2.waitKey(1); continue

        # ─ TRACK: 색지 보임 ───────────────────────────────────────────
        if found:
            park_state = "TRACK"

            if fm >= THRESH_SLOW:
                w = -KP_ROT * err_x
                v = APPROACH_V
            else:
                w_cam   = -KP_ROT * err_x
                w_lidar = adir * 0.7
                if fm < THRESH_STOP:
                    v, w = 0.09, w_lidar
                elif fm < THRESH_TURN:
                    v = 0.13
                    w = 0.7 * w_lidar + 0.3 * w_cam
                else:
                    v = 0.18
                    w = 0.3 * w_lidar + 0.7 * w_cam

            send_cmd(v, w)
            cv2.putText(frame, f"TRACK [{target}]  front={fm:.0f}cm  {'LIDAR_OFF' if fm >= THRESH_SLOW else 'LIDAR_ON'}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.48, draw, 1)

        # ─ 색지 사라짐 ────────────────────────────────────────────────
        else:
            if was_in_bottom:
                park_state = "PARKING"
                park_t     = time.time()
                stop_robot()
                print(f"[{target}] 하단 소실 (last_y={last_bottom_y}px) → {PARK_SEC}s 정차")
            else:
                park_state = "SEARCH"
                v = 0.0
                w = -1.30 if last_seen_x > cx_mid else 1.30
                send_cmd(v, w)
                cv2.putText(frame, f"SEARCH [{target}]  last_x={last_seen_x}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 0), 1)
            cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)

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
