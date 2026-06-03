#!/usr/bin/env python3
"""
Project 3: Color-Zone Tracking Robot
  - 카메라 + 제어: 메인 루프 (단일 스레드, 렉 없음)
  - LiDAR       : 별도 스레드
  - 모터        : Arduino Serial ("v,w\n")
  - LiDAR       : 직접 시리얼 파싱 (프로젝트2 방식)
"""

import cv2
import serial
import numpy as np
import threading
import time
import math
import logging

# ═════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═════════════════════════════════════════════════════════════════════════════

# ── Serial ────────────────────────────────────────────────────────────────────
ARDUINO_PORT  = "/dev/serial0"
ARDUINO_BAUD  = 115200
LIDAR_PORT    = "/dev/ttyUSB0"
LIDAR_BAUD    = 460800

# ── Camera ────────────────────────────────────────────────────────────────────
CAMERA_INDEX  = 0
FRAME_W       = 320
FRAME_H       = 240

# ── Color detection ───────────────────────────────────────────────────────────
MIN_AREA      = 500
CENTER_DEAD   = 15       # px, dead-zone

# ── 통과 판정 ─────────────────────────────────────────────────────────────────
PASS_HOLD_SEC = 1.0      # 색 사라진 후 이 시간 유지되면 통과

# ── LiDAR / FGM (프로젝트2 파라미터) ──────────────────────────────────────────
SCAN_LIMIT         = 150
FRONT_RANGE        = 60
ROBOT_RADIUS       = 17.0
SAFE_DIST          = 14
INFLATION_MAX_DIST = 25
FRONT_CLEAR_DIST   = 12
FRONT_CLEAR_RANGE  = 15
EMA_ALPHA          = 0.3
MEDIAN_K           = 2
SMOOTHING_NORMAL   = 0.55
SMOOTHING_DANGER   = 0.20
DANGER_DIST        = 8

# ── Emergency ─────────────────────────────────────────────────────────────────
EMERGENCY_DIST   = 6
REVERSE_DURATION = 0.18
ROTATE_DURATION  = 1.00
REVERSE_SPEED    = -0.10
ROTATE_W         = 0.9

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

# ── 모터 속도 ─────────────────────────────────────────────────────────────────
MAX_SPEED  = 0.20
MIN_SPEED  = 0.12
MAX_W      = 1.0
TURN_GAIN  = 1.8
ALIGN_THR  = 10

# ── Fusion 가중치 ─────────────────────────────────────────────────────────────
W_COLOR_BASE  = 0.70
W_AVOID_BASE  = 0.30
AVOID_TRIGGER = 30    # cm
AVOID_FULL    = 12    # cm

# ── 스핀 탐색 ────────────────────────────────────────────────────────────────
SPIN_W = 0.9   # rad/s

# ── HSV 색상 범위 (실측 기반) ──────────────────────────────────────────────────
COLOR_PROFILES = {
    "red": [
        (np.array([0,   100, 150]), np.array([5,   255, 255])),
        (np.array([165, 100, 150]), np.array([180, 255, 255])),
    ],
    "yellow": [
        (np.array([20, 80, 150]), np.array([35, 255, 255])),
    ],
    "blue": [
        (np.array([105, 90, 150]), np.array([120, 255, 255])),
    ],
}

TARGET_COLORS = ["red", "yellow", "blue"]

SHOW_DISPLAY  = True

# ═════════════════════════════════════════════════════════════════════════════
#  Logging
# ═════════════════════════════════════════════════════════════════════════════
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("robot")

# ═════════════════════════════════════════════════════════════════════════════
#  LiDAR 공유 데이터 (스레드 → 메인)
# ═════════════════════════════════════════════════════════════════════════════
scan_lock = threading.Lock()
scan_data = np.full(360, float(SCAN_LIMIT), dtype=np.float32)

def lidar_update_scan(angle, dist_cm):
    with scan_lock:
        scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * dist_cm

def lidar_apply_median():
    with scan_lock:
        k = MEDIAN_K
        filtered = np.empty(360, dtype=np.float32)
        for i in range(360):
            idx = [(i + d) % 360 for d in range(-k, k + 1)]
            filtered[i] = np.median(scan_data[idx])
        scan_data[:] = filtered

def get_scan():
    with scan_lock:
        return scan_data.copy()

stop_event = threading.Event()

# ═════════════════════════════════════════════════════════════════════════════
#  LiDAR Thread (프로젝트2 직접 파싱)
# ═════════════════════════════════════════════════════════════════════════════
class LidarThread(threading.Thread):
    def __init__(self):
        super().__init__(name="LidarThread", daemon=True)

    def run(self):
        try:
            ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
        except Exception as e:
            log.warning(f"LiDAR 연결 실패 ({e}) – 시뮬레이션 모드")
            return

        ser.write(bytes([0xA5, 0x40]))
        time.sleep(2)
        ser.reset_input_buffer()
        ser.write(bytes([0xA5, 0x20]))
        ser.read(7)
        log.info("LiDAR 시작")

        try:
            while not stop_event.is_set():
                raw = ser.read(5)
                if len(raw) != 5:
                    continue
                s_flag = raw[0] & 0x01
                if (raw[0] & 0x02) >> 1 != (1 - s_flag): continue
                if (raw[1] & 0x01) != 1:                  continue
                if (raw[0] >> 2) < 3:                     continue
                angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
                dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
                if 3 < dist_cm < SCAN_LIMIT:
                    lidar_update_scan(angle, dist_cm)
                if s_flag == 1:
                    lidar_apply_median()
        except Exception as e:
            log.error(f"LiDAR 오류: {e}")
        finally:
            ser.write(bytes([0xA5, 0x25]))
            ser.close()

# ═════════════════════════════════════════════════════════════════════════════
#  모터 (Arduino Serial)
# ═════════════════════════════════════════════════════════════════════════════
class Motor:
    def __init__(self):
        try:
            self.ser     = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
            self.enabled = True
            time.sleep(0.5)
            log.info("Arduino 연결됨")
        except Exception as e:
            log.warning(f"Arduino 없음 ({e}) – 시뮬레이션")
            self.ser     = None
            self.enabled = False

    def send(self, v, w):
        v = float(np.clip(v, -0.30, 0.30))
        w = float(np.clip(w, -0.80, 0.80))
        if self.enabled and self.ser:
            self.ser.write(f"{v:.3f},{-w:.3f}\n".encode())
        else:
            log.debug(f"[SIM] v={v:.3f} w={w:.3f}")

    def stop(self):
        self.send(0.0, 0.0)

    def cleanup(self):
        self.stop()
        if self.ser:
            self.ser.close()

# ═════════════════════════════════════════════════════════════════════════════
#  HSV Blob 검출 (practice6/7 스타일)
# ═════════════════════════════════════════════════════════════════════════════
def detect_blob(frame, color_name):
    """최대 contour의 (cx, area) 반환. 없으면 (None, 0)"""
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_PROFILES.get(color_name, []):
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

    kernel = np.ones((3, 3), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0, frame

    c    = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA:
        return None, 0, frame

    rect         = cv2.minAreaRect(c)
    (cx, cy), _, _ = rect
    cx           = int(cx)

    # 시각화
    vis = frame.copy()
    box = np.int32(cv2.boxPoints(rect))
    cv2.drawContours(vis, [box], 0, (0, 255, 0), 2)
    cv2.circle(vis, (cx, int(cy)), 5, (0, 0, 255), -1)
    col = {"red":(0,0,255),"yellow":(0,220,220),"blue":(255,100,0)}.get(color_name,(255,255,255))
    cv2.putText(vis, f"{color_name} area:{int(area)}", (cx-40, int(cy)-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    return cx, area, vis

# ═════════════════════════════════════════════════════════════════════════════
#  FGM (프로젝트2 로직)
# ═════════════════════════════════════════════════════════════════════════════
def inflate_obstacles(dists):
    proc = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= INFLATION_MAX_DIST: continue
        alpha = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
        lo, hi = max(0, int(i-alpha)), min(len(dists)-1, int(i+alpha))
        proc[lo:hi+1] = 0.0
    return proc

def find_gaps(proc):
    gaps, start = [], None
    for i, d in enumerate(proc):
        if d > SAFE_DIST:
            if start is None: start = i
        else:
            if start is not None: gaps.append((start, i-1)); start = None
    if start is not None: gaps.append((start, len(proc)-1))
    return gaps

def score_gap(gap, proc, angles):
    s, e = gap
    return (e-s)*0.5 + np.mean(proc[s:e+1])*1.2 - abs(angles[int((s+e)/2)])*0.6

def best_idx_in_gap(gap, proc, angles):
    s, e = gap
    bi, bs = int((s+e)/2), -1e9
    for i in range(s, e+1):
        sc = proc[i]*1.0 + min(i-s, e-i)*0.8 - abs(angles[i])*1.5
        if sc > bs: bs, bi = sc, i
    return bi

def fgm_direction(scan, smoothing, prev_angle):
    angles     = np.arange(-FRONT_RANGE, FRONT_RANGE+1)
    dists      = np.array([scan[a % 360] for a in angles], dtype=np.float32)
    proc       = inflate_obstacles(dists)
    gaps       = find_gaps(proc)
    if not gaps: return None, prev_angle

    best_gap   = max(gaps, key=lambda g: score_gap(g, proc, angles))
    idx        = best_idx_in_gap(best_gap, proc, angles)
    gap_angle  = float(angles[idx])

    front_clear = float(np.min(scan[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE+1) % 360]))

    if front_clear > FRONT_CLEAR_DIST:
        target, label = gap_angle * 0.2, "STRAIGHT"
    elif front_clear > EMERGENCY_DIST * 2:
        target, label, smoothing = gap_angle, "GAP", SMOOTHING_DANGER
    else:
        target, label, smoothing = gap_angle, "CRITICAL", 0.0

    target = prev_angle * smoothing + target * (1.0 - smoothing)
    return (target, label, front_clear), target

def fuse(color_ang, fgm_ang, front_cm):
    if front_cm >= AVOID_TRIGGER:
        wc, wa = W_COLOR_BASE, W_AVOID_BASE
    elif front_cm <= AVOID_FULL:
        wc, wa = 0.05, 0.95
    else:
        wa = W_AVOID_BASE + (AVOID_TRIGGER-front_cm)/(AVOID_TRIGGER-AVOID_FULL) * (0.95-W_AVOID_BASE)
        wc = 1.0 - wa
    return float(np.clip(wc*color_ang + wa*fgm_ang, -90, 90)), wc, wa

def compute_cmd(target_angle, scan):
    w   = float(np.clip(math.radians(target_angle) * TURN_GAIN, -MAX_W, MAX_W))
    rmin = float(np.min(scan[np.arange(-FRONT_RANGE, FRONT_RANGE+1) % 360]))
    if abs(target_angle) > ALIGN_THR:
        return 0.05, w
    v = max(MAX_SPEED * min(rmin/25.0, 1.0), MIN_SPEED)
    return v, w

# ═════════════════════════════════════════════════════════════════════════════
#  Main Loop (단일 스레드: 카메라 + 제어)
# ═════════════════════════════════════════════════════════════════════════════
def main():
    motor = Motor()

    lidar_thread = LidarThread()
    lidar_thread.start()
    time.sleep(2.0)   # LiDAR 워밍업

    # 카메라 (메인 스레드에서 직접)
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    if not cap.isOpened():
        log.error("카메라 열기 실패")
        return

    # ── 상태 변수 ──────────────────────────────────────────────────────────
    color_idx       = 0
    color_ever_seen = False
    pass_timer      = None

    spin_searching  = False
    spin_end        = 0.0

    em_state        = STATE_NORMAL
    maneuver_end    = 0.0
    rotate_dir      = 1
    prev_fgm_angle  = 0.0

    last_seen_x     = FRAME_W // 2

    log.info(f"=== MISSION START | 타깃 순서: {TARGET_COLORS} ===")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # 버퍼 비우기 – 항상 최신 프레임 사용
            cap.grab()

            frame = cv2.flip(frame, 1)
            now   = time.time()
            scan  = get_scan()

            # ── DONE ────────────────────────────────────────────────────────
            if color_idx >= len(TARGET_COLORS):
                motor.stop()
                cv2.putText(frame, "MISSION COMPLETE", (30, FRAME_H//2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
                if SHOW_DISPLAY:
                    cv2.imshow("Robot", frame)
                    cv2.waitKey(1)
                time.sleep(0.1)
                continue

            color_name = TARGET_COLORS[color_idx]
            cx, area, vis = detect_blob(frame, color_name)
            color_found   = cx is not None

            if color_found:
                last_seen_x = cx

            front_min = float(np.min(scan[np.arange(-10, 11) % 360]))

            # ── Emergency state machine ──────────────────────────────────────
            if em_state == STATE_REVERSE:
                if now < maneuver_end:
                    motor.send(REVERSE_SPEED, 0.0)
                else:
                    em_state     = STATE_ROTATE
                    maneuver_end = now + ROTATE_DURATION
                _hud(vis, "REVERSE", front_min, color_name)
                _show(vis)
                continue

            if em_state == STATE_ROTATE:
                if now < maneuver_end:
                    motor.send(0.0, ROTATE_W * rotate_dir)
                else:
                    rotate_dir    *= -1
                    em_state       = STATE_NORMAL
                    prev_fgm_angle = 0.0
                _hud(vis, "ROTATE", front_min, color_name)
                _show(vis)
                continue

            if front_min < EMERGENCY_DIST:
                la = float(np.mean(scan[1:90]))
                ra = float(np.mean(scan[271:360]))
                rotate_dir   = 1 if la >= ra else -1
                em_state     = STATE_REVERSE
                maneuver_end = now + REVERSE_DURATION
                motor.send(REVERSE_SPEED, 0.0)
                _show(vis)
                continue

            # ── 스핀 탐색 ────────────────────────────────────────────────────
            if spin_searching:
                if color_found:
                    spin_searching = False
                    log.info("스핀 탐색 성공 → 추적 시작")
                else:
                    motor.send(0.0, SPIN_W)
                    _hud(vis, "SPIN SEARCH", front_min, color_name)
                    _show(vis)
                    continue

            # ── FGM ──────────────────────────────────────────────────────────
            smoothing  = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
            fgm_result, prev_fgm_angle = fgm_direction(scan, smoothing, prev_fgm_angle)

            if fgm_result is None:
                la = float(np.mean(scan[1:90]))
                ra = float(np.mean(scan[271:360]))
                rotate_dir   = 1 if la >= ra else -1
                em_state     = STATE_REVERSE
                maneuver_end = now + REVERSE_DURATION
                _show(vis)
                continue

            fgm_angle, bias_label, front_clear = fgm_result

            # ── Fusion + 모터 ────────────────────────────────────────────────
            if color_found:
                blended, wc, wa = fuse(
                    (cx - FRAME_W//2) / (FRAME_W/2) * 90.0,
                    fgm_angle, front_clear
                )
            else:
                blended, wc, wa = fgm_angle, 0.0, 1.0

            v, w = compute_cmd(blended, scan)
            motor.send(v, w)

            # ── 통과 판정 ────────────────────────────────────────────────────
            if color_found:
                color_ever_seen = True
                pass_timer      = None
            else:
                if color_ever_seen:
                    if pass_timer is None:
                        pass_timer = now
                        log.info(f"[{color_name}] 색 사라짐 → 1초 대기…")
                    elif now - pass_timer >= PASS_HOLD_SEC:
                        log.info(f"[{color_name}] PASSED → 다음 색 탐색 스핀 시작")
                        motor.stop()
                        color_idx      += 1
                        color_ever_seen = False
                        pass_timer      = None
                        if color_idx < len(TARGET_COLORS):
                            spin_searching = True
                            spin_end       = now + (2 * math.pi / SPIN_W)

            # ── HUD + 화면 ───────────────────────────────────────────────────
            label = f"{bias_label} | v:{v:.2f} w:{w:.2f} wC:{wc:.2f} f:{front_clear:.0f}cm"
            _hud(vis, label, front_min, color_name)
            _show(vis)

    except KeyboardInterrupt:
        log.info("중단")
    finally:
        stop_event.set()
        motor.cleanup()
        cap.release()
        if SHOW_DISPLAY:
            cv2.destroyAllWindows()
        log.info("=== 종료 ===")

# ─── HUD 헬퍼 ─────────────────────────────────────────────────────────────────
def _hud(frame, state_str, front_min, color_name):
    if frame is None: return
    cv2.putText(frame, state_str,           (10, 25),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),   2)
    cv2.putText(frame, f"TARGET:{color_name}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
    cv2.putText(frame, f"front:{front_min:.0f}cm", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,0), 1)

def _show(frame):
    if not SHOW_DISPLAY or frame is None: return
    cv2.imshow("Robot", frame)
    if cv2.waitKey(1) == 27:
        stop_event.set()

if __name__ == "__main__":
    main()
