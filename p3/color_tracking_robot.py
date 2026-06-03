#!/usr/bin/env python3
"""
Project 3: Color-Zone Tracking Robot
=====================================
모터/LiDAR 제어 방식: 프로젝트2 코드 기반
  - 모터  : Arduino Serial (/dev/serial0, 115200) → "v,w\n" 형식
  - LiDAR : 직접 시리얼 파싱 (/dev/ttyUSB0, 460800)  ← rplidar 라이브러리 불필요
  - Camera: OpenCV HSV blob 검출 (practice6/7 스타일)
  - FGM   : 프로젝트2 gap/inflation 로직 그대로 재사용
  - Fusion: color_angle + FGM safe_angle 가중 평균
  - State Machine: TARGET_COLORS 순차 추적 → 마지막 색 통과 시 정지
"""

import cv2
import numpy as np
import threading
import time
import math
import logging
import serial

# ═════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═════════════════════════════════════════════════════════════════════════════

# ── Serial 포트 ───────────────────────────────────────────────────────────────
ARDUINO_PORT   = "/dev/serial0"
ARDUINO_BAUD   = 115200
LIDAR_PORT     = "/dev/ttyUSB0"
LIDAR_BAUD     = 460800

# ── Camera ────────────────────────────────────────────────────────────────────
CAMERA_INDEX   = 0
FRAME_W        = 640
FRAME_H        = 480
MIN_BLOB_AREA  = 1500      # 최소 contour 면적 (practice6 스타일)
PASS_AREA_THR  = 40000     # 통과 판정 blob 면적 (px²)
CENTER_DEAD    = 30        # 중앙 dead-zone (±px)

# ── LiDAR / FGM (프로젝트2 파라미터 그대로) ──────────────────────────────────
SCAN_LIMIT          = 150   # cm, 유효 인식 거리
FRONT_RANGE         = 60    # ±deg, 탐색 반경
ROBOT_RADIUS        = 17.0  # cm
SAFE_DIST           = 14    # cm, gap 인정 기준
INFLATION_MAX_DIST  = 25    # cm
FRONT_CLEAR_DIST    = 12    # cm
FRONT_CLEAR_RANGE   = 15    # deg
EMA_ALPHA           = 0.3
MEDIAN_K            = 2
SMOOTHING_NORMAL    = 0.55
SMOOTHING_DANGER    = 0.20
DANGER_DIST         = 8     # cm

# ── Emergency / State Machine (프로젝트2) ─────────────────────────────────────
EMERGENCY_DIST    = 6       # cm
REVERSE_DURATION  = 0.18    # s
ROTATE_DURATION   = 1.00    # s
REVERSE_SPEED     = -0.10   # m/s
ROTATE_W          = 0.9     # rad/s

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

# ── 모터 속도 (프로젝트2 단위: m/s, rad/s) ────────────────────────────────────
MAX_SPEED     = 0.20
MIN_SPEED     = 0.12
MAX_W         = 1.0
TURN_GAIN     = 1.8
ALIGN_THR     = 10          # deg, 이 이하면 직진 취급

# ── Fusion 가중치 ─────────────────────────────────────────────────────────────
W_COLOR_BASE  = 0.70        # 장애물 없을 때
W_AVOID_BASE  = 0.30
AVOID_TRIGGER = 30          # cm
AVOID_FULL    = 12          # cm

# ── HSV 색상 프로파일 (practice6/7 기반) ──────────────────────────────────────
COLOR_PROFILES = {
    "red": [
        (np.array([0,   120,  70]), np.array([10,  255, 255])),
        (np.array([170, 120,  70]), np.array([180, 255, 255])),
    ],
    "yellow": [
        (np.array([20, 100, 100]), np.array([40, 255, 255])),
    ],
    "blue": [
        (np.array([100, 150,  50]), np.array([130, 255, 255])),
    ],
    "green": [
        (np.array([40,  80,  80]), np.array([80,  255, 255])),
    ],
}

TARGET_COLORS = ["red", "yellow", "blue"]   # 마지막 색 통과 시 정지

SHOW_DISPLAY  = True   # VNC True, headless False

# ═════════════════════════════════════════════════════════════════════════════
#  Logging
# ═════════════════════════════════════════════════════════════════════════════
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s %(threadName)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("robot")

# ═════════════════════════════════════════════════════════════════════════════
#  Shared State
# ═════════════════════════════════════════════════════════════════════════════
class SharedState:
    def __init__(self):
        self._lock       = threading.Lock()
        self.color_angle = 0.0
        self.blob_area   = 0.0
        self.blob_cx     = FRAME_W // 2
        self.color_found = False
        self.debug_frame = None
        # LiDAR (프로젝트2와 동일: cm 단위 scan_data 배열)
        self.scan_data   = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
        self.lidar_ready = False
        # State machine
        self.color_idx   = 0
        self.robot_state = "TRACKING"   # TRACKING / DONE

    def update_camera(self, angle, area, cx, found, frame=None):
        with self._lock:
            self.color_angle = angle
            self.blob_area   = area
            self.blob_cx     = cx
            self.color_found = found
            if frame is not None:
                self.debug_frame = frame

    def update_scan(self, angle, dist_cm):
        """EMA 적용 후 저장 (LidarThread에서 호출)"""
        with self._lock:
            self.scan_data[angle] = (
                (1.0 - EMA_ALPHA) * self.scan_data[angle] + EMA_ALPHA * dist_cm
            )

    def apply_median(self):
        with self._lock:
            k = MEDIAN_K
            filtered = np.empty(360, dtype=np.float32)
            for i in range(360):
                indices = [(i + d) % 360 for d in range(-k, k + 1)]
                filtered[i] = np.median(self.scan_data[indices])
            self.scan_data[:] = filtered
            self.lidar_ready = True

    def get_scan_copy(self):
        with self._lock:
            return self.scan_data.copy()

    def read_cam(self):
        with self._lock:
            return (self.color_angle, self.blob_area, self.blob_cx,
                    self.color_found, self.debug_frame)

    def read_state(self):
        with self._lock:
            return self.color_idx, self.robot_state

    def next_color(self):
        with self._lock:
            self.color_idx += 1
            if self.color_idx >= len(TARGET_COLORS):
                self.robot_state = "DONE"
            else:
                self.robot_state = "TRACKING"

state      = SharedState()
stop_event = threading.Event()

# ═════════════════════════════════════════════════════════════════════════════
#  Arduino Serial (모터 명령 송신)
# ═════════════════════════════════════════════════════════════════════════════
class MotorController:
    def __init__(self):
        try:
            self.ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
            time.sleep(0.5)
            log.info(f"Arduino connected: {ARDUINO_PORT}")
            self.enabled = True
        except Exception as e:
            log.warning(f"Arduino not found ({e}) – simulation mode")
            self.ser     = None
            self.enabled = False

    def send(self, v, w):
        """프로젝트2 동일 형식: 'v,w\n'  (w 부호: -w 전송)"""
        if self.enabled and self.ser:
            try:
                self.ser.write(f"{v:.3f},{-w:.3f}\n".encode())
            except Exception as e:
                log.error(f"Serial write error: {e}")
        else:
            log.debug(f"[SIM] v={v:.3f}  w={w:.3f}")

    def stop(self):
        self.send(0.0, 0.0)
        log.info("Motors stopped")

    def cleanup(self):
        self.stop()
        if self.ser:
            self.ser.close()

# ═════════════════════════════════════════════════════════════════════════════
#  HSV Blob Detection  (practice6 / practice7 스타일)
# ═════════════════════════════════════════════════════════════════════════════
def detect_color_blob(frame, color_name):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in COLOR_PROFILES.get(color_name, []):
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    vis = frame.copy()
    best_cx, best_cy, best_area = FRAME_W // 2, FRAME_H // 2, 0.0

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area > MIN_BLOB_AREA:
            x, y, w, h = cv2.boundingRect(largest)
            best_cx, best_cy, best_area = x + w // 2, y + h // 2, area
            col = {"red":(0,0,255),"yellow":(0,220,220),
                   "blue":(255,0,0),"green":(0,255,0)}.get(color_name,(255,255,255))
            cv2.rectangle(vis, (x, y), (x+w, y+h), col, 2)
            cv2.circle(vis, (best_cx, best_cy), 6, (0,0,255), -1)
            cv2.putText(vis, f"{color_name}|cx:{best_cx} area:{int(area)}",
                        (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

    mask_overlay = cv2.merge([mask//3, mask//3, mask//3])
    vis = cv2.addWeighted(vis, 1.0, mask_overlay, 0.3, 0)
    return best_cx, best_cy, best_area, vis

def compute_color_angle(cx):
    offset = cx - (FRAME_W // 2)
    if abs(offset) < CENTER_DEAD:
        return 0.0
    return float(np.clip((offset / (FRAME_W / 2)) * 90.0, -90, 90))

# ═════════════════════════════════════════════════════════════════════════════
#  Camera Thread
# ═════════════════════════════════════════════════════════════════════════════
class CameraThread(threading.Thread):
    def __init__(self):
        super().__init__(name="CameraThread", daemon=True)

    def run(self):
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if not cap.isOpened():
            log.error("Camera open failed!")
            return
        log.info("Camera thread started")

        while not stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            cidx, rstst = state.read_state()
            if rstst == "DONE" or cidx >= len(TARGET_COLORS):
                state.update_camera(0, 0, FRAME_W//2, False, frame)
                continue

            color_name = TARGET_COLORS[cidx]
            cx, cy, area, vis = detect_color_blob(frame, color_name)
            found = area > MIN_BLOB_AREA
            angle = compute_color_angle(cx) if found else 0.0

            cv2.putText(vis, f"Target: {color_name} [{cidx+1}/{len(TARGET_COLORS)}]",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(vis, f"Angle:{angle:.1f}  Area:{int(area)}",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,255,200), 2)
            cv2.putText(vis, f"State: {rstst}",
                        (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,255), 2)
            cv2.line(vis, (FRAME_W//2, 0), (FRAME_W//2, FRAME_H), (128,128,128), 1)
            state.update_camera(angle, area, cx, found, vis)

        cap.release()
        log.info("Camera thread stopped")

# ═════════════════════════════════════════════════════════════════════════════
#  LiDAR Thread  (프로젝트2 직접 시리얼 파싱 방식)
# ═════════════════════════════════════════════════════════════════════════════
class LidarThread(threading.Thread):
    def __init__(self):
        super().__init__(name="LidarThread", daemon=True)

    def run(self):
        try:
            ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
        except Exception as e:
            log.warning(f"LiDAR not found ({e}) – simulation mode")
            while not stop_event.is_set():
                time.sleep(0.1)
            return

        # LiDAR 시작 (프로젝트2 동일)
        ser.write(bytes([0xA5, 0x40]))   # Reset
        time.sleep(2)
        ser.reset_input_buffer()
        ser.write(bytes([0xA5, 0x20]))   # Scan Start
        ser.read(7)                       # 응답 헤더 소비
        log.info("LiDAR thread started")

        try:
            while not stop_event.is_set():
                raw = ser.read(5)
                if len(raw) != 5:
                    continue

                # 프로젝트2 파싱 로직 그대로
                s_flag = raw[0] & 0x01
                if (raw[0] & 0x02) >> 1 != (1 - s_flag):
                    continue
                if (raw[1] & 0x01) != 1:
                    continue
                if (raw[0] >> 2) < 3:
                    continue

                angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
                dist_cm = (raw[3] | (raw[4] << 8)) / 40.0

                if 3 < dist_cm < SCAN_LIMIT:
                    state.update_scan(angle, dist_cm)

                # 한 바퀴 완료(s_flag==1) 시 median 필터 적용
                if s_flag == 1:
                    state.apply_median()

        except Exception as e:
            log.error(f"LiDAR error: {e}")
        finally:
            ser.write(bytes([0xA5, 0x25]))   # Scan Stop
            ser.close()
            log.info("LiDAR thread stopped")

# ═════════════════════════════════════════════════════════════════════════════
#  FGM  (프로젝트2 로직 그대로)
# ═════════════════════════════════════════════════════════════════════════════
def inflate_obstacles(dists):
    proc = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= INFLATION_MAX_DIST:
            continue
        alpha     = math.degrees(math.asin(min(ROBOT_RADIUS / d, 1.0)))
        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists) - 1, int(i + alpha))
        proc[start_idx:end_idx+1] = 0.0
    return proc

def find_gaps(proc_dists):
    gaps, gap_start = [], None
    for i, d in enumerate(proc_dists):
        if d > SAFE_DIST:
            if gap_start is None: gap_start = i
        else:
            if gap_start is not None:
                gaps.append((gap_start, i-1))
                gap_start = None
    if gap_start is not None:
        gaps.append((gap_start, len(proc_dists)-1))
    return gaps

def score_gap(gap, proc_dists, angles):
    start, end  = gap
    center_i    = (start + end) / 2.0
    center_angle = angles[int(center_i)]
    avg_dist    = np.mean(proc_dists[start:end+1])
    return (end - start) * 0.5 + avg_dist * 1.2 - abs(center_angle) * 0.6

def find_best_index_in_gap(gap, proc_dists, angles):
    start, end = gap
    best_idx, best_score = int((start+end)/2), -1e9
    for i in range(start, end+1):
        margin = min(i - start, end - i)
        s = proc_dists[i] * 1.0 + margin * 0.8 - abs(angles[i]) * 1.5
        if s > best_score:
            best_score, best_idx = s, i
    return best_idx

def fgm_find_direction(scan_data, smoothing, prev_angle):
    angles     = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists      = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    proc_dists = inflate_obstacles(dists)
    gaps       = find_gaps(proc_dists)

    if not gaps:
        return None, prev_angle

    best_gap  = max(gaps, key=lambda g: score_gap(g, proc_dists, angles))
    best_idx  = find_best_index_in_gap(best_gap, proc_dists, angles)
    gap_angle = float(angles[best_idx])

    front_clear = float(np.min(
        scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE+1) % 360]
    ))
    CRITICAL = EMERGENCY_DIST * 2

    if front_clear > FRONT_CLEAR_DIST:
        target, label = gap_angle * 0.2, "STRAIGHT"
    elif front_clear > CRITICAL:
        target, label, smoothing = gap_angle * 1.0, "GAP", SMOOTHING_DANGER
    else:
        target, label, smoothing = gap_angle * 1.0, "CRITICAL", 0.0

    target     = prev_angle * smoothing + target * (1.0 - smoothing)
    return (target, label, front_clear), target

# ═════════════════════════════════════════════════════════════════════════════
#  Fusion: color_angle + FGM gap_angle → blended steering angle
# ═════════════════════════════════════════════════════════════════════════════
def fuse_angles(color_ang, fgm_ang, front_dist_cm):
    if front_dist_cm >= AVOID_TRIGGER:
        w_c, w_a = W_COLOR_BASE, W_AVOID_BASE
    elif front_dist_cm <= AVOID_FULL:
        w_c, w_a = 0.05, 0.95
    else:
        ratio = (AVOID_TRIGGER - front_dist_cm) / (AVOID_TRIGGER - AVOID_FULL)
        w_a   = W_AVOID_BASE + ratio * (0.95 - W_AVOID_BASE)
        w_c   = 1.0 - w_a
    blended = w_c * color_ang + w_a * fgm_ang
    return float(np.clip(blended, -90, 90)), w_c, w_a

# ═════════════════════════════════════════════════════════════════════════════
#  angle → (v, w)  (프로젝트2 compute_cmd 동일)
# ═════════════════════════════════════════════════════════════════════════════
def compute_cmd(target_angle, scan_data):
    w = float(np.clip(math.radians(target_angle) * TURN_GAIN, -MAX_W, MAX_W))

    search_idx   = np.arange(-FRONT_RANGE, FRONT_RANGE+1) % 360
    relevant_min = float(np.min(scan_data[search_idx]))

    if abs(target_angle) > ALIGN_THR:
        return 0.05, w   # 회전 중 최소 전진

    obstacle_scale = min(relevant_min / 25.0, 1.0)
    v = max(MAX_SPEED * obstacle_scale, MIN_SPEED)
    return v, w

# ═════════════════════════════════════════════════════════════════════════════
#  Control Loop + State Machine  (main thread)
# ═════════════════════════════════════════════════════════════════════════════
def control_loop(motor: MotorController):
    # Emergency state machine (프로젝트2)
    em_state         = STATE_NORMAL
    maneuver_end     = 0.0
    rotate_dir       = 1
    prev_fgm_angle   = 0.0

    pass_timer       = None
    PASS_HOLD_SEC    = 1.0

    log.info(f"Control loop started. Targets: {TARGET_COLORS}")

    while not stop_event.is_set():
        t_start = time.time()
        scan    = state.get_scan_copy()
        color_ang, blob_area, blob_cx, color_found, dbg_frame = state.read_cam()
        cidx, rstst = state.read_state()
        now = time.time()

        # ── DONE ──────────────────────────────────────────────────────────
        if rstst == "DONE":
            motor.stop()
            if SHOW_DISPLAY and dbg_frame is not None:
                cv2.putText(dbg_frame, "DONE - ALL TARGETS REACHED",
                            (60, FRAME_H//2), cv2.FONT_HERSHEY_SIMPLEX,
                            0.9, (0,255,255), 2)
                cv2.imshow("Color Tracking Robot", dbg_frame)
                cv2.waitKey(1)
            time.sleep(0.1)
            continue

        # ── Emergency State Machine (프로젝트2) ───────────────────────────
        front_min = float(np.min(scan[np.arange(-10, 11) % 360]))

        if em_state == STATE_REVERSE:
            if now < maneuver_end:
                motor.send(REVERSE_SPEED, 0.0)
            else:
                em_state     = STATE_ROTATE
                maneuver_end = now + ROTATE_DURATION
            _update_display(dbg_frame, "REVERSE", 0, 0, 0, 0, front_min)
            _tick(t_start)
            continue

        if em_state == STATE_ROTATE:
            if now < maneuver_end:
                motor.send(0.0, ROTATE_W * rotate_dir)
            else:
                rotate_dir   *= -1
                em_state      = STATE_NORMAL
                prev_fgm_angle = 0.0
                for a in range(-45, 46):
                    scan[a % 360] = float(SCAN_LIMIT)
            _update_display(dbg_frame, "ROTATE", 0, 0, 0, 0, front_min)
            _tick(t_start)
            continue

        if front_min < EMERGENCY_DIST:
            left_avg  = float(np.mean(scan[1:90]))
            right_avg = float(np.mean(scan[271:360]))
            rotate_dir   = 1 if left_avg >= right_avg else -1
            em_state     = STATE_REVERSE
            maneuver_end = now + REVERSE_DURATION
            motor.send(REVERSE_SPEED, 0.0)
            _tick(t_start)
            continue

        # ── FGM ──────────────────────────────────────────────────────────
        smoothing  = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        fgm_result, prev_fgm_angle = fgm_find_direction(scan, smoothing, prev_fgm_angle)

        if fgm_result is None:
            left_avg  = float(np.mean(scan[1:90]))
            right_avg = float(np.mean(scan[271:360]))
            rotate_dir   = 1 if left_avg >= right_avg else -1
            em_state     = STATE_REVERSE
            maneuver_end = now + REVERSE_DURATION
            _tick(t_start)
            continue

        fgm_angle, bias_label, front_clear = fgm_result

        # ── Fusion ────────────────────────────────────────────────────────
        if color_found:
            blended, w_c, w_a = fuse_angles(color_ang, fgm_angle, front_clear)
        else:
            # 색 미탐지 → FGM 단독 (탐색 회전)
            blended, w_c, w_a = fgm_angle, 0.0, 1.0

        # ── Motor command ─────────────────────────────────────────────────
        v, w = compute_cmd(blended, scan)
        motor.send(v, w)

        # ── 색상 통과 판정 ─────────────────────────────────────────────────
        if rstst == "TRACKING":
            cx_ok   = abs(blob_cx - FRAME_W//2) < FRAME_W // 4
            area_ok = blob_area > PASS_AREA_THR
            if color_found and cx_ok and area_ok:
                if pass_timer is None:
                    pass_timer = now
                    log.info(f"[{TARGET_COLORS[cidx]}] Pass condition detected…")
                elif now - pass_timer >= PASS_HOLD_SEC:
                    log.info(f"[{TARGET_COLORS[cidx]}] PASSED! → next color")
                    state.next_color()
                    pass_timer = None
            else:
                pass_timer = None

        # ── Debug display ─────────────────────────────────────────────────
        log.info(f"TRG:{blended:+.1f}° v:{v:.2f} w:{w:.2f} "
                 f"f:{front_clear:.1f}cm wC:{w_c:.2f}/wA:{w_a:.2f} {bias_label}")
        _update_display(dbg_frame, bias_label, blended, v, w, w_c, front_clear)
        _tick(t_start)

    motor.stop()

# ── 디스플레이 헬퍼 ────────────────────────────────────────────────────────────
def _update_display(frame, label, angle, v, w, w_c, front_cm):
    if not SHOW_DISPLAY or frame is None:
        return
    info = (f"{label} | ang:{angle:+.1f} v:{v:.2f} w:{w:.2f} "
            f"wC:{w_c:.2f} front:{front_cm:.1f}cm")
    cv2.putText(frame, info, (10, FRAME_H-15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255,255,0), 1)
    cv2.imshow("Color Tracking Robot", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop_event.set()

def _tick(t_start):
    elapsed = time.time() - t_start
    time.sleep(max(0.0, 0.033 - elapsed))

# ═════════════════════════════════════════════════════════════════════════════
#  Main
# ═════════════════════════════════════════════════════════════════════════════
def main():
    log.info("=== Color Tracking Robot starting ===")
    log.info(f"Target sequence: {TARGET_COLORS}")

    motor        = MotorController()
    cam_thread   = CameraThread()
    lidar_thread = LidarThread()

    cam_thread.start()
    lidar_thread.start()
    time.sleep(2.0)   # LiDAR 워밍업

    try:
        control_loop(motor)
    except KeyboardInterrupt:
        log.info("KeyboardInterrupt – stopping")
    finally:
        stop_event.set()
        cam_thread.join(timeout=3)
        lidar_thread.join(timeout=3)
        motor.cleanup()
        if SHOW_DISPLAY:
            cv2.destroyAllWindows()
        log.info("=== Robot shutdown complete ===")

if __name__ == "__main__":
    main()
