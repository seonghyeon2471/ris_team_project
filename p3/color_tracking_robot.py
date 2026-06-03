#!/usr/bin/env python3
"""
Project 3: Color-Zone Tracking Robot
=====================================
Raspberry Pi + Camera + LiDAR + L298N Motor (Differential Drive)

Architecture:
  - CameraThread  : HSV blob detection → desired_angle  (practice6/7 style)
  - LidarThread   : FGM obstacle avoidance → safe_angle
  - ControlLoop   : Fusion + State Machine → motor PWM
  - StateMachine  : target_colors 순차 추적, "통과" 판정, 마지막 색상 정지

HW 가정:
  - RPi Camera (or USB cam) at /dev/video0
  - RPLIDAR A1/A2 via rplidar (pip install rplidar-roboticia)
  - L298N: IN1/IN2/ENA → left motor,  IN3/IN4/ENB → right motor
    GPIO BCM 핀은 MOTOR_PINS 딕셔너리에서 변경

Usage:
  python3 color_tracking_robot.py
  (VNC로 접속, X 서버 불필요 - imshow 비활성화 가능)
"""

import cv2
import numpy as np
import threading
import time
import math
import logging

# ── GPIO / Motor ──────────────────────────────────────────────────────────────
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("[WARN] RPi.GPIO not found – motor output disabled (simulation mode)")

# ── LiDAR ─────────────────────────────────────────────────────────────────────
try:
    from rplidar import RPLidar
    LIDAR_AVAILABLE = True
except ImportError:
    LIDAR_AVAILABLE = False
    print("[WARN] rplidar not found – LiDAR disabled (simulation mode)")

# ═════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═════════════════════════════════════════════════════════════════════════════
# ── Camera ────────────────────────────────────────────────────────────────────
CAMERA_INDEX   = 0
FRAME_W        = 640
FRAME_H        = 480
MIN_BLOB_AREA  = 1500          # 너무 작은 contour 무시 (practice6 스타일)
PASS_AREA_THR  = 40000         # 이 면적 이상이면 "통과" 판정
CENTER_DEAD    = 30            # ±px, 중앙 dead-zone

# ── LiDAR / FGM ───────────────────────────────────────────────────────────────
LIDAR_PORT     = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 115200
FGM_SAFE_DIST  = 400           # mm, 이 거리 이내 장애물 → gap 탐색
FGM_GAP_THR    = 300           # mm, gap으로 인정하는 최소 반경
MAX_LIDAR_DIST = 3500          # mm, 유효 거리 상한

# ── Fusion ────────────────────────────────────────────────────────────────────
W_COLOR_BASE   = 0.70          # 장애물 없을 때 color 가중치
W_AVOID_BASE   = 0.30          # 장애물 없을 때 FGM 가중치
AVOID_TRIGGER  = 500           # mm, 이 거리 이하 → 회피 우선
AVOID_FULL     = 250           # mm, 이 거리 이하 → 완전 회피

# ── Motor (GPIO BCM) ─────────────────────────────────────────────────────────
MOTOR_PINS = {
    "IN1": 17, "IN2": 18, "ENA": 12,   # Left
    "IN3": 27, "IN4": 22, "ENB": 13,   # Right
}
PWM_FREQ        = 1000          # Hz
BASE_SPEED      = 55            # % PWM (0~100)
MAX_SPEED       = 80
TURN_GAIN       = 0.35          # angle → differential speed
STOP_DURATION   = 2.0           # 마지막 색상 통과 후 정지 유지 시간(s)

# ── HSV 색상 프로파일 (practice6/7 기반) ──────────────────────────────────────
# 각 항목: (lower1, upper1, lower2, upper2)  – 빨강은 Hue wrap-around 처리
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

# ── State Machine: 추적할 색상 순서 ──────────────────────────────────────────
TARGET_COLORS = ["red", "yellow", "blue"]   # 마지막 색 통과 시 정지

# ── Display ────────────────────────────────────────────────────────────────────
SHOW_DISPLAY = True    # VNC 연결 시 True, headless 시 False


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
#  Shared State (thread-safe with Lock)
# ═════════════════════════════════════════════════════════════════════════════
class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        # Camera output
        self.color_angle   = 0.0    # deg, –90(left)…+90(right)
        self.blob_area     = 0.0    # px²
        self.blob_cx       = FRAME_W // 2
        self.color_found   = False
        self.debug_frame   = None
        # LiDAR output
        self.safe_angle    = 0.0    # deg
        self.min_dist      = 9999.0 # mm, front arc minimum
        self.lidar_ready   = False
        # State machine
        self.color_idx     = 0
        self.robot_state   = "TRACKING"  # TRACKING / PASSING / DONE

    def update_camera(self, angle, area, cx, found, frame=None):
        with self._lock:
            self.color_angle = angle
            self.blob_area   = area
            self.blob_cx     = cx
            self.color_found = found
            if frame is not None:
                self.debug_frame = frame

    def update_lidar(self, safe_ang, min_d):
        with self._lock:
            self.safe_angle  = safe_ang
            self.min_dist    = min_d
            self.lidar_ready = True

    def read(self):
        with self._lock:
            return (self.color_angle, self.blob_area, self.blob_cx,
                    self.color_found, self.safe_angle, self.min_dist,
                    self.color_idx, self.robot_state, self.debug_frame)

    def next_color(self):
        with self._lock:
            self.color_idx += 1
            if self.color_idx >= len(TARGET_COLORS):
                self.robot_state = "DONE"
            else:
                self.robot_state = "TRACKING"

    def set_state(self, s):
        with self._lock:
            self.robot_state = s


state = SharedState()
stop_event = threading.Event()


# ═════════════════════════════════════════════════════════════════════════════
#  HSV Blob Detection  (practice6 / practice7 style)
# ═════════════════════════════════════════════════════════════════════════════
def detect_color_blob(frame, color_name):
    """
    Returns (cx, cy, area, annotated_frame)
    practice6: bounding rect + largest contour
    practice7: centroid + distance estimation style
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    ranges = COLOR_PROFILES.get(color_name, [])

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

    # Morphological cleanup
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    vis = frame.copy()
    best_cx, best_cy, best_area = FRAME_W // 2, FRAME_H // 2, 0.0

    if contours:
        # Largest contour (practice6/7 스타일)
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area > MIN_BLOB_AREA:
            x, y, w, h = cv2.boundingRect(largest)
            best_cx = x + w // 2
            best_cy = y + h // 2
            best_area = area

            # Draw (practice7 스타일: bounding rect + centroid + info)
            col_bgr = {"red": (0,0,255), "yellow": (0,220,220),
                       "blue": (255,0,0), "green": (0,255,0)}.get(color_name, (255,255,255))
            cv2.rectangle(vis, (x, y), (x+w, y+h), col_bgr, 2)
            cv2.circle(vis, (best_cx, best_cy), 6, (0, 0, 255), -1)
            info = f"{color_name} | cx:{best_cx} area:{int(best_area)}"
            cv2.putText(vis, info, (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col_bgr, 2)

    # Mask overlay (semi-transparent)
    mask_color = cv2.merge([mask//3, mask//3, mask//3])
    vis = cv2.addWeighted(vis, 1.0, mask_color, 0.3, 0)

    return best_cx, best_cy, best_area, vis


def compute_color_angle(cx):
    """
    Frame center → desired steering angle (deg)
    Left of center → negative (turn left), right → positive (turn right)
    """
    offset = cx - (FRAME_W // 2)
    if abs(offset) < CENTER_DEAD:
        return 0.0
    # Normalize: max offset = FRAME_W/2 → ±90 deg
    angle = (offset / (FRAME_W / 2)) * 90.0
    return float(np.clip(angle, -90, 90))


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

            # Read current target color from state
            with state._lock:
                cidx  = state.color_idx
                rstst = state.robot_state

            if rstst == "DONE" or cidx >= len(TARGET_COLORS):
                state.update_camera(0, 0, FRAME_W//2, False, frame)
                continue

            color_name = TARGET_COLORS[cidx]
            cx, cy, area, vis = detect_color_blob(frame, color_name)

            found = (area > MIN_BLOB_AREA)
            angle = compute_color_angle(cx) if found else 0.0

            # HUD overlay
            cv2.putText(vis, f"Target: {color_name}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(vis, f"Angle: {angle:.1f}  Area: {int(area)}", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,255,200), 2)
            cv2.putText(vis, f"State: {rstst}  [{cidx+1}/{len(TARGET_COLORS)}]",
                        (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,255), 2)
            # Crosshair
            cv2.line(vis, (FRAME_W//2, 0), (FRAME_W//2, FRAME_H), (128,128,128), 1)

            state.update_camera(angle, area, cx, found, vis)

        cap.release()
        log.info("Camera thread stopped")


# ═════════════════════════════════════════════════════════════════════════════
#  LiDAR + FGM (Follow the Gap Method)
# ═════════════════════════════════════════════════════════════════════════════
class FGM:
    """
    Follow the Gap Method:
    1. 전방 ±FOV 스캔 데이터에서 안전 거리 이하 포인트를 장애물로 표시
    2. 가장 넓은 gap 찾기
    3. Gap 중앙 방향을 safe_angle 로 반환

    Angles: 0 deg = 로봇 정면, +right / -left (로봇 좌표계)
    """
    FOV = 180  # 전방 ±90 deg 사용

    def process(self, scan_dict):
        """
        scan_dict: {angle_deg: distance_mm, ...}  (0~360 범위)
        Returns (safe_angle_deg, min_front_dist_mm)
        """
        if not scan_dict:
            return 0.0, 9999.0

        # 1. 전방 ±(FOV/2) 범위만 추출 및 정규화
        #    RPLIDAR: 0=front, 90=right (or 90=left depending on mounting)
        #    → 여기서는 0=front, CCW positive 가정
        angles = np.arange(-self.FOV//2, self.FOV//2 + 1, dtype=float)
        distances = np.full(len(angles), MAX_LIDAR_DIST, dtype=float)

        for raw_ang, dist in scan_dict.items():
            # 0~360 → -180~180
            ang = raw_ang if raw_ang <= 180 else raw_ang - 360
            if dist <= 0 or dist > MAX_LIDAR_DIST:
                dist = MAX_LIDAR_DIST
            # 가장 가까운 bin
            idx = int(round(ang)) + self.FOV // 2
            if 0 <= idx < len(distances):
                distances[idx] = min(distances[idx], dist)

        # 2. 장애물 bubble 처리 (장애물 주변 안전 반경 확장)
        obstacle_mask = distances < FGM_SAFE_DIST
        bubble_mask = np.zeros_like(obstacle_mask)
        bubble_r = 5  # ±5 deg 버블
        for i in np.where(obstacle_mask)[0]:
            lo = max(0, i - bubble_r)
            hi = min(len(distances)-1, i + bubble_r)
            bubble_mask[lo:hi+1] = True

        safe_mask = ~bubble_mask  # True = 통과 가능

        # 3. Min front distance (–30 ~ +30 deg)
        front_lo = self.FOV // 2 - 30
        front_hi = self.FOV // 2 + 30
        front_dists = distances[front_lo:front_hi+1]
        min_front = float(np.min(front_dists)) if len(front_dists) > 0 else 9999.0

        # 4. Gap 찜하기 – 연속된 safe 구간 중 가장 넓은 것
        gaps = []
        in_gap = False
        gap_start = 0
        for i, safe in enumerate(safe_mask):
            if safe and not in_gap:
                gap_start = i
                in_gap = True
            elif not safe and in_gap:
                gaps.append((gap_start, i - 1))
                in_gap = False
        if in_gap:
            gaps.append((gap_start, len(safe_mask) - 1))

        if not gaps:
            # 완전 막힌 경우 → 뒤로 or 제자리 회전 (0 반환)
            return 0.0, min_front

        # 5. 가장 넓고 가까운 gap 선택
        best_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_center_idx = (best_gap[0] + best_gap[1]) // 2
        safe_angle = float(angles[gap_center_idx])   # deg

        return safe_angle, min_front


class LidarThread(threading.Thread):
    def __init__(self):
        super().__init__(name="LidarThread", daemon=True)
        self.fgm = FGM()

    def run(self):
        if not LIDAR_AVAILABLE:
            log.warning("LiDAR simulation: always clear")
            while not stop_event.is_set():
                state.update_lidar(0.0, 9999.0)
                time.sleep(0.1)
            return

        try:
            lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
            lidar.connect()
            lidar.start_motor()
            log.info("LiDAR thread started")

            for scan in lidar.iter_scans(max_buf_meas=500):
                if stop_event.is_set():
                    break
                # scan: list of (quality, angle, distance)
                scan_dict = {a: d for (_, a, d) in scan if d > 0}
                safe_ang, min_d = self.fgm.process(scan_dict)
                state.update_lidar(safe_ang, min_d)

            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception as e:
            log.error(f"LiDAR error: {e}")
            # Fallback: clear path
            while not stop_event.is_set():
                state.update_lidar(0.0, 9999.0)
                time.sleep(0.1)

        log.info("LiDAR thread stopped")


# ═════════════════════════════════════════════════════════════════════════════
#  Motor Controller (L298N + PWM)
# ═════════════════════════════════════════════════════════════════════════════
class MotorController:
    def __init__(self):
        self.enabled = GPIO_AVAILABLE
        self.left_pwm  = None
        self.right_pwm = None
        if self.enabled:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            for pin in MOTOR_PINS.values():
                GPIO.setup(pin, GPIO.OUT)
            self.left_pwm  = GPIO.PWM(MOTOR_PINS["ENA"], PWM_FREQ)
            self.right_pwm = GPIO.PWM(MOTOR_PINS["ENB"], PWM_FREQ)
            self.left_pwm.start(0)
            self.right_pwm.start(0)
            log.info("Motor controller initialized")

    def _set_motor(self, in1, in2, pwm_obj, speed):
        """speed: –100(back) … +100(fwd)"""
        if not self.enabled:
            return
        spd = int(abs(np.clip(speed, -100, 100)))
        if speed >= 0:
            GPIO.output(MOTOR_PINS[in1], GPIO.HIGH)
            GPIO.output(MOTOR_PINS[in2], GPIO.LOW)
        else:
            GPIO.output(MOTOR_PINS[in1], GPIO.LOW)
            GPIO.output(MOTOR_PINS[in2], GPIO.HIGH)
        pwm_obj.ChangeDutyCycle(spd)

    def set_speeds(self, left_spd, right_spd):
        self._set_motor("IN1", "IN2", self.left_pwm,  left_spd)
        self._set_motor("IN3", "IN4", self.right_pwm, right_spd)
        if not self.enabled:
            log.debug(f"  [SIM] L={left_spd:+.1f}  R={right_spd:+.1f}")

    def stop(self):
        self.set_speeds(0, 0)
        log.info("Motors stopped")

    def cleanup(self):
        self.stop()
        if self.enabled:
            if self.left_pwm:  self.left_pwm.stop()
            if self.right_pwm: self.right_pwm.stop()
            GPIO.cleanup()


# ═════════════════════════════════════════════════════════════════════════════
#  Angle → Differential Speed
# ═════════════════════════════════════════════════════════════════════════════
def angle_to_speeds(target_angle_deg, base_spd=BASE_SPEED):
    """
    target_angle: deg (–=left, +=right)
    Returns (left_speed, right_speed)  0~MAX_SPEED range
    """
    t = np.clip(target_angle_deg / 90.0, -1.0, 1.0)  # –1 … +1
    if t >= 0:
        left  = base_spd
        right = base_spd * (1.0 - t * TURN_GAIN * 2)
    else:
        left  = base_spd * (1.0 + t * TURN_GAIN * 2)
        right = base_spd
    return float(np.clip(left, -MAX_SPEED, MAX_SPEED)), \
           float(np.clip(right, -MAX_SPEED, MAX_SPEED))


# ═════════════════════════════════════════════════════════════════════════════
#  Fusion: color_angle + safe_angle → blended_angle
# ═════════════════════════════════════════════════════════════════════════════
def fuse_angles(color_ang, safe_ang, min_dist):
    """
    장애물이 가까울수록 FGM(safe_angle) 가중치 증가
    """
    if min_dist >= AVOID_TRIGGER:
        w_c, w_a = W_COLOR_BASE, W_AVOID_BASE
    elif min_dist <= AVOID_FULL:
        w_c, w_a = 0.05, 0.95
    else:
        # 선형 보간
        ratio = (AVOID_TRIGGER - min_dist) / (AVOID_TRIGGER - AVOID_FULL)
        w_a = W_AVOID_BASE + ratio * (0.95 - W_AVOID_BASE)
        w_c = 1.0 - w_a

    blended = w_c * color_ang + w_a * safe_ang
    return float(np.clip(blended, -90, 90)), w_c, w_a


# ═════════════════════════════════════════════════════════════════════════════
#  State Machine + Control Loop  (main thread)
# ═════════════════════════════════════════════════════════════════════════════
def control_loop(motor: MotorController):
    """
    State Machine:
      TRACKING  → 현재 색상 blob 추적
      PASSING   → blob이 충분히 크고 중앙 → 통과 판정 대기
      DONE      → 정지
    """
    pass_timer    = None
    PASS_HOLD_SEC = 1.0   # 통과 조건 지속 시간

    log.info(f"Control loop started. Targets: {TARGET_COLORS}")

    while not stop_event.is_set():
        t_start = time.time()

        (color_ang, blob_area, blob_cx,
         color_found, safe_ang, min_dist,
         cidx, rstst, dbg_frame) = state.read()

        # ── DONE ──────────────────────────────────────────────────────────
        if rstst == "DONE":
            motor.stop()
            log.info("=== All targets visited. STOP ===")
            if SHOW_DISPLAY and dbg_frame is not None:
                cv2.putText(dbg_frame, "DONE - ALL TARGETS REACHED",
                            (80, FRAME_H//2), cv2.FONT_HERSHEY_SIMPLEX,
                            0.9, (0,255,255), 2)
                cv2.imshow("Color Tracking Robot", dbg_frame)
                cv2.waitKey(1)
            time.sleep(0.1)
            continue

        # ── Fusion ────────────────────────────────────────────────────────
        if color_found:
            blended_ang, w_c, w_a = fuse_angles(color_ang, safe_ang, min_dist)
        else:
            # 색상 미탐지 → 천천히 제자리 회전 (탐색)
            blended_ang = 20.0   # 약간 우회전으로 탐색
            w_c, w_a = 0.0, 1.0

        # ── Motor command ─────────────────────────────────────────────────
        l_spd, r_spd = angle_to_speeds(blended_ang)
        motor.set_speeds(l_spd, r_spd)

        # ── State Machine ─────────────────────────────────────────────────
        if rstst == "TRACKING":
            # 통과 조건: blob이 크고(PASS_AREA_THR) 중앙 부근
            cx_ok   = abs(blob_cx - FRAME_W//2) < FRAME_W // 4
            area_ok = blob_area > PASS_AREA_THR
            if color_found and cx_ok and area_ok:
                if pass_timer is None:
                    pass_timer = time.time()
                    log.info(f"[{TARGET_COLORS[cidx]}] Pass condition detected, holding…")
                elif time.time() - pass_timer >= PASS_HOLD_SEC:
                    log.info(f"[{TARGET_COLORS[cidx]}] PASSED! → next color")
                    state.next_color()
                    pass_timer = None
            else:
                pass_timer = None

        # ── Debug display ─────────────────────────────────────────────────
        if SHOW_DISPLAY and dbg_frame is not None:
            h_info = (f"blend:{blended_ang:+.1f} "
                      f"wC:{w_c:.2f}/wA:{w_a:.2f}  "
                      f"minD:{min_dist:.0f}mm  "
                      f"L:{l_spd:.0f} R:{r_spd:.0f}")
            cv2.putText(dbg_frame, h_info, (10, FRAME_H - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.imshow("Color Tracking Robot", dbg_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

        # ~30 Hz
        elapsed = time.time() - t_start
        time.sleep(max(0.0, 0.033 - elapsed))

    motor.stop()


# ═════════════════════════════════════════════════════════════════════════════
#  Main
# ═════════════════════════════════════════════════════════════════════════════
def main():
    log.info("=== Color Tracking Robot starting ===")
    log.info(f"Target sequence: {TARGET_COLORS}")

    motor = MotorController()

    cam_thread   = CameraThread()
    lidar_thread = LidarThread()

    cam_thread.start()
    lidar_thread.start()

    # LiDAR warmup
    time.sleep(2.0)

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
