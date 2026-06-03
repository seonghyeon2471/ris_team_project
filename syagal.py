import cv2
import serial
import numpy as np
import time
import threading

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# =========================================
# LIDAR START
# =========================================
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR START")

# =========================================
# LIDAR PARAMETERS
# =========================================
MAX_SPEED         = 0.30
MIN_SPEED         = 0.09
MAX_W             = 0.9
THRESH_30         = 32.0
THRESH_20         = 22.0
THRESH_10         = 12.0
FRONT_CHECK_RANGE = 45

EMA_ALPHA = 0.35
MEDIAN_K  = 2

_scan_buf    = np.full(360, 150.0, dtype=np.float32)
_scan_shared = np.full(360, 150.0, dtype=np.float32)
scan_lock    = threading.Lock()

# =========================================
# CAMERA PARAMETERS (★신속성 및 회전 강화 튜닝)
# =========================================
MAX_V       = 0.24      # 주행 최고속도 상향
MIN_V       = 0.10      # ★ 최저 주행속도 상향 (목표 직전 굼뜸 현상 해결)
KP_ROT      = 0.003     # 회전 반응성 약간 상향
X_TOL       = 35        # ★ 중앙 근삿값 허용 (기존 22 -> 35로 대폭 완화)
MIN_AREA    = 500
TARGET_AREA = 15000     # 면적 임계값 최적화
PARK_SEC    = 3.0

# =========================================
# 색상별 HSV + BGR 범위
# =========================================
COLOR_CFG = {
    "red": {
        "hsv1": ([0,   70, 70], [12,  255, 255]),
        "hsv2": ([160, 70, 70], [179, 255, 255]),
        "bgr":  ([40,  20, 120], [210, 170, 255]),
        "draw": (0, 0, 255),
    },
    "yellow": {
        "hsv1": ([18,  80, 80], [38,  255, 255]),
        "hsv2": None,
        "bgr":  ([0,  150, 150], [100, 255, 255]),
        "draw": (0, 200, 255),
    },
    "blue": {
        "hsv1": ([95,  80, 60], [135, 255, 255]),
        "hsv2": None,
        "bgr":  ([100, 50,  0], [255, 150,  80]),
        "draw": (255, 80, 0),
    },
}

MISSION = ["red", "yellow", "blue"]

# =========================================
# 상태 변수
# =========================================
mission_index = 0
state         = "SEARCH"
last_seen_x   = 160
park_start    = None

# =========================================
# LIDAR UTIL
# =========================================
def _apply_ema(angle, dist_cm):
    if dist_cm <= 0:
        return
    _scan_buf[angle] = (1.0 - EMA_ALPHA) * _scan_buf[angle] + EMA_ALPHA * dist_cm

def _apply_median_filter():
    k = MEDIAN_K
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
        filtered[i] = np.sort(_scan_buf[idx])[k]
    _scan_buf[:] = filtered

def _publish_scan():
    with scan_lock:
        _scan_shared[:] = _scan_buf

def get_front_min():
    with scan_lock:
        indices = np.arange(-FRONT_CHECK_RANGE, FRONT_CHECK_RANGE + 1) % 360
        return float(np.min(_scan_shared[indices]))

def choose_avoid_direction():
    with scan_lock:
        left_avg  = float(np.mean(_scan_shared[1:90]))
        right_avg = float(np.mean(_scan_shared[271:360]))
    return 1 if left_avg >= right_avg else -1

# =========================================
# LIDAR 스레드
# =========================================
def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue
        s_flag = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - s_flag) \
                or (raw[1] & 0x01) != 1 \
                or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150:
            _apply_ema(angle, dist_cm)
        if s_flag == 1:
            _apply_median_filter()
            _publish_scan()

lidar_thread = threading.Thread(target=lidar_loop, daemon=True)
lidar_thread.start()
print("LIDAR THREAD START")

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    v = np.clip(v, -0.3, 0.3)
    w = np.clip(w, -0.8, 0.8)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# 마스크 생성
# =========================================
def make_mask(frame, hsv, color_name):
    cfg    = COLOR_CFG[color_name]
    kernel = np.ones((3, 3), np.uint8)

    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    hsv_mask = cv2.inRange(hsv, lo1, hi1)
    if cfg["hsv2"] is not None:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        hsv_mask = cv2.bitwise_or(hsv_mask, cv2.inRange(hsv, lo2, hi2))

    bgr_mask = cv2.inRange(frame,
                           np.array(cfg["bgr"][0]),
                           np.array(cfg["bgr"][1]))

    mask = cv2.bitwise_and(hsv_mask, bgr_mask)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask

# =========================================
# 동작 우선순위
# =========================================
def decide_cmd(cam_v, cam_w, front_min):
    if front_min < THRESH_10:
        direction = choose_avoid_direction()
        return MIN_SPEED, direction * MAX_W, True

    elif front_min < THRESH_20:
        direction = choose_avoid_direction()
        avoid_w   = direction * 0.8
        if direction * cam_w >= 0:
            blended_w = 0.3 * cam_w + 0.7 * avoid_w
        else:
            blended_w = avoid_w
        return 0.12, blended_w, True

    elif front_min < THRESH_30:
        direction = choose_avoid_direction()
        avoid_w   = direction * 0.7
        if direction * cam_w >= 0:
            blended_w = 0.5 * cam_w + 0.5 * avoid_w
        else:
            blended_w = avoid_w
        v = min(cam_v, 0.15)
        return v, blended_w, True

    else:
        return cam_v, cam_w, False

# =========================================
# 카메라 버퍼 flush
# =========================================
def flush_camera_buffer(n=3):
    for _ in range(n):
        cap.grab()

# =========================================
# START
# =========================================
print("=" * 45)
print(f"   주차 미션 시작: {MISSION}")
print(f"   첫 번째 목표  : [{MISSION[0]}]")
print("   ESC 키로 종료")
print("=" * 45)
time.sleep(1.0)

try:
    while True:

        ret, frame = cap.read()
        if not ret:
            continue

        frame    = cv2.flip(frame, 1)
        HEIGHT, WIDTH = frame.shape[:2]
        frame_cx = WIDTH  // 2
        frame_cy = HEIGHT // 2

        hsv       = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        front_min = get_front_min()

        cv2.circle(frame, (frame_cx, frame_cy), 5, (0, 255, 255), -1)

        lidar_color = (0, 255, 255) if front_min > THRESH_30 else (0, 0, 255)
        cv2.putText(frame, f"LIDAR:{front_min:.0f}cm",
                    (WIDTH - 110, HEIGHT - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, lidar_color, 1)

        # ── 미션 완료 ──────────────────────────
        if mission_index >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "MISSION COMPLETE!",
                        (20, HEIGHT // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        target = MISSION[mission_index]
        draw   = COLOR_CFG[target]["draw"]

        mask = make_mask(frame, hsv, target)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # ── PARKING 상태 ──────────────────────────
        if state == "PARKING":
            stop_robot()

            elapsed = time.time() - park_start
            remain   = max(0.0, PARK_SEC - elapsed)

            cv2.putText(frame, f"PARKING [{target}] {remain:.1f}s",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, draw, 2)
            cv2.putText(frame, f"MISSION {mission_index+1}/{len(MISSION)}",
                        (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

            if front_min < THRESH_10:
                cv2.putText(frame, "! OBSTACLE NEAR !",
                            (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            if elapsed >= PARK_SEC:
                print(f"✅ [{target}] 정차 완료!")
                mission_index += 1
                if mission_index >= len(MISSION):
                    print("🏁 미션 전체 완료!")
                else:
                    # ★ 개정: 다음 구역 인식을 위해 강제 탐색 상태로 세팅
                    state = "FORCED_SEARCH" 
                    last_seen_x = 0  # 한쪽 방향(왼쪽)으로 강제 회전 유도 부호 설정
                    flush_camera_buffer()
                    print(f"➡️  다음 목표: [{MISSION[mission_index]}] 탐색을 위해 강제 회전을 시작합니다.")

            cv2.imshow("frame", frame)
            cv2.imshow("mask",  mask)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        cam_v     = 0.0
        cam_w     = 0.0

        # ── 객체 감지 및 주행 제어 ──────────────────────────
        if contours:
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                # 목표 색상을 일단 찾았다면 강제 탐색 모드 해제 및 정상 TRACK 전환
                if state == "FORCED_SEARCH":
                    state = "TRACK"

                rect = cv2.minAreaRect(c)
                (cx, cy), (rw, rh), angle = rect
                cx = int(cx)
                cy = int(cy)

                last_seen_x = cx

                box = np.int32(cv2.boxPoints(rect))
                cv2.drawContours(frame, [box], 0, draw, 2)
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

                error_x = cx - frame_cx

                # ★ ARRIVED 판정 (X_TOL 35로 근삿값 허용)
                if area > TARGET_AREA:
                    if abs(error_x) < X_TOL:
                        stop_robot()
                        state      = "PARKING"
                        park_start = time.time()
                        print(f"🅿️  [{target}] 도착(A:{int(area)}) → 정차 시작")
                        cv2.imshow("frame", frame)
                        cv2.imshow("mask",  mask)
                        cv2.waitKey(1)
                        continue
                    else:
                        # ★ 개정: 면적은 찼는데 중심 정렬중일 때 멈추지 않고 미세 전진(0.08) 결합하여 신속 진입
                        cam_v = 0.08
                        cam_w = -KP_ROT * error_x
                        cam_state = "ALIGNING"
                else:
                    cam_w     = -KP_ROT * error_x
                    rem_area  = max(0.0, TARGET_AREA - area)
                    # 선형 감속폭 완화 (최소 속도를 MIN_V=0.10 으로 잡아서 답답함 해소)
                    cam_v     = MIN_V + (MAX_V - MIN_V) * (rem_area / TARGET_AREA)
                    cam_v     = np.clip(cam_v, MIN_V, MAX_V)
                    cam_state = "TRACK"

                cv2.putText(frame, f"A:{int(area)}",
                            (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            else:
                cam_state = "SMALL"
                stop_robot()

        else:
            # ★ 개정: 목표 색상이 아예 안 보일 때의 예외 처리 로직 고도화
            if state == "FORCED_SEARCH":
                # 주차 직후 사각지대 탈출을 위한 빠르고 적극적인 제자리 피벗 회전
                cam_v, cam_w = 0.0, 0.55  
                cam_state    = "FORCED_SEARCH"
            else:
                # 일반 탐색 상황
                cam_state = "SEARCH LEFT" if last_seen_x <= frame_cx else "SEARCH RIGHT"
                if last_seen_x > frame_cx:
                    cam_v, cam_w = 0.04, -0.38  # 회전 속도 소폭 상향
                else:
                    cam_v, cam_w = 0.04,  0.38

        # ── LiDAR 우선순위 적용 ────────────────
        final_v, final_w, avoid_on = decide_cmd(cam_v, cam_w, front_min)
        send_cmd(final_v, final_w)

        state = f"AVOID {front_min:.0f}cm" if avoid_on else cam_state

        # ── HUD ────────────────────────────────
        cv2.putText(frame, state,
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame,
                    f"TARGET:{target} ({mission_index+1}/{len(MISSION)})",
                    (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, draw, 2)

        cv2.imshow("frame", frame)
        cv2.imshow("mask",  mask)

        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
    print("SHUTDOWN COMPLETE")
