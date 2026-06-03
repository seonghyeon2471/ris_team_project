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
# CAMERA & HARDWARE FIX (★빛 번짐 및 자동 제어 오작동 해결)
# =========================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

time.sleep(1.0)  # 카메라 안정화 대기
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)        # 1: 수동(Manual), 3: 자동(Auto)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)              # 0: 자동 화이트 밸런스 비활성화

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
# CAMERA PARAMETERS (★초고속 탐색 및 벽 없는 내부 정지 튜닝)
# =========================================
MAX_V       = 0.24      
MIN_V       = 0.10      
KP_ROT      = 0.003     
X_TOL       = 35        
MIN_AREA    = 500
TARGET_AREA = 15000     
PARK_SEC    = 3.0

# [벽 없는 환경 핵심] 객체가 카메라에서 사라진 후 내부 중심까지 파고들 추가 전진 시간 (초)
APPROACH_DRIVE_SEC = 1.0  

# [고속화 핵심] 제자리 탐색은 짧고 굵게! 2.2초 동안 빠르게 돌고 없으면 바로 탈출 주행
SEARCH_TIMEOUT = 2.2    

# =========================================
# 색상별 HSV 범위 (★과노출/빛 바램 대응 채도 하한선 튜닝)
# =========================================
COLOR_CFG = {
    "red": {
        "hsv1": ([0,   45,  50], [15,  255, 255]),
        "hsv2": ([160, 45,  50], [179, 255, 255]),
        "bgr":  ([0, 0, 0], [255, 255, 255]), # BGR 왜곡 간섭 차단
        "draw": (0, 0, 255),
    },
    "yellow": {
        "hsv1": ([15,  40,  50], [42,  255, 255]), 
        "hsv2": None,
        "bgr":  ([0, 0, 0], [255, 255, 255]),
        "draw": (0, 200, 255),
    },
    "blue": {
        "hsv1": ([90,  45,  40], [140, 255, 255]),
        "hsv2": None,
        "bgr":  ([0, 0, 0], [255, 255, 255]),
        "draw": (255, 80, 0),
    },
}

MISSION = ["red", "yellow", "blue"]

# =========================================
# 상태 변수
# =========================================
mission_index     = 0
state             = "SEARCH"
last_seen_x       = 160
park_start        = None
search_start_time = None  
approach_start_time = None # 내부 진입 타이머 변수

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
    kernel = np.ones((5, 5), np.uint8) # 모폴로지 5x5로 확장 (노이즈 억제 향상)

    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    hsv_mask = cv2.inRange(hsv, lo1, hi1)
    if cfg["hsv2"] is not None:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        hsv_mask = cv2.bitwise_or(hsv_mask, cv2.inRange(hsv, lo2, hi2))

    bgr_mask = cv2.inRange(frame,
                           np.array(cfg["bgr"][0]),
                           np.array(cfg["bgr"][1]))

    mask = cv2.bitwise_and(hsv_mask, bgr_mask)
    # OPEN 후 CLOSE 추가하여 내부 진입 시 파편화되는 블롭 결합
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

# =========================================
# 동작 우선순위 (장애물 회피용)
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
# 카메라 버퍼 flush (★고속 반응 핵심)
# =========================================
def flush_camera_buffer(n=8):  
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

            if elapsed >= PARK_SEC:
                print(f"✅ [{target}] 정차 완료!")
                mission_index += 1
                if mission_index >= len(MISSION):
                    print("🏁 미션 전체 완료!")
                else:
                    # [고속화] 주차 해제 즉시 버퍼를 비워 잔상으로 인한 딜레이 컷
                    flush_camera_buffer(n=8) 
                    
                    state = "FORCED_SEARCH" 
                    search_start_time = time.time()  
                    last_seen_x = 160  # 센터 리셋
                    print(f"➡️  다음 목표: [{MISSION[mission_index]}] 초고속 탐색 시작")

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
                if state in ["FORCED_SEARCH", "WANDERING"]:
                    print(f"🎯 [{target}] 객체 발견! 즉시 추적 상태 돌입.")
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

                # [내부정지] 1단계: 목표 크기 도달 시 APPROACH(진입) 상태 기동
                if area > TARGET_AREA and state == "TRACK":
                    state = "APPROACH"
                    approach_start_time = time.time()
                    print(f"📥 [{target}] 객체 내부 진입 시작 (Area:{int(area)})")

                # [내부정지] 2단계: 보면서 진입 주행 컨트롤
                if state == "APPROACH":
                    cam_w = -KP_ROT * error_x * 0.5 
                    cam_v = 0.12  
                    cam_state = "APPROACH"

                    # 벽이 없으므로 화면이 노란색으로 포화(Area > 28000)되면 정지
                    if area > 28000: 
                        stop_robot()
                        state      = "PARKING"
                        park_start = time.time()
                        print(f"🅿️  [{target}] 내부 면적 포화 즉시 정지 (Area:{int(area)})")
                        continue
                else:
                    # 일반 TRACK 모드
                    cam_w     = -KP_ROT * error_x
                    rem_area  = max(0.0, TARGET_AREA - area)
                    cam_v     = MIN_V + (MAX_V - MIN_V) * (rem_area / TARGET_AREA)
                    cam_v     = np.clip(cam_v, MIN_V, MAX_V)
                    cam_state = "TRACK"

                cv2.putText(frame, f"A:{int(area)}",
                            (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            else:
                cam_state = "SMALL"
                stop_robot()

        else:
            # ── [벽 없음/고속화 핵심] 객체가 시야에서 완전히 사라졌을 때 처리 ──
            if state == "APPROACH":
                cv2.putText(frame, "APPROACH (INSIDE BLIND)", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                
                # 내부로 완전히 깔렸으므로 조향 핸들은 고정하고 직진
                cam_v, cam_w = 0.12, 0.0 
                cam_state    = "APPROACH"

                # 시야에서 사라진 시점부터 APPROACH_DRIVE_SEC(1.0초) 동안 전진 후 정확히 중간에서 정지
                if time.time() - approach_start_time > APPROACH_DRIVE_SEC:
                    stop_robot()
                    state      = "PARKING"
                    park_start = time.time()
                    print(f"🅿️  [{target}] 내부 진입 후 시간 조건 정지 완료!")
                    continue

            elif state == "FORCED_SEARCH":
                # [고속화] 2.2초 타이머가 끝날 때까지 엄청 빠르게(w=0.75) 회전 스캔
                if time.time() - search_start_time > SEARCH_TIMEOUT:
                    state = "WANDERING"
                    print(f"⚠️ 컷! [{target}] 안 보임 -> 맵 크게 돌러 출발")
                    cam_v, cam_w = 0.20, 0.0  # 전진 주행 속도 업
                    cam_state    = "WANDERING"
                else:
                    cam_v, cam_w = 0.0, 0.75  
                    cam_state    = "FORCED_SEARCH"
            
            elif state == "WANDERING":
                cam_v, cam_w = 0.20, 0.0  
                cam_state    = "WANDERING"
                
            else:
                # 일반 서칭 턴 속도도 민첩하게 증가
                cam_state = "SEARCH LEFT" if last_seen_x <= frame_cx else "SEARCH RIGHT"
                if last_seen_x > frame_cx:
                    cam_v, cam_w = 0.02, -0.55  
                else:
                    cam_v, cam_w = 0.02,  0.55

        # ── LiDAR 회피 간섭 분기 ──
        # 객체 내부로 정밀 진입(APPROACH) 하거나 멈춘 상태(PARKING)에서는 라이다 회피를 끕니다.
        if state in ["APPROACH", "PARKING"]:
            final_v, final_w, avoid_on = cam_v, cam_w, False
        else:
            final_v, final_w, avoid_on = decide_cmd(cam_v, cam_w, front_min)
            
        send_cmd(final_v, final_w)

        # 디버깅 스크린 출력 상태 명시
        disp_state = f"AVOID {front_min:.0f}cm" if avoid_on else cam_state

        # ── HUD ────────────────────────────────
        cv2.putText(frame, disp_state,
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
