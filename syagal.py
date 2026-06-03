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
# CAMERA & HARDWARE FIX
# =========================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

time.sleep(1.0)  
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)        
cap.set(cv2.CAP_PROP_AUTO_WB, 0)              

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
# CAMERA PARAMETERS
# =========================================
MAX_V       = 0.24      
MIN_V       = 0.10      
KP_ROT      = 0.003     
X_TOL       = 35        
TARGET_AREA = 14000     
PARK_SEC    = 3.0       

APPROACH_DRIVE_SEC = 1.2  

# ── ★ [탐색 반경 확장 및 초정밀 포착 튜닝 변수] ──
TRACK_MIN_AREA = 400         # 정식 추적(TRACK) 진입 면적
GLIMPSE_MIN_AREA = 80        # ★ 스치듯 인식되어도 포착할 최소 픽셀 면적 (초민감 세팅)
SEARCH_TIMEOUT = 3.5         # 제자리 고속 스캔 시간 확장 (3.5초)
HIGH_TURN_W = 0.85           # 제자리 회전 속도 극대화

# 맵 순항 탐색(WANDERING) 선회 반경 극대화 세팅
WANDERING_V = 0.22           # 직진 속도를 키우고
WANDERING_W = 0.25           # 회전 속도를 줄여서 원을 매우 크게 그리며 전진하도록 수정

# =========================================
# 색상별 HSV 범위
# =========================================
COLOR_CFG = {
    "red": {
        "hsv1": ([0,   45,  50], [15,  255, 255]),
        "hsv2": ([160, 45,  50], [179, 255, 255]),
        "bgr":  ([0, 0, 0], [255, 255, 255]), 
        "draw": (0, 0, 255),
    },
    "yellow": {
        "hsv1": ([15,  30,  60], [40,  255, 255]), 
        "hsv2": ([10,  0,   190], [45,  45,  255]), 
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
mission_index       = 0
state               = "SEARCH"
last_seen_x         = 160
park_start          = None
search_start_time   = None  
approach_start_time = None  

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

# =========================================
# MOTOR WHEEL CONTROL
# =========================================
def send_cmd(v, w):
    v = np.clip(v, -0.3, 0.3)
    w = np.clip(w, -0.8, 0.8)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

def make_mask(frame, hsv, color_name):
    cfg    = COLOR_CFG[color_name]
    kernel = np.ones((5, 5), np.uint8) 

    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    hsv_mask = cv2.inRange(hsv, lo1, hi1)
    
    if cfg["hsv2"] is not None:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        hsv_mask = cv2.bitwise_or(hsv_mask, cv2.inRange(hsv, lo2, hi2))

    bgr_mask = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    mask = cv2.bitwise_and(hsv_mask, bgr_mask)
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def decide_cmd(cam_v, cam_w, front_min):
    if front_min < THRESH_10:
        direction = choose_avoid_direction()
        return MIN_SPEED, direction * MAX_W, True
    elif front_min < THRESH_20:
        direction = choose_avoid_direction()
        avoid_w   = direction * 0.8
        blended_w = 0.3 * cam_w + 0.7 * avoid_w if direction * cam_w >= 0 else avoid_w
        return 0.12, blended_w, True
    elif front_min < THRESH_30:
        direction = choose_avoid_direction()
        avoid_w   = direction * 0.7
        blended_w = 0.5 * cam_w + 0.5 * avoid_w if direction * cam_w >= 0 else avoid_w
        return min(cam_v, 0.15), blended_w, True
    else:
        return cam_v, cam_w, False

def flush_camera_buffer(n=8):  
    for _ in range(n):
        cap.grab()

# =========================================
# MAIN LOOP
# =========================================
print("🏁 MISSION CONTROL START")

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

        if mission_index >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSION COMPLETE!", (20, HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == 27: break
            continue

        target = MISSION[mission_index]
        draw   = COLOR_CFG[target]["draw"]

        # ── [PARKING 격리 레이어] ──────────────────
        if state == "PARKING":
            send_cmd(0.0, 0.0) 
            
            elapsed = time.time() - park_start
            remain   = max(0.0, PARK_SEC - elapsed)
            
            cv2.putText(frame, f"STATE: PARKING", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"WAIT [{target}]: {remain:.1f}s", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
            cv2.imshow("frame", frame)
            
            if elapsed >= PARK_SEC:
                print(f"✅ [{target}] 미션 시간 종료 -> 타겟 인덱스 상승")
                mission_index += 1
                
                if mission_index < len(MISSION):
                    flush_camera_buffer(n=15)         
                    state = "FORCED_SEARCH" 
                    search_start_time = time.time()  
                    print(f"➡️  NEXT TARGET: [{MISSION[mission_index]}] 탐색 시퀀스 개시")
                else:
                    print("🏁 모든 미션 클리어")
            
            if cv2.waitKey(1) & 0xFF == 27: break
            continue  

        # 주행 연산 레이어
        mask = make_mask(frame, hsv, target)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cam_v, cam_w = 0.0, 0.0
        cam_state = state

        # ── [객체 인지 구역] ──────────────────────
        if contours:
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            # ★ [핵심 기능 1]: 탐색 중에 목표 색상이 아주 조금이라도(GLIMPSE_MIN_AREA 이상) 잡힌 경우
            if area > GLIMPSE_MIN_AREA:
                rect = cv2.minAreaRect(c)
                (cx, cy), _, _ = rect
                cx, cy = int(cx), int(cy)
                last_seen_x = cx

                cv2.drawContours(frame, [np.int32(cv2.boxPoints(rect))], 0, draw, 2)
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                error_x = cx - frame_cx

                # 아직 확실하게 정중앙 조준이 안 되었거나 면적이 작은 상태라면
                if state in ["FORCED_SEARCH", "WANDERING", "SEARCH"] or area < TRACK_MIN_AREA:
                    # 일단 직진 속도를 0으로 하여 그 자리에 스냅 정지(SNAP STOP)
                    cam_v = 0.0 
                    # 목표물이 있는 방향으로 미세 조향 회전축 가동 (중앙정렬 유도)
                    cam_w = -0.45 if error_x > 0 else 0.45
                    cam_state = "SNAP STOP & ALIGN"
                    
                    # 만약 화면 오차가 좁혀지고 면적도 충분히 커졌다면 정식 추적(TRACK)으로 완전 전환
                    if abs(error_x) < X_TOL and area >= TRACK_MIN_AREA:
                        state = "TRACK"
                        print(f"🎯 [{target}] 정밀 조준 완료 -> TRACK 모드 전환")
                
                # 정식 추적 모드 및 접근 모드 구역
                else:
                    if state == "APPROACH" or (area > TARGET_AREA and state == "TRACK"):
                        if state != "APPROACH":
                            state = "APPROACH"
                            approach_start_time = time.time()
                            print(f"📥 [{target}] 내부 진입 (Area 크기 만족: {int(area)})")

                        cam_w = -KP_ROT * error_x * 0.5 
                        cam_v = 0.12  
                        cam_state = "APPROACH"

                        if area > 25000: 
                            state      = "PARKING"
                            park_start = time.time()
                            print(f"🅿️  [{target}] 내부 면적 포화 정지")
                    else:
                        cam_w     = -KP_ROT * error_x
                        rem_area  = max(0.0, TARGET_AREA - area)
                        cam_v     = MIN_V + (MAX_V - MIN_V) * (rem_area / TARGET_AREA)
                        cam_v     = np.clip(cam_v, MIN_V, MAX_V)
                        cam_state = "TRACK"

                cv2.putText(frame, f"Area: {int(area)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                # 노이즈 수준의 극소 면적은 탐색 제어 유지
                if state in ["FORCED_SEARCH", "WANDERING"]: pass
                else: cam_v, cam_w = 0.0, 0.0
        
        # ── [객체 유실 구역 - 광대역 선회 탐색] ──
        else:
            if state == "APPROACH":
                cam_v, cam_w = 0.12, 0.0  
                cam_state    = "APPROACH_BLIND"
                if time.time() - approach_start_time > APPROACH_DRIVE_SEC:
                    state      = "PARKING"
                    park_start = time.time()
                    print(f"🅿️  [{target}] 사각지대 시간 완료 -> 주차 가동")

            # 1단계: 미션 전환 직후 제자리 초고속 회전 스캔 (3.5초간)
            elif state == "FORCED_SEARCH":
                elapsed_search = time.time() - search_start_time
                if elapsed_search > SEARCH_TIMEOUT:
                    state = "WANDERING"
                    print(f"⚠️ 1단계 실패 -> 2단계: 광대역 순항 탐색(WANDERING) 돌입")
                else:
                    search_dir = -HIGH_TURN_W if last_seen_x > frame_cx else HIGH_TURN_W
                    cam_v, cam_w = 0.03, search_dir  
                    cam_state    = f"F_SEARCH ({SEARCH_TIMEOUT - elapsed_search:.1f}s)"
            
            # 2단계: 선회 반경을 대폭 넓혀 운동장 크게 돌듯이 맵 순항
            elif state == "WANDERING":
                # 높은 전진 속도(WANDERING_V)와 부드러운 회전(WANDERING_W)으로 큰 타원을 그리며 사각지대 제거
                # 벽을 만나면 하단 라이다가 튕겨내 주어 자동 순항맵 청소기 모드 작동
                cruise_dir = -WANDERING_W if last_seen_x > frame_cx else WANDERING_W
                cam_v = WANDERING_V
                cam_w = cruise_dir
                cam_state = "WANDERING (WIDE CRUISE)"
                
            else:
                cam_state = "SEARCH LEFT" if last_seen_x <= frame_cx else "SEARCH RIGHT"
                cam_v, cam_w = (0.03, -HIGH_TURN_W) if last_seen_x > frame_cx else (0.03, HIGH_TURN_W)

        # ── [라이다 제어 분기 및 명령 최종 전달] ──────────────────
        if state in ["APPROACH", "APPROACH_BLIND", "PARKING"]:
            final_v, final_w = cam_v, cam_w
        else:
            # WANDERING 상태로 크게 돌다가 벽을 만나면 라이다가 안전하게 튕겨내 선회를 유지해 줌
            final_v, final_w, _ = decide_cmd(cam_v, cam_w, front_min)
            
        send_cmd(final_v, final_w)

        # 디스플레이 업데이트
        cv2.putText(frame, f"STATE: {cam_state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"TARGET: {target}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
        
        cv2.imshow("frame", frame)
        cv2.imshow("mask",  mask)

        if cv2.waitKey(1) & 0xFF == 27: 
            break

except KeyboardInterrupt:
    print("USER INTERRUPT")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
    print("SYSTEM SHUTDOWN")
