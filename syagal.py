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
MIN_AREA    = 400       
TARGET_AREA = 14000     
PARK_SEC    = 3.0       

APPROACH_DRIVE_SEC = 1.2  
SEARCH_TIMEOUT = 2.2    

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
        "hsv2": ([10,  0,   190], [45,  45,  255]), # 화이트아웃 구원 필터 유지
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

        # 전체 미션 완료 핸들러
        if mission_index >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSION COMPLETE!", (20, HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == 27: break
            continue

        target = MISSION[mission_index]
        draw   = COLOR_CFG[target]["draw"]

        # ── [★ 핵심 수정: PARKING 최상단 격리 레이어] ──────────────────
        # 인식 결과나 유실 여부와 무관하게 주차 상태면 무조건 여기서 카운트다운하고 루프를 탈출(continue)시킴
        if state == "PARKING":
            send_cmd(0.0, 0.0) # 강제 정지 신호 지속 송신
            
            elapsed = time.time() - park_start
            remain   = max(0.0, PARK_SEC - elapsed)
            
            # 주차 화면 강제 렌더링
            cv2.putText(frame, f"STATE: PARKING", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"WAIT [{target}]: {remain:.1f}s", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
            cv2.imshow("frame", frame)
            
            if elapsed >= PARK_SEC:
                print(f"✅ [{target}] 주차 시간 완료! 다음 타겟 전환.")
                mission_index += 1
                
                if mission_index < len(MISSION):
                    flush_camera_buffer(n=15)         # 이전 타겟 잔상 파괴
                    state = "FORCED_SEARCH" 
                    search_start_time = time.time()  
                    last_seen_x = 160                 
                    print(f"➡️  NEXT TARGET: [{MISSION[mission_index]}] 탐색 개시")
                else:
                    print("🏁 모든 미션 클리어")
            
            if cv2.waitKey(1) & 0xFF == 27: break
            continue  # ★ 중요: 주차 중일 때는 하단의 이미지 처리/주행 코드를 완전히 무시하고 다음 프레임으로 통과

        # ── 일반 주행 시퀀스 (PARKING이 아닐 때만 도달함) ──
        mask = make_mask(frame, hsv, target)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cam_v, cam_w = 0.0, 0.0
        cam_state = state

        # ── [객체 인지 구역] ──────────────────────
        if contours:
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                if state in ["FORCED_SEARCH", "WANDERING", "SEARCH"]:
                    state = "TRACK"
                    print(f"🎯 [{target}] 포착 -> TRACK 모드 체인지")

                rect = cv2.minAreaRect(c)
                (cx, cy), _, _ = rect
                cx, cy = int(cx), int(cy)
                last_seen_x = cx

                cv2.drawContours(frame, [np.int32(cv2.boxPoints(rect))], 0, draw, 2)
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                error_x = cx - frame_cx

                # APPROACH 진입 판정
                if state == "APPROACH" or (area > TARGET_AREA and state == "TRACK"):
                    if state != "APPROACH":
                        state = "APPROACH"
                        approach_start_time = time.time()
                        print(f"📥 [{target}] 내부 진입 (Area 크기 만족: {int(area)})")

                    cam_w = -KP_ROT * error_x * 0.5 
                    cam_v = 0.12  
                    cam_state = "APPROACH"

                    # 면적 극대화로 정지하는 경우
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
                if state in ["FORCED_SEARCH", "WANDERING"]: pass
                else: cam_v, cam_w = 0.0, 0.0
        
        # ── [객체 유실 구역] ──────────────────────
        else:
            if state == "APPROACH":
                cam_v, cam_w = 0.12, 0.0  
                cam_state    = "APPROACH_BLIND"

                # 아래로 사라진 뒤 직진 타임아웃 도달 시 주차 전환
                if time.time() - approach_start_time > APPROACH_DRIVE_SEC:
                    state      = "PARKING"
                    park_start = time.time()
                    print(f"🅿️  [{target}] 사각지대 시간 완료 -> 주차 가동")

            elif state == "FORCED_SEARCH":
                if time.time() - search_start_time > SEARCH_TIMEOUT:
                    state = "WANDERING"
                    print(f"⚠️ 타임아웃 -> 맵 전체 탐색(WANDERING) 기동")
                    cam_v, cam_w = 0.20, 0.0  
                else:
                    cam_v, cam_w = 0.0, 0.75  
            
            elif state == "WANDERING":
                cam_v, cam_w = 0.20, 0.0  
                
            else:
                cam_state = "SEARCH LEFT" if last_seen_x <= frame_cx else "SEARCH RIGHT"
                cam_v, cam_w = (0.02, -0.55) if last_seen_x > frame_cx else (0.02,  0.55)

        # ── [라이다 제어 분기 및 명령 최종 전달] ──────────────────
        if state in ["APPROACH", "APPROACH_BLIND", "PARKING"]:
            final_v, final_w = cam_v, cam_w
        else:
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
