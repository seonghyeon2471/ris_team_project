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
# LIDAR PARAMETERS (백그라운드 수신 유지용)
# =========================================
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
MIN_AREA    = 900       # ★ 모폴로지 제거로 인한 잔돌 노이즈 필터링을 위해 면적 하한선 상향
TARGET_AREA = 13000     
PARK_SEC    = 3.0       

APPROACH_DRIVE_SEC = 1.2  
SEARCH_TIMEOUT = 2.2    
APPROACH_MAX_TIMEOUT = 2.0   

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
# LIDAR UTIL (수신 유지용 스레드)
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
    cfg = COLOR_CFG[color_name]

    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    hsv_mask = cv2.inRange(hsv, lo1, hi1)
    
    if cfg["hsv2"] is not None:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        hsv_mask = cv2.bitwise_or(hsv_mask, cv2.inRange(hsv, lo2, hi2))

    bgr_mask = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    mask = cv2.bitwise_and(hsv_mask, bgr_mask)
    
    # ★ [변경]: 기존의 MORPH_OPEN / MORPH_CLOSE 연산을 완전히 들어내어 딜레이 최소화
    return mask

def flush_camera_buffer(n=8):  
    for _ in range(n):
        cap.grab()

# =========================================
# MAIN LOOP
# =========================================
print("🏁 MISSION CONTROL START (MORPHOLOGY REMOVED)")

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
                print(f"✅ [{target}] 미션 타임아웃 완료 -> 넥스트 미션")
                mission_index += 1
                
                if mission_index < len(MISSION):
                    flush_camera_buffer(n=15)         
                    state = "FORCED_SEARCH" 
                    search_start_time = time.time()  
                    last_seen_x = 160                 
                    print(f"➡️  NEXT TARGET: [{MISSION[mission_index]}] 탐색 개시")
                else:
                    print("🏁 모든 미션 클리어")
            
            if cv2.waitKey(1) & 0xFF == 27: break
            continue  

        # ── [APPROACH 내부 강제 타임아웃 안전장치] ──
        if state == "APPROACH" and approach_start_time is not None:
            if time.time() - approach_start_time > APPROACH_MAX_TIMEOUT:
                print(f"🚨 [안전장치 트리거] APPROACH 강제 {APPROACH_MAX_TIMEOUT}초 만료 -> PARKING!")
                state      = "PARKING"
                park_start = time.time()
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

            if area > MIN_AREA:
                if state in ["FORCED_SEARCH", "WANDERING", "SEARCH"]:
                    state = "TRACK"
                    print(f"🎯 [{target}] 포착 -> TRACK 모드")

                rect = cv2.minAreaRect(c)
                (cx, cy), _, _ = rect
                cx, cy = int(cx), int(cy)
                last_seen_x = cx

                cv2.drawContours(frame, [np.int32(cv2.boxPoints(rect))], 0, draw, 2)
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                error_x = cx - frame_cx

                # 돌격 진입 조건문
                if state == "APPROACH" or area > TARGET_AREA:
                    if state != "APPROACH":
                        state = "APPROACH"
                        approach_start_time = time.time() 
                        print(f"📥 [{target}] 강제 돌격 (Area 만족: {int(area)})")

                    cam_w = -KP_ROT * error_x * 0.5 
                    cam_v = 0.12  
                    cam_state = "APPROACH"

                    if area > 24000: 
                        state      = "PARKING"
                        park_start = time.time()
                        print(f"🅿️  [{target}] 내부 포화 정상 정지 완료")
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

                if time.time() - approach_start_time > APPROACH_DRIVE_SEC:
                    state      = "PARKING"
                    park_start = time.time()
                    print(f"🅿️  [{target}] 사각지대 시간 완료 -> 주차 가동")

            elif state == "FORCED_SEARCH":
                if time.time() - search_start_time > SEARCH_TIMEOUT:
                    state = "WANDERING"
                    print(f"⚠️ 타임아웃 -> WANDERING")
                    cam_v, cam_w = 0.20, 0.0  
                else:
                    cam_v, cam_w = 0.03, 0.80  
            
            elif state == "WANDERING":
                cam_v, cam_w = 0.20, 0.0  
                
            else:
                cam_state = "SEARCH LEFT" if last_seen_x <= frame_cx else "SEARCH RIGHT"
                cam_v, cam_w = (0.03, -0.65) if last_seen_x > frame_cx else (0.03,  0.65)

        # 라이다 간섭 필터가 영구 차단된 모터 전달부
        send_cmd(cam_v, cam_w)

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
