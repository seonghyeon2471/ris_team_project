import cv2
import serial
import numpy as np
import time
import math
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
EMA_ALPHA = 0.35
MEDIAN_K  = 2

_scan_buf    = np.full(360, 150.0, dtype=np.float32)
_scan_shared = np.full(360, 150.0, dtype=np.float32)
scan_lock    = threading.Lock()

# =========================================
# 카메라 기하학 파라미터
# =========================================
CAM_H        = 73.0   
CAM_PITCH    = 30.0   
CAM_FOV_V    = 50.0   
RES_W, RES_H = 320, 240
CY           = RES_H / 2   
FY           = (RES_H / 2) / math.tan(math.radians(CAM_FOV_V / 2))  

def pixel_to_ground_dist(y_px):
    delta_deg  = math.degrees(math.atan2(y_px - CY, FY))
    total_deg  = CAM_PITCH + delta_deg
    if total_deg <= 0:
        return float('inf')
    return CAM_H / math.tan(math.radians(total_deg))

# =========================================
# 제어 및 주행 파라미터
# =========================================
MAX_V       = 0.24
MIN_V       = 0.10
KP_ROT      = 0.003
MIN_AREA    = 900        
TARGET_AREA = 6000       

APPROACH_V           = 0.24  
BLIND_V              = 0.29  

APPROACH_DRIVE_SEC   = 1.2   
APPROACH_MAX_TIMEOUT = 4.5   
SEARCH_TIMEOUT       = 1.6   

PARK_SEC        = 3.0     

# 바운더리 파라미터
BOUNDARY_PHASE1_SEC = 1.0
BOUNDARY_PHASE2_SEC = 1.8
BOUNDARY_PHASE3_SEC = 0.8
BOUNDARY_W          = 0.55
BOUNDARY_BACK_V     = -0.10

# =========================================
# HSV 범위
# =========================================
COLOR_CFG = {
    "red": {
        "hsv1": ([0,   80,  70], [10,  255, 255]),
        "hsv2": ([170, 80,  70], [179, 255, 255]),
        "bgr":  ([0, 0, 0], [255, 255, 255]),
        "draw": (0, 0, 255),
    },
    "yellow": {
        "hsv1": ([15,  30,  60], [40,  255, 255]),
        "hsv2": ([10,  15,  190], [45,  45,  255]),
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
mission_index             = 0
state                     = "SEARCH"
last_seen_x               = 160
last_seen_y               = 120    
park_start                = None
search_start_time         = None
approach_start_time       = None
blind_dash_start_time     = None  

last_dist_cm         = 150.0  
boundary_phase       = 0
boundary_phase_start = None
boundary_direction   = 1

morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

# 🌟 출력을 제어하기 위한 프레임 카운터 추가
frame_count = 0

# =========================================
# LIDAR 쓰레드 함수들
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
        if ((raw[0] & 0x02) >> 1) != (1 - s_flag) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
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
# MOTOR COMM
# =========================================
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# IMAGE MASK
# =========================================
def make_mask(frame, hsv, color_name):
    cfg = COLOR_CFG[color_name]
    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    hsv_mask = cv2.inRange(hsv, lo1, hi1)
    if cfg["hsv2"] is not None:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        hsv_mask = cv2.bitwise_or(hsv_mask, cv2.inRange(hsv, lo2, hi2))
    bgr_mask = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    
    mask = cv2.bitwise_and(hsv_mask, bgr_mask)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, morph_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morph_kernel)
    return mask

def flush_camera_buffer(n=8):
    for _ in range(n):
        cap.grab()

# =========================================
# BOUNDARY SEARCH
# =========================================
def start_boundary_search(lost_x, frame_cx):
    global boundary_phase, boundary_phase_start, boundary_direction, state
    boundary_direction = 1 if lost_x > frame_cx else -1
    boundary_phase = 1
    boundary_phase_start = time.time()
    state = "BOUNDARY"
    print(f"🔍 바운더리 탐색 시작 | 방향={'우' if boundary_direction > 0 else '좌'}")

def run_boundary_search():
    global boundary_phase, boundary_phase_start, state, search_start_time
    elapsed = time.time() - boundary_phase_start

    if boundary_phase == 1:
        if elapsed < BOUNDARY_PHASE1_SEC: return 0.03, BOUNDARY_W * boundary_direction
        boundary_phase = 2; boundary_phase_start = time.time(); elapsed = 0.0
    if boundary_phase == 2:
        if elapsed < BOUNDARY_PHASE2_SEC: return 0.03, BOUNDARY_W * -boundary_direction
        boundary_phase = 3; boundary_phase_start = time.time(); elapsed = 0.0
    if boundary_phase == 3:
        if elapsed < BOUNDARY_PHASE3_SEC: return BOUNDARY_BACK_V, 0.0
        boundary_phase = 0; state = "SEARCH"; search_start_time = time.time()
    return 0.0, 0.0

# =========================================
# MAIN LOOP
# =========================================
print("🏁 MISSION CONTROL v7.3 Active (Auto Data Telemetry Engaged)")

try:
    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame_count += 1
        frame = cv2.flip(frame, 1)
        HEIGHT, WIDTH = frame.shape[:2]
        frame_cx = WIDTH // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if mission_index >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSION COMPLETE!", (20, HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == 27: break
            continue

        target = MISSION[mission_index]
        draw   = COLOR_CFG[target]["draw"]
        mask   = make_mask(frame, hsv, target)

        contours_full, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        with scan_lock:
            front_angles = list(range(345, 360)) + list(range(0, 15))
            front_dists = [_scan_shared[a] for a in front_angles if 3.0 < _scan_shared[a] < 150.0]
            min_front_lidar = min(front_dists) if front
