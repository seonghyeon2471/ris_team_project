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
PARK_SEC    = 2.0       # ★ 주차 정지 대기 시간 단축 (3.0 -> 2.0초)

APPROACH_DRIVE_SEC   = 1.5   # 0.2초 여유 추가
SEARCH_TIMEOUT       = 2.2    
APPROACH_MAX_TIMEOUT = 4.0   # ★ 2.0 → 4.0 (핵심 수정)
APPROACH_V           = 0.22  # ★ 신규 — APPROACH 전용 속도 상수
PARK_AREA            = 18000 # ★ 신규 — PARKING 진입 면적 기준 (24000 → 18000)

# =========================================
# 색상별 HSV 범위
# =========================================
COLOR_CFG = {
    "red": {
        "hsv1": ([0,   45,  50], [15,  255, 255]),
        "hsv2": ([160, 45,  50], [179, 255, 255]),
        "
