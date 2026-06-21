"""
좌/우(L/R) 각도 매핑이 실제 방향과 맞는지 확인하는 테스트 스크립트.

사용법:
1. 로봇(또는 라이다)을 제자리에 고정.
2. 로봇 기준 '왼쪽'에만 장애물(벽, 박스 등)을 30~50cm 정도 거리에 둔다. (오른쪽은 비워둠)
3. 이 스크립트 실행 후 출력되는 L_raw / R_raw 값을 확인.
   - 왼쪽에 장애물을 뒀는데 L_raw 값이 작게(가깝게) 나오면 -> 매핑 정상
   - 왼쪽에 장애물을 뒀는데 R_raw 값이 작게(가깝게) 나오면 -> 매핑 반대 (코드에서 L/R 각도 구간을 swap해야 함)
4. Ctrl+C로 종료.
"""

import serial
import numpy as np
import time
import threading

lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

EMA_ALPHA = 0.35
MEDIAN_K  = 2

_scan     = np.full(360, 150.0, dtype=np.float32)
_scan_pub = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

def _ema(a, d):
    if d > 0:
        _scan[a] = (1 - EMA_ALPHA) * _scan[a] + EMA_ALPHA * d

def _median():
    k = MEDIAN_K
    buf = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
        buf[i] = np.sort(_scan[idx])[k]
    _scan[:] = buf

def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150:
            _ema(angle, dist_cm)
        if sf == 1:
            _median()
            with scan_lock:
                _scan_pub[:] = _scan

def get_scan():
    with scan_lock:
        return _scan_pub.copy()

# 라이다 부팅
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK\n")

threading.Thread(target=lidar_loop, daemon=True).start()
time.sleep(1.0)  # 스캔 데이터 누적 대기

# 원본 코드와 동일한 L/R 각도 구간 (수정 전 기준)
def side_min_raw(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

print("왼쪽에 장애물을 두고 30~50cm 거리에서 테스트하세요. (Ctrl+C로 종료)\n")

try:
    while True:
        scan = get_scan()
        # 원본 코드 기준: "L" = 65~95도, "R" = 265~295도
        L_raw = side_min_raw(scan, 65, 96)
        R_raw = side_min_raw(scan, 265, 296)
        front = float(np.min(scan[np.concatenate([np.arange(0, 91), np.arange(270, 360)])]))

        print(f"FRONT: {front:6.1f} cm | L_raw(65~95deg): {L_raw:6.1f} cm | R_raw(265~295deg): {R_raw:6.1f} cm")
        time.sleep(0.3)

except KeyboardInterrupt:
    print("\n종료")
finally:
    lidar_ser.write(bytes([0xA5, 0x25]))
