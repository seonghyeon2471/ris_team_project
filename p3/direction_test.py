#!/usr/bin/env python3
"""
lidar_direction_check.py
─────────────────────────────────────────────────────────────────────
메인 코드가 가정하는 라이다 각도 규칙이 실제 하드웨어와 맞는지
점검하기 위한 스크립트.

가정하는 규칙 (메인 코드 전체에서 사용 중인 규칙):
    0°   = 전방 (Front)
    90°  = 왼쪽 (Left)
    180° = 후방 (Back)
    270° = 오른쪽 (Right)
  → 각도는 반시계 방향으로 증가

사용법:
  1. 라이다만 연결한 상태로 이 스크립트를 실행 (모터/카메라 불필요)
  2. 로봇 코앞(정면)에 손이나 물체를 가까이 대본다
     → 콘솔의 "전방(F)" 값이 작아지고, 레이더 창에서 점이 위쪽(F)에 찍혀야 함
  3. 로봇의 왼쪽에 물체를 대본다
     → "왼쪽(L)" 값이 작아지고, 레이더 창 왼쪽(L)에 점이 찍혀야 함
  4. 로봇의 오른쪽에 물체를 대본다
     → "오른쪽(R)" 값이 작아지고, 레이더 창 오른쪽(R)에 점이 찍혀야 함
  5. 콘솔에 찍히는 "가장 가까운 지점" 각도/방향이 실제로 물체를 댄 방향과
     일치하는지 확인. 어긋나면 메인 코드의 각도 매핑(0/90/180/270 가정)을
     실제 라이다 장착 방향에 맞게 보정해야 한다.

ESC 키로 종료.
"""

import serial
import numpy as np
import time
import threading
import cv2

# ── SERIAL (라이다만) ───────────────────────────────────────────────
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# ── LIDAR BOOT (메인 코드와 동일) ─────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR 파싱 (메인 코드와 동일) ─────────────────────────────────────
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
        if len(raw) != 5: continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150: _ema(angle, dist_cm)
        if sf == 1:
            _median()
            with scan_lock: _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock: return _scan_pub.copy()

# ── 방향별 최소거리 (±5° 밴드, 노이즈 완화) ───────────────────────────
BAND = 5

def band_min(scan, center_deg):
    idx = np.arange(center_deg - BAND, center_deg + BAND + 1) % 360
    return float(np.min(scan[idx]))

DIRS = [("전방(F)", 0), ("왼쪽(L)", 90), ("후방(B)", 180), ("오른쪽(R)", 270)]

# ── 레이더 시각화 ──────────────────────────────────────────────────
SIZE   = 520
CX, CY = SIZE // 2, SIZE // 2
MAX_R  = SIZE // 2 - 40
MAX_D  = 150.0
SCALE  = MAX_R / MAX_D

def draw_radar(scan, closest_deg, closest_d):
    img = np.zeros((SIZE, SIZE, 3), dtype=np.uint8)

    # 거리 동심원 (50cm 단위)
    for r_cm in (50, 100, 150):
        cv2.circle(img, (CX, CY), int(r_cm * SCALE), (50, 50, 50), 1)

    # 컴퍼스 십자선
    cv2.line(img, (CX, CY - MAX_R), (CX, CY + MAX_R), (40, 40, 40), 1)
    cv2.line(img, (CX - MAX_R, CY), (CX + MAX_R, CY), (40, 40, 40), 1)

    # 라벨: F=위, L=왼쪽, R=오른쪽, B=아래  (코드가 가정하는 규칙대로 배치)
    cv2.putText(img, "F(0)",   (CX - 20, CY - MAX_R - 12), 0, 0.55, (0, 255, 255), 2)
    cv2.putText(img, "L(90)",  (CX - MAX_R - 60, CY + 5),  0, 0.55, (0, 255, 255), 2)
    cv2.putText(img, "R(270)", (CX + MAX_R + 8, CY + 5),   0, 0.55, (0, 255, 255), 2)
    cv2.putText(img, "B(180)", (CX - 25, CY + MAX_R + 28), 0, 0.55, (0, 255, 255), 2)

    # 로봇 자신 표시 (중심점, 코는 위쪽=전방)
    cv2.circle(img, (CX, CY), 5, (255, 255, 255), -1)
    cv2.line(img, (CX, CY), (CX, CY - 18), (255, 255, 255), 2)  # 전방 화살표

    # 360° 전체 스캔 점 찍기
    # 로봇 좌표계: 전방=+X(=cos), 왼쪽=+Y(=sin) → 화면: 전방=위, 왼쪽=좌
    for deg in range(360):
        d = float(scan[deg])
        if d <= 0 or d >= MAX_D - 1: continue
        theta = np.deg2rad(deg)
        x_robot = d * np.cos(theta)   # 전방 성분
        y_robot = d * np.sin(theta)   # 왼쪽 성분
        px = int(CX - y_robot * SCALE)
        py = int(CY - x_robot * SCALE)
        cv2.circle(img, (px, py), 2, (0, 255, 0), -1)

    # 가장 가까운 지점 강조 (빨간 점 + 각도 표시)
    if closest_d < MAX_D - 1:
        theta = np.deg2rad(closest_deg)
        x_robot = closest_d * np.cos(theta)
        y_robot = closest_d * np.sin(theta)
        px = int(CX - y_robot * SCALE)
        py = int(CY - x_robot * SCALE)
        cv2.circle(img, (px, py), 7, (0, 0, 255), 2)
        cv2.putText(img, f"{closest_deg}deg / {closest_d:.0f}cm",
                    (px + 10, py), 0, 0.45, (0, 0, 255), 1)

    return img

print("정면 → 왼쪽 → 오른쪽 순서로 물체를 가까이 대보세요. (ESC로 종료)")
last_print = 0.0

try:
    while True:
        scan = get_scan()

        # 가장 가까운 지점(전체 360°) 찾기 — 어느 쪽이 실제로 가장 가까운지 확인용
        closest_deg = int(np.argmin(scan))
        closest_d   = float(scan[closest_deg])

        now = time.time()
        if now - last_print > 0.4:
            last_print = now
            readings = " | ".join(
                f"{name}={band_min(scan, deg):5.1f}cm" for name, deg in DIRS
            )
            print(f"{readings}   ||  최근접: {closest_deg:3d}° / {closest_d:5.1f}cm")

        img = draw_radar(scan, closest_deg, closest_d)
        cv2.imshow("LiDAR Direction Check", img)
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("STOP")
finally:
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
