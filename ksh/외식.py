#!/usr/bin/env python3
"""
RP라이다 기반 장애물 회피 - 라즈베리파이 메인 제어 코드
아두이노 통신 프로토콜: "v,w\n" (선속도 m/s, 각속도 rad/s)
"""

import time
import math
import serial
import numpy as np
from rplidar import RPLidar

# ── 포트 설정 ──────────────────────────────────────
LIDAR_PORT   = '/dev/ttyUSB0'   # RP라이다 USB 포트
ARDUINO_PORT = '/dev/ttyAMA0'   # 아두이노 UART (GPIO 14/15)
ARDUINO_BAUD = 115200

# ── 로봇 치수 (m) ──────────────────────────────────
ROBOT_WIDTH      = 0.16
ROBOT_LENGTH     = 0.20
LIDAR_FROM_FRONT = 0.025
LIDAR_OFFSET     = (ROBOT_LENGTH / 2) - LIDAR_FROM_FRONT  # 0.075m
MIN_GAP_WIDTH    = ROBOT_WIDTH + 0.10   # 통과 가능 최소 갭 폭 0.26m

# ── 거리 임계값 (m) ────────────────────────────────
OBSTACLE_THRESHOLD  = 0.55   # 장애물 판정 거리
EMERGENCY_STOP_DIST = 0.22   # 긴급 정지 거리 (로봇 앞면 기준)

# ── 탐색 각도 ──────────────────────────────────────
SCAN_HALF_ANGLE  = 90    # 전방 ±90° 탐색
FRONT_CHECK_ANGLE = 20   # 긴급정지 판단 ±20°

# ── 속도 파라미터 ──────────────────────────────────
MAX_LINEAR  = 0.18   # 최대 선속도 m/s
MAX_ANGULAR = 1.4    # 최대 각속도 rad/s
KP_ANGULAR  = 1.3    # 각도 → 각속도 게인


def normalize_angle(deg):
    deg = deg % 360
    if deg > 180:
        deg -= 360
    return deg


def parse_scan(scan, max_dist_m=6.0):
    """rplidar 스캔 → {각도°: 거리m} 딕셔너리. 0°=정면, +좌/-우"""
    ANGLE_OFFSET = 0   # 라이다 마운팅 오프셋 — 실제 장착 후 조정
    data = {}
    for _, angle_raw, dist_mm in scan:
        if dist_mm == 0:
            continue
        dist_m = dist_mm / 1000.0
        if dist_m > max_dist_m:
            continue
        key = round(normalize_angle(angle_raw + ANGLE_OFFSET), 1)
        if key not in data or dist_m < data[key]:
            data[key] = dist_m
    return data


def front_min_dist(scan_dict):
    """정면 ±FRONT_CHECK_ANGLE° 최소 거리 (라이다 기준)"""
    dists = [d for a, d in scan_dict.items() if abs(a) <= FRONT_CHECK_ANGLE]
    return min(dists) if dists else float('inf')


def find_gaps(scan_dict):
    """전방 스캔에서 로봇 통과 가능한 갭 목록 반환"""
    angles = sorted([a for a in scan_dict if -SCAN_HALF_ANGLE <= a <= SCAN_HALF_ANGLE])
    if not angles:
        return []

    # 각 각도별 자유 공간 판정 (라이다 위치 보정 포함)
    free_flags = {a: (scan_dict[a] - LIDAR_OFFSET > OBSTACLE_THRESHOLD) for a in angles}

    gaps = []
    in_gap = False
    gap_angles, gap_dists = [], []

    def flush_gap():
        if len(gap_angles) < 2:
            return
        span  = abs(gap_angles[-1] - gap_angles[0])
        avg_d = float(np.mean(gap_dists))
        width = 2.0 * avg_d * math.sin(math.radians(span / 2.0))
        if width >= MIN_GAP_WIDTH:
            gaps.append({
                'center_angle': (gap_angles[0] + gap_angles[-1]) / 2.0,
                'angular_span': span,
                'width_m':      width,
                'min_dist_m':   float(min(gap_dists)),
            })

    for a in angles:
        if free_flags[a]:
            if not in_gap:
                in_gap = True
                gap_angles, gap_dists = [a], [scan_dict[a]]
            else:
                gap_angles.append(a)
                gap_dists.append(scan_dict[a])
        else:
            if in_gap:
                flush_gap()
                in_gap = False
                gap_angles, gap_dists = [], []

    if in_gap:
        flush_gap()

    return gaps


def select_best_gap(gaps):
    """정면에 가깝고 넓은 갭 선택"""
    if not gaps:
        return None
    return min(gaps, key=lambda g: abs(g['center_angle']) * 1.0 - g['width_m'] * 8.0)


def compute_cmd(best_gap, min_front_lidar):
    """갭 방향으로 v, w 계산"""
    angle_rad  = math.radians(best_gap['center_angle'])
    dist_ratio = max(min((min_front_lidar - EMERGENCY_STOP_DIST) /
                         (OBSTACLE_THRESHOLD - EMERGENCY_STOP_DIST), 1.0), 0.0)
    angle_ratio = 1.0 - 0.5 * min(abs(angle_rad) / (math.pi / 2), 1.0)
    v = MAX_LINEAR * dist_ratio * angle_ratio
    w = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * angle_rad))
    return round(v, 4), round(w, 4)


def send_cmd(ser, v, w):
    """아두이노에 "v,w\n" 전송"""
    ser.write(f"{v:.3f},{w:.3f}\n".encode('ascii'))


def main():
    print("=" * 50)
    print(" RP라이다 장애물 회피 시작")
    print("=" * 50)

    arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
    time.sleep(2)
    print(f"아두이노 연결 완료: {ARDUINO_PORT}")

    lidar = RPLidar(LIDAR_PORT, baudrate=115200)
    lidar.start_motor()
    time.sleep(1.5)
    print(f"RP라이다 연결 완료: {LIDAR_PORT}\n")
    print(f"최소 갭: {MIN_GAP_WIDTH*100:.0f}cm | 장애물 판정: {OBSTACLE_THRESHOLD*100:.0f}cm")
    print("-" * 50)

    prev_v, prev_w = 0.0, 0.0

    try:
        for scan in lidar.iter_scans(max_buf_meas=500):
            scan_dict = parse_scan(scan)
            if not scan_dict:
                continue

            min_front_lidar = front_min_dist(scan_dict)
            min_front_robot = min_front_lidar - LIDAR_OFFSET

            # 긴급 정지
            if min_front_robot < EMERGENCY_STOP_DIST:
                if prev_v != 0.0 or prev_w != 0.0:
                    send_cmd(arduino, 0.0, 0.0)
                    prev_v, prev_w = 0.0, 0.0
                print(f"[긴급정지] 앞면 {min_front_robot*100:.1f}cm")
                continue

            # 갭 탐색 및 주행
            best = select_best_gap(find_gaps(scan_dict))

            if best is None:
                v, w = 0.0, MAX_ANGULAR * 0.55
                send_cmd(arduino, v, w)
                print(f"[탐색회전] 갭 없음 | w={w:.2f}")
            else:
                v, w = compute_cmd(best, min_front_lidar)
                send_cmd(arduino, v, w)
                print(f"전방: {min_front_robot*100:5.1f}cm | "
                      f"갭중심: {best['center_angle']:+6.1f}° | "
                      f"갭폭: {best['width_m']*100:5.1f}cm | "
                      f"v={v:+.3f}  w={w:+.4f}")

            prev_v, prev_w = v, w

    except KeyboardInterrupt:
        print("\n종료 중...")
    except Exception as e:
        print(f"\n오류: {e}")
    finally:
        send_cmd(arduino, 0.0, 0.0)
        time.sleep(0.3)
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        arduino.close()
        print("종료 완료")


if __name__ == '__main__':
    main()
