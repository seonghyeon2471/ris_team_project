"""
RPLIDAR C1 + Follow the Gap (FTG) 장애물 회피
=============================================
수정 사항:
  1. RPLIDAR C1 표준 5바이트 패킷 파싱 구현 (동기화 비트 검증 포함)
  2. 한 바퀴 완료 후에만 FTG 계산 실행 (타이밍 수정)
  3. 전방 오프셋 설정 (라이다 장착 방향 보정)
  4. 음수 각도 인덱싱 버그 수정
  5. 속도 공식 수정 (FRONT_RANGE 기준으로 정규화)
  6. 갭 선택 방식 개선 (최대 깊이 + 갭 폭 중심 방식)
  7. 히스테리시스 스무딩 비율 조정 (새 방향 반응성 향상)
  8. 스레드 분리 (라이다 파싱 / 제어 / 출력)
  9. 안전 정지 로직 강화

하드웨어:
  - Raspberry Pi (Debian 기반)
  - RPLIDAR C1 → /dev/ttyUSB0, 460800 baud
  - Arduino  → /dev/serial0,  115200 baud
"""

import serial
import math
import time
import threading
import numpy as np

# =====================================================================
# 하드웨어 설정
# =====================================================================
ARDUINO_PORT  = "/dev/serial0"
ARDUINO_BAUD  = 115200

LIDAR_PORT    = "/dev/ttyUSB0"
LIDAR_BAUD    = 460800

# =====================================================================
# 로봇 물리 파라미터
# =====================================================================
ROBOT_RADIUS  = 0.15    # 외접원 반지름 [m] (여유 포함)
WHEEL_BASE    = 0.17    # 휠베이스 [m]

# =====================================================================
# RPLIDAR C1 프로토콜 상수
# =====================================================================
# 스캔 시작 요청: 0xA5 0x20
SCAN_REQUEST  = bytes([0xA5, 0x20])
# 응답 디스크립터: 0xA5 0x5A 로 시작하는 7바이트
RESP_DESC_LEN = 7
# 데이터 패킷: 5바이트 고정
PACKET_LEN    = 5

# =====================================================================
# FTG 주행 파라미터
# =====================================================================
MAX_SPEED     = 0.20    # 최대 전진 속도 [m/s]
MIN_SPEED     = 0.05    # 최소 전진 속도 [m/s] (급선회 시)
MAX_W         = 1.2     # 최대 각속도 [rad/s]

SCAN_LIMIT    = 3.5     # 라이다 유효 거리 [m] (C1 최대 측정 12m, 실내 3.5m 권장)
FRONT_RANGE   = 90      # 전방 탐색 범위 [°] (±90° = 정면 180° 전체)

# 라이다 장착 방향 보정
# 라이다 0° 가 로봇 전방을 가리키면 0,
# 라이다를 뒤집어 장착했으면 180, 90° 돌렸으면 90 등
FRONT_OFFSET  = 0       # [°] 실제 장착 방향에 맞게 조정

# 히스테리시스
HYSTERESIS_THRESHOLD = 12.0  # [°] 이 이하 변화는 이전 방향 유지
SMOOTHING            = 0.35  # 낮을수록 새 방향 반영 빠름 (기존 0.6 → 0.35)

# 전방 긴급 정지 기준
EMERGENCY_DIST  = 0.25   # [m] 이 거리 이내 장애물 → 즉시 정지
EMERGENCY_ANGLE = 20     # [°] 정면 ±20° 이내에서만 체크

# =====================================================================
# 공유 데이터 (스레드 간)
# =====================================================================
scan_data  = [SCAN_LIMIT] * 360   # 인덱스 = 각도(정수), 값 = 거리[m]
scan_lock  = threading.Lock()
new_scan   = threading.Event()    # 한 바퀴 완료 시 set()
stop_event = threading.Event()    # 전체 종료 플래그

# =====================================================================
# RPLIDAR C1 파싱 함수
# =====================================================================
def init_lidar(ser: serial.Serial) -> bool:
    """
    라이다에 스캔 요청을 보내고 응답 디스크립터(7바이트)를 소비한다.
    성공하면 True.
    """
    ser.reset_input_buffer()
    ser.write(SCAN_REQUEST)
    time.sleep(0.1)

    # 응답 디스크립터 대기 (0xA5 0x5A 헤더 탐색)
    start = time.time()
    header_found = False
    while time.time() - start < 3.0:
        b = ser.read(1)
        if b == b'\xa5':
            b2 = ser.read(1)
            if b2 == b'\x5a':
                # 나머지 5바이트 버림
                ser.read(5)
                header_found = True
                break
    if not header_found:
        print("[LIDAR] 응답 디스크립터를 찾지 못했습니다.")
        return False
    print("[LIDAR] 초기화 성공, 스캔 시작")
    return True


def is_packet_valid(b0: int, b1: int) -> bool:
    """
    RPLIDAR C1 동기화 비트 검증.
      - b0의 bit0(S)과 bit1(S̄)이 서로 달라야 함 (XOR == 1)
      - b1의 bit0(C)이 1이어야 함
    """
    s_bit  = (b0 >> 0) & 0x01
    sn_bit = (b0 >> 1) & 0x01
    c_bit  = (b1 >> 0) & 0x01
    return (s_bit ^ sn_bit == 1) and (c_bit == 1)


def parse_packet(raw: bytes):
    """
    5바이트 패킷 → (angle_deg: float, dist_m: float, quality: int)
    angle_deg : 0~359.xx
    dist_m    : 미터 단위 거리 (0이면 측정 실패)
    quality   : 0~63 (낮으면 신뢰도 낮음)
    """
    b0, b1, b2, b3, b4 = raw

    quality    = (b0 >> 2) & 0x3F           # 상위 6비트
    angle_q6   = ((b2 << 7) | (b1 >> 1))   # 15비트 각도 (Q6 고정소수점)
    angle_deg  = angle_q6 / 64.0
    dist_q2    = (b4 << 8) | b3             # 16비트 거리 (Q2 고정소수점)
    dist_mm    = dist_q2 / 4.0
    dist_m     = dist_mm / 1000.0

    return angle_deg, dist_m, quality


def lidar_thread(ser: serial.Serial):
    """
    백그라운드 스레드: 연속으로 5바이트 패킷을 읽어 scan_data 갱신.
    새 한 바퀴(new_scan_flag) 감지 → new_scan 이벤트 발생.
    """
    global scan_data

    if not init_lidar(ser):
        stop_event.set()
        return

    buf       = bytearray()
    prev_angle = None
    local_scan = [SCAN_LIMIT] * 360

    while not stop_event.is_set():
        chunk = ser.read(PACKET_LEN * 10)   # 최대 10패킷씩 읽기
        if not chunk:
            continue
        buf.extend(chunk)

        while len(buf) >= PACKET_LEN:
            # 동기화 비트 맞을 때까지 한 바이트씩 앞으로 이동
            if not is_packet_valid(buf[0], buf[1]):
                buf.pop(0)
                continue

            raw    = bytes(buf[:PACKET_LEN])
            buf    = buf[PACKET_LEN:]

            angle_deg, dist_m, quality = parse_packet(raw)

            # 품질이 너무 낮거나 거리 0이면 SCAN_LIMIT 처리
            if quality < 5 or dist_m <= 0.0:
                dist_m = SCAN_LIMIT
            dist_m = min(dist_m, SCAN_LIMIT)

            # 전방 오프셋 적용
            corrected_angle = (angle_deg + FRONT_OFFSET) % 360.0
            idx = int(corrected_angle) % 360

            local_scan[idx] = dist_m

            # 한 바퀴 완료 감지: 각도가 0°를 다시 지나갈 때
            if prev_angle is not None and prev_angle > 300 and corrected_angle < 60:
                with scan_lock:
                    scan_data = local_scan.copy()
                new_scan.set()
                # 다음 바퀴를 위해 초기화
                local_scan = [SCAN_LIMIT] * 360

            prev_angle = corrected_angle


# =====================================================================
# Follow the Gap 알고리즘 (개선)
# =====================================================================
def get_best_gap_center(proc_dists: np.ndarray, angles: np.ndarray,
                        min_gap_deg: int = 20) -> int:
    """
    연속된 갭(장애물 없는 구간)을 찾고, 가장 넓으면서 가장 깊은 갭의 중심 인덱스 반환.
    단순 argmax 대신 '갭 폭 × 평균 깊이' 점수로 선택.
    """
    in_gap      = False
    gaps        = []        # (start_idx, end_idx, mean_depth)
    gap_start   = 0
    gap_depths  = []

    for i, d in enumerate(proc_dists):
        if d > 0:
            if not in_gap:
                in_gap     = True
                gap_start  = i
                gap_depths = [d]
            else:
                gap_depths.append(d)
        else:
            if in_gap:
                in_gap = False
                width = i - gap_start
                if width >= min_gap_deg:
                    gaps.append((gap_start, i - 1, np.mean(gap_depths)))
                gap_depths = []

    # 마지막 갭 처리
    if in_gap and len(gap_depths) >= min_gap_deg:
        gaps.append((gap_start, len(proc_dists) - 1, np.mean(gap_depths)))

    if not gaps:
        # 갭이 없으면 (전방이 막힌 경우) 가장 먼 단일 포인트
        return int(np.argmax(proc_dists))

    # 점수 = 갭 폭 × 평균 깊이
    best_gap   = max(gaps, key=lambda g: (g[1] - g[0] + 1) * g[2])
    center_idx = (best_gap[0] + best_gap[1]) // 2
    return center_idx


def get_gap_navigation(scan_data_local: list, prev_angle: float):
    """
    FTG 알고리즘 메인 함수.
    Returns: (v [m/s], w [rad/s], final_angle [°])
    """
    # ── 1. 전방 ±FRONT_RANGE° 슬라이싱 ──────────────────────────────
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists  = np.array([
        float(scan_data_local[(int(a) + 360) % 360]) for a in angles
    ])

    # ── 2. 긴급 정지 판단 ────────────────────────────────────────────
    # 전방 전체가 막힌 경우는 아래 4번 로직에서 제자리 회전으로 처리.
    # 여기서는 '일부 장애물이 매우 가까이 있는' 경우만 정지
    emg_mask   = np.abs(angles) <= EMERGENCY_ANGLE
    emg_dists  = dists[emg_mask]
    all_blocked = np.all(dists < EMERGENCY_DIST)    # 전방 전체 차단 여부
    if np.any(emg_dists < EMERGENCY_DIST) and not all_blocked:
        # 정면 일부에 매우 가까운 장애물 → 즉시 정지
        return 0.0, 0.0, prev_angle

    # ── 3. 장애물 팽창 (Bubble Expansion) ───────────────────────────
    proc_dists = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if 0 < d < 1.5:
            ratio = ROBOT_RADIUS / max(d, 0.05)
            alpha = math.degrees(math.asin(min(ratio, 1.0)))
            s_idx = max(0, int(i - alpha))
            e_idx = min(len(dists) - 1, int(i + alpha))
            proc_dists[s_idx:e_idx + 1] = 0.0

    # ── 4. 최적 갭 중심 탐색 ─────────────────────────────────────────
    if np.max(proc_dists) <= 0:
        # 전방이 완전히 막힌 경우 → 제자리 회전
        return 0.0, MAX_W * 0.5, prev_angle

    best_idx   = get_best_gap_center(proc_dists, angles)
    new_angle  = float(angles[best_idx])

    # ── 5. 히스테리시스 + 스무딩 ─────────────────────────────────────
    if abs(new_angle - prev_angle) < HYSTERESIS_THRESHOLD:
        final_angle = prev_angle
    else:
        final_angle = (prev_angle * SMOOTHING) + (new_angle * (1.0 - SMOOTHING))

    # ── 6. v, w 계산 ─────────────────────────────────────────────────
    # 방향이 전방에 가까울수록 빠르게, 옆이면 느리게
    v = MAX_SPEED * (1.0 - abs(final_angle) / FRONT_RANGE)
    v = float(np.clip(v, MIN_SPEED, MAX_SPEED))

    w = math.radians(final_angle) * 1.5
    w = float(np.clip(w, -MAX_W, MAX_W))

    return v, w, final_angle


# =====================================================================
# 출력 스레드 (주기적 콘솔 출력으로 오버헤드 감소)
# =====================================================================
_last_v   = 0.0
_last_w   = 0.0
_last_ang = 0.0

def print_thread():
    while not stop_event.is_set():
        print(f"v={_last_v:.2f} m/s | w={_last_w:.2f} rad/s | angle={_last_ang:.1f}°")
        time.sleep(0.5)


# =====================================================================
# 메인
# =====================================================================
def main():
    global _last_v, _last_w, _last_ang

    # ── 시리얼 포트 초기화 ──────────────────────────────────────────
    try:
        arduino_ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
        lidar_ser   = serial.Serial(LIDAR_PORT,   LIDAR_BAUD,   timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERROR] 포트 열기 실패: {e}")
        return

    time.sleep(0.5)

    # ── 스레드 시작 ─────────────────────────────────────────────────
    t_lidar = threading.Thread(target=lidar_thread, args=(lidar_ser,), daemon=True)
    t_print = threading.Thread(target=print_thread, daemon=True)
    t_lidar.start()
    t_print.start()

    print("=== RPLIDAR C1 + Follow the Gap 시작 ===")
    print("종료: Ctrl+C")

    last_angle = 0.0

    try:
        while not stop_event.is_set():
            # 새 한 바퀴 스캔 완료까지 대기 (최대 200ms)
            if not new_scan.wait(timeout=0.2):
                # 타임아웃: 라이다 데이터 아직 없음 → 정지 유지
                arduino_ser.write(b"0.00,0.00\n")
                continue

            new_scan.clear()

            # 스캔 데이터 복사
            with scan_lock:
                local_scan = scan_data.copy()

            # FTG 계산
            v, w, last_angle = get_gap_navigation(local_scan, last_angle)

            _last_v   = v
            _last_w   = w
            _last_ang = last_angle

            # 아두이노로 전송
            msg = f"{v:.3f},{w:.3f}\n"
            arduino_ser.write(msg.encode())

    except KeyboardInterrupt:
        print("\n[INFO] 정지 명령 전송...")

    finally:
        stop_event.set()
        # 안전 정지
        try:
            arduino_ser.write(b"0.000,0.000\n")
            time.sleep(0.1)
        except Exception:
            pass
        arduino_ser.close()
        lidar_ser.close()
        print("[INFO] 종료 완료")


if __name__ == "__main__":
    main()
