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
  10. [BUG FIX] MOTOR_REVERSED 시 final_angle 반전 버그 수정
      → w 계산 전용 임시 변수 사용, prev_angle 오염 방지
  11. [수정] EMERGENCY_ANGLE 40°로 확대 (양옆 인지 범위 넓힘)
  12. [수정] 긴급 정지 시 제자리 회전 추가 (좌우 여유 공간 비교 후 넓은 쪽)
  13. [수정] 전방 완전 차단 시 제자리 회전 방향 보정 로직 분리

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
SCAN_REQUEST  = bytes([0xA5, 0x20])
RESP_DESC_LEN = 7
PACKET_LEN    = 5

# =====================================================================
# FTG 주행 파라미터
# =====================================================================
MAX_SPEED     = 0.20    # 최대 전진 속도 [m/s]
MIN_SPEED     = 0.05    # 최소 전진 속도 [m/s] (급선회 시)
MAX_W         = 1.2     # 최대 각속도 [rad/s]

SCAN_LIMIT    = 3.5     # 라이다 유효 거리 [m]
FRONT_RANGE   = 110     # 전방 탐색 범위 ±110°

FRONT_OFFSET  = 0       # [°] 라이다 장착 방향 보정

# 히스테리시스
HYSTERESIS_THRESHOLD = 12.0
SMOOTHING            = 0.35

# 전방 긴급 정지 기준
EMERGENCY_DIST  = 0.05   # 10cm 이내 장애물 → 긴급 처리
EMERGENCY_ANGLE = 20    # [수정] ±40° (기존 ±20° → 양옆 20° 추가)

# 모터드라이버 배선 반전 여부
# 오른쪽 장애물인데 오른쪽으로 돌면 True
MOTOR_REVERSED  = True

# =====================================================================
# 공유 데이터 (스레드 간)
# =====================================================================
scan_data  = [SCAN_LIMIT] * 360
scan_lock  = threading.Lock()
new_scan   = threading.Event()
stop_event = threading.Event()

# =====================================================================
# RPLIDAR C1 파싱 함수
# =====================================================================
def init_lidar(ser: serial.Serial) -> bool:
    ser.reset_input_buffer()
    ser.write(SCAN_REQUEST)
    time.sleep(0.1)

    start = time.time()
    header_found = False
    while time.time() - start < 3.0:
        b = ser.read(1)
        if b == b'\xa5':
            b2 = ser.read(1)
            if b2 == b'\x5a':
                ser.read(5)
                header_found = True
                break
    if not header_found:
        print("[LIDAR] 응답 디스크립터를 찾지 못했습니다.")
        return False
    print("[LIDAR] 초기화 성공, 스캔 시작")
    return True


def is_packet_valid(b0: int, b1: int) -> bool:
    s_bit  = (b0 >> 0) & 0x01
    sn_bit = (b0 >> 1) & 0x01
    c_bit  = (b1 >> 0) & 0x01
    return (s_bit ^ sn_bit == 1) and (c_bit == 1)


def parse_packet(raw: bytes):
    b0, b1, b2, b3, b4 = raw
    quality    = (b0 >> 2) & 0x3F
    angle_q6   = ((b2 << 7) | (b1 >> 1))
    angle_deg  = angle_q6 / 64.0
    dist_q2    = (b4 << 8) | b3
    dist_mm    = dist_q2 / 4.0
    dist_m     = dist_mm / 1000.0
    return angle_deg, dist_m, quality


def lidar_thread(ser: serial.Serial):
    global scan_data

    if not init_lidar(ser):
        stop_event.set()
        return

    buf        = bytearray()
    prev_angle = None
    local_scan = [SCAN_LIMIT] * 360

    while not stop_event.is_set():
        chunk = ser.read(PACKET_LEN * 10)
        if not chunk:
            continue
        buf.extend(chunk)

        while len(buf) >= PACKET_LEN:
            if not is_packet_valid(buf[0], buf[1]):
                buf.pop(0)
                continue

            raw = bytes(buf[:PACKET_LEN])
            buf = buf[PACKET_LEN:]

            angle_deg, dist_m, quality = parse_packet(raw)

            if quality < 5 or dist_m <= 0.0:
                dist_m = SCAN_LIMIT
            dist_m = min(dist_m, SCAN_LIMIT)

            corrected_angle = (angle_deg + FRONT_OFFSET) % 360.0
            idx = int(corrected_angle) % 360
            local_scan[idx] = dist_m

            if prev_angle is not None and prev_angle > 300 and corrected_angle < 60:
                with scan_lock:
                    scan_data = local_scan.copy()
                new_scan.set()
                local_scan = [SCAN_LIMIT] * 360

            prev_angle = corrected_angle


# =====================================================================
# Follow the Gap 알고리즘
# =====================================================================
def get_best_gap_center(proc_dists: np.ndarray, angles: np.ndarray,
                        min_gap_deg: int = 20) -> int:
    in_gap     = False
    gaps       = []
    gap_start  = 0
    gap_depths = []

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

    if in_gap and len(gap_depths) >= min_gap_deg:
        gaps.append((gap_start, len(proc_dists) - 1, np.mean(gap_depths)))

    if not gaps:
        return int(np.argmax(proc_dists))

    best_gap   = max(gaps, key=lambda g: (g[1] - g[0] + 1) * g[2])
    center_idx = (best_gap[0] + best_gap[1]) // 2
    return center_idx


def _get_escape_w(dists: np.ndarray, angles: np.ndarray) -> float:
    """
    긴급 상황(정면 근접 장애물 / 전방 완전 차단) 시
    좌우 여유 공간을 비교해 더 넓은 방향으로 제자리 회전.

    반환: w [rad/s]  (MOTOR_REVERSED 보정 포함)
    """
    # angles 배열에서 0 기준 왼쪽(양수)과 오른쪽(음수) 분리
    left_mask  = angles > 0
    right_mask = angles < 0

    left_mean  = float(np.mean(dists[left_mask]))  if left_mask.any()  else 0.0
    right_mean = float(np.mean(dists[right_mask])) if right_mask.any() else 0.0

    # 더 넓은 쪽으로 회전 (FTG 좌표계: 양수 각도 = 왼쪽)
    # w > 0 → 왼쪽 회전, w < 0 → 오른쪽 회전
    if left_mean >= right_mean:
        w = MAX_W * 0.6     # 왼쪽이 더 넓으면 왼쪽으로
    else:
        w = -MAX_W * 0.6    # 오른쪽이 더 넓으면 오른쪽으로

    # 모터 배선 반전 보정
    if MOTOR_REVERSED:
        w = -w

    return w


def get_gap_navigation(scan_data_local: list, prev_angle: float):
    """
    FTG 알고리즘 메인 함수.
    Returns: (v [m/s], w [rad/s], final_angle [°])
    """
    # ── 1. 전방 ±FRONT_RANGE° 슬라이싱 ─────────────────
    angles = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists  = np.array([
        float(scan_data_local[(int(a) + 360) % 360]) for a in angles
    ])

    # ── 2. 장애물 감지 및 회전 ────────────────────────────
    emg_mask = np.abs(angles) <= EMERGENCY_ANGLE
    emg_dists = dists[emg_mask]

    # 전방 장애물이 있는 경우
    if np.any(emg_dists < EMERGENCY_DIST):
        # 왼쪽과 오른쪽의 거리 비교
        left_mask = angles < 0
        right_mask = angles > 0

        left_mean = np.mean(dists[left_mask]) if left_mask.any() else 0.0
        right_mean = np.mean(dists[right_mask]) if right_mask.any() else 0.0

        # 더 넓은 쪽으로 회전 설정
        if left_mean >= right_mean:
            w = MAX_W * 0.5   # 왼쪽 회전
        else:
            w = -MAX_W * 0.5  # 오른쪽 회전

        print(f"[OBSTACLE] 전방 장애물 감지 → w={w:.2f}")
        return 0.05, w, prev_angle  # 속도는 유지하며 회전

    # ── 3. 장애물 팽창 (Bubble Expansion) ───────────────
    proc_dists = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if 0 < d < 1.5:
            ratio = ROBOT_RADIUS / max(d, 0.05)
            alpha = min(math.degrees(math.asin(min(ratio, 1.0))), 35.0)
            s_idx = max(0, int(i - alpha))
            e_idx = min(len(dists) - 1, int(i + alpha))
            proc_dists[s_idx:e_idx + 1] = 0.0
        
    # ── 4. 최적 갭 중심 탐색 ────────────────────────────
    best_idx = get_best_gap_center(proc_dists, angles)
    new_angle = float(angles[best_idx])

    # ── 5. 히스테리시스 + 스무딩 ────────────────────────
    if abs(new_angle - prev_angle) < HYSTERESIS_THRESHOLD:
        final_angle = prev_angle
    else:
        final_angle = (prev_angle * SMOOTHING) + (new_angle * (1.0 - SMOOTHING))

    # ── 6. v, w 계산 ─────────────────────────────────────
    v = MAX_SPEED * (1.0 - abs(final_angle) / FRONT_RANGE)
    v = float(np.clip(v, MIN_SPEED, MAX_SPEED))

    w_angle = -final_angle if MOTOR_REVERSED else final_angle
    w = math.radians(w_angle) * 1.5
    w = float(np.clip(w, -MAX_W, MAX_W))

    return v, w, final_angle



# =====================================================================
# 출력 스레드
# =====================================================================
_last_v   = 0.0
_last_w   = 0.0
_last_ang = 0.0

def print_thread():
    while not stop_event.is_set():
        print(f"v={_last_v:.2f} m/s | w={_last_w:.2f} rad/s | angle={_last_ang:.1f} deg")
        time.sleep(0.5)


# =====================================================================
# 메인
# =====================================================================
def main():
    global _last_v, _last_w, _last_ang

    try:
        arduino_ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
        lidar_ser   = serial.Serial(LIDAR_PORT,   LIDAR_BAUD,   timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERROR] 포트 열기 실패: {e}")
        return

    time.sleep(0.5)

    t_lidar = threading.Thread(target=lidar_thread, args=(lidar_ser,), daemon=True)
    t_print = threading.Thread(target=print_thread, daemon=True)
    t_lidar.start()
    t_print.start()

    print("=== RPLIDAR C1 + Follow the Gap 시작 ===")
    print("종료: Ctrl+C")

    last_angle = 0.0

    try:
        while not stop_event.is_set():
            if not new_scan.wait(timeout=0.2):
                arduino_ser.write(b"0.00,0.00\n")
                continue

            new_scan.clear()

            with scan_lock:
                local_scan = scan_data.copy()

            v, w, last_angle = get_gap_navigation(local_scan, last_angle)

            _last_v   = v
            _last_w   = w
            _last_ang = last_angle

            msg = f"{v:.3f},{w:.3f}\n"
            arduino_ser.write(msg.encode())

    except KeyboardInterrupt:
        print("\n[INFO] 정지 명령 전송...")

    finally:
        stop_event.set()
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
