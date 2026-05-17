import serial
import math
import time
import numpy as np

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# =========================================
# LIDAR START
# =========================================
lidar_ser.write(bytes([0xA5, 0x40]))   # Reset
time.sleep(2)
lidar_ser.reset_input_buffer()

lidar_ser.write(bytes([0xA5, 0x20]))   # Scan Start
lidar_ser.read(7)                       # 응답 헤더 소비

print("LIDAR START")

# =========================================
# ROBOT PHYSICAL PARAMETER
# =========================================
ROBOT_RADIUS = 17.0   # 일반 주행 시 안전 반지름 (cm)
WHEEL_BASE   = 17.0   # 차동구동 휠 베이스 (cm)

# =========================================
# DRIVE PARAMETER
# =========================================
MAX_SPEED    = 0.14   # 최대 선속도 (m/s)
MIN_SPEED    = 0.11   # 최소 속도 하한선 유지
MAX_W        = 0.8    # 기본 최대 각속도 (rad/s)
TURN_GAIN    = 1.8    # 조향 게인

SCAN_LIMIT   = 150    # 유효 인식 거리 (cm)
FRONT_RANGE  = 65     # 탐색 반경 (±65°)

# =========================================
# FILTER PARAMETER
# =========================================
EMA_ALPHA    = 0.3    # EMA 필터 계수
MEDIAN_K     = 2      # 중앙값 필터 범위

# =========================================
# SMOOTHING PARAMETER
# =========================================
SMOOTHING_NORMAL = 0.55
SMOOTHING_DANGER = 0.20
DANGER_DIST      = 12     # 위험 감지 거리 (cm)

# =========================================
# GAP & INFLATION PARAMETER
# =========================================
SAFE_DIST          = 14   # Gap 유효 최소 거리 (cm)
INFLATION_MAX_DIST = 25   # 팽창 적용 최대 거리 (cm)

FRONT_CLEAR_DIST   = 23   # 직진 편향 판단 거리 (cm)
FRONT_CLEAR_RANGE  = 15   # 직진 편향 범위 (±15°)

# =========================================
# GOAL PARAMETER (목적지 편향 가중치)
# =========================================
GOAL_ANGLE  = 0       # 로봇 기준 정면 목적지 방향
GOAL_WEIGHT = 1.5     # Gap 스코어링용 가중치

# =========================================
# STATE MACHINE
# =========================================
STATE_NORMAL   = 0
STATE_REVERSE  = 1
STATE_ROTATE   = 2
STATE_REORIENT = 3    # 탈출 후 목적지 방향 재정렬

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

EMERGENCY_DIST    = 6      # 긴급회피 거리 (cm)
REVERSE_DURATION  = 0.18
ROTATE_DURATION   = 1.00
REVERSE_SPEED     = -0.10
ROTATE_W          = 0.9

# REORIENT 파라미터
REORIENT_W          = 0.6    # 재정렬 중 각속도
REORIENT_CLEAR_DIST = 25     # 재정렬 완료 판단 거리
REORIENT_TIMEOUT    = 4.0    # 최대 재정렬 시간 (초)
reorient_dir        = 1      # 재정렬 회전 방향
reorient_start_time = 0.0    # 재정렬 시작 시각

# 갇힘 탈출(Stuck) 카운터
stuck_counter   = 0
STUCK_THRESHOLD = 3          # 연속 "NO GAP" 발생 시 180도 완전 회전

# =========================================
# STATE DATA
# =========================================
scan_data  = np.full(360, float(SCAN_LIMIT), dtype=np.float32)
prev_angle = 0.0

# =========================================
# UTIL & FILTER
# =========================================
def apply_ema(angle, new_dist_cm):
    scan_data[angle] = ((1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm)

def apply_median_filter():
    k      = MEDIAN_K
    window = 2 * k + 1
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        indices  = [(i + d) % 360 for d in range(-k, k + 1)]
        values   = np.sort(scan_data[indices])
        filtered[i] = values[window // 2]
    scan_data[:] = filtered

# =========================================
# DYNAMIC OBSTACLE INFLATION (동적 부풀리기)
# =========================================
def inflate_obstacles_dynamic(dists, angles):
    """
    [핵심 수정] 틈새가 목적지 방향과 일치하고 차체 통과가 가능해 보인다면
    순간적으로 부풀리기 반지름 마진을 축소하여 숨은 통로(Gap)를 열어줍니다.
    """
    proc = dists.copy()
    for i in range(len(dists)):
        d = dists[i]
        if d < 5 or d >= INFLATION_MAX_DIST:
            continue
            
        angle = angles[i]
        current_radius = ROBOT_RADIUS  # 기본 17.0cm
        
        # 조건: 1) 정면 목적지 방향(±20도 이내)이고 2) 실측 거리가 최소 한계치(13cm) 이상인 경우
        if abs(angle - GOAL_ANGLE) <= 20 and d > 13.0:
            current_radius = 11.5  # 로봇 순수 물리 크기에 가깝게 마진 축소
            
        alpha     = math.degrees(math.asin(min(current_radius / d, 1.0)))
        start_idx = max(0, int(i - alpha))
        end_idx   = min(len(dists) - 1, int(i + alpha))
        proc[start_idx : end_idx + 1] = 0.0
    return proc

# =========================================
# GAP SEARCH & SCORE
# =========================================
def find_gaps(proc_dists, angles):
    gaps      = []
    gap_start = None
    for i, d in enumerate(proc_dists):
        if d > SAFE_DIST:
            if gap_start is None: gap_start = i
        else:
            if gap_start is not None:
                gaps.append((gap_start, i - 1))
                gap_start = None
    if gap_start is not None: gaps.append((gap_start, len(proc_dists) - 1))
    return gaps

def score_gap(gap, proc_dists, angles):
    start, end   = gap
    width        = end - start
    center_i     = (start + end) / 2.0
    center_angle = angles[int(center_i)]
    avg_dist     = np.mean(proc_dists[start : end + 1])

    goal_diff  = abs(center_angle - GOAL_ANGLE)
    goal_bonus = max(0.0, (FRONT_RANGE - goal_diff) / FRONT_RANGE) * GOAL_WEIGHT

    return (width    * 0.5
            + avg_dist * 1.2
            - abs(center_angle) * 0.6
            + goal_bonus)

def select_best_gap(gaps, proc_dists, angles):
    best_gap, best_score = None, -1e9
    for gap in gaps:
        s = score_gap(gap, proc_dists, angles)
        if s > best_score:
            best_score, best_gap = s, gap
    return best_gap

def find_best_index_in_gap(best_gap, proc_dists, angles):
    start, end      = best_gap
    best_idx        = int((start + end) / 2)
    max_local_score = -1e9
    
    for i in range(start, end + 1):
        dist         = proc_dists[i]
        angle        = angles[i]
        
        left_margin  = i - start
        right_margin = end - i
        margin       = min(left_margin, right_margin)
        
        # 목적지 가중치 결합: 정면(0도)과 가까울수록 가점 부여
        goal_diff_local = abs(angle - GOAL_ANGLE)
        goal_bonus_local = max(0.0, (FRONT_RANGE - goal_diff_local) / FRONT_RANGE) * 12.0
        
        local_score  = (dist * 1.0) + (margin * 0.8) - (abs(angle) * 1.5) + goal_bonus_local
        
        if local_score > max_local_score:
            max_local_score = local_score
            best_idx        = i
            
    return best_idx

# =========================================
# REORIENT 방향 결정
# =========================================
def choose_reorient_direction():
    left_avg  = float(np.mean(scan_data[1:91]))
    right_avg = float(np.mean(scan_data[270:360]))

    if left_avg >= right_avg:
        print(f"  [REORIENT DIR] LEFT  (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return 1    
    else:
        print(f"  [REORIENT DIR] RIGHT (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return -1   

# =========================================
# PLANNING
# =========================================
def find_best_direction(smoothing):
    global prev_angle
    angles     = np.arange(-FRONT_RANGE, FRONT_RANGE + 1)
    dists      = np.array([scan_data[a % 360] for a in angles], dtype=np.float32)
    
    # [수정] 동적 팽창 함수 호출로 변경하여 틈새 데이터 활성화
    proc_dists = inflate_obstacles_dynamic(dists, angles)
    gaps       = find_gaps(proc_dists, angles)

    if not gaps: return None

    best_gap  = select_best_gap(gaps, proc_dists, angles)
    best_idx  = find_best_index_in_gap(best_gap, proc_dists, angles)
    gap_angle = float(angles[best_idx])

    front_clear = float(np.min(
        scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]
    ))

    CRITICAL_DIST = EMERGENCY_DIST * 2   # 12cm

    if front_clear > FRONT_CLEAR_DIST:   # > 23cm : 직진 편향
        target     = gap_angle * 0.2
        bias_label = "STRAIGHT"
    elif front_clear > CRITICAL_DIST:    # 12~23cm : Gap 완전 추종
        target     = gap_angle * 1.0
        smoothing  = SMOOTHING_DANGER
        bias_label = "GAP"
    else:                                # ≤ 12cm : 즉각 반응
        target     = gap_angle * 1.0
        smoothing  = 0.0
        bias_label = "CRITICAL"

    target     = prev_angle * smoothing + target * (1.0 - smoothing)
    prev_angle = target
    return target, bias_label, front_clear

# =========================================
# CONTROL (좁은 틈새 진입용 순간 선회 제한 완화)
# =========================================
ALIGN_THRESHOLD = 10

def compute_cmd(target_angle):
    # 🎯 [수정] 좁은 틈새 진입 조향 시, 둔하게 도는 현상을 막기 위해 최대 각속도 한계를 일시적으로 완화 (0.8 -> 1.1)
    dynamic_max_w = 1.1 if abs(target_angle) > ALIGN_THRESHOLD else MAX_W
    
    w = math.radians(target_angle) * TURN_GAIN
    w = float(np.clip(w, -dynamic_max_w, dynamic_max_w))

    search_indices = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    relevant_min   = float(np.min(scan_data[search_indices]))

    # 조향 타겟이 틀어져 있다면 제자리 선회(Pivot Turn)로 전환하여 측면 긁힘 방지
    if abs(target_angle) > ALIGN_THRESHOLD:
        return 0.0, w

    obstacle_scale = min(relevant_min / 25.0, 1.0)
    speed          = max(MAX_SPEED * obstacle_scale, MIN_SPEED)
    return speed, w

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

def choose_avoid_direction():
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    if left_avg >= right_avg:
        print(f"  [AVOID DIR] LEFT  (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return 1
    else:
        print(f"  [AVOID DIR] RIGHT (L:{left_avg:.1f}cm R:{right_avg:.1f}cm)")
        return -1

# =========================================
# MAIN LOOP
# =========================================
print("NAVIGATION START")
try:
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5: continue

        s_flag = raw[0] & 0x01
        if (raw[0] & 0x02) >> 1 != (1 - s_flag): continue
        if (raw[1] & 0x01) != 1: continue
        if (raw[0] >> 2) < 3: continue

        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < SCAN_LIMIT:
            apply_ema(angle, dist_cm)

        if s_flag != 1: continue
        apply_median_filter()
        now = time.time()

        # =================================
        # STATE MACHINE
        # =================================

        # ── STATE_REVERSE ──
        if state == STATE_REVERSE:
            if now < maneuver_end_time:
                send_cmd(REVERSE_SPEED, 0.0)
                print(f"  [REVERSE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                state             = STATE_ROTATE
                maneuver_end_time = now + ROTATE_DURATION
                print(f"  [ROTATE START] dir:{'+' if rotate_dir > 0 else '-'}")
            continue

        # ── STATE_ROTATE ──
        if state == STATE_ROTATE:
            if now < maneuver_end_time:
                send_cmd(0.0, ROTATE_W * rotate_dir)
                print(f"  [ROTATE] remaining:{maneuver_end_time - now:.2f}s")
            else:
                rotate_dir         *= -1
                reorient_dir        = choose_reorient_direction()
                reorient_start_time = now
                state               = STATE_REORIENT
                prev_angle          = 0.0
                for a in range(-45, 46):
                    scan_data[a % 360] = float(SCAN_LIMIT)
                print(f"  [REORIENT START] dir:{'+' if reorient_dir > 0 else '-'}")
            continue

        # ── STATE_REORIENT : 목적지 방향 재정렬 ──
        if state == STATE_REORIENT:
            elapsed     = now - reorient_start_time
            front_clear = float(np.min(
                scan_data[np.arange(-FRONT_CLEAR_RANGE, FRONT_CLEAR_RANGE + 1) % 360]
            ))

            if front_clear > REORIENT_CLEAR_DIST:
                state = STATE_NORMAL
                print(f"  [REORIENT→NORMAL] 전방 개방 ({front_clear:.1f}cm) 재정렬 완료")
                continue

            if elapsed > REORIENT_TIMEOUT:
                state = STATE_NORMAL
                print(f"  [REORIENT→NORMAL] 타임아웃 ({elapsed:.1f}s) 강제 복귀")
                continue

            send_cmd(0.0, REORIENT_W * reorient_dir)
            print(f"  [REORIENT] elapsed:{elapsed:.1f}s front:{front_clear:.1f}cm dir:{'+' if reorient_dir > 0 else '-'}")
            continue

        # ── STATE_NORMAL ──
        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))
        if front_min < EMERGENCY_DIST:
            rotate_dir        = choose_avoid_direction()
            state             = STATE_REVERSE
            maneuver_end_time = now + REVERSE_DURATION
            send_cmd(REVERSE_SPEED, 0.0)
            print(f"  EMERGENCY! front:{front_min:.1f}cm → REVERSE")
            continue

        smoothing = SMOOTHING_DANGER if front_min < DANGER_DIST else SMOOTHING_NORMAL
        result    = find_best_direction(smoothing)

        # 주변이 통째로 막혀 갈 수 있는 갭이 하나도 없을 때 (Stuck 예외 처리)
        if result is None:
            stuck_counter += 1
            print(f"  [NO GAP WARNING] 연속 갇힘 횟수: {stuck_counter}/{STUCK_THRESHOLD}")
            
            if stuck_counter >= STUCK_THRESHOLD:
                rotate_dir        = choose_avoid_direction()
                state             = STATE_ROTATE
                maneuver_end_time = now + (ROTATE_DURATION * 2.0)
                stuck_counter     = 0
                print("  🔥🔥 [CRITICAL STUCK] 사방 고립! 180도 회전 탈출 시도! 🔥🔥")
            else:
                rotate_dir        = choose_avoid_direction()
                state             = STATE_REVERSE
                maneuver_end_time = now + REVERSE_DURATION
                send_cmd(REVERSE_SPEED, 0.0)
            continue

        # 정상 주행 시 Stuck 카운터 리셋
        stuck_counter = 0
        target_angle, bias_label, front_clear = result
        v, w = compute_cmd(target_angle)
        send_cmd(v, w)

        print(f"TRG:{target_angle:5.1f}° | v:{v:.2f} | w:{w:.2f} | f_min:{front_min:.1f} | clear:{front_clear:.1f} | bias:{bias_label}")

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
