import cv2
import serial
import numpy as np
import time

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# =========================================
# CONTROL
# =========================================
KP_ROT       = 0.0045
MIN_AREA     = 500
TARGET_AREA  = 18000   # 이 면적까지 접근
MIN_FORWARD  = 0.05
MIN_BACKWARD = -0.08

# =========================================
# 도착 판정 파라미터
# =========================================
# 구역이 화면 하단에 걸쳐있을 때 y 기준
# 240 해상도에서 하단 20% = y > 192
BOTTOM_THRESHOLD = 200   # 윤곽 하단이 이 y 이상이면 "구역이 발 밑에 있음"
NEAR_AREA        = 12000  # 이 면적 이상일 때만 도착 판정 고려
ARRIVE_CONFIRM   = 8      # 연속 N프레임 유지

arrive_count  = 0

# 도착 확정 후 구역 중앙까지 전진
# 구역이 하단에서 사라질 때 앞 끝이 구역에 걸쳐있는 상태
# 속도 0.06으로 1.2초 ≈ 7cm 전진 → 구역 중앙
FINAL_SPEED    = 0.06
FINAL_DURATION = 1.2

# =========================================
# 색상 범위
# =========================================
# RED (주황 오인식 방지: H 0~10, 165~179)
lower_red_hsv1 = np.array([165, 100, 100])
upper_red_hsv1 = np.array([179, 255, 255])
lower_red_hsv2 = np.array([0,   100, 100])
upper_red_hsv2 = np.array([10,  255, 255])
lower_red_bgr  = np.array([80,  30,  150])
upper_red_bgr  = np.array([200, 210, 255])

# YELLOW (H 15~35)
lower_yellow_hsv = np.array([15, 100, 150])
upper_yellow_hsv = np.array([35, 255, 255])
lower_yellow_bgr = np.array([0,  120, 150])
upper_yellow_bgr = np.array([180,255, 255])

# BLUE (H 100~135)
lower_blue_hsv = np.array([100, 80,  70])
upper_blue_hsv = np.array([135, 255, 255])
lower_blue_bgr = np.array([80,  50,  30])
upper_blue_bgr = np.array([255, 180, 180])

# =========================================
# FSM
# =========================================
mission_order = ["RED", "YELLOW", "BLUE"]
mission_idx   = 0
target_color  = mission_order[mission_idx]

found_once   = False
search_dir   = 1
search_timer = 0
last_seen_x  = 160

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    v = np.clip(v, -0.30, 0.30)
    w = np.clip(w, -0.80, 0.80)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0, 0)

def get_mask(frame, hsv, color):
    if color == "RED":
        m1 = cv2.inRange(hsv, lower_red_hsv1, upper_red_hsv1)
        m2 = cv2.inRange(hsv, lower_red_hsv2, upper_red_hsv2)
        hsv_mask = cv2.bitwise_or(m1, m2)
        bgr_mask = cv2.inRange(frame, lower_red_bgr, upper_red_bgr)
    elif color == "YELLOW":
        hsv_mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
        bgr_mask = cv2.inRange(frame, lower_yellow_bgr, upper_yellow_bgr)
    else:
        hsv_mask = cv2.inRange(hsv, lower_blue_hsv, upper_blue_hsv)
        bgr_mask = cv2.inRange(frame, lower_blue_bgr, upper_blue_bgr)

    mask = cv2.bitwise_and(hsv_mask, bgr_mask)
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

def do_arrival():
    """도착 확정 후 처리"""
    global arrive_count, mission_idx, target_color, found_once, search_timer

    # 구역 중앙까지 짧게 전진 후 정지
    print(f"[{target_color}] 구역 진입 - 전진 {FINAL_DURATION}초")
    send_cmd(FINAL_SPEED, 0)
    time.sleep(FINAL_DURATION)

    stop_robot()
    print(f"[{target_color}] 1초 정지")
    time.sleep(1.0)
    print(f"[{target_color}] DONE")

    arrive_count = 0
    mission_idx += 1

    if mission_idx >= len(mission_order):
        print("=== MISSION COMPLETE ===")
        stop_robot()
        return True  # 전체 완료

    target_color = mission_order[mission_idx]
    print(f"다음 목표: {target_color}")
    found_once   = False
    search_timer = 0
    return False

print("=== MISSION START ===")
print(f"첫 번째 목표: {target_color}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        area = 0
        frame = cv2.flip(frame, 1)
        HEIGHT, WIDTH = frame.shape[:2]
        frame_cx = WIDTH // 2

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = get_mask(frame, hsv, target_color)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        state       = "SEARCH"
        bottom_y    = 0

        if contours:
            found_once   = True
            search_timer = 0

            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:

                # 무게중심
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = frame_cx, HEIGHT // 2

                last_seen_x = cx
                error_x     = cx - frame_cx

                # 윤곽 하단 y좌표 (구역이 발밑에 얼마나 걸쳐있는지)
                bottom_y = int(c[:, :, 1].max())

                # 시각화
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                cv2.line(frame, (frame_cx, 0), (frame_cx, HEIGHT),
                         (255, 255, 0), 1)
                # 하단 기준선 표시
                cv2.line(frame, (0, BOTTOM_THRESHOLD),
                         (WIDTH, BOTTOM_THRESHOLD), (0, 165, 255), 1)

                w = -KP_ROT * error_x

                # ─────────────────────────────────────
                # 도착 판정
                # 조건: x 정렬 + 면적 충분 + 하단이 화면 끝에 걸침
                # ─────────────────────────────────────
                centered    = abs(error_x) < 25
                near_enough = area > NEAR_AREA
                at_bottom   = bottom_y > BOTTOM_THRESHOLD

                if centered and near_enough and at_bottom:
                    arrive_count += 1
                    state = f"CONFIRM {arrive_count}/{ARRIVE_CONFIRM}"

                    # 확정 전까지 천천히 전진 (정렬 유지)
                    send_cmd(0.04, w)

                    if arrive_count >= ARRIVE_CONFIRM:
                        done = do_arrival()
                        if done:
                            break
                        continue

                else:
                    arrive_count = 0

                    distance_error = TARGET_AREA - area
                    if distance_error > 0:
                        v = distance_error * 0.000025
                        v = np.clip(v, MIN_FORWARD, 0.18)
                        state = "TRACK"
                    else:
                        v = distance_error * 0.000020
                        v = np.clip(v, MIN_BACKWARD, -0.03)
                        state = "BACKWARD"

                    send_cmd(v, w)

        else:
            arrive_count = 0
            search_timer += 1

            if not found_once:
                send_cmd(0, 0.35 * search_dir)
                state = "INIT SEARCH"
                if search_timer > 70:
                    search_dir  *= -1
                    search_timer = 0
            else:
                if last_seen_x > frame_cx:
                    send_cmd(0.03, -0.30)
                    state = "SEARCH RIGHT"
                else:
                    send_cmd(0.03,  0.30)
                    state = "SEARCH LEFT"

        # HUD
        cv2.putText(frame, state,
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"TARGET:{target_color}",
                    (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"AREA:{int(area)}  BOT:{bottom_y}",
                    (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2)
        cv2.putText(frame, f"MISSION:{mission_idx+1}/{len(mission_order)}",
                    (20, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
    pass

finally:
    stop_robot()
    cap.release()
    cv2.destroyAllWindows()
