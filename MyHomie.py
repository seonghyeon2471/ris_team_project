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
KP_ROT         = 0.0045
MIN_AREA       = 500
TARGET_AREA    = 22000
ARRIVE_AREA    = 28000
MIN_FORWARD    = 0.05
MIN_BACKWARD   = -0.08
ARRIVE_CONFIRM = 12

# =========================================
# 색상 범위 (샘플 BGR 기반 HSV 변환 + 조명 보정)
# =========================================

# ── RED ──────────────────────────────────
# 주황 오인식 방지: H 0~10(저H) + 165~179(고H) 로 좁게 제한
# 주황은 H≈15~20 이므로 10 이하로 끊음
lower_red_hsv1 = np.array([165, 100, 100])
upper_red_hsv1 = np.array([179, 255, 255])

lower_red_hsv2 = np.array([0,   100, 100])
upper_red_hsv2 = np.array([10,  255, 255])

lower_red_bgr  = np.array([80,  30,  150])
upper_red_bgr  = np.array([200, 210, 255])

# ── YELLOW ───────────────────────────────
# H: 15~35, 조명 밝아서 S 하한 낮춤
lower_yellow_hsv = np.array([15, 100, 150])
upper_yellow_hsv = np.array([35, 255, 255])

lower_yellow_bgr = np.array([0,   120, 150])
upper_yellow_bgr = np.array([180, 255, 255])

# ── BLUE ─────────────────────────────────
# H: 100~135
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

found_once    = False
search_dir    = 1
search_timer  = 0
last_seen_x   = 160
arrive_count  = 0

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    v = np.clip(v, -0.30, 0.30)
    w = np.clip(w, -0.80, 0.80)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0, 0)

# =========================================
# 마스크 생성
# =========================================
def get_mask(frame, hsv, color):
    if color == "RED":
        m1 = cv2.inRange(hsv, lower_red_hsv1, upper_red_hsv1)
        m2 = cv2.inRange(hsv, lower_red_hsv2, upper_red_hsv2)
        hsv_mask = cv2.bitwise_or(m1, m2)
        bgr_mask = cv2.inRange(frame, lower_red_bgr, upper_red_bgr)
    elif color == "YELLOW":
        hsv_mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
        bgr_mask = cv2.inRange(frame, lower_yellow_bgr, upper_yellow_bgr)
    else:  # BLUE
        hsv_mask = cv2.inRange(hsv, lower_blue_hsv, upper_blue_hsv)
        bgr_mask = cv2.inRange(frame, lower_blue_bgr, upper_blue_bgr)

    mask = cv2.bitwise_and(hsv_mask, bgr_mask)
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

# =========================================
# MAIN LOOP
# =========================================
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

        state = "SEARCH"

        if contours:
            found_once   = True
            search_timer = 0

            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:

                # 무게중심으로 정확한 중심 계산
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = frame_cx, HEIGHT // 2

                last_seen_x = cx
                error_x     = cx - frame_cx

                # 시각화
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                cv2.line(frame, (frame_cx, 0), (frame_cx, HEIGHT),
                         (255, 255, 0), 1)  # 중앙선

                w = -KP_ROT * error_x

                # ─────────────────────────────────────
                # 도착 판정: 중앙정렬 + 면적 충분
                # ─────────────────────────────────────
                centered     = abs(error_x) < 20
                close_enough = area > ARRIVE_AREA

                if centered and close_enough:
                    arrive_count += 1
                    state = f"CONFIRM {arrive_count}/{ARRIVE_CONFIRM}"
                    send_cmd(0.03, w)  # 확정 전 아주 천천히

                    if arrive_count >= ARRIVE_CONFIRM:

                        # ★ 도착 확정 → 정지 1초
                        stop_robot()
                        print(f"[{target_color}] 구역 도착 - 1초 정지")
                        time.sleep(1.0)
                        print(f"[{target_color}] DONE")

                        # ★ 카운터 리셋 후 다음 미션
                        arrive_count = 0
                        mission_idx += 1

                        if mission_idx >= len(mission_order):
                            print("=== MISSION COMPLETE ===")
                            stop_robot()
                            break

                        target_color = mission_order[mission_idx]
                        print(f"다음 목표: {target_color}")

                        found_once   = False
                        search_timer = 0
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
        cv2.putText(frame, f"AREA:{int(area)}",
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
