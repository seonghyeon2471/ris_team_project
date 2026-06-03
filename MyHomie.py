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
KP_ROT = 0.0045
MIN_AREA = 500

TARGET_AREA  = 22000   # 이 면적이 되면 속도 줄임
ARRIVE_AREA  = 28000   # 이 면적 이상 + 중앙 정렬 → 정지 (구역 위에 올라선 상태)

MIN_FORWARD  = 0.05
MIN_BACKWARD = -0.08

# 도착 확정: 연속 N프레임 조건 유지해야 정지
# (1프레임 오인식 방지)
ARRIVE_CONFIRM = 12
arrive_count = 0

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

# =========================================
# 색상 범위
# =========================================
lower_red1    = np.array([0,   50,  80]); upper_red1    = np.array([15,  255, 255])
lower_red2    = np.array([155, 50,  80]); upper_red2    = np.array([179, 255, 255])
lower_red_bgr = np.array([30,  30, 120]); upper_red_bgr = np.array([210, 220, 255])

lower_yellow_hsv = np.array([12,  60, 100]); upper_yellow_hsv = np.array([45,  255, 255])
lower_yellow_bgr = np.array([0,  100, 120]); upper_yellow_bgr = np.array([180, 255, 255])

lower_blue_hsv = np.array([90,  50,  50]); upper_blue_hsv = np.array([140, 255, 255])
lower_blue_bgr = np.array([80,  60,  60]); upper_blue_bgr = np.array([255, 180, 180])

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
        m1 = cv2.inRange(hsv, lower_red1, upper_red1)
        m2 = cv2.inRange(hsv, lower_red2, upper_red2)
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

print("=== MISSION START ===")
print(f"첫 번째 목표: {target_color}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

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

                # ★ minAreaRect 대신 moments로 정확한 무게중심
                M  = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = frame_cx, HEIGHT // 2

                last_seen_x = cx

                # 무게중심 표시
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)

                error_x = cx - frame_cx

                # 회전 제어
                w = -KP_ROT * error_x

                # ─────────────────────────────────────
                # 도착 판정
                # 조건: x 중앙 정렬 + 면적 충분히 큼
                # → 연속 ARRIVE_CONFIRM 프레임 유지
                # ─────────────────────────────────────
                centered     = abs(error_x) < 20   # x 오차 20px 이내
                close_enough = area > ARRIVE_AREA

                if centered and close_enough:
                    arrive_count += 1
                    state = f"CONFIRM {arrive_count}/{ARRIVE_CONFIRM}"

                    # 확정 전까지 아주 천천히 전진 (관성 최소화)
                    send_cmd(0.03, w)

                    if arrive_count >= ARRIVE_CONFIRM:
                        # ★ 완전 정지 후 1초 주차
                        stop_robot()
                        print(f"[{target_color}] 구역 도착 - 1초 정지")
                        time.sleep(1.0)
                        print(f"[{target_color}] DONE")

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
                    # 도착 조건 미충족 → 카운터 리셋
                    arrive_count = 0

                    # 거리 제어
                    distance_error = TARGET_AREA - area

                    if distance_error > 0:
                        # 멀다 → 전진, 가까울수록 느리게
                        v = distance_error * 0.000025
                        v = np.clip(v, MIN_FORWARD, 0.18)
                    else:
                        # 너무 가깝다 → 후진
                        v = distance_error * 0.000020
                        v = np.clip(v, MIN_BACKWARD, -0.03)

                    send_cmd(v, w)
                    state = "TRACK" if distance_error > 0 else "BACKWARD"

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
        cv2.putText(frame, state, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"TARGET:{target_color}", (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"AREA:{int(area) if contours else 0}", (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2)
        cv2.putText(frame, f"{mission_idx+1}/{len(mission_order)}", (20, 145),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
    pass

finally:
    stop_robot()
    cap.release()
    cv2.destroyAllWindows()
