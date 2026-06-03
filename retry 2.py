import cv2
import serial
import numpy as np
import time

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0",115200,timeout=0.1)

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)

WIDTH = 640
HEIGHT = 480

cap.set(3, WIDTH)
cap.set(4, HEIGHT)

# =========================================
# MOTOR
# =========================================
MAX_V = 0.28
TURN_W = 0.55

def send_cmd(v,w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0,0)

# =========================================
# RED RANGE
# (네가 준 값 기반)
# =========================================

# HSV
lower_hsv = np.array([160,130,150])
upper_hsv = np.array([179,255,255])

# BGR
lower_bgr = np.array([80,30,170])
upper_bgr = np.array([150,100,255])

# =========================================
# MEMORY
# =========================================
last_seen_x = WIDTH//2

print("RED TRACK START")

try:

    while True:

        ret, frame = cap.read()

        if not ret:
            continue

        frame = cv2.flip(frame,1)

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        # --------------------
        # HSV MASK
        # --------------------
        hsv_mask = cv2.inRange(
            hsv,
            lower_hsv,
            upper_hsv
        )

        # --------------------
        # BGR MASK
        # --------------------
        bgr_mask = cv2.inRange(
            frame,
            lower_bgr,
            upper_bgr
        )

        # 둘 다 만족
        mask = cv2.bitwise_and(
            hsv_mask,
            bgr_mask
        )

        kernel = np.ones((5,5),np.uint8)

        mask = cv2.erode(
            mask,
            kernel,
            iterations=1
        )

        mask = cv2.dilate(
            mask,
            kernel,
            iterations=2
        )

        contours,_ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # 중앙 정지 박스
        box = 120

        x1 = WIDTH//2 - box//2
        x2 = WIDTH//2 + box//2

        y1 = HEIGHT//2 - box//2
        y2 = HEIGHT//2 + box//2

        cv2.rectangle(
            frame,
            (x1,y1),
            (x2,y2),
            (255,255,0),
            2
        )

        if contours:

            c = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(c)

            if area > 700:

                x,y,w,h = cv2.boundingRect(c)

                cx = x + w//2
                cy = y + h//2

                last_seen_x = cx

                cv2.rectangle(
                    frame,
                    (x,y),
                    (x+w,y+h),
                    (0,255,0),
                    2
                )

                cv2.circle(
                    frame,
                    (cx,cy),
                    5,
                    (255,0,0),
                    -1
                )

                # ====================
                # 사각형 안 도착
                # ====================
                if x1 < cx < x2 and y1 < cy < y2:

                    v = 0
                    w = 0
                    state = "STOP"

                else:

                    error = cx - WIDTH//2

                    if abs(error) > 80:

                        v = 0.12

                        if error > 0:
                            w = -TURN_W
                            state = "RIGHT"

                        else:
                            w = TURN_W
                            state = "LEFT"

                    else:

                        v = MAX_V
                        w = 0
                        state = "FORWARD"

                send_cmd(v,w)

            else:

                stop_robot()
                state="SMALL"

        else:

            # ====================
            # 기억 기반 탐색
            # ====================

            if last_seen_x > WIDTH//2:

                send_cmd(
                    0.08,
                    -0.45
                )

                state="SEARCH RIGHT"

            else:

                send_cmd(
                    0.08,
                    0.45
                )

                state="SEARCH LEFT"

        cv2.putText(
            frame,
            state,
            (20,40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0,255,0),
            2
        )

        cv2.imshow("frame",frame)
        cv2.imshow("mask",mask)

        if cv2.waitKey(1)==27:
            break

except KeyboardInterrupt:
    pass

finally:

    stop_robot()

    cap.release()

    cv2.destroyAllWindows()
