import cv2
import serial
import numpy as np
import time

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.1
)

# =========================================
# CAMERA
# =========================================
WIDTH = 640
HEIGHT = 480

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

# =========================================
# CONTROL PARAMETER
# =========================================
MAX_V = 0.22
MIN_V = 0.08

MAX_W = 0.55

KP_ROT = 0.0018
KP_FORWARD = 0.0012

X_TOL = 20
Y_TOL = 20

MIN_AREA = 700

# =========================================
# RED RANGE
# =========================================

# HSV 범위
lower_hsv = np.array([160,130,150])
upper_hsv = np.array([179,255,255])

# BGR 범위
lower_bgr = np.array([80,30,170])
upper_bgr = np.array([150,100,255])

# =========================================
# MEMORY
# =========================================
last_seen_x = WIDTH//2

# =========================================
# MOTOR
# =========================================
def send_cmd(v,w):

    v = np.clip(v,-0.3,0.3)
    w = np.clip(w,-0.8,0.8)

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():

    send_cmd(0,0)

# =========================================
# START
# =========================================
print("COLOR CENTER TRACK START")

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

        # ==========================
        # MASK
        # ==========================
        hsv_mask = cv2.inRange(
            hsv,
            lower_hsv,
            upper_hsv
        )

        bgr_mask = cv2.inRange(
            frame,
            lower_bgr,
            upper_bgr
        )

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

        frame_cx = WIDTH//2
        frame_cy = HEIGHT//2

        # 카메라 중심 표시
        cv2.circle(
            frame,
            (frame_cx,frame_cy),
            7,
            (0,255,255),
            -1
        )

        cv2.line(
            frame,
            (frame_cx,0),
            (frame_cx,HEIGHT),
            (0,255,255),
            1
        )

        cv2.line(
            frame,
            (0,frame_cy),
            (WIDTH,frame_cy),
            (0,255,255),
            1
        )

        # ====================================
        # OBJECT FOUND
        # ====================================
        if contours:

            c = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(c)

            if area > MIN_AREA:

                rect = cv2.minAreaRect(c)

                (cx,cy),(rw,rh),angle = rect

                cx = int(cx)
                cy = int(cy)

                last_seen_x = cx

                box = cv2.boxPoints(rect)
                box = np.int32(box)

                cv2.drawContours(
                    frame,
                    [box],
                    0,
                    (0,255,0),
                    2
                )

                cv2.circle(
                    frame,
                    (cx,cy),
                    6,
                    (255,0,0),
                    -1
                )

                # ====================
                # ERROR
                # ====================
                error_x = cx - frame_cx
                error_y = frame_cy - cy

                # ====================
                # CENTERED
                # ====================
                if (
                    abs(error_x) < X_TOL and
                    abs(error_y) < Y_TOL
                ):

                    v = 0
                    w = 0

                    state = "CENTERED"

                else:

                    # 회전 제어
                    w = -KP_ROT * error_x

                    # 전진 제어
                    if abs(error_x) < 80:

                        v = KP_FORWARD * abs(error_y)

                        v = np.clip(
                            v,
                            MIN_V,
                            MAX_V
                        )

                    else:

                        v = MIN_V

                    state = "TRACK"

                send_cmd(v,w)

            else:

                stop_robot()
                state = "SMALL OBJECT"

        # ====================================
        # LOST TARGET
        # ====================================
        else:

            if last_seen_x > frame_cx:

                send_cmd(
                    0.05,
                    -0.35
                )

                state="SEARCH RIGHT"

            else:

                send_cmd(
                    0.05,
                    0.35
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

        cv2.imshow(
            "frame",
            frame
        )

        cv2.imshow(
            "mask",
            mask
        )

        if cv2.waitKey(1) & 0xFF == 27:

            break

except KeyboardInterrupt:

    pass

finally:

    stop_robot()

    cap.release()

    cv2.destroyAllWindows()
