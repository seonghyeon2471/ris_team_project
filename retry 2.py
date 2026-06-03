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
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

cap.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*'MJPG')
)

# =========================================
# CONTROL
# =========================================
MAX_V = 0.22
MIN_V = 0.06

KP_ROT = 0.004
MAX_W = 0.55

X_TOL = 15

MIN_AREA = 500
TARGET_AREA = 18000

# =========================================
# RED RANGE
# =========================================

# HSV red lower region
lower_red1 = np.array([0,70,70])
upper_red1 = np.array([12,255,255])

# HSV red upper region
lower_red2 = np.array([160,70,70])
upper_red2 = np.array([179,255,255])

# BGR filter
lower_bgr = np.array([40,20,120])
upper_bgr = np.array([210,170,255])

# =========================================
# MEMORY
# =========================================
last_seen_x = 160

# =========================================
# MOTOR
# =========================================
def send_cmd(v,w):

    v=np.clip(v,-0.3,0.3)
    w=np.clip(w,-0.8,0.8)

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():

    send_cmd(0,0)

# =========================================
# START
# =========================================
print("RED CENTER TRACK START")

try:

    while True:

        ret,frame = cap.read()

        if not ret:
            continue

        frame = cv2.flip(
            frame,
            1
        )

        HEIGHT, WIDTH = frame.shape[:2]

        frame_cx = WIDTH//2
        frame_cy = HEIGHT//2

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        # =====================================
        # MASK
        # =====================================

        mask1 = cv2.inRange(
            hsv,
            lower_red1,
            upper_red1
        )

        mask2 = cv2.inRange(
            hsv,
            lower_red2,
            upper_red2
        )

        hsv_mask = cv2.bitwise_or(
            mask1,
            mask2
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

        kernel = np.ones(
            (3,3),
            np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        contours,_ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # 화면 중심 표시
        cv2.circle(
            frame,
            (frame_cx,frame_cy),
            5,
            (0,255,255),
            -1
        )

        state="SEARCH"

        # =====================================
        # OBJECT FOUND
        # =====================================
        if contours:

            c=max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(c)

            if area > MIN_AREA:

                rect = cv2.minAreaRect(c)

                (cx,cy),(rw,rh),angle = rect

                cx=int(cx)
                cy=int(cy)

                last_seen_x = cx

                box = cv2.boxPoints(rect)
                box=np.int32(box)

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
                    5,
                    (255,0,0),
                    -1
                )

                error_x = cx - frame_cx

                # =====================
                # ARRIVED
                # =====================
                if (
                    abs(error_x) < X_TOL and
                    area > TARGET_AREA
                ):

                    v=0
                    w=0

                    state="ARRIVED"

                else:

                    w = -KP_ROT * error_x

                    distance_error = TARGET_AREA - area

                    v = distance_error * 0.00003

                    v = np.clip(
                        v,
                        MIN_V,
                        MAX_V
                    )

                    state="TRACK"

                send_cmd(v,w)

                cv2.putText(
                    frame,
                    f"A:{int(area)}",
                    (20,80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0,255,0),
                    2
                )

            else:

                stop_robot()

                state="SMALL"

        # =====================================
        # LOST TARGET
        # =====================================
        else:

            if last_seen_x > frame_cx:

                send_cmd(
                    0.04,
                    -0.32
                )

                state="SEARCH RIGHT"

            else:

                send_cmd(
                    0.04,
                    0.32
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

        key = cv2.waitKey(1)

        if key == 27:
            break

except KeyboardInterrupt:

    pass

finally:

    stop_robot()

    cap.release()

    cv2.destroyAllWindows()
