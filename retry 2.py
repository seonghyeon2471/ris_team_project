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

WIDTH  = 320
HEIGHT = 240

FRAME_CX = WIDTH // 2
FRAME_CY = HEIGHT // 2

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
    send_cmd(0.0,0.0)

# =========================================
# RED RANGE (좁힘)
# =========================================
LOWER_RED1 = np.array([0,110,120])
UPPER_RED1 = np.array([8,255,255])

LOWER_RED2 = np.array([170,110,120])
UPPER_RED2 = np.array([179,255,255])

MIN_AREA = 500

KP_ROT = 0.004

MOVE_DONE = False

# =========================================
# MAIN
# =========================================
try:

    while True:

        ret,frame = cap.read()

        if not ret:
            continue

        frame = cv2.flip(frame,1)

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        mask1 = cv2.inRange(
            hsv,
            LOWER_RED1,
            UPPER_RED1
        )

        mask2 = cv2.inRange(
            hsv,
            LOWER_RED2,
            UPPER_RED2
        )

        mask = cv2.bitwise_or(
            mask1,
            mask2
        )

        kernel=np.ones((5,5),np.uint8)

        mask=cv2.erode(
            mask,
            kernel,
            iterations=1
        )

        mask=cv2.dilate(
            mask,
            kernel,
            iterations=2
        )

        contours,_ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # 카메라 중심
        cv2.circle(
            frame,
            (FRAME_CX,FRAME_CY),
            6,
            (255,0,0),
            -1
        )

        if len(contours)>0:

            c=max(
                contours,
                key=cv2.contourArea
            )

            area=cv2.contourArea(c)

            if area>MIN_AREA:

                x,y,w,h = cv2.boundingRect(c)

                cx=x+w//2
                cy=y+h//2

                # 초록 박스
                cv2.rectangle(
                    frame,
                    (x,y),
                    (x+w,y+h),
                    (0,255,0),
                    2
                )

                # 물체 중심
                cv2.circle(
                    frame,
                    (cx,cy),
                    6,
                    (0,0,255),
                    -1
                )

                cv2.line(
                    frame,
                    (FRAME_CX,FRAME_CY),
                    (cx,cy),
                    (255,255,0),
                    2
                )

                error = cx - FRAME_CX

                print(
                    "CENTER ERROR:",
                    error
                )

                if not MOVE_DONE:

                    # 중심 안맞음
                    if abs(error) > 20:

                        rot = KP_ROT * error

                        send_cmd(
                            0.0,
                            -rot
                        )

                    else:

                        print(
                            "3cm MOVE"
                        )

                        send_cmd(
                            0.10,
                            0.0
                        )

                        time.sleep(0.35)

                        stop_robot()

                        MOVE_DONE=True

        else:

            stop_robot()

        cv2.imshow(
            "FRAME",
            frame
        )

        cv2.imshow(
            "MASK",
            mask
        )

        key=cv2.waitKey(1)

        if key==27:
            break

finally:

    stop_robot()

    cap.release()

    cv2.destroyAllWindows()

    arduino_ser.close()
